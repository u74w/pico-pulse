use cortex_m::singleton;
use defmt::info;
use pio::{
    ArrayVec, Assembler, JmpCondition, MovDestination, MovOperation, MovSource, SideSet,
    WaitSource, RP2040_MAX_PROGRAM_SIZE,
};
use rp2040_hal::{
    dma::{
        self, single_buffer, Channel, ChannelIndex, DMAExt, ReadTarget, SingleChannel, WriteTarget,
        CH0, CH1,
    },
    pac::{DMA, PIO0, RESETS},
    pio::{
        Buffers::OnlyTx, PIOBuilder, PIOExt, PinDir, Running, StateMachine, Tx, ValidStateMachine,
        PIO, PIO0SM0, PIO0SM1,
    },
};

pub const NUM_PULSES_MAX: usize = 32;
const DMA_BUF_LEN: usize = NUM_PULSES_MAX * 2 + 1;

pub enum EdgePolarity {
    Rising,
    Falling,
}

pub struct EdgeTrigger {
    pub index: u8,
    pub polarity: EdgePolarity,
}

pub enum Trigger {
    Edge(EdgeTrigger),
    Immediate,
}

pub struct PulseParameter {
    index: u8,
    delay: ArrayVec<u32, NUM_PULSES_MAX>,
    width: ArrayVec<u32, NUM_PULSES_MAX>,
    trigger_edge_count: u32,
}

impl PulseParameter {
    fn new(index: u8) -> Self {
        Self {
            index,
            delay: ArrayVec::new(),
            width: ArrayVec::new(),
            trigger_edge_count: 0,
        }
    }
}

pub struct PulseGeneratorChannel<SM, CH>
where
    SM: ValidStateMachine,
    CH: SingleChannel,
{
    sm: Option<StateMachine<SM, Running>>,
    tx: Option<Tx<SM>>,
    dma_ch: Option<CH>,
    tx_transfer: Option<single_buffer::Transfer<CH, &'static mut [u32; 5], Tx<SM>>>,
    param: PulseParameter,
}

impl<SM: ValidStateMachine, CH: SingleChannel> Default for PulseGeneratorChannel<SM, CH> {
    fn default() -> Self {
        Self {
            sm: None,
            tx: None,
            dma_ch: None,
            tx_transfer: None,
            param: PulseParameter::new(15),
        }
    }
}

impl<SM: ValidStateMachine, CH: SingleChannel> PulseGeneratorChannel<SM, CH> {
    pub fn set_delay(&mut self, cycle: u32) {
        self.param.delay.push(cycle.saturating_sub(1));
    }

    pub fn set_width(&mut self, cycle: u32) {
        self.param.width.push(cycle.saturating_sub(1));
    }

    pub fn set_trigger_edge_count(&mut self, count: u32) {
        self.param.trigger_edge_count = count.saturating_sub(1);
    }

    pub fn arm(&mut self) -> Result<(), ()> {
        if self.param.delay.len() != self.param.width.len() {
            return Err(());
        }

        let sm = self.sm.take().unwrap();
        let sm = sm.stop();
        let tx = self.tx.take().unwrap();

        let mut buf: ArrayVec<u32, 5> = ArrayVec::new();
        buf.push(self.param.trigger_edge_count);

        loop {
            if let Some(delay) = self.param.delay.pop_at(0) {
                buf.push(delay);
            }
            if let Some(width) = self.param.width.pop_at(0) {
                buf.push(width);
                continue;
            }
            break;
        }

        info!("{:?}", buf.as_slice());

        let dma_ch = self.dma_ch.take().unwrap();

        //let buf = buf.into_inner().unwrap();
        let buf = buf.into_inner().unwrap();
        info!("{}", buf);
        let tx_buf = singleton!(: [u32; 5] = buf).unwrap();

        let tx_transfer = single_buffer::Config::new(dma_ch, tx_buf, tx).start();

        let sm = sm.start();
        self.sm = Some(sm);
        self.tx_transfer = Some(tx_transfer);
        Ok(())
    }
}

pub struct PulseGenerator {
    pio: PIO<PIO0>,
    pub ch0: PulseGeneratorChannel<PIO0SM0, Channel<CH0>>,
    pub ch1: PulseGeneratorChannel<PIO0SM1, Channel<CH1>>,
    trigger: Trigger,
}

impl PulseGenerator {
    pub fn new(pio: PIO0, dma: DMA, trigger: Trigger, resets: &mut RESETS) -> Self {
        let (mut pio, sm0, _, _, _) = pio.split(resets);
        let dma = dma.split(resets);

        // Install program
        let program = assemble(&trigger);
        let installed_program = pio.install(&program).unwrap();

        // Configure SM0
        let (mut sm0, _, tx) = PIOBuilder::from_installed_program(installed_program)
            .buffers(OnlyTx)
            .side_set_pin_base(15)
            .in_pin_base(0)
            .build(sm0);
        sm0.set_pindirs([(15, PinDir::Output)]);
        let sm0 = sm0.start();
        Self {
            pio,
            ch0: PulseGeneratorChannel {
                sm: Some(sm0),
                tx: Some(tx),
                dma_ch: Some(dma.ch0),
                tx_transfer: None,
                param: PulseParameter::new(15),
            },
            ch1: PulseGeneratorChannel {
                sm: None,
                tx: None,
                dma_ch: None,
                tx_transfer: None,
                param: PulseParameter::new(16),
            },
            trigger,
        }
    }
}

pub fn assemble(trigger: &Trigger) -> pio::Program<RP2040_MAX_PROGRAM_SIZE> {
    let sideset = SideSet::new(true, 1, false);
    let mut asm: Assembler<RP2040_MAX_PROGRAM_SIZE> = Assembler::new_with_side_set(sideset);

    match trigger {
        Trigger::Edge(edge_trigger) => {
            // Get number of edges before triggering
            asm.pull(false, true);
            asm.mov(MovDestination::Y, MovOperation::None, MovSource::OSR);

            // Wait number of edges
            let mut edge_label = asm.label();
            asm.bind(&mut edge_label);
            match edge_trigger.polarity {
                EdgePolarity::Falling => {
                    asm.wait(1, WaitSource::PIN, 0, false);
                    asm.wait(0, WaitSource::PIN, 0, false);
                }
                EdgePolarity::Rising => {
                    asm.wait(0, WaitSource::PIN, 0, false);
                    asm.wait(1, WaitSource::PIN, 0, false);
                }
            }
            asm.jmp(JmpCondition::YDecNonZero, &mut edge_label);
        }
        Trigger::Immediate => (),
    }

    // Get delay cycles
    let mut loop_label = asm.label();
    asm.bind(&mut loop_label);
    asm.pull(false, true);
    asm.mov(MovDestination::X, MovOperation::None, MovSource::OSR);

    // Get width cycles
    asm.pull(false, true);
    asm.mov(MovDestination::Y, MovOperation::None, MovSource::OSR);

    // Wait delay cycles
    let mut delay_label = asm.label();
    asm.bind(&mut delay_label);
    asm.jmp(JmpCondition::XDecNonZero, &mut delay_label);

    // Wait width cycles (Pulse High)
    let mut width_label = asm.label();
    asm.bind(&mut width_label);
    asm.jmp_with_side_set(JmpCondition::YDecNonZero, &mut width_label, 1);

    // Loop (Pulse Low)
    asm.jmp_with_side_set(JmpCondition::Always, &mut loop_label, 0);

    asm.assemble_program().set_origin(Some(0))
}
