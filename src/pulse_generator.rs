use defmt::info;
use pio::{
    ArrayVec, Assembler, JmpCondition, MovDestination, MovOperation, MovSource, SideSet, WaitSource,
};
use rp2040_hal::{
    pac::{DMA, PIO0, RESETS},
    pio::{
        Buffers::OnlyTx, PIOBuilder, PIOExt, PinDir, Running, Rx, StateMachine, StateMachineIndex,
        Tx, ValidStateMachine, PIO, PIO0SM0, PIO0SM1, SM0, SM1, SM2, SM3,
    },
};

pub const NUM_PULSES_MAX: usize = 32;

enum EdgePolarity {
    Rising,
    Falling,
}

pub struct EdgeTrigger {
    index: u8,
    polarity: EdgePolarity,
    count: u32,
}

pub struct PulseParameter {
    index: u8,
    delay: ArrayVec<u32, NUM_PULSES_MAX>,
    width: ArrayVec<u32, NUM_PULSES_MAX>,
}

impl PulseParameter {
    fn new(index: u8) -> Self {
        Self {
            index,
            delay: ArrayVec::new(),
            width: ArrayVec::new(),
        }
    }
}

pub enum ChannelIndex {
    CH0,
    CH1,
    CH2,
    CH3,
}

struct PulseGeneratorChannel<SM>
where
    SM: ValidStateMachine,
{
    sm: Option<StateMachine<SM, Running>>,
    tx: Option<Tx<SM>>,
    rx: Option<Rx<SM>>,
    param: PulseParameter,
}

impl<SM: ValidStateMachine> PulseGeneratorChannel<SM> {
    fn set_delay(&mut self, cycle: u32) {
        self.param.delay.push(cycle.saturating_sub(1));
    }

    fn set_width(&mut self, cycle: u32) {
        self.param.width.push(cycle.saturating_sub(1));
    }
}

pub struct PulseGenerator {
    pio: PIO<PIO0>,
    pub ch0: PulseGeneratorChannel<PIO0SM0>,
    pub ch1: PulseGeneratorChannel<PIO0SM1>,
}

impl PulseGenerator {
    pub fn new(pio: PIO0, _dma: DMA, resets: &mut RESETS) -> Self {
        let (mut pio, sm0, _, _, _) = pio.split(resets);
        let mut asm = Assembler::new();
        asm.push(true, true);
        let program = pio.install(&asm.assemble_program()).unwrap();
        let (sm, rx, tx) = PIOBuilder::from_installed_program(program)
            .buffers(OnlyTx)
            .build(sm0);
        let sm = sm.start();
        Self {
            pio,
            ch0: PulseGeneratorChannel {
                sm: Some(sm),
                tx: Some(tx),
                rx: Some(rx),
                param: PulseParameter::new(15),
            },
            ch1: PulseGeneratorChannel {
                sm: None,
                tx: None,
                rx: None,
                param: PulseParameter::new(16),
            },
        }
    }

    pub fn check(&mut self) {
        let sm = self.ch0.sm.take().unwrap();
        let sm = sm.stop();
        info!("sm address: {}", sm.instruction_address());
        let sm = sm.start();
        self.ch0.sm = Some(sm);
    }

    pub fn arm(&mut self) {
        info!("arm");
        match self.ch0.sm.take() {
            Some(sm) => {
                let (sm, old) = sm.uninit(self.ch0.rx.take().unwrap(), self.ch0.tx.take().unwrap());
                self.pio.uninstall(old);
                let program = self.compile();
                info!("Program[3]: {}", program.code[3]);
                info!("install");
                let program = self.pio.install(&program).unwrap();
                info!("sm");
                let (mut sm, rx, mut tx) = PIOBuilder::from_installed_program(program)
                    .buffers(OnlyTx)
                    .side_set_pin_base(15)
                    .in_pin_base(0)
                    .build(sm);
                sm.set_pindirs([(15, PinDir::Output)]);
                info!("edge");
                tx.write(0); // number of trigger edges
                info!("delay");
                tx.write(self.ch0.param.delay[0]);
                info!("width");
                tx.write(self.ch0.param.width[0]);

                let sm = sm.start();
                self.ch0.sm = Some(sm);
                self.ch0.tx = Some(tx);
                self.ch0.rx = Some(rx);
            }
            _ => unreachable!(),
        }
    }

    pub fn set_delay(&mut self, delay: u32) {
        self.ch0.param.delay.push(delay.saturating_sub(1));
    }

    pub fn set_width(&mut self, width: u32) {
        self.ch0.param.width.push(width.saturating_sub(1));
    }

    pub fn compile(&mut self) -> pio::Program<32> {
        let sideset = SideSet::new(true, 1, false);
        let mut asm: Assembler<32> = Assembler::new_with_side_set(sideset);

        // Get number of edges before triggering
        asm.pull(false, true);
        asm.mov(MovDestination::Y, MovOperation::None, MovSource::OSR);

        // Wait number of edges
        let mut edge_label = asm.label();
        asm.bind(&mut edge_label);
        asm.wait(0, WaitSource::PIN, 0, false);
        asm.wait(1, WaitSource::PIN, 0, false);
        asm.jmp(JmpCondition::YDecNonZero, &mut edge_label);

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
}
