use pio::{
    ArrayVec, Assembler, JmpCondition, MovDestination, MovOperation, MovSource, SideSet,
    WaitSource, RP2040_MAX_PROGRAM_SIZE,
};
use rp2040_hal::{
    pac::{DMA, PIO0, RESETS},
    pio::{
        Buffers::OnlyTx, PIOBuilder, PIOExt, PinDir, Running, Rx, StateMachine, Tx,
        ValidStateMachine, PIO, PIO0SM0, PIO0SM1,
    },
};

pub const NUM_PULSES_MAX: usize = 32;

pub enum EdgePolarity {
    Rising,
    Falling,
}

pub struct EdgeTrigger {
    pub index: u8,
    pub polarity: EdgePolarity,
    pub count: u32,
}

pub enum Trigger {
    Edge(EdgeTrigger),
    Immediate,
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

pub struct PulseGeneratorChannel<SM>
where
    SM: ValidStateMachine,
{
    sm: Option<StateMachine<SM, Running>>,
    tx: Option<Tx<SM>>,
    rx: Option<Rx<SM>>,
    param: PulseParameter,
}

impl<SM: ValidStateMachine> Default for PulseGeneratorChannel<SM> {
    fn default() -> Self {
        Self {
            sm: None,
            tx: None,
            rx: None,
            param: PulseParameter::new(15),
        }
    }
}

impl<SM: ValidStateMachine> PulseGeneratorChannel<SM> {
    pub fn set_delay(&mut self, cycle: u32) {
        self.param.delay.push(cycle.saturating_sub(1));
    }

    pub fn set_width(&mut self, cycle: u32) {
        self.param.width.push(cycle.saturating_sub(1));
    }

    pub fn arm(&mut self) {
        let sm = self.sm.take().unwrap();
        let sm = sm.stop();
        let mut tx = self.tx.take().unwrap();
        tx.write(0);
        tx.write(self.param.delay[0]);
        tx.write(self.param.width[0]);
        let sm = sm.start();
        self.sm = Some(sm);
        self.tx = Some(tx);
    }
}

pub struct PulseGenerator {
    pio: PIO<PIO0>,
    pub ch0: PulseGeneratorChannel<PIO0SM0>,
    pub ch1: PulseGeneratorChannel<PIO0SM1>,
    trigger: Trigger,
}

impl PulseGenerator {
    pub fn new(pio: PIO0, _dma: DMA, trigger: Trigger, resets: &mut RESETS) -> Self {
        let (mut pio, sm0, _, _, _) = pio.split(resets);

        // Install program
        let program = assemble(&trigger);
        let installed_program = pio.install(&program).unwrap();

        // Configure SM0
        let (mut sm0, rx, tx) = PIOBuilder::from_installed_program(installed_program)
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
                rx: Some(rx),
                param: PulseParameter::new(15),
            },
            ch1: PulseGeneratorChannel {
                sm: None,
                tx: None,
                rx: None,
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
