use defmt::info;
use pio::{
    ArrayVec, Assembler, JmpCondition, MovDestination, MovOperation, MovSource, SideSet, WaitSource,
};
use rp2040_hal::{
    pac::{DMA, PIO0, RESETS},
    pio::{Buffers::OnlyTx, PIOBuilder, PIOExt, Running, Rx, StateMachine, Tx, PIO, SM0},
};

pub const NUM_PULSES_MAX: usize = 32;

enum Polarity {
    Rising,
    Falling,
}

pub struct EdgeTrigger {
    index: u8,
    polarity: Polarity,
    count: u32,
}

pub struct PulseParameter {
    delay: ArrayVec<u32, NUM_PULSES_MAX>,
    width: ArrayVec<u32, NUM_PULSES_MAX>,
}

impl PulseParameter {
    fn new() -> Self {
        Self {
            delay: ArrayVec::new(),
            width: ArrayVec::new(),
        }
    }
}

pub struct PulseGenerator {
    pio: PIO<PIO0>,
    sm: Option<StateMachine<(PIO0, SM0), Running>>,
    tx: Option<Tx<(PIO0, SM0)>>,
    rx: Option<Rx<(PIO0, SM0)>>,
    params: [PulseParameter; 2],
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
            sm: Some(sm),
            tx: Some(tx),
            rx: Some(rx),
            params: [PulseParameter::new(), PulseParameter::new()],
        }
    }

    pub fn check(&mut self) {
        let sm = self.sm.take().unwrap();
        let sm = sm.stop();
        info!("sm address: {}", sm.instruction_address());
        let sm = sm.start();
        self.sm = Some(sm);
    }

    pub fn arm(&mut self) {
        info!("arm");
        match self.sm.take() {
            Some(sm) => {
                let (sm, old) = sm.uninit(self.rx.take().unwrap(), self.tx.take().unwrap());
                self.pio.uninstall(old);
                let program = self.compile();
                info!("Program[3]: {}", program.code[3]);
                info!("install");
                let program = self.pio.install(&program).unwrap();
                info!("sm");
                let (sm, rx, mut tx) = PIOBuilder::from_installed_program(program)
                    .buffers(OnlyTx)
                    .side_set_pin_base(15)
                    .in_pin_base(0)
                    .build(sm);
                info!("edge");
                tx.write(0); // number of trigger edges
                info!("delay");
                tx.write(self.params[0].delay[0]);
                info!("width");
                tx.write(self.params[0].width[0]);

                let sm = sm.start();
                self.sm = Some(sm);
                self.tx = Some(tx);
                self.rx = Some(rx);
            }
            _ => unreachable!(),
        }
    }

    pub fn set_delay(&mut self, delay: u32) {
        self.params[0].delay.push(delay.saturating_sub(1));
    }

    pub fn set_width(&mut self, width: u32) {
        self.params[0].width.push(width.saturating_sub(1));
    }

    pub fn compile(&mut self) -> pio::Program<32> {
        let sideset = SideSet::new(true, 1, true);
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
