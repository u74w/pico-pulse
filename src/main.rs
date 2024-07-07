// Pico Pulse Generator
#![no_std]
#![no_main]

use bsp::entry;
use cortex_m::singleton;
use defmt::*;
use defmt_rtt as _;
use embedded_hal::digital::OutputPin;
use panic_probe as _;

use rp_pico as bsp;

use bsp::hal::{
    clocks::{Clock, ClockSource, ClocksManager},
    fugit::HertzU32,
    gpio::FunctionPio0,
    pac,
    pll::{common_configs::PLL_USB_48MHZ, setup_pll_blocking, PLLConfig},
    sio::Sio,
    usb::UsbBus,
    xosc::setup_xosc_blocking,
};

use usb_device::{
    bus::UsbBusAllocator,
    device::{StringDescriptors, UsbDeviceBuilder, UsbVidPid},
    LangID, UsbError,
};
use usbd_serial::{SerialPort, USB_CLASS_CDC};

mod pulse_generator;
use pulse_generator::{EdgePolarity, EdgeTrigger, PulseGenerator, Trigger};

// External high-speed crystal on the pico board is 12Mhz
const XTAL_FREQ: HertzU32 = HertzU32::MHz(12);
const PLL_SYS_250MHZ: PLLConfig = PLLConfig {
    vco_freq: HertzU32::MHz(1500),
    refdiv: 1,
    post_div1: 6,
    post_div2: 1,
};

#[entry]
fn main() -> ! {
    let mut pac = pac::Peripherals::take().unwrap();
    let sio = Sio::new(pac.SIO);

    let mut clocks = ClocksManager::new(pac.CLOCKS);

    let xosc = setup_xosc_blocking(pac.XOSC, XTAL_FREQ).unwrap();

    let pll_sys = setup_pll_blocking(
        pac.PLL_SYS,
        xosc.operating_frequency(),
        PLL_SYS_250MHZ,
        &mut clocks,
        &mut pac.RESETS,
    )
    .unwrap();

    let pll_usb = setup_pll_blocking(
        pac.PLL_USB,
        xosc.operating_frequency(),
        PLL_USB_48MHZ,
        &mut clocks,
        &mut pac.RESETS,
    )
    .unwrap();

    clocks
        .system_clock
        .configure_clock(&pll_sys, pll_sys.get_freq())
        .unwrap();
    info!(
        "System Clock: {} MHz",
        clocks.system_clock.get_freq().to_MHz()
    );

    clocks
        .usb_clock
        .configure_clock(&pll_usb, pll_usb.get_freq())
        .unwrap();

    clocks
        .peripheral_clock
        .configure_clock(&clocks.system_clock, clocks.system_clock.freq())
        .unwrap();

    let pins = bsp::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let usb_bus = UsbBus::new(
        pac.USBCTRL_REGS,
        pac.USBCTRL_DPRAM,
        clocks.usb_clock,
        true,
        &mut pac.RESETS,
    );
    let usb_bus: &'static UsbBusAllocator<UsbBus> =
        singleton!(: UsbBusAllocator<UsbBus> = UsbBusAllocator::new(usb_bus)).unwrap();
    let mut serial = SerialPort::new(usb_bus);
    let descriptor = StringDescriptors::new(LangID::EN_US).product("Pico-Pulse");
    let mut usb_dev = UsbDeviceBuilder::new(usb_bus, UsbVidPid(0x16c0, 0x27dd))
        .strings(&[descriptor])
        .unwrap()
        .device_class(USB_CLASS_CDC)
        .build();

    // init pins for PIO
    pins.gpio0.into_function::<FunctionPio0>();
    pins.gpio1.into_function::<FunctionPio0>();
    pins.gpio2.into_function::<FunctionPio0>();
    pins.gpio3.into_function::<FunctionPio0>();
    pins.gpio4.into_function::<FunctionPio0>();
    pins.gpio5.into_function::<FunctionPio0>();
    pins.gpio6.into_function::<FunctionPio0>();
    pins.gpio7.into_function::<FunctionPio0>();
    pins.gpio8.into_function::<FunctionPio0>();
    pins.gpio9.into_function::<FunctionPio0>();
    pins.gpio10.into_function::<FunctionPio0>();
    pins.gpio11.into_function::<FunctionPio0>();
    pins.gpio12.into_function::<FunctionPio0>();
    pins.gpio13.into_function::<FunctionPio0>();
    pins.gpio14.into_function::<FunctionPio0>();
    pins.gpio15.into_function::<FunctionPio0>();
    pins.gpio16.into_function::<FunctionPio0>();
    pins.gpio17.into_function::<FunctionPio0>();
    pins.gpio18.into_function::<FunctionPio0>();
    pins.gpio19.into_function::<FunctionPio0>();
    pins.gpio20.into_function::<FunctionPio0>();
    pins.gpio21.into_function::<FunctionPio0>();
    pins.gpio22.into_function::<FunctionPio0>();

    let mut led_pin = pins.led.into_push_pull_output();
    led_pin.set_high().unwrap();

    let trigger = Trigger::Edge(EdgeTrigger {
        index: 0,
        polarity: EdgePolarity::Rising,
    });
    let mut pulse_gen = PulseGenerator::new(pac.PIO0, pac.DMA, trigger, &mut pac.RESETS);
    pulse_gen.ch0.set_trigger_edge_count(1);
    pulse_gen.ch0.set_delay(10_000);
    pulse_gen.ch0.set_width(10_000);
    pulse_gen.ch0.set_delay(20_000);
    pulse_gen.ch0.set_width(20_000);
    pulse_gen.ch0.set_delay(30_000);
    pulse_gen.ch0.set_width(30_000);
    pulse_gen.ch0.arm().unwrap();

    pulse_gen.ch1.set_trigger_edge_count(1);
    pulse_gen.ch1.set_delay(20_000);
    pulse_gen.ch1.set_width(10_000);
    pulse_gen.ch1.set_delay(40_000);
    pulse_gen.ch1.set_width(15_000);
    pulse_gen.ch1.arm().unwrap();

    loop {
        if !usb_dev.poll(&mut [&mut serial]) {
            continue;
        }

        let mut buf = [0u8; 64];
        match serial.read(&mut buf[..]) {
            Ok(_count) => {
                serial.write(&buf).unwrap();
                info!("{}", pulse_gen.ch0.triggered());
            }
            Err(UsbError::WouldBlock) => {} // No data received
            Err(_err) => {}                 // An error occurred
        };
    }
}
