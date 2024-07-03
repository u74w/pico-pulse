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
    dma::DMAExt,
    fugit::{HertzU32, RateExtU32},
    pac,
    pio::PIOExt,
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
use pulse_generator::PulseGenerator;

// External high-speed crystal on the pico board is 12Mhz
const XTAL_FREQ_HZ: u32 = 12_000_000;
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

    let xosc = setup_xosc_blocking(pac.XOSC, XTAL_FREQ_HZ.Hz()).unwrap();

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

    let mut led_pin = pins.led.into_push_pull_output();
    led_pin.set_high().unwrap();

    let (pio, sm0, sm1, _, _) = pac.PIO0.split(&mut pac.RESETS);
    let dma = pac.DMA.split(&mut pac.RESETS);
    let mut pulse_gen = PulseGenerator::new(pio, sm0, sm1, dma.ch0, dma.ch1);
    pulse_gen.set_delay(10);
    pulse_gen.set_width(10);
    pulse_gen.arm();

    loop {
        if !usb_dev.poll(&mut [&mut serial]) {
            continue;
        }

        let mut buf = [0u8; 64];
        match serial.read(&mut buf[..]) {
            Ok(_count) => {
                serial.write(&buf).unwrap();
            }
            Err(UsbError::WouldBlock) => {} // No data received
            Err(_err) => {}                 // An error occurred
        };
    }
}
