#![no_std]
#![no_main]

use cortex_m_rt::entry;

use embedded_time::duration::*;
use embedded_time::rate::Extensions;

use panic_probe as _;

use rp_pico::hal::prelude::*;

use rp_pico::hal::pac;

use rp_pico::hal;

use defmt::*;
use defmt_rtt as _;

use is31fl3731::devices::CharlieBonnet;

#[entry]
fn main() -> ! {
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();

    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);

    let clocks = hal::clocks::init_clocks_and_plls(
        rp_pico::XOSC_CRYSTAL_FREQ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let sio = hal::Sio::new(pac.SIO);

    let pins = rp_pico::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let i2c = hal::I2C::i2c0(
        pac.I2C0,
        pins.gpio16.into_mode::<hal::gpio::FunctionI2C>(),
        pins.gpio17.into_mode::<hal::gpio::FunctionI2C>(),
        100.kHz(),
        &mut pac.RESETS,
        clocks.peripheral_clock,
    );

    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().integer());
    let mut matrix = CharlieBonnet::configure(i2c, &mut delay);
    matrix.setup().expect("Failed to setup display");

    info!("Setup everything");

    matrix
        .fill(1, None, 0)
        .expect("Failed to fill matrix with display brightness of 1");

    loop {}
}
