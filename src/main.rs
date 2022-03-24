#![no_std]
#![no_main]

use core::ops;
use cortex_m_rt::entry;
use defmt::*;
use embedded_hal::digital::v2::OutputPin;
use embedded_time::duration::*;
use embedded_time::rate::Extensions;
use is31fl3731::devices::CharlieBonnet;
use oorandom::Rand32;
use rp_pico::hal;
use rp_pico::hal::pac;
use rp_pico::hal::prelude::*;
use {defmt_rtt as _, panic_probe as _};

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
	let mut matrix = CharlieBonnet::configure(i2c);
	matrix.setup(&mut delay).expect("Failed to setup display");

	info!("Setup everything");
	pins.led
		.into_push_pull_output()
		.set_high()
		.expect("Failed to set LED to high");

	let mut rng = Rand32::new(0);
	loop {
		let x: u8 = rng
			.rand_range(ops::Range { start: 0, end: 16 })
			.try_into()
			.unwrap();
		let y: u8 = rng
			.rand_range(ops::Range { start: 0, end: 8 })
			.try_into()
			.unwrap();
		info!("Setting LED at ({}, {})", x, y);
		matrix.pixel(x, y, 5).expect("Failed to set pixel light on");
		delay.delay_ms(30);
		matrix
			.pixel(x, y, 0)
			.expect("Failed to set pixel light off");
	}
}
