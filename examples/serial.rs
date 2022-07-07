//! UART test program, used to test buad rates.
//!
//! run with `cargo run --example serial`
//! `cargo run --manifest-path misc/serial/Cargo.toml -- -b <baud rate> -p <device path>` in `core` is the
//! host side counterpart

#![no_std]
#![no_main]

extern crate panic_write as _; // TODO?
extern crate tm4c123x_hal as hal;

use cortex_m_rt::entry;
use hal::prelude::*;

use core::fmt::Write;

#[entry]
fn main() -> ! {
    let p = hal::Peripherals::take().unwrap();

    let mut sc = p.SYSCTL.constrain();
    sc.clock_setup.oscillator = hal::sysctl::Oscillator::Main(
        hal::sysctl::CrystalFrequency::_16mhz,
        hal::sysctl::SystemClock::UsePll(hal::sysctl::PllOutputFrequency::_80_00mhz),
    );

    let clocks = sc.clock_setup.freeze();

    let mut porta = p.GPIO_PORTA.split(&sc.power_control);
    let u0 = p.UART0;

    // Activate UART
    let mut uart = hal::serial::Serial::uart0(
        u0,
        porta
            .pa1
            .into_af_push_pull::<hal::gpio::AF1>(&mut porta.control),
        porta
            .pa0
            .into_af_push_pull::<hal::gpio::AF1>(&mut porta.control),
        (),
        (),
        // 3_000_000_u32.bps(),
        // 1_000_000_u32.bps(),
        // 1_843_200_u32.bps(),
        // _000_000_u32.bps(),
        // 3_000_000_u32.bps(),
        // 3_686_400_u32.bps(),
        1_500_000.bps(),
        // 1_500_000.bps(),
        hal::serial::NewlineMode::SwapLFtoCRLF,
        // hal::serial::NewlineMode::Binary,
        &clocks,
        &sc.power_control,
    );

    #[inline(never)]
    #[no_mangle]
    extern "C" fn delay() {
        let mut _foo: u8 = 0;

        for _ in 0..1024 {
            _foo = unsafe { core::ptr::read_volatile(_foo as _) };
        }
    }

    writeln!(uart, "hello!").unwrap();

    const MSG: &[u8] = b"hello!\n";
    loop {
        // brrrrr
        uart.write_all(MSG);
        delay();
    }
}
