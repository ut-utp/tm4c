#![no_std]
#![no_main]

extern crate tm4c123x_hal as hal;

use core::fmt::Write;

use cortex_m_rt::entry;
use hal::gpio::GpioExt;
use hal::prelude::*;

use utp_tm4c::*;
use utp_tm4c::tm4c_clock::{Tm4cTimerTrait, get_innards};
use tm4c123x_hal::time::Millis;

#[entry]
fn main() -> ! {
    let p = hal::Peripherals::take().unwrap();

    let mut sc = p.SYSCTL.constrain();
    sc.clock_setup.oscillator = hal::sysctl::Oscillator::Main(
        hal::sysctl::CrystalFrequency::_16mhz,
        hal::sysctl::SystemClock::UsePll(hal::sysctl::PllOutputFrequency::_80_00mhz),
    );
    let clocks = sc.clock_setup.freeze();

    let hal::gpio::gpioa::Parts { control: mut porta_ctrl, pa0, pa1, .. } = p.GPIO_PORTA.split(&sc.power_control);
    let u0 = p.UART0;

    let uart = setup_uart(pa0, pa1, &mut porta_ctrl, u0, &sc.power_control, &clocks);
    with_panic_handler(uart, |tx, _rx| {
        let mut timer2_raw = Some(p.WTIMER2);

        let mut test_period = |period_in_ms| {
            let mut timer2 = hal::timer::Timer::<tm4c123x::WTIMER2>::new_checked(timer2_raw.take().unwrap(), MillisU16(Millis(period_in_ms)), &sc.power_control, &clocks).unwrap();
            timer2.count_up();

            writeln!(tx, "Testing: {period_in_ms}!").unwrap();
            let inner = unsafe { get_innards(&timer2) };
            let reload_a = inner.tim.tailr.read().bits();
            let reload_b = inner.tim.tbilr.read().bits();
            // inner.tim.tamr.write(|w| w.tamr());
            writeln!(tx, "A: {reload_a:#12} ({reload_a:#12X})").unwrap();
            writeln!(tx, "B: {reload_b:#12} ({reload_b:#12X})").unwrap();

            let mut max = 0;
            let mut no_new_max = 0;
            loop {
                // writeln!(uart, "{:?}", timer2.read_raw());
                let new = timer2.read_raw();
                if new > max {
                    max = new;
                    write!(tx, "{:#12} ({:#10})     \r", max, (max / (period_in_ms as u64))).unwrap();

                    no_new_max = 0;
                } else {
                    writeln!(tx, "{:#12} ({:#10}) {:#12}", max, (max / (period_in_ms as u64)), new).unwrap();
                    no_new_max += 1;

                    if no_new_max == 5 {
                        writeln!(tx, "-----------------------------").unwrap();
                        break;
                    }
                }
            }

            timer2_raw = Some(timer2.free());
        };

        for p in [10, 100, 1000, 10_000, 50_000, 65_536, 100_000] {
            test_period(p);
        }

        writeln!(tx, "\nFIN").unwrap();
        loop { }
    })
}
