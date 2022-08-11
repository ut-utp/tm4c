//! Crude measurements of execution speed.

#![no_std]
#![no_main]

use core::fmt::Write;

use cortex_m_rt::entry;
use lc3_traits::control::{rpc::SimpleEventFutureSharedState, Control};
use panic_write::PanicHandler;
use tm4c123x_hal as hal;

use ubench::{metrics::*, reporters::*, *};
use utp_tm4c::*;

#[entry]
fn main() -> ! {
    let state: SimpleEventFutureSharedState = SimpleEventFutureSharedState::new();
    let (mut sim, aux) = setup(&state);

    let (mut porta_ctl, pa0, pa1, u0, pc, clocks) = aux;
    let uart0 = setup_uart(pa0, pa1, &mut porta_ctl, u0, &pc, &clocks);

    let mut uart0 = PanicHandler::new_with_hook(uart0, |w, panic_info| {
        writeln!(w, "\n{}", PANIC_DELIM).unwrap();
        writeln!(w, "{panic_info}").unwrap();
        writeln!(w, "{}", PANIC_DELIM).unwrap();
    });

    // TODO: replace this once we swap the peripherals out
    let mut core_p = unsafe { hal::CorePeripherals::steal() };
    // let mut core_p = hal::CorePeripherals::take().unwrap();

    let mut m = CortexMCycleCount::new(&mut core_p.DWT, &mut core_p.DCB).unwrap();
    let mut r = BasicReporter::new_with_serial::<u8, _, _>(&mut *uart0);

    // TODO! Add better tests!

    BenchmarkRunner::new()
        .set_iterations(20)
        .add(single(
            "empty execution",
            |i: &_| {
                for _ in 0..*i {
                    sim.step();
                }
            },
            (1..10).map(|i| 2u32.pow(i)),
        ))
        .run(&mut m, &mut r);

    // Signal success if we got here!
    writeln!(uart0, "\n{}", END_DELIM).unwrap();
    loop {}
}
