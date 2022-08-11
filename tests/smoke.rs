//! Test that the simulator can at least start without panicking.

#![no_std]
#![no_main]

use core::fmt::Write;

use cortex_m_rt::entry;
use panic_write::PanicHandler;
use lc3_traits::control::{Control, rpc::SimpleEventFutureSharedState};

use utp_tm4c::*;

#[entry]
fn main() -> ! {
    let state: SimpleEventFutureSharedState = SimpleEventFutureSharedState::new();
    let (mut sim, aux) = setup(&state);

    let (mut porta_ctl, pa0, pa1, u0, pc, clocks) = aux;
    let uart0 = setup_uart(pa0, pa1, &mut porta_ctl, u0, &pc, &clocks);

    let mut uart0 = PanicHandler::new_with_hook(
        uart0,
        |w, panic_info| {
            writeln!(w, "\n{}", PANIC_DELIM).unwrap();
            writeln!(w, "{panic_info}").unwrap();
            writeln!(w, "{}", PANIC_DELIM).unwrap();
        }
    );

    let _ = sim.step();

    // Signal success if we got here!
    writeln!(uart0, "\n{}", END_DELIM).unwrap();
    loop {}
}
