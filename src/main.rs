// #![feature(generic_associated_types)]

//! Impl of the UTP platform for the TI TM4C.

#![no_std]
#![no_main]

use cortex_m_rt::entry;
use lc3_traits::control::rpc::SimpleEventFutureSharedState;
use utp_tm4c::*;

#[entry]
fn main() -> ! {
    let state: SimpleEventFutureSharedState = SimpleEventFutureSharedState::new();
    let (mut sim, aux) = setup(&state);

    let (mut porta_ctrl, pa0, pa1, u0, pc, clocks) = aux;
    let uart0 = setup_uart(pa0, pa1, &mut porta_ctrl, u0, &pc, &clocks);

    run(&mut sim, uart0)
}
