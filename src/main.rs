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

    let (porta, u0, pc, clocks) = aux;
    let uart0 = setup_uart(porta, u0, &pc, &clocks);

    run(&mut sim, uart0)
}
