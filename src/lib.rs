//! Impl of the UTP platform for the TI TM4C.
//!
//! TODO!

//! ## Mappings
//! Gonna do this for pins for now:
//!
//! ```text
//!          [For reference]            |              [Assignments]           |
//!                                     |                                      |
//!    J1    J3           J4    J2      |     J1    J3           J4    J2      |
//!  ┏━━━━━┯━━━━━┓      ┏━━━━━┯━━━━━┓   |   ┏━━━━━┯━━━━━┓      ┏━━━━━┯━━━━━┓   |
//!  ┃ 3.3 │ 5.0 ┃      ┃ PF2 │ GND ┃   |   ┃ 3.3 │ 5.0 ┃      ┃ GA1 │ GND ┃   |
//!  ┠─────┼─────┨      ┠─────┼─────┨   |   ┠─────┼─────┨      ┠─────┼─────┨   |
//!  ┃ PB5 │ GND ┃      ┃ PF3 │ PB2 ┃   |   ┃ GC5 │ GND ┃      ┃ GA2 │ GC2 ┃   |
//!  ┠─────┼─────┨      ┠─────┼─────┨   |   ┠─────┼─────┨      ┠─────┼─────┨   |
//!  ┃ PB0 │ PD0 ┃      ┃ PB3 │ PE0 ┃   |   ┃ GC0 │ --- ┃      ┃ GC3 │  A0 ┃   |
//!  ┠─────┼─────┨      ┠─────┼─────┨   |   ┠─────┼─────┨      ┠─────┼─────┨   |
//!  ┃ PB1 │ PD1 ┃      ┃ PC4 │ PF0 ┃   |   ┃ GC1 │ GA7*┃      ┃ GA4 │ GA3 ┃   |
//!  ┠─────┼─────┨      ┠─────┼─────┨   |   ┠─────┼─────┨      ┠─────┼─────┨   |
//!  ┃ PE4 │ PD2 ┃      ┃ PC5 │ RST ┃   |   ┃  A4 │ GB0 ┃      ┃ GA5 │ RST ┃   |
//!  ┠─────┼─────┨      ┠─────┼─────┨   |   ┠─────┼─────┨      ┠─────┼─────┨   |
//!  ┃ PE5 │ PD3 ┃      ┃ PC6 │ PB7 ┃   |   ┃  A5 │ GB1 ┃      ┃  P0 │ GC7 ┃   |
//!  ┠─────┼─────┨      ┠─────┼─────┨   |   ┠─────┼─────┨      ┠─────┼─────┨   |
//!  ┃ PB4 │ PE1 ┃      ┃ PC7 │ PB6 ┃   |   ┃ GC4 │  A1 ┃      ┃  P1 │ GC6 ┃   |
//!  ┠─────┼─────┨      ┠─────┼─────┨   |   ┠─────┼─────┨      ┠─────┼─────┨   |
//!  ┃ PA5 │ PE2 ┃      ┃ PD6 │ PA4 ┃   |   ┃ GB5 │  A2 ┃      ┃BT_TX│ GB5 ┃   |
//!  ┠─────┼─────┨      ┠─────┼─────┨   |   ┠─────┼─────┨      ┠─────┼─────┨   |
//!  ┃ PA6 │ PE3 ┃      ┃ PD7 │ PA3 ┃   |   ┃ GB6 │  A3 ┃      ┃BT_RX│ GB3 ┃   |
//!  ┠─────┼─────┨      ┠─────┼─────┨   |   ┠─────┼─────┨      ┠─────┼─────┨   |
//!  ┃ PA7 │ PF1 ┃      ┃ PF4 │ PA2 ┃   |   ┃ GB7 │ GA0 ┃      ┃ GA4 │ GB2 ┃   |
//!  ┗━━━━━┷━━━━━┛      ┗━━━━━┷━━━━━┛   |   ┗━━━━━┷━━━━━┛      ┗━━━━━┷━━━━━┛   |
//!        ┏━━━━━┓      ┏━━━━━┓         |         ┏━━━━━┓      ┏━━━━━┓         |
//!        ┃ GND ┃      ┃ GND ┃         |         ┃ GND ┃      ┃ GND ┃         |
//!        ┠─────┨      ┠─────┨         |         ┠─────┨      ┠─────┨         |
//!        ┃ GND ┃      ┃ GND ┃         |         ┃ GND ┃      ┃ GND ┃         |
//!        ┠─────┨      ┠─────┨         |         ┠─────┨      ┠─────┨         |
//!        ┃ 5.0 ┃      ┃ 3.3 ┃         |         ┃ 5.0 ┃      ┃ 3.3 ┃         |
//!        ┗━━━━━┛      ┗━━━━━┛         |         ┗━━━━━┛      ┗━━━━━┛         |
//!  ┏━━━━━┯━━━━━┓      ┏━━━━━┓         |   ┏━━━━━┯━━━━━┓      ┏━━━━━┓         |
//!  ┃ PD0<->PB6 ┃      ┃D7^VD┃         |   ┃ PD0<->PB6 ┃      ┃D7^VD┃         |
//!  ┠─────┼─────┨      ┗━━━━━┛         |   ┠─────┼─────┨      ┗━━━━━┛         |
//!  ┃ PD1<->PB7 ┃                      |   ┃ PD1<->PB7 ┃                      |
//!  ┗━━━━━┷━━━━━┛                      |   ┗━━━━━┷━━━━━┛                      |
//!                                     |                                      |
//! ------------------------------------|--------------------------------------|
//! ```
//!
//! #### Gpio
//!
//! | Pin # | Pin Name | Hardware Pin |
//! |:-----:|:--------:| :----------: |
//! |   0   |   GA0    |     PF1      | # Red     (TODO: add suggestion that G0 be an LED)
//! |   1   |   GA1    |     PF2      | # Green
//! |   2   |   GA2    |     PF3      | # Blue
//! |   3   |   GA3    |     PF0      | # Button
//! |   4   |   GA4    |     PF4      | # Button
//! |   5   |   GA5    |     PC4      |           (pessimizing GPIO port a here...)
//! |   6   |   GA6    |     PC5      |
//! |   7   |   GA7    |     PD1*     | # Tied to PB7 aka GC7...
//! |   8   |   GB0    |     PD2      |
//! |   9   |   GB1    |     PD3      |
//! |  10   |   GB2    |     PA2      | # Matches Port A
//! |  11   |   GB3    |     PA3      |
//! |  12   |   GB4    |     PA4      |
//! |  13   |   GB5    |     PA5      |
//! |  14   |   GB6    |     PA6      |
//! |  15   |   GB7    |     PA7      |
//! |  16   |   GC0    |     PB0      | # Matches Port B
//! |  17   |   GC1    |     PB1      |
//! |  18   |   GC2    |     PB2      |
//! |  19   |   GC3    |     PB3      |
//! |  20   |   GC4    |     PB4      |
//! |  21   |   GC5    |     PB5      |
//! |  22   |   GC6    |     PB6      |
//! |  23   |   GC7    |     PB7*     | # Tied to PD1 aka GA7 (i.e. if you want to use the very last pin, remove the shunt resistors)
//!
//! #### Adc
//!
//! | Pin # | TM4C Analog Input | Hardware Pin |
//! |:-----:| :---------------: | :----------: |
//! |  A0   |       AIN3        |     PE0      | # Matches Port E
//! |  A1   |       AIN2        |     PE1      |
//! |  A2   |       AIN1        |     PE2      |
//! |  A3   |       AIN0        |     PE3      |
//! |  A4   |       AIN9        |     PE4      |
//! |  A5   |       AIN8        |     PE5      |
//!
//! #### Pwm
//!
//! | Pin # | TM4C PWM Output | Hardware Pin |
//! |:-----:| :-------------: | :----------: |
//! |   P0  |      M0PWM6     |     PC6      | # Specifically *not* using LED pins for this.
//! |   P1  |      M0PWM7     |     PC7      |
//!
//! #### Misc
//!
//! Keeping PD6 and PD7 reserved for a secondary UART (debugging, bluetooth, etc).
//!

#![doc(test(attr(deny(rust_2018_idioms, warnings))))]
#![doc(html_logo_url = "")] // TODO!
#![no_std]

#[cfg(not(all(target_arch = "arm", target_os = "none")))]
compile_error!(
    "

    This crate only builds for `thumbv7em-none-eabihf`!

    Please either pass `--target thumbv7em-none-eabihf` to `cargo` or
    use one of the aliases (like `cargo r` to run) defined in `.cargo/config`.


"
);

pub const PANIC_DELIM: &str = "++++++++++";
pub const END_DELIM: &str = "==========";

extern crate tm4c123x_hal as hal;

use core::fmt::Write;

// use crate::flash::Flash_Unit;
// use crate::paging::RAM_Pages;

use core::convert::Infallible;

use hal::gpio::{AlternateFunction, IsUnlocked, PushPull, Tristate, AF1};
use hal::prelude::*;


use lc3_traits::control::rpc::{
    SimpleEventFutureSharedState, Device, RequestMessage, ResponseMessage, EventFuture
};
use lc3_baseline_sim::interp::{
    Interpreter,
    PeripheralInterruptFlags, OwnedOrRef, MachineState,
};
use lc3_baseline_sim::sim::Simulator;
use lc3_traits::control::Control;
use lc3_traits::peripherals::stubs::PwmStub;
use lc3_traits::peripherals::{
    PeripheralSet,
    stubs::{
        /*PeripheralsStub,*/ InputStub, OutputStub
    },
};
use lc3_device_support::{
    memory::PartialMemory,
    rpc::{
        transport::uart_simple::UartTransport,
        encoding::{PostcardEncode, PostcardDecode, Cobs},
    },
    peripherals::adc::generic_adc_unit as GenericAdc,
    peripherals::timer::generic_timer_unit as GenericTimer,
    peripherals::clock::generic_clock_unit as GenericClock,
    util::Fifo,
};

mod generic_gpio;
mod flash;
mod paging;
mod memory_trait_RAM_flash;
mod tm4c_clock;


// GPIO Board specifics

use tm4c123x_hal::gpio::{
    self as gp,
    gpioa::{self, PA0, PA1, PA2, PA3, PA4, PA5, PA6, PA7},
    gpiob::{self, PB0, PB1, PB2, PB3, PB4, PB5, PB6, PB7},
    gpioc::{self, PC4, PC5 /* , PC6, PC7 */},
    gpiod::{self, PD1, PD2, PD3},
    gpiof::{self, PF0, PF1, PF2, PF3, PF4},
    PullUp,
};

generic_gpio::multi_bank! {
    for banks {
        //! TODO: module doc comment!
        for pins {
            /** ... (red)   */ PF1 as GA0, /** ... (green) */ PF2 as GA1,
            /** ... (blue)  */ PF3 as GA2, /** ... button  */ PF0 as GA3,
            /** ... button  */ PF4 as GA4, /** ...         */ PC4 as GA5,
            /** ...         */ PC5 as GA6, /** ...         */ PD1 as GA7,
        } as GpioBankA;

        //! TODO: module doc comment!
        for pins {
            /** ...         */ PD2 as GB0, /** ...         */ PD3 as GB1,
            /** ...         */ PA2 as GB2, /** ...         */ PA3 as GB3,
            /** ...         */ PA4 as GB4, /** ...         */ PA5 as GB5,
            /** ...         */ PA6 as GB6, /** ...         */ PA7 as GB7,
        } as GpioBankB;

        //! TODO: module doc comment!
        for pins {
            /** ...         */ PB0 as GC0, /** ...         */ PB1 as GC1,
            /** ...         */ PB2 as GC2, /** ...         */ PB3 as GC3,
            /** ...         */ PB4 as GC4, /** ...         */ PB5 as GC5,
            /** ...         */ PB6 as GC6, /** ...         */ PB7 as GC7,
        } as GpioBankC;
    }: generic_gpio::io_pins_with_typestate {

        type Ctx = ();
        type Error = Infallible;

        type Disabled = gp::Tristate;
        type Input = gp::Input<PullUp>;
        type Output = gp::Output<PushPull>;

        => disabled = |x, ()| Ok(x.into_tri_state())
        => input    = |x, ()| Ok(x.into_pull_up_input())
        => output   = |x, ()| Ok(x.into_push_pull_output())

        => enable  interrupts = |inp, ()| Ok(inp.set_interrupt_mode(gp::InterruptMode::EdgeRising))
        => disable interrupts = |inp, ()| {
            inp.clear_interrupt();
            Ok(inp.set_interrupt_mode(gp::InterruptMode::Disabled))
        }

        => interrupts {
            check = |i, ()| i.get_interrupt_status();
            reset = |i, ()| i.clear_interrupt();
        }
    }
}

//Timer board Specifics
extern crate embedded_time;
use embedded_time as hal_time;
use hal_time::duration::Milliseconds;
use hal_time::fixed_point::FixedPoint;

use tm4c123x_hal::timer::*;
use tm4c123x_hal::time::*;

pub struct MillisU16(Millis);

impl Into<Millis> for MillisU16 {
    fn into(self) -> Millis{
        self.0
    }
}
impl From<Milliseconds> for MillisU16{
    fn from(val: Milliseconds) -> Self {
        MillisU16(u32::millis(val.integer()))
    }
}

impl Into<Milliseconds> for MillisU16{
    fn into(self) -> Milliseconds{
        Milliseconds::new(self.0.0)
    }

}

//Clock board Specifics
// struct WrappingU16(u16);

// impl From<u64> for WrappingU16{
//     fn from(val: u64) -> Self {
//         unimplemented!()
//     }
// }

// impl Into<u16> for WrappingU16{
//     fn into(self) -> u16{
//         unimplemented!()
//     }

// }

pub static FLAGS: PeripheralInterruptFlags = PeripheralInterruptFlags::new();

type PortA = tm4c123x::GPIO_PORTA;
type Uart0 = hal::serial::UART0;
type PowerControl = hal::sysctl::PowerControl;
type Clocks = hal::sysctl::Clocks;

pub fn setup(
    state: &SimpleEventFutureSharedState,
) -> (impl Control<EventFuture = EventFuture<'_, SimpleEventFutureSharedState>> + '_, (PortA, Uart0, PowerControl, Clocks)) {
    let p = hal::Peripherals::take().unwrap();

    let mut sc = p.SYSCTL.constrain();
    sc.clock_setup.oscillator = hal::sysctl::Oscillator::Main(
        hal::sysctl::CrystalFrequency::_16mhz,
        hal::sysctl::SystemClock::UsePll(hal::sysctl::PllOutputFrequency::_80_00mhz),
    );

    let clocks = sc.clock_setup.freeze();

    // Peripheral Init:
    // Peripheral Init:
    let peripheral_set = {
        let mut portf = p.GPIO_PORTF.split(&sc.power_control);
        let pf0 = portf.pf0.unlock(&mut portf.control);  //pf0 is special pin to be unlocked
        let portb = p.GPIO_PORTB;

        let gpiof::Parts { pf1: g1, pf2: g2, pf3: g3, pf4: g4, .. } = portf;
        let gpiob::Parts { pb5: g5, pb6: g6, pb7: g7, .. } = portb.split(&sc.power_control);
        let gpio = Tm4cGpio::new(pf0, g1, g2, g3, g4, g5, g6, g7);


        let porte = p.GPIO_PORTE.split(&sc.power_control);
        let pe3 = porte.pe3.into_analog_state();
        let pe2 = porte.pe2.into_analog_state();
        let pe1 = porte.pe1.into_analog_state();
        let pe0 = porte.pe0.into_analog_state();
        let pe5 = porte.pe5.into_analog_state();
        let pe4 = porte.pe4.into_analog_state();
        let adc_unit = hal::adc::Adc::adc0(p.ADC0, &sc.power_control);
        let adc = GenericAdc::new(adc_unit, pe3, pe2, pe1, pe0, pe5, pe4);

        // let portb = unsafe { hal::Peripherals::steal() }.GPIO_PORTB;
        // let portd = p.GPIO_PORTD;
        // let pwm0 = p.PWM0;
        // let pwm1 = p.PWM1;
        // Note: This will spin forever if you make the mistake of using an `lm4f` (which
        // does not have PWM...).
        // Perhaps we should have a `feature` for this at the very least?
        // let pwm = Tm4cPwm::new(
        //     PwmComponents {
        //         portb,
        //         portd,
        //         pwm0,
        //         pwm1,
        //     },
        //     &sc.power_control,
        // );
        let pwm = PwmStub;

        let tm4c_timer0 = Timer::<tm4c123x::WTIMER0>::wtimer0(p.WTIMER0, MillisU16(Millis(4)), &sc.power_control, &clocks);
        let tm4c_timer1 = Timer::<tm4c123x::WTIMER1>::wtimer1(p.WTIMER1, MillisU16(Millis(4)), &sc.power_control, &clocks);

        let utp_timer = GenericTimer::<MillisU16, _, _, _>::new(tm4c_timer0, tm4c_timer1);
        let timers = utp_timer;

        let tm4c_clock = tm4c_clock::Tm4cClock::new(p.WTIMER2, &sc.power_control, &clocks);
        let utp_clock = GenericClock::<_, u64>::new(tm4c_clock);
        let clock = utp_clock;

        PeripheralSet::new(
            gpio,
            adc,
            pwm,
            timers,
            clock,
            InputStub,
            OutputStub,
        )
    };

    static mut MEMORY: PartialMemory = PartialMemory::new();

    let interp: Interpreter<'static, _, _> = Interpreter::new(
        // SAFETY: we have exclusive access!
        unsafe {
            &mut MEMORY
        },
        peripheral_set,
        OwnedOrRef::Ref(&FLAGS),
        [0; 8],
        0x200,
        MachineState::Running,
    );

    let sim = Simulator::new_with_state(interp, state);
    let aux = (p.GPIO_PORTA, p.UART0, sc.power_control, clocks);

    (sim, aux)
}

pub type Serial0 = hal::serial::Serial<
    tm4c123x::UART0,
    PA1<AlternateFunction<AF1, PushPull>>,
    PA0<AlternateFunction<AF1, PushPull>>,
    (),
    (),
>;
pub fn setup_uart(porta: PortA, u0: Uart0, pc: &PowerControl, clocks: &Clocks) -> Serial0 {
    let mut porta = porta.split(pc);

    // Activate UART
    hal::serial::Serial::uart0(
        u0,
        porta
            .pa1
            .into_af_push_pull::<hal::gpio::AF1>(&mut porta.control),
        porta
            .pa0
            .into_af_push_pull::<hal::gpio::AF1>(&mut porta.control),
        (),
        (),
        1_500_000_u32.bps(),
        // hal::serial::NewlineMode::SwapLFtoCRLF,
        hal::serial::NewlineMode::Binary,
        &clocks,
        pc,
    )
}

// TODO: do away with this once we update `core`.
struct MutRefWrite<'w, W: embedded_hal::serial::Write<u8>>(&'w mut W);
impl<'w, W: embedded_hal::serial::Write<u8>> embedded_hal::serial::Write<u8> for MutRefWrite<'w, W> {
    type Error = W::Error;

    fn write(&mut self, word: u8) -> nb::Result<(), Self::Error> {
        self.0.write(word)
    }

    fn flush(&mut self) -> nb::Result<(), Self::Error> {
        self.0.flush()
    }
}

pub fn run<C: Control>(sim: &mut C, uart: Serial0) -> !
where
    <C as Control>::EventFuture: Unpin,
{
    let func: &dyn Fn() -> Cobs<Fifo<u8>> = &|| Cobs::try_new(Fifo::new()).unwrap();
    let enc = PostcardEncode::<ResponseMessage, _, _>::new(func);
    let dec = PostcardDecode::<RequestMessage, Cobs<Fifo<u8>>>::new();

    let (tx, rx) = uart.split();

    let mut tx = panic_write::PanicHandler::new_with_hook(tx, |w, panic_info| {
        // TODO: multi-plexing friendly hook!

        writeln!(w, "\n{}", PANIC_DELIM).unwrap();
        writeln!(w, "{panic_info}").unwrap();
        writeln!(w, "{}", PANIC_DELIM).unwrap();
    });
    let tx = MutRefWrite(&mut *tx);

    let mut device = Device::<UartTransport<_, _>, _, RequestMessage, ResponseMessage, _, _>::new(
        enc,
        dec,
        UartTransport::new(rx, tx),
    );

    loop {
        let _ = device.step(sim);
    }
}
