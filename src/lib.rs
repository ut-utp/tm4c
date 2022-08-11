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

use core::cell::RefCell;
use core::fmt::Write;

// use crate::flash::Flash_Unit;
// use crate::paging::RAM_Pages;

use core::convert::Infallible;
use core::panic::PanicInfo;

use hal::gpio::{AlternateFunction, IsUnlocked, PushPull, Tristate, AF1};
use hal::prelude::*;

use lc3_baseline_sim::interp::{Interpreter, MachineState};
use lc3_baseline_sim::sim::Simulator;
use lc3_device_support::rpc::encoding::DynFifoBorrow;
use lc3_device_support::util::PeripheralInterruptFlags;
use lc3_device_support::{
    memory::PartialMemory,
    peripherals::adc::GenericAdc,
    peripherals::clock::GenericClock,
    peripherals::timer::GenericTimers,
    rpc::{
        encoding::{Cobs, PostcardDecode, PostcardEncode},
        transport::uart_simple::device::UartTransport,
    },
    util::Fifo,
};
use lc3_traits::control::rpc::{
    Device, EventFuture, RequestMessage, ResponseMessage, SimpleEventFutureSharedState,
};
use lc3_traits::control::Control;
use lc3_traits::peripherals::stubs::PwmStub;
use lc3_traits::peripherals::{
    stubs::{/*PeripheralsStub,*/ InputStub, OutputStub},
    PeripheralSet,
};

mod flash;
mod generic_gpio;
mod memory_trait_RAM_flash;
mod paging;
pub mod tm4c_clock;

use panic_write::PanicHandler;
use tm4c_clock::Tm4cTimerTrait;

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

use tm4c123x_hal::time::*;
use tm4c123x_hal::timer::*;

#[derive(Clone, Copy)]
pub struct MillisU16(pub Millis);

impl Into<Millis> for MillisU16 {
    fn into(self) -> Millis {
        self.0
    }
}
impl From<Milliseconds> for MillisU16 {
    fn from(val: Milliseconds) -> Self {
        MillisU16(u32::millis(val.integer()))
    }
}

impl Into<Milliseconds> for MillisU16 {
    fn into(self) -> Milliseconds {
        Milliseconds::new(self.0 .0)
    }
}

pub static FLAGS: PeripheralInterruptFlags = PeripheralInterruptFlags::new();

type PortACtrl = hal::gpio::gpioa::GpioControl;
type Uart0 = hal::serial::UART0;
type PowerControl = hal::sysctl::PowerControl;
type Clocks = hal::sysctl::Clocks;

pub fn setup(
    state: &SimpleEventFutureSharedState,
) -> (
    impl Control<EventFuture = EventFuture<'_, SimpleEventFutureSharedState>> + '_,
    (
        PortACtrl,
        PA0<Tristate>,
        PA1<Tristate>,
        Uart0,
        PowerControl,
        Clocks,
    ),
) {
    let p = hal::Peripherals::take().unwrap();

    let mut sc = p.SYSCTL.constrain();
    sc.clock_setup.oscillator = hal::sysctl::Oscillator::Main(
        hal::sysctl::CrystalFrequency::_16mhz,
        hal::sysctl::SystemClock::UsePll(hal::sysctl::PllOutputFrequency::_80_00mhz),
    );

    let clocks = sc.clock_setup.freeze();
    let porta = p.GPIO_PORTA.split(&sc.power_control);
    let gpioa::Parts {
        pa0,
        pa1,
        pa2,
        pa3,
        pa4,
        pa5,
        pa6,
        pa7,
        control: porta_control,
    } = porta;

    // Peripheral Init:
    let peripheral_set = {
        let (gpio_a, gpio_b, gpio_c) = {
            let portb = p.GPIO_PORTB;
            let portc = p.GPIO_PORTC;
            let portd = p.GPIO_PORTD;
            let portf = p.GPIO_PORTF;

            let (gb2, gb3, gb4, gb5, gb6, gb7) = (pa2, pa3, pa4, pa5, pa6, pa7);
            let gpiob::Parts {
                pb0: gc0,
                pb1: gc1,
                pb2: gc2,
                pb3: gc3,
                pb4: gc4,
                pb5: gc5,
                pb6: gc6,
                pb7: gc7,
                control: _,
            } = portb.split(&sc.power_control);
            let gpioc::Parts {
                pc4: ga5, pc5: ga6, ..
            } = portc.split(&sc.power_control);
            let gpiod::Parts {
                pd1: ga7,
                pd2: gb0,
                pd3: gb1,
                ..
            } = portd.split(&sc.power_control);
            let gpiof::Parts {
                pf0,
                pf1: ga0,
                pf2: ga1,
                pf3: ga2,
                pf4: ga4,
                control: mut portf_control,
                ..
            } = portf.split(&sc.power_control);

            let pf0 = pf0.unlock(&mut portf_control); // pf0 is special pin to be unlocked
            let ga3 = pf0;

            let gpio_a = GpioBankA::new(ga0, ga1, ga2, ga3, ga4, ga5, ga6, ga7);
            let gpio_b = GpioBankB::new(gb0, gb1, gb2, gb3, gb4, gb5, gb6, gb7);
            let gpio_c = GpioBankC::new(gc0, gc1, gc2, gc3, gc4, gc5, gc6, gc7);

            (gpio_a, gpio_b, gpio_c)
        };

        let adc = {
            let porte = p.GPIO_PORTE.split(&sc.power_control);
            let pe0 = porte.pe0.into_analog_state();
            let pe1 = porte.pe1.into_analog_state();
            let pe2 = porte.pe2.into_analog_state();
            let pe3 = porte.pe3.into_analog_state();
            let pe4 = porte.pe4.into_analog_state();
            let pe5 = porte.pe5.into_analog_state();
            let adc_unit = hal::adc::Adc::adc0(p.ADC0, &sc.power_control);
            GenericAdc::<12, _, _, _, _, _, _, _, _, _>::new(adc_unit, pe0, pe1, pe2, pe3, pe4, pe5)
        };

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

        let timers = {
            let tm4c_timer0 = Timer::<tm4c123x::WTIMER0>::wtimer0(
                p.WTIMER0,
                MillisU16(Millis(400_000)),
                &sc.power_control,
                &clocks,
            );
            let tm4c_timer1 = Timer::<tm4c123x::WTIMER1>::wtimer1(
                p.WTIMER1,
                MillisU16(Millis(400_000)),
                &sc.power_control,
                &clocks,
            );

            GenericTimers::<MillisU16, _, _, _>::new(tm4c_timer0, tm4c_timer1)
        };

        let clock = {
            let timer2 = Timer::<tm4c123x::WTIMER2>::new_checked(
                p.WTIMER2,
                MillisU16(Millis(65_536)),
                &sc.power_control,
                &clocks,
            )
            .unwrap();
            GenericClock::new(timer2.as_clock())
        };

        PeripheralSet::new(gpio_a, adc, pwm, timers, clock, InputStub, OutputStub)
            .with_gpio_bank_b(gpio_b)
            .with_gpio_bank_c(gpio_c)
    };

    static mut MEMORY: PartialMemory = PartialMemory::new();

    let interp: Interpreter<_, _> = Interpreter::new(
        // SAFETY: we have exclusive access!
        unsafe { &mut MEMORY },
        peripheral_set,
        [0; 8],
        0x200,
        MachineState::Running,
    );

    let sim = Simulator::new_with_state(interp, state);
    let aux = (porta_control, pa0, pa1, p.UART0, sc.power_control, clocks);

    (sim, aux)
}

pub type Serial0 = hal::serial::Serial<
    tm4c123x::UART0,
    PA1<AlternateFunction<AF1, PushPull>>,
    PA0<AlternateFunction<AF1, PushPull>>,
    (),
    (),
>;
pub type Serial0TxInner =
    hal::serial::Tx<tm4c123x::UART0, PA1<AlternateFunction<AF1, PushPull>>, ()>;
pub type PanicHandlerFunc = fn(&mut Serial0TxInner, &PanicInfo);
pub type Serial0Tx = PanicHandler<Serial0TxInner, PanicHandlerFunc>;
pub type Serial0Rx = hal::serial::Rx<tm4c123x::UART0, PA0<AlternateFunction<AF1, PushPull>>, ()>;

pub fn setup_uart(
    pa0: PA0<impl IsUnlocked>,
    pa1: PA1<impl IsUnlocked>,
    porta_control: &mut PortACtrl,
    u0: Uart0,
    pc: &PowerControl,
    clocks: &Clocks,
) -> Serial0 {
    // Activate UART
    hal::serial::Serial::uart0(
        u0,
        pa1.into_af_push_pull::<hal::gpio::AF1>(porta_control),
        pa0.into_af_push_pull::<hal::gpio::AF1>(porta_control),
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
impl<'w, W: embedded_hal::serial::Write<u8>> embedded_hal::serial::Write<u8>
    for MutRefWrite<'w, W>
{
    type Error = W::Error;

    fn write(&mut self, word: u8) -> nb::Result<(), Self::Error> {
        self.0.write(word)
    }

    fn flush(&mut self) -> nb::Result<(), Self::Error> {
        self.0.flush()
    }
}

fn panic_handler_func(w: &mut Serial0TxInner, panic_info: &PanicInfo) {
    // TODO: multi-plexing friendly hook!
    writeln!(w, "\n{}", PANIC_DELIM).unwrap();
    writeln!(w, "{panic_info}").unwrap();
    writeln!(w, "{}", PANIC_DELIM).unwrap();
}

// Note: doesn't currently give you the uart back.
pub fn with_panic_handler<R>(
    uart: Serial0,
    func: impl FnOnce(&mut Serial0TxInner, Serial0Rx) -> R,
) -> R {
    let (tx, rx) = uart.split();

    let handler: PanicHandlerFunc = panic_handler_func;
    let tx = panic_write::PanicHandler::new_with_hook(tx, handler);
    pin_utils::pin_mut!(tx);
    tx.register();
    // TODO: fix unsafety in ^; we can move the Pin!
    // TODO: when register took `Pin<&mut Self>` we seemed to be able to
    // make a copy of the Pin????

    func(tx.get_inner(), rx)
}

pub fn run<C: Control>(sim: &mut C, uart: Serial0) -> !
where
    <C as Control>::EventFuture: Unpin,
{
    let fifo = RefCell::new(<Fifo<_>>::new());
    let fifo = &fifo;
    let enc = PostcardEncode::<ResponseMessage, Cobs<DynFifoBorrow<'_, 256>>, _>::new(|| Cobs::try_new(DynFifoBorrow(fifo.borrow_mut())).unwrap());

    // let mut fifo = &mut fifo;
    // let fifo: &&mut _ = &fifo;
    // let mut fifo = FifoBorrow(&mut fifo);
    // // let func: &dyn Fn() -> Cobs<Fifo<u8>> = &|| Cobs::try_new(Fifo::new()).unwrap();

    // struct FifoBorrow<'f>(&'f mut Fifo<u8>);
    // impl<'f> FifoBorrow<'f> {
    //     fn get<'b>(&'b mut self) -> &'b mut Fifo<u8>
    //     where
    //         'f: 'b
    //     {
    //         self.0
    //     }
    // }

    // fn func_cast<'fifo, F: FnMut() -> &'fifo mut Fifo<u8>>(func: F) -> F { func }
    // let func: &dyn for<'a> FnMut() -> &'a mut Fifo<u8> = (|| {
    //     let reborrow = &mut *fifo;
    //     reborrow
    // }) as _;

    // let func: &dyn Fn() -> Cobs<Fifo<u8>> = &|| Cobs::try_new(Fifo::new()).unwrap();
    // let enc = PostcardEncode::<ResponseMessage, _, _>::new(func);

    let dec = PostcardDecode::<RequestMessage, Cobs<Fifo<u8>>>::new();

    with_panic_handler(uart, |tx, rx| {
        let tx = MutRefWrite(tx);

        let mut device =
            Device::<UartTransport<_, _>, _, RequestMessage, ResponseMessage, _, _>::new(
                enc,
                dec,
                UartTransport::new(rx, tx),
            );

        loop {
            let _ = device.step(sim);
        }
    })
}
