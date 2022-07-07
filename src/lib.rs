//! Impl of the UTP platform for the TI TM4C.
//!
//! TODO!

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

extern crate tm4c123x_hal as hal;
use hal::gpio::{AlternateFunction, AF1, PushPull};
use hal::gpio::gpioa::{PA1, PA0};
use hal::prelude::*;

use lc3_baseline_sim::interp::{Interpreter, MachineState, OwnedOrRef, PeripheralInterruptFlags};
use lc3_baseline_sim::sim::Simulator;
use lc3_device_support::{
    memory::PartialMemory,
    rpc::{
        encoding::{Cobs, PostcardDecode, PostcardEncode},
        transport::uart_simple::UartTransport,
    },
    util::Fifo,
};
use lc3_traits::control::rpc::{
    Device, RequestMessage, ResponseMessage, SimpleEventFutureSharedState, EventFuture,
};
use lc3_traits::control::Control;
use lc3_traits::peripherals::stubs::{ClockStub, GpioStub, PwmStub};
use lc3_traits::peripherals::{
    stubs::{/*PeripheralsStub,*/ InputStub, OutputStub},
    PeripheralSet,
};

// use hal::{gpio::*, gpio::gpioe::*};
use lc3_tm4c::peripherals_tm4c::{
    // gpio::{
    //     required_components as GpioComponents,
    //     physical_pins as Tm4cGpio,
    //     // GpioShim exists but it's not used for anything and doesn't impl Gpio?
    // },
    adc::{required_components as AdcComponents, AdcShim as Tm4cAdc},
    // clock::{required_components as ClockComponents, Tm4cClock},
    // pwm::{required_components as PwmComponents, PwmShim as Tm4cPwm},
    timers::{required_components as TimerComponents, TimersShim as Tm4cTimers},
};

// Unforuntately, this type alias is incomplete.
// use lc3_tm4c::peripherals_tm4c::Peripheralstm4c;

pub static FLAGS: PeripheralInterruptFlags = PeripheralInterruptFlags::new();

// type Tm4cPeripheralSet<'int> = PeripheralSet<
//     'int,
//     Tm4cGpio<'int>,
//     Tm4cAdc,
//     Tm4cPwm,
//     Tm4cTimers<'int>,
//     Tm4cClock,
//     InputStub,
//     OutputStub,
// >;

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
    let peripheral_set = {
        // let portf = p.GPIO_PORTF;
        // let portb = p.GPIO_PORTB;

        // let gpio = Tm4cGpio::new(
        //     &sc.power_control,
        //     GpioComponents {
        //         portf,
        //         portb,
        //     },
        // );
        let gpio = GpioStub;

        let adc0 = p.ADC0;
        let adc1 = p.ADC1;
        let porte = p.GPIO_PORTE;
        let adc = Tm4cAdc::new(&sc.power_control, AdcComponents { adc0, adc1, porte });

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

        let timer0 = p.TIMER0;
        let timer1 = p.TIMER1;
        let timers = Tm4cTimers::new(&sc.power_control, TimerComponents { timer0, timer1 });

        // let timer = p.TIMER2;
        // let clock = Tm4cClock::new(
        //     ClockComponents {
        //         timer,
        //     },
        //     &sc.power_control,
        // );
        let clock = ClockStub;

        PeripheralSet::new(gpio, adc, pwm, timers, clock, InputStub, OutputStub)
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

pub fn run<C: Control>(sim: &mut C, uart: Serial0) -> !
where
    <C as Control>::EventFuture: Unpin,
{
    let func: &dyn Fn() -> Cobs<Fifo<u8>> = &|| Cobs::try_new(Fifo::new()).unwrap();
    let enc = PostcardEncode::<ResponseMessage, _, _>::new(func);
    let dec = PostcardDecode::<RequestMessage, Cobs<Fifo<u8>>>::new();

    let (tx, rx) = uart.split();

    let mut device = Device::<UartTransport<_, _>, _, RequestMessage, ResponseMessage, _, _>::new(
        enc,
        dec,
        UartTransport::new(rx, tx),
    );

    loop {
        let _ = device.step(sim);
    }
}
