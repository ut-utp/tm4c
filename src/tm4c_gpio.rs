use lc3_traits::peripherals::gpio::{self as lc3_gp};
use lc3_traits::peripherals::gpio::{GpioPinArr, GpioMiscError};
use core::sync::atomic::AtomicBool;

use tm4c123x_hal::gpio::{
    self as gp,
    PullUp,
    gpiof::{self, PF0, PF1, PF2, PF3, PF4},
    gpiob::*,
    gpioc::*,
    gpiod::*,
    gpioa::*,
    Tristate
};
//use tm4c123x_hal::gpio::Tristate;

pub struct Tm4cGpio{
    p0: PA2<Tristate>,
    p1: PA3<Tristate>,
    p2: PA4<Tristate>,
    p3: PB2<Tristate>,
    p4: PB3<Tristate>, 
    p5: PB6<Tristate>, 
    p6: PB7<Tristate>, 
    p7: PC4<Tristate>,
    p8: PC5<Tristate>,
    p9: PC6<Tristate>,
    p10: PC7<Tristate>,
    p11: PD2<Tristate>,
    p12: PD3<Tristate>,
    p13: PD6<Tristate>,
    p14: PF0<Tristate>,
    p15: PF1<Tristate>, 
    p16: PF2<Tristate>,
    p17: PF3<Tristate>,
    p18: PF4<Tristate>,
    p19: PB5<Tristate>,
    p20: PB0<Tristate>,
    p21: PB1<Tristate>,
    p22: PB2<Tristate>,
    p23: PB4<Tristate>,

}

impl Tm4cGpio{
    pub fn new(p0: PA2<Tristate>, p1: PA3<Tristate>, p2: PA4<Tristate>, p3: PB2<Tristate>, p4: PB3<Tristate>, p5: PB6<Tristate>, p6: PB7<Tristate>, p7: PC4<Tristate>,
               p8: PC5<Tristate>, p9: PC6<Tristate>, p10: PC7<Tristate>, p11: PD2<Tristate>, p12: PD3<Tristate>, p13: PD6<Tristate>, p14: PF0<Tristate>, p15: PF1<Tristate>, 
               p16: PF2<Tristate>, p17: PF3<Tristate>, p18: PF4<Tristate>) -> Self{
        Self{

        }
    }
}

#[allow(clippy::toplevel_ref_arg)]
impl <'i> lc3_gp::Gpio<'i> for Tm4cGpio
{
    fn set_state(&mut self, pin: lc3_gp::GpioPin, state: lc3_gp::GpioState) -> Result<(), GpioMiscError> {
        // pin_proxy!(
        //     (self) pins[pin] as ref mut p => {
        //         p.set_state(state, self.ctx.as_mut())
        //     }
        // )
        unimplemented!()
    }

    fn get_state(&self, pin: lc3_gp::GpioPin) -> lc3_gp::GpioState {
        //pin_proxy!((self) pins[pin] as ref p => p.get_state())
        unimplemented!()
    }

    // TODO: change ReadError to be more specific? not sure
    //
    // definitely also allow for "Other" errors...
    //
    // also don't let us return the pin, it doesn't make any sense to..
    fn read(&self, pin: lc3_gp::GpioPin) -> Result<bool, lc3_gp::GpioReadError> {
        //pin_proxy!((self) pins[pin] as ref p => p.read())
        unimplemented!()
    }


    fn write(&mut self, pin: lc3_gp::GpioPin, bit: bool) -> Result<(), lc3_gp::GpioWriteError> {
        unimplemented!()
    }


    fn register_interrupt_flags(&mut self, _flags: & 'i GpioPinArr<AtomicBool>) {
        /* todo: remove this! */
    }


    fn interrupt_occurred(&self, pin: lc3_gp::GpioPin) -> bool {
        unimplemented!()
    }


    fn reset_interrupt_flag(&mut self, pin: lc3_gp::GpioPin) {
        unimplemented!()
    }

}