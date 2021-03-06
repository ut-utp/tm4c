use core::num::NonZeroU8;
use lc3_traits::peripherals::pwm::{
    Pwm, PwmPin, PwmPinArr, PwmState,
};

extern crate tm4c123x;

//use hal::Pwm;
use core::sync::atomic::{AtomicBool, Ordering};
use tm4c123x_hal::gpio;
use tm4c123x_hal::gpio::*;
use tm4c123x_hal::gpio::{gpiob::*, gpioe::*};
use tm4c123x_hal::sysctl::Clocks;
use tm4c123x_hal::time::MegaHertz;
use tm4c123x_hal::{prelude::*, Peripherals};
use tm4c123x_hal::sysctl;
//use core::num::NonZeroU8;
//use core::ops::{Index, IndexMut};

static PWM_SHIM_PINS: PwmPinArr<AtomicBool> =
    PwmPinArr([AtomicBool::new(false), AtomicBool::new(false)]);

pub enum PhysicalPins {
    p0(tm4c123x_hal::gpio::gpiob::PB6<AlternateFunction<AF4, PushPull>>),
    p1(tm4c123x_hal::gpio::gpiob::PB7<AlternateFunction<AF4, PushPull>>),
}

pub struct PwmShim {
    states: PwmPinArr<PwmState>,
    duty_cycle: PwmPinArr<u8>,
   // pwm_physical_pins: [PhysicalPins; 2],
    //pub pwm0: tm4c123x::PWM0,
    //pub pwm1: tm4c123x::PWM1,
 //   components: Option<required_components>,
    //guards: PwmPinArr<Option<timer::Guard>>,
}

pub struct required_components {
   // pub sysctl: tm4c123x::SYSCTL,
    pub pb6: PB6<AlternateFunction<AF4, PushPull>>,
    pub pb7: PB7<AlternateFunction<AF4, PushPull>>,
    pub pwm0: tm4c123x::PWM0,
    pub pwm1: tm4c123x::PWM1,
    //pub pb7: tm4c123x_hal::gpio::gpiob::PB7<AlternateFunction<AF::4, PushPull>>,

}

impl Default for PwmShim {
    fn default() -> Self {
       unimplemented!()
    }
}

impl PwmShim {
    pub fn new(mut peripheral_set: required_components, power: &sysctl::PowerControl) -> Self {
       // let sys = peripheral_set.sysctl.constrain();


        let p = unsafe { &*tm4c123x::SYSCTL::ptr() };
        sysctl::control_power(
            power, sysctl::Domain::Pwm0,
            sysctl::RunMode::Run, sysctl::PowerState::On);
        sysctl::reset(power, sysctl::Domain::Pwm0);
       // sysctl::control_power(
       //      power, sysctl::Domain::Pwm1,
       //      sysctl::RunMode::Run, sysctl::PowerState::On);
       //  sysctl::reset(power, sysctl::Domain::Pwm1);
        let pb6 = peripheral_set.pb6; //pwm0 pb6
        //let pb7 = portd.pb7.into_af_push_pull::<gpio::AF4>(&mut portb.control); //pwm0 pb7
      //  let pd0 = portd.pd0.into_af_push_pull::<gpio::AF4>(&mut portd.control); //pwm0 pb7
        let pb7 = peripheral_set.pb7; //pwm0 pb7
        //let pd1 = portd.pd1.into_af_push_pull::<gpio::AF5>(&mut portd.control); //pwm0 pb7
        // let pwm_divider = p
        //     .rcc
        //     .write(|w| unsafe { w.bits((p.rcc.read().bits() & !0x000E0000) | (0x00100000)) });
        //let portb_sysctl = peripheral_set.sysctl.rcgcgpio.write(|w| unsafe{w.bits(2)});
        peripheral_set.pwm0.ctl.write(|w| unsafe { w.bits(0) });
      //  peripheral_set.pwm1.ctl.write(|w| unsafe { w.bits(0) });
        peripheral_set
            .pwm0
            ._0_gena
            .write(|w| unsafe { w.bits(0x00C8) });
        peripheral_set
            .pwm0
            ._0_genb
            .write(|w| unsafe { w.bits(0x0C08) });

        // peripheral_set
        //     .pwm1
        //     ._1_gena
        //     .write(|w| unsafe { w.bits(0xC8) });
        // peripheral_set
        //     .pwm1
        //     ._1_genb
        //     .write(|w| unsafe { w.bits(0x0C8) });

        Self {
            states: PwmPinArr([PwmState::Disabled; PwmPin::NUM_PINS]),
            duty_cycle: PwmPinArr([0; PwmPin::NUM_PINS]), // start with duty_cycle low
            // components: Some(required_components{
            //     pb6:   pb6,
            //     pb7:   pb7,
            //     pwm0:  peripheral_set.pwm0,
            //     pwm1:  peripheral_set.pwm1,



            // }),
            // pwm_physical_pins: [PhysicalPins::p0(pb6), PhysicalPins::p1(pb7)],
            // pwm0: ,
            // pwm1: ,
            //                                               //components: Some(peripheral_set),
                                                          // guards: PwmPinArr([None, None]),
        }
    }
    pub fn get_pin_state(&self, pin: PwmPin) -> PwmState {
        self.states[pin].into()
    }
}
impl Pwm for PwmShim {
    fn set_state(&mut self, pin: PwmPin, state: PwmState) {
        use PwmState::*;
        let x = usize::from(pin);
        match x {
            0 => {
                 match state {
                Enabled(dut) =>{
                        //let NonZeroU8(extract)=dut;
                        let p = unsafe { &*tm4c123x::PWM0::ptr() };
                        p._0_load.write(|w| unsafe{w.bits(255)});
                     //   p._1_load.write(|w| unsafe{w.bits(0x018F)});
                        p._0_cmpa.write(|w| unsafe{w.bits(dut.get().into())});
                        p._0_ctl.write(|w| unsafe{w.bits(p._0_ctl.read().bits() | 1)});
                    //    p.enable
                      //      .write(|w| unsafe { w.bits(p.enable.read().bits() & !3) });
                        p.enable
                            .write(|w| unsafe { w.bits(p.enable.read().bits() | 1) });                  

                    }
                    
                    Disabled => {
                        let p = unsafe { &*tm4c123x::PWM0::ptr() };
                        //let p = Peripherals::take().unwrap().PWM1;
                        p.enable
                            .write(|w| unsafe { w.bits(p.enable.read().bits() & !1) });
                    }
                }
                // match state {
                //     Enabled(_) => {
                //         //let p = unsafe { &*tm4c123x::PWM0::ptr() };
                //         let mut handle = {
                //             unsafe {
                //                 core::mem::replace(
                //                     &mut self.pwm_physical_pins[0],
                //                     core::mem::uninitialized(),
                //                 )
                //             }
                //         };

                //         match handle {
                //             PhysicalPins::p0(pin_out) =>{
                //                  self.pwm0._0_load.write(|w| unsafe { w.bits(40000) });
                //                  self.pwm0._0_cmpa.write(|w| unsafe { w.bits(4000) });
                //                  self.pwm0._0_ctl
                //                     .write(|w| unsafe { w.bits(p._0_ctl.read().bits() | 1) });
                //                  self.pwm0.enable
                //                     .write(|w| unsafe { w.bits(p.enable.read().bits() | 1) }); 

                //                      core::mem::replace(
                //                         &mut self.pwm_physical_pins[0],
                //                         PhysicalPins::p0(pin_out));                              
                //             },

                //             _ =>{},


                //         }

                //     }
                //     Disabled => {
                //         let p = unsafe { &*tm4c123x::PWM0::ptr() };
                //         p.enable
                //             .write(|w| unsafe { w.bits(p.enable.read().bits() & !1) });
                //     }
                // }
            }

            1 => {
                match state {
                    Enabled(dut) => {
                        // let p = Peripherals::take().unwrap().PWM1;
                        //let NonZeroU8(extract)=dut;
                        let d: u32 = dut.get().into();
                        let clon = d;
                        let p = unsafe { &*tm4c123x::PWM0::ptr() };
                        p._0_load.write(|w| unsafe{w.bits(255)});
                        p._0_cmpb.write(|w| unsafe{w.bits(clon)});
                        p._0_ctl.write(|w| unsafe{w.bits(p._0_ctl.read().bits() | 1)});
                   
                        p.enable
                            .write(|w| unsafe { w.bits(p.enable.read().bits() | 2) });

                    }
                    Disabled => {
                        let p = unsafe { &*tm4c123x::PWM0::ptr() };
                        //let p = Peripherals::take().unwrap().PWM1;
                        p.enable
                            .write(|w| unsafe { w.bits(p.enable.read().bits() & !2) });
                    }
                }
            }

            _ => {}
        }
        self.states[pin] = state;

       // Ok(())
    }

    fn get_state(&self, pin: PwmPin) -> PwmState {
        self.states[pin]
    }

    // fn get_pin(&self, pin: PwmPin) -> bool {
    //     return PWM_SHIM_PINS[pin].load(Ordering::SeqCst);
    // }

    fn set_duty_cycle(&mut self, pin: PwmPin, duty: u8) {
        let x = usize::from(pin);
        match x {
            0 => {
                let p = unsafe { &*tm4c123x::PWM0::ptr() };
                p.enable
                    .write(|w| unsafe { w.bits(p.enable.read().bits() & !1) });

                let period = p._0_load.read().bits();

                let new_duty = ((duty as u32) * period / 256);
                p._0_cmpa.write(|w| unsafe { w.bits(new_duty) });
                p.enable
                    .write(|w| unsafe { w.bits(p.enable.read().bits() | 1) });
            },

            1 => {
                let p = unsafe { &*tm4c123x::PWM0::ptr() };
                p.enable
                    .write(|w| unsafe { w.bits(p.enable.read().bits() & !2) });

                let period = p._0_load.read().bits();

                let new_duty = ((duty as u32) * period / 256);
                p._0_cmpb.write(|w| unsafe { w.bits(new_duty) });
                p.enable
                    .write(|w| unsafe { w.bits(p.enable.read().bits() | 2) });
            }
            _=> {}
        }
        self.duty_cycle[pin] = duty;
        //Ok(())
    }

    fn get_duty_cycle(&self, pin: PwmPin) -> u8 {
        self.duty_cycle[pin]
    }
}
