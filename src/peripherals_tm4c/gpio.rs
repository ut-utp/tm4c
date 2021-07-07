use core::ops::{Index, IndexMut};
use core::sync::atomic::{AtomicBool, Ordering};
use lc3_traits::peripherals::gpio::GpioState::Interrupt;
use lc3_traits::peripherals::gpio::{
    Gpio, GpioMiscError, GpioPin, GpioPinArr, GpioReadError, GpioState, GpioWriteError,
};
extern crate embedded_hal;
extern crate tm4c123x;
use tm4c123x_hal::gpio::*;
use tm4c123x_hal::gpio::{gpiob::*, gpiof::*};
use tm4c123x_hal::timer;
use tm4c123x_hal::{prelude::*, Peripherals};
extern crate cortex_m;
use cortex_m::interrupt as cortex_int;

use tm4c123x::NVIC as nvic;


static mut GPIO_INTERRUPTS: [u8; 8] = [0; 8];

#[derive(Copy, Clone, Debug, PartialEq)]

pub enum State {
    Input(bool),
    Output(bool),
    Interrupt(bool),
    Disabled,
}

pub struct RequiredComponents {
    pub pf1: PF1<Output<PushPull>>,
    pub pf2: PF2<Output<PushPull>>,
    pub pf4: PF4<Output<PushPull>>,
    pub pb0: PB0<Output<PushPull>>,
    pub pb1: PB1<Input<PullUp>>,
    pub pb2: PB2<Input<PullUp>>,
    pub pb3: PB3<Input<PullUp>>,
    pub pb4: PB4<Input<PullUp>>,
}

impl From<State> for GpioState {
    fn from(state: State) -> GpioState {
        use GpioState::*;

        match state {
            State::Input(_) => Input,
            State::Output(_) => Output,
            State::Interrupt(_) => Interrupt,
            State::Disabled => Disabled,
        }
    }
}

enum physical_pin_mappings {
    GPIO0(PF1<Output<PushPull>>),
    GPIO1(PF2<Output<PushPull>>),
    GPIO2(PF4<Output<PushPull>>),
    GPIO3(PB0<Output<PushPull>>),
    GPIO4(PB1<Input<PullUp>>),
    GPIO5(PB2<Input<PullUp>>),
    GPIO6(PB3<Input<PullUp>>),
    GPIO7(PB4<Input<PullUp>>),
}

pub struct Mapping<T>(pub [T; GpioPin::NUM_PINS]);

pub enum State2<In, Out>
where
    In: embedded_hal::digital::v2::InputPin,
    Out: embedded_hal::digital::v2::OutputPin,
{
    Input(In),
    Output(Out),
    Interrupt(In),
    Disabled,
}

pub enum PhysicalPins {
    G0(State2<PF1<Input<PullUp>>, PF1<Output<PushPull>>>),
    G1(State2<PF2<Input<PullUp>>, PF2<Output<PushPull>>>),
    G2(State2<PF4<Input<PullUp>>, PF4<Output<PushPull>>>),
    G3(State2<PB0<Input<PullUp>>, PB0<Output<PushPull>>>),
    G4(State2<PB1<Input<PullUp>>, PB1<Output<PushPull>>>),
    G5(State2<PB2<Input<PullUp>>, PB2<Output<PushPull>>>),
    G6(State2<PB3<Input<PullUp>>, PB3<Output<PushPull>>>),
    G7(State2<PB4<Input<PullUp>>, PB4<Output<PushPull>>>),
}

pub struct physical_pins<'a> {
    states: GpioPinArr<State>,
    flags:  Option<&'a GpioPinArr<AtomicBool>>,
    mappinG2: [PhysicalPins; 8],
    Peripheral_set: Option<&'a mut tm4c123x_hal::Peripherals>,
}
impl Default for physical_pins<'_> {
    fn default() -> Self {
        unimplemented!()
    }
}

impl physical_pins<'_> {
    pub fn new<'a>(
        power: &tm4c123x_hal::sysctl::PowerControl,
        peripheral_set: RequiredComponents,
    ) -> Self {
        let mut states_init = [
            State::Output(false),
            State::Output(false),
            State::Output(false),
            State::Output(false),
            State::Input(false),
            State::Input(false),
            State::Input(false),
            State::Input(false),
        ];
        let p_st = peripheral_set;

        let mut nvic_field;
        unsafe{
        let p_core = tm4c123x_hal::CorePeripherals::steal();
        nvic_field = p_core.NVIC;
        };
        let mut gpiof1 = p_st.pf1;
        gpiof1.set_low();
        let mut gpiof2 = p_st.pf2;
        gpiof2.set_low();
        let mut gpiof4 = p_st.pf4;
        gpiof4.set_low();
        let mut gpiob0 = p_st.pb0;
        let mut gpiob1 = p_st.pb1;
        let mut gpiob2 = p_st.pb2;
        let mut gpiob3 = p_st.pb3;
        let mut gpiob4 = p_st.pb4;

       unsafe{nvic::unmask(tm4c123x::Interrupt::GPIOF);};
       unsafe{nvic_field.set_priority(tm4c123x::Interrupt::GPIOF, 1);};
       unsafe{nvic_field.enable(tm4c123x::Interrupt::GPIOF);};
       unsafe{cortex_int::enable();};


        Self {
            states: GpioPinArr(states_init),
            flags: None,
            mappinG2: ([
                PhysicalPins::G0(State2::<PF1<Input<PullUp>>, PF1<Output<PushPull>>>::Output(
                    gpiof1,
                )),
                PhysicalPins::G1(State2::<PF2<Input<PullUp>>, PF2<Output<PushPull>>>::Output(
                    gpiof2,
                )),
                PhysicalPins::G2(State2::<PF4<Input<PullUp>>, PF4<Output<PushPull>>>::Output(
                    gpiof4,
                )),
                PhysicalPins::G3(State2::<PB0<Input<PullUp>>, PB0<Output<PushPull>>>::Output(
                    gpiob0,
                )),
                PhysicalPins::G4(State2::<PB1<Input<PullUp>>, PB1<Output<PushPull>>>::Input(
                    gpiob1,
                )),
                PhysicalPins::G5(State2::<PB2<Input<PullUp>>, PB2<Output<PushPull>>>::Input(
                    gpiob2,
                )),
                PhysicalPins::G6(State2::<PB3<Input<PullUp>>, PB3<Output<PushPull>>>::Input(
                    gpiob3,
                )),
                PhysicalPins::G7(State2::<PB4<Input<PullUp>>, PB4<Output<PushPull>>>::Input(
                    gpiob4,
                )),
            ]),
            Peripheral_set: None,
        }
    }
}

pub struct GpioShim<'a> {
    states: GpioPinArr<State>,
    flags: GpioPinArr<Option<&'a AtomicBool>>,
}

impl Index<GpioPin> for physical_pins<'_> {
    type Output = State;

    fn index(&self, pin: GpioPin) -> &State {
        &self.states[pin]
    }
}

impl IndexMut<GpioPin> for physical_pins<'_> {
    fn index_mut(&mut self, pin: GpioPin) -> &mut State {
        &mut self.states[pin]
    }
}

impl physical_pins<'_> {

    pub fn set_pin(&mut self, pin: GpioPin, bit: bool) -> Option<()> {
        use physical_pin_mappings::*;
        use PhysicalPins::*;
        use State::*;
         macro_rules! set_pin {
            ($ pin : expr, $resp:path) => {
                        let mut handle = {
                            unsafe {
                                core::mem::replace(
                                    &mut self.mappinG2[pin as usize],
                                    core::mem::uninitialized(),
                                )
                            }
                        };
                        match handle {
                            $resp(mut vb) => match vb {
                                State2::Input(mut ins) => {
                                    core::mem::replace(
                                        &mut self.mappinG2[pin as usize],
                                        $resp(State2::Input(ins)),
                                    );                                  
                                }
                                State2::Output(mut out) => {
                                    {
                                        if bit {
                                            out.set_high();
                                        } else {
                                            out.set_low();
                                        };
                                    };
                                    core::mem::replace(
                                        &mut self.mappinG2[pin as usize],
                                        $resp(State2::Output(out)),
                                    );
                                }

                                State2::Interrupt(mut ins) => {
                                    core::mem::replace(
                                        &mut self.mappinG2[pin as usize],
                                        $resp(State2::Interrupt(ins)),
                                    );                                  
                                }
                                _ => {}
                            },
                            _ => {}
                        }

            };


        }

          let mut disabled_flag = 0;
          match self.states[pin] {
            State::Disabled =>{
                disabled_flag = 1;
            },
            _=>{},

          } ;    


        if(disabled_flag == 1){
            self.states[pin] = State::Output(false);

        }

        match self[pin] {
            _ => {
                self[pin] = Output(bit);
                let mut x = usize::from(pin);
                match x {

                    0 => {
                        set_pin!(0, G0);
                    }
                    1 => {
                        set_pin!(1, G1);
                    }
                    2 => {
                        set_pin!(2, G2);
                    }
                    3 => {
                        set_pin!(3, G3);
                    }
                    4 => {
                        set_pin!(4, G4);
                    }
                    5 => {
                        set_pin!(5, G5);
                    }
                    6 => {
                        set_pin!(6, G6);
                    }
                    7 => {
                        set_pin!(7, G7);
                    }
                    _ => {}
                };
            }

        };

        if(disabled_flag == 1){
            self.states[pin] = State::Disabled;

        }

        Some(())

    }

    fn raise_interrupt(&self, pin: GpioPin) {

    }

    fn update_flags(&self){
        unsafe{
        for i in 0..8 {
        if(GPIO_INTERRUPTS[i]==1){
        match self.flags {
            Some(flags) => {
                match i {
                    0 =>{
                     flags[GpioPin::G0].store(true, Ordering::SeqCst);
                    }
                    1 =>{
                     flags[GpioPin::G1].store(true, Ordering::SeqCst);
                    }
                    2 =>{
                     flags[GpioPin::G2].store(true, Ordering::SeqCst);
                    }
                    3 =>{
                     flags[GpioPin::G3].store(true, Ordering::SeqCst);
                    }
                    4 =>{
                     flags[GpioPin::G4].store(true, Ordering::SeqCst);
                    }
                    5 =>{
                     flags[GpioPin::G5].store(true, Ordering::SeqCst);
                    }
                    6 =>{
                     flags[GpioPin::G6].store(true, Ordering::SeqCst);
                    }
                    7 =>{
                     flags[GpioPin::G7].store(true, Ordering::SeqCst);
                    }
                    _=>{}
                }

               
            }
            None => unreachable!(),
        }
        }
        }
    };

    }

    pub fn get_pin(&self, pin: GpioPin) -> Option<bool> {
        use State::*;

        match self[pin] {
            Input(b) | Output(b) | Interrupt(b) => Some(b),
            Disabled => None,
        }
    }

    pub fn get_pin_state(&self, pin: GpioPin) -> GpioState {
        self[pin].into()
    }
}

impl<'a> Gpio<'a> for physical_pins<'a> {
    fn set_state(&mut self, pin: GpioPin, state: GpioState) -> Result<(), GpioMiscError> {
        use GpioState::*;


        macro_rules! set_state_input {
            ($ pin : expr, $resp:path) => {
                        let mut handle = {
                            unsafe {
                                core::mem::replace(
                                    &mut self.mappinG2[pin as usize],
                                    core::mem::uninitialized(),
                                )
                            }
                        };

                          match handle {
                             $resp(x) => {
                                match x{
                                State2::Input(mut ins) => {
                                    let new_out = ins.into_pull_up_input();
                                    core::mem::replace(
                                        &mut self.mappinG2[pin as usize],
                                        $resp(State2::Input(new_out)),
                                    );
                                }
                                State2::Output(mut out) => {
                                    let new_out = out.into_pull_up_input();
                                    core::mem::replace(
                                        &mut self.mappinG2[pin as usize],
                                        $resp(State2::Input(new_out)),
                                    );
                                }
                                State2::Interrupt(mut ins) =>{
                                    let new_in = ins.into_pull_up_input();
                                    core::mem::replace(
                                        &mut self.mappinG2[pin as usize],
                                        $resp(State2::Interrupt(new_out)),
                                    );

                                },
                                    _=> {}
                                }
                          },
                             _ => {}
                         } 

            };


        }



        match state {
            Input => {
                self[pin] = State::Input(false);
                let mut x = usize::from(pin);
                match x {
                    0 => {
                        set_state_input!(0, PhysicalPins::G0);
                    }
                    1 => {
                        set_state_input!(1, PhysicalPins::G1);
                    }
                    2 => {
                        set_state_input!(2, PhysicalPins::G2);
                    }
                    3 => {
                        set_state_input!(3, PhysicalPins::G3);
                    }
                    4 => {
                        set_state_input!(4, PhysicalPins::G4);
                    }
                    5 => {
                        set_state_input!(5, PhysicalPins::G5);
                    }
                    6 => {
                        set_state_input!(6, PhysicalPins::G6);
                    }
                    7 => {
                        set_state_input!(7, PhysicalPins::G7);
                    }
                    _ => {}
                }
            }
            Output => {

        macro_rules! set_state_output {
            ($ pin : expr, $resp:path) => {
                        let mut handle = {
                            unsafe {
                                core::mem::replace(
                                    &mut self.mappinG2[pin as usize],
                                    core::mem::uninitialized(),
                                )
                            }
                        };

                          match handle {
                             $resp(x) => {
                                match x{
                                State2::Input(mut ins) => {
                                    let new_out = ins.into_push_pull_output();
                                    core::mem::replace(
                                        &mut self.mappinG2[pin as usize],
                                        $resp(State2::Output(new_out)),
                                    );
                                }
                                State2::Output(mut out) => {
                                    let new_out = out.into_push_pull_output();
                                    core::mem::replace(
                                        &mut self.mappinG2[pin as usize],
                                        $resp(State2::Output(new_out)),
                                    );
                                }
                                State2::Interrupt(mut ins) =>{
                                    let new_in = ins.into_pull_up_input();
                                    let new_out = new_in.into_push_pull_output();
                                    core::mem::replace(
                                        &mut self.mappinG2[pin as usize],
                                        $resp(State2::Interrupt(new_out)),
                                    );

                                },
                                    _=> {}
                                }
                          },
                             _ => {}
                         } 

            };


        }

                self[pin] = State::Output(false);
                let mut x = usize::from(pin);
                match x {
                    0 => {
                        set_state_output!(0, PhysicalPins::G0);
                    }
                    1 => {
                        set_state_output!(1, PhysicalPins::G1); 
                    }
                    2 => {
                         set_state_output!(2, PhysicalPins::G2);
                    }
                    3 => {
                         set_state_output!(3, PhysicalPins::G3);
                    }
                    4 => {
                        set_state_output!(4, PhysicalPins::G4);
                    }
                    5 => {
                         set_state_output!(5, PhysicalPins::G5);
                    }
                    6 => {
                         set_state_output!(6, PhysicalPins::G6);
                    }
                    7 => {
                         set_state_output!(7, PhysicalPins::G7);
                    }
                    _ => {}
                }
            }
            Interrupt => {
                
        macro_rules! set_state_interrupt {
            ($ pin : expr, $resp:path) => {
                        let mut handle = {
                            unsafe {
                                core::mem::replace(
                                    &mut self.mappinG2[pin as usize],
                                    core::mem::uninitialized(),
                                )
                            }
                        };

                          match handle {
                             $resp(x) => {
                                match x{
                                State2::Input(mut ins) => {
                                    ins.set_interrupt_mode(tm4c123x_hal::gpio::InterruptMode::EdgeRising);
                                    core::mem::replace(
                                        &mut self.mappinG2[pin as usize],
                                        $resp(State2::Interrupt(ins)),
                                    );
                                    self[pin] = State::Interrupt(false);
                                }
                                State2::Output(mut out) => {
                                    core::mem::replace(
                                        &mut self.mappinG2[pin as usize],
                                        $resp(State2::Output(out)),
                                    );
                                    self[pin] = State::Interrupt(false);
                                }
                                State2::Interrupt(mut ins) =>{
                                    ins.set_interrupt_mode(tm4c123x_hal::gpio::InterruptMode::EdgeRising);
                                    core::mem::replace(
                                        &mut self.mappinG2[pin as usize],
                                        $resp(State2::Interrupt(ins)),
                                    );
                                    self[pin] = State::Interrupt(false);
                                },
                                    _=> {

                                    }
                                }
                          },
                             _ => {}
                         } 

            };


        }
                let mut x = usize::from(pin);
                match x {
                    0 => {
                        set_state_interrupt!(0, PhysicalPins::G0);
                    }
                    1 => {
                        set_state_interrupt!(1, PhysicalPins::G1); 
                    }
                    2 => {
                         set_state_interrupt!(2, PhysicalPins::G2);
                    }
                    3 => {
                         set_state_interrupt!(3, PhysicalPins::G3);
                    }
                    4 => {
                        set_state_interrupt!(4, PhysicalPins::G4);
                    }
                    5 => {
                         set_state_interrupt!(5, PhysicalPins::G5);
                    }
                    6 => {
                         set_state_interrupt!(6, PhysicalPins::G6);
                    }
                    7 => {
                         set_state_interrupt!(7, PhysicalPins::G7);
                    }
                    _ => {}
                }

            },
            Disabled => {
                self[pin] = State::Disabled;
                self.set_pin(pin, false);
            },
        };

        Ok(())
    }

    fn get_state(&self, pin: GpioPin) -> GpioState {
        self.get_pin_state(pin)
    }

    fn read(&self, pin: GpioPin) -> Result<bool, GpioReadError> {
        use State::*;

        if let Input(b) | Interrupt(b) = self[pin] {
            Ok(b)
        } else {
            Err(GpioReadError((pin, self[pin].into())))
        }
    }

    fn write(&mut self, pin: GpioPin, bit: bool) -> Result<(), GpioWriteError> {
        use State::*;
        self.set_pin(pin, bit);
        Ok(())
    }

    fn register_interrupt_flags(&mut self, flags: &'a GpioPinArr<AtomicBool>) {

        unsafe{GPIO_INTERRUPTS = [0; 8];};
        self.flags = match self.flags {
            None => Some(flags),
            Some(_) => {
                Some(flags)
            }
        }
    }

    fn interrupt_occurred(&self, pin: GpioPin) -> bool {

        self.update_flags();
        match self.flags {
            Some(flag) => {
                let occurred = flag[pin].load(Ordering::SeqCst);
                self.interrupts_enabled(pin) && occurred
            }
            None => unreachable!(),
        }
    }

    fn reset_interrupt_flag(&mut self, pin: GpioPin) {
        match self.flags {
            Some(flags) => flags[pin].store(false, Ordering::SeqCst),
            None => unreachable!(),
        }

        unsafe{GPIO_INTERRUPTS[usize::from(pin)]=0;};
    }

    fn interrupts_enabled(&self, pin: GpioPin) -> bool {
        self.get_state(pin) == Interrupt
    }
}

use cortex_m_rt_macros::interrupt;
use tm4c123x::Interrupt as interrupt;

#[interrupt]
fn GPIOF(){

    unsafe{
        let mut sc = &*tm4c123x::GPIO_PORTF::ptr();
        //sc.
        let bits = sc.ris.read().bits();

        let trail_zeros = bits.trailing_zeros();
        if(bits & 0x02 == 0x02){
        GPIO_INTERRUPTS[0] = 1;
        sc.icr.write(|w| unsafe{w.bits(0x02)}); 
        }  
        if(bits & 0x04 == 0x04){
        GPIO_INTERRUPTS[1] = 1;
        sc.icr.write(|w| unsafe{w.bits(0x04)}); 
        } 
        if(bits & 0x10 == 0x10){
        GPIO_INTERRUPTS[2] = 1;
        sc.icr.write(|w| unsafe{w.bits(0x10)}); 
        } 

    //DEBUG
    // let mut p = unsafe { &*tm4c123x::GPIO_PORTF::ptr() };
    // let mut bits = p.data.read().bits();
    // bits ^= 0x02;
    // p.data.write(|w| unsafe { w.bits(bits) });
    // p.icr.write(|w| unsafe { w.bits(0x10) });
    //let button = portb.pb3.into_pull_up_input(); 
    
    };

}

#[interrupt]
fn GPIOB(){

    unsafe{
        let mut sc = &*tm4c123x::GPIO_PORTB::ptr();

        let bits = sc.ris.read().bits();

        let trail_zeros = bits.trailing_zeros();
        if((bits & 0x01) == 0x01){
        GPIO_INTERRUPTS[3] = 1;
        sc.icr.write(|w| unsafe{w.bits(0x01)}); 
        }  
        if(bits & 0x02 == 0x02){
        GPIO_INTERRUPTS[4] = 1;
        sc.icr.write(|w| unsafe{w.bits(0x02)}); 
        } 
        if(bits & 0x04 == 0x04){
        GPIO_INTERRUPTS[5] = 1;
        sc.icr.write(|w| unsafe{w.bits(0x04)}); 
        } 
        if(bits & 0x08 == 0x08){
        GPIO_INTERRUPTS[6] = 1;
        sc.icr.write(|w| unsafe{w.bits(0x08)}); 
        }  
        if(bits & 0x10 == 0x10){
        GPIO_INTERRUPTS[7] = 1;
        sc.icr.write(|w| unsafe{w.bits(0x10)}); 
        } 

    };
}