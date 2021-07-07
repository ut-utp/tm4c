use lc3_traits::peripherals::adc::{
    Adc, AdcMiscError, AdcPin as Pin, AdcPinArr as PinArr, AdcReadError as ReadError, AdcState,
    AdcStateMismatch as StateMismatch,
};
extern crate tm4c123x;
use tm4c123x_hal::prelude::*;
use tm4c123x_hal::sysctl;

pub struct AdcShim {
    states: PinArr<State>,
}

#[derive(Copy, Clone, Debug, PartialEq)]
pub enum State {
    Enabled(u8),
    Disabled,
}

pub struct RequiredComponents{
    pub adc0:  tm4c123x::ADC0,
    pub adc1:  tm4c123x::ADC1,
    pub porte: tm4c123x::GPIO_PORTE,
}

impl From<State> for AdcState {
    fn from(state: State) -> AdcState {
        use AdcState::*;
        match state {
            State::Enabled(_) => Enabled,
            State::Disabled => Disabled,
        }
    }
}

const INIT_VALUE: u8 = 0;

impl Default for AdcShim {
    fn default() -> Self {
        Self {
            states: PinArr([State::Disabled; Pin::NUM_PINS]),
        }
    }
}

#[derive(Copy, Clone, Debug, PartialEq)]
pub struct SetError(StateMismatch);

impl AdcShim {
    pub fn new(power: &tm4c123x_hal::sysctl::PowerControl, peripheral_set: RequiredComponents) -> Self {

        let mut ad0 = peripheral_set.adc0;
        let mut ad1 = peripheral_set.adc1;
        let mut porte = peripheral_set.porte.split(power);
        let porte_ams = unsafe { &*tm4c123x::GPIO_PORTE::ptr() }; 
        porte_ams.dir.write(|w| unsafe{w.bits(porte_ams.dir.read().bits() & !0x003F )});
        porte_ams.afsel.write(|w| unsafe{w.bits(porte_ams.afsel.read().bits() | 0x003F )});
        porte_ams.den.write(|w| unsafe{w.bits(porte_ams.den.read().bits() & !0x003F )});
        porte_ams.amsel.write(|w| unsafe{w.bits(porte_ams.amsel.read().bits() | 0x003F )});
        
        let p = unsafe { &*tm4c123x::SYSCTL::ptr() };
        sysctl::control_power(
            power, sysctl::Domain::Adc0,
            sysctl::RunMode::Run, sysctl::PowerState::On);
        sysctl::reset(power, sysctl::Domain::Adc0);
        ad0.sspri.write(|w| unsafe{w.bits(0x0123)});
        ad0.actss.write(|w| unsafe{w.bits(ad0.actss.read().bits() & !0x0008)});
        ad0.emux.write(|w| unsafe{w.bits(ad0.emux.read().bits() & !0xF000)});
        ad0.ssmux3.write(|w| unsafe{w.bits(ad0.ssmux3.read().bits() & !0x000F )});
        ad0.ssmux3.write(|w| unsafe{w.bits(ad0.ssmux3.read().bits() + 9 )});
        ad0.ssctl3.write(|w| unsafe{w.bits(0x06)});
        ad0.im.write(|w| unsafe{w.bits(ad0.im.read().bits() & !0x0008 )});

        Self{
            states: PinArr([State::Disabled; Pin::NUM_PINS]),
        }
    }

    pub fn set_value(&mut self, pin: Pin, value: u8) -> Result<(), SetError> {
        use State::*;
        self.states[pin] = match self.states[pin] {
            Enabled(_) => Enabled(value),
            Disabled => return Err(SetError((pin, self.get_state(pin)))),
        };
        Ok(())
    }
}

impl Adc for AdcShim {
    fn set_state(&mut self, pin: Pin, state: AdcState) -> Result<(), AdcMiscError> {
        use AdcState::*;
        match state {
            Enabled => {
              self.states[pin] =   State::Enabled(INIT_VALUE);
               let ad0 = unsafe { &*tm4c123x::ADC0::ptr() }; 
              let x = usize::from(pin);
              match x{
                0 => {
                    ad0.ssmux3.write(|w| unsafe{w.bits(ad0.ssmux3.read().bits() & !0x000F )});
                    ad0.ssmux3.write(|w| unsafe{w.bits(ad0.ssmux3.read().bits() + 0 )});
                }
                1 => {
                    ad0.ssmux3.write(|w| unsafe{w.bits(ad0.ssmux3.read().bits() & !0x000F )});
                    ad0.ssmux3.write(|w| unsafe{w.bits(ad0.ssmux3.read().bits() + 1 )});
                }
                2 => {
                    ad0.ssmux3.write(|w| unsafe{w.bits(ad0.ssmux3.read().bits() & !0x000F )});
                    ad0.ssmux3.write(|w| unsafe{w.bits(ad0.ssmux3.read().bits() + 2 )});

                }
                3 => {
                    ad0.ssmux3.write(|w| unsafe{w.bits(ad0.ssmux3.read().bits() & !0x000F )});
                    ad0.ssmux3.write(|w| unsafe{w.bits(ad0.ssmux3.read().bits() + 3 )});
                }
                4 => {
                    ad0.ssmux3.write(|w| unsafe{w.bits(ad0.ssmux3.read().bits() & !0x000F )});
                    ad0.ssmux3.write(|w| unsafe{w.bits(ad0.ssmux3.read().bits() + 8 )});
                }
                5 => {
                    ad0.ssmux3.write(|w| unsafe{w.bits(ad0.ssmux3.read().bits() & !0x000F )});
                    ad0.ssmux3.write(|w| unsafe{w.bits(ad0.ssmux3.read().bits() + 9 )});
                }


                 _=> {

                }


              } 
              ad0.actss.write(|w| unsafe{w.bits(ad0.actss.read().bits() | 0x0008 )});


            },
            Disabled => {
                self.states[pin] = State::Disabled;
                let ad0 = unsafe { &*tm4c123x::ADC0::ptr() }; 
            },
        };
        Ok(())
    }

    fn get_state(&self, pin: Pin) -> AdcState {
        self.states[pin].into()
    }

    fn read(&self, pin: Pin) -> Result<u8, ReadError> {
        use State::*;
        match self.states[pin] {
            Enabled(value) => {
                let ad0 = unsafe { &*tm4c123x::ADC0::ptr() };
              let x = usize::from(pin);
              match x{
                0 => {
                    ad0.ssmux3.write(|w| unsafe{w.bits(ad0.ssmux3.read().bits() & !0x000F )});
                    ad0.ssmux3.write(|w| unsafe{w.bits(ad0.ssmux3.read().bits() + 0 )});
                }
                1 => {
                    ad0.ssmux3.write(|w| unsafe{w.bits(ad0.ssmux3.read().bits() & !0x000F )});
                    ad0.ssmux3.write(|w| unsafe{w.bits(ad0.ssmux3.read().bits() + 1 )});
                }
                2 => {
                    ad0.ssmux3.write(|w| unsafe{w.bits(ad0.ssmux3.read().bits() & !0x000F )});
                    ad0.ssmux3.write(|w| unsafe{w.bits(ad0.ssmux3.read().bits() + 2 )});

                }
                3 => {
                    ad0.ssmux3.write(|w| unsafe{w.bits(ad0.ssmux3.read().bits() & !0x000F )});
                    ad0.ssmux3.write(|w| unsafe{w.bits(ad0.ssmux3.read().bits() + 3 )});
                }
                4 => {
                    ad0.ssmux3.write(|w| unsafe{w.bits(ad0.ssmux3.read().bits() & !0x000F )});
                    ad0.ssmux3.write(|w| unsafe{w.bits(ad0.ssmux3.read().bits() + 8 )});
                }
                5 => {
                    ad0.ssmux3.write(|w| unsafe{w.bits(ad0.ssmux3.read().bits() & !0x000F )});
                    ad0.ssmux3.write(|w| unsafe{w.bits(ad0.ssmux3.read().bits() + 9 )});
                }


                 _=> {

                }


              } 
                ad0.pssi.write(|w| unsafe{w.bits(0x0008)});
                while((ad0.ris.read().bits()&0x08)==0){};
                let out = ad0.ssfifo3.read().bits()& 0x0FFF;
                ad0.isc.write(|w| unsafe{w.bits(0x00008)});
                Ok((out/16) as u8)

            },
            valueless => Err(ReadError((pin, valueless.into()))),
        }
    }
}