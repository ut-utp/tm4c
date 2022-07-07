//Simple Embedded-time clock trait impl for a tm4c wide timer.
//This is not meant to be a comprehensive embedded-time impl which
//will be a much more involved effort and to be done in the tm4c-hal fork repo.
//Just a temporary impl to use for now for the utp device clock
//TODO: Make the full embedded-time impl in tm4c-hal repo someday 
//      (if it is really worth it and there are no better/more standardized HAL abstractions by then )
extern crate embedded_time;
extern crate tm4c123x;
extern crate tm4c123x_hal;

use embedded_time as hal_time;
use hal_time::fraction::Fraction;
use hal_time::clock::{Error, Clock};
use hal_time::{Instant, TimeInt};
use hal_time::duration::{Milliseconds, Generic};
use hal_time::fixed_point::FixedPoint;

use tm4c123x_hal::sysctl;
use tm4c123x_hal::sysctl::Clocks;

pub struct Tm4cClock {
	timer: tm4c123x::WTIMER2, //wide timer 0,1 used for the timer peripherals
	clocks: Clocks,
}

impl Tm4cClock {
	pub fn new(tim: tm4c123x::WTIMER2,
           pc: &sysctl::PowerControl,
           clocks: &Clocks) -> Self {
                    // power up
                    sysctl::control_power(
                        pc, sysctl::Domain::WideTimer2,
                        sysctl::RunMode::Run, sysctl::PowerState::On);
                    sysctl::reset(pc, sysctl::Domain::WideTimer2);

                    // Stop Timers
                    tim.ctl.write(|w|
                                  w.taen().clear_bit()
                                  .tben().clear_bit()
                                  .tastall().set_bit()
                    );

                    // GPTMCFG = 0x0 (chained - 2x16 = 32bits) This
                    // will not force 32bits wide timer, this will
                    // really force the wider range to be used (32 for
                    // 16/32bits timers, 64 for 32/64).
                    tim.cfg.write(|w| w.cfg()._32_bit_timer());

                    tim.tamr.write(|w| w.tamr().period());

                    tim.tamr.write(|w| w.tacdir().set_bit()); //Set to up count

                    //tim.tbmr.write(|w| w.tbcdir().set_bit()); unnecessary when in combined mode

                    tim.tav.write(|w| unsafe { w.bits(0xFFFF_FFFF) }); //Just set to max upcount range so you don't need to worry about roll overs in your lifetime
                    tim.tailr.write(|w| unsafe { w.bits(0xFFFF_FFFF) });

                    tim.tbv.write(|w| unsafe { w.bits(0xFFFF_FFFF) });
                    tim.tbilr.write(|w| unsafe { w.bits(0xFFFF_FFFF) });

                    // // start counter
                    tim.ctl.modify(|_, w|
                        w.taen().set_bit()
                        .tben().set_bit()
                    );

                    let mut clock = Tm4cClock {
                        timer: tim,
                        clocks: *clocks,
                    };

                    clock
	}
}

//TODO: This is just assuming sec return for now and this is partly due to the way the generic hal impl is done in device-support
//      which doesn't yet support anything other than s -> ms. Need to fix the generic impl
impl Clock for Tm4cClock {
    type T = u64;  //Just hard specifying a u64 here; will do more generic options later if needed

    const SCALING_FACTOR: Fraction = Fraction::new(1, 1); //Likewise hard setting scaling to unity

    fn try_now(&self) -> Result<Instant<Self>, Error> {
    	let current_ticks: u64 = (self.timer.tav.read().bits() as u64) + (((self.timer.tbv.read().bits()) as u64) << 32) as u64;
    	let millis: u64 = current_ticks / (((self.clocks.sysclk.0) as u64));
    	
    	Ok(Instant::new(millis))
    }
}