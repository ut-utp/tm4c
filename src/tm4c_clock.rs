// TODO: move this into `tm4c123x_hal`.

use core::hash::Hash;

use embedded_time::duration::Generic;
use embedded_time::{self as hal_time, TimeInt};
use hal_time::clock::{Clock, Error};
use hal_time::fraction::Fraction;
use hal_time::Instant;

use tm4c123x_hal::sysctl::Clocks;
use tm4c123x_hal::sysctl::PowerControl;
use tm4c123x_hal::time::Millis;
use tm4c123x_hal::timer::{Event, Timer as Tm4cTimer};

#[derive(Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Clone, Copy)]
pub enum TimerWidth {
    _16bits,
    _24bits, // 16 bits with 8 bit prescaler
    _32bits,
    _48bits, // 32 bits with 16 bit prescaler
    _64bits,
}

#[allow(unused)] // TODO: remove after moving!
impl TimerWidth {
    pub fn width(&self) -> u32 {
        use TimerWidth::*;
        match self {
            _16bits => 16,
            _24bits => 24,
            _32bits => 32,
            _48bits => 48,
            _64bits => 64,
        }
    }
}

pub trait Tm4cTimerTrait: Sized {
    type Inner;
    type RawReading;
    const TIMER_WIDTH: TimerWidth;

    // TODO: this should actually still be in `Hertz`, probably
    fn new<T: Into<Millis>>(
        timer: Self::Inner,
        timeout: T,
        pc: &PowerControl,
        clocks: &Clocks,
    ) -> Self;

    /// Returns `Err(())` if the timeout is too large for the timer.
    fn new_checked<T: Into<Millis> + Clone>(
        timer: Self::Inner,
        timeout: T,
        pc: &PowerControl,
        clocks: &Clocks,
    ) -> Result<Self, ()> {
        let timeout_in_ms = timeout.clone().into().0 as u64;
        let sysclk_hertz = clocks.sysclk().0 as u64;
        let ticks_in_a_ms = sysclk_hertz.checked_div(1000).unwrap() as u64;

        let max_reload_value = if Self::TIMER_WIDTH == TimerWidth::_64bits {
            u64::MAX
        } else {
            2u64.pow(Self::TIMER_WIDTH.width())
        };
        let reload_value = ticks_in_a_ms.checked_mul(timeout_in_ms).ok_or(())?;

        if reload_value > max_reload_value {
            return Err(());
        }

        Ok(Self::new(timer, timeout, pc, clocks))
    }

    fn count_up(&mut self) -> &mut Self;
    fn count_down(&mut self) -> &mut Self;
    fn listen(&mut self, event: Event) -> &mut Self;
    fn unlisten(&mut self, event: Event) -> &mut Self;

    fn free(self) -> Self::Inner;

    fn read_raw(&self) -> Self::RawReading;
    fn read(&self) -> Generic<Self::RawReading>;
    fn as_clock(self) -> Tm4cTimerToClockAdapter<Self> {
        Tm4cTimerToClockAdapter::new(self)
    }
}

// Where `U` is the first element of `T` and where we're assuming repr(Rust)
// isn't reordering the struct...
//
// (this is just a dumb bad workaround for getting around visbility; once we
// move this stuff into tm4c123x_hal proper we can get rid of this)
unsafe fn get_innards_mut<T>(a: &mut Tm4cTimer<T>) -> &mut TimerLookalike<T> {
    core::mem::transmute(a)
}

pub unsafe fn get_innards<T>(a: &Tm4cTimer<T>) -> &TimerLookalike<T> {
    core::mem::transmute(a)
}

pub struct TimerLookalike<TIM> {
    pub tim: TIM,
    pub clocks: Clocks,
    pub timeout: Millis,
}

macro_rules! tm4c_timer_trait_impl {
    ($(($t:ty, $w:ident, $n:ident))*) => {$(
        #[deny(unconditional_recursion)]
        impl Tm4cTimerTrait for Tm4cTimer<$t> {
            type Inner = $t;
            const TIMER_WIDTH: TimerWidth = TimerWidth::$w;
            type RawReading = tm4c_timer_trait_impl!(read_ty: $w);

            // according to the PLL frequencies used, this always corresponds to
            // a whole number of clock cycles (all the sys clock frequencies are
            // multiples of 1000)
            //
            // this probably isn't _actually_ true but it's also probably an
            // acceptable approximation
            fn new<T: Into<Millis>>(timer: Self::Inner, timeout: T, pc: &PowerControl, clocks: &Clocks) -> Self {
                Tm4cTimer::<Self::Inner>::$n::<T>(timer, timeout, pc, clocks)
            }

            // We run all our timers in concatenated mode so we don't have to
            // set `TBMR`, just `TAMR`.
            fn count_up(&mut self) -> &mut Self { // TODO: record in typestate so we can limit `Clock` impls to counting up timers?
                let inner = &unsafe { get_innards_mut(self) }.tim;
                inner.tamr.write(|w| w.tacdir().set_bit()); // Count up!

                self
            }
            fn count_down(&mut self) -> &mut Self {
                let inner = &unsafe { get_innards_mut(self) }.tim;
                inner.tamr.write(|w| w.tacdir().clear_bit());

                self
            }

            fn listen(&mut self, event: Event) -> &mut Self { Self::listen(self, event); self }
            fn unlisten(&mut self, event: Event) -> &mut Self { self.unlisten(event); self }
            fn free(self) -> Self::Inner { self.free() }
            tm4c_timer_trait_impl!(read_impl: $w $t);

            fn read(&self) -> Generic<Self::RawReading> {
                let reading_in_ticks = self.read_raw();
                let sysclk_hertz = unsafe { get_innards(self) }.clocks.sysclk().0;

                let scale = Fraction::new(1, sysclk_hertz);

                Generic::new(reading_in_ticks, scale)
            }
        }
    )*};

    (read_ty: _32bits) => { u32 };
    (read_ty: _64bits) => { u64 };

    (read_impl: _32bits $t:ty) => {
        fn read_raw(&self) -> u32 {
            let inner = &unsafe { get_innards(self) }.tim;
            inner.tav.read().bits()
        }
    };
    (read_impl: _64bits $t:ty) => {
        fn read_raw(&self) -> u64 {
            let inner = &unsafe { get_innards(self) }.tim;

            // When using a 64 bit timer, the lower 32 bits are in TAV and the
            // upper 32 bits are in TBV.
            //
            // Because we cannot read these registers together, atomically,
            // there is potential for value mismatch here: i.e. we can read
            // TBV and then TAV and report back an incorrect time because
            // TBV changed out from under us.
            //
            // Consider (assuming counting down):
            //   1) we read TBV as 0x51 (at this time, TBA is 0x0000_0000)
            //   2) another tick goes by, TBV is now 0x50 and TBA is 0xFFFF_FFFF
            //   3) we read TBA as 0xFFFF_FFFFF
            //   4) we incorrectly report: `0x51_FFFF_FFFFF`, a time in the past
            //
            // To guard against this we read TBV twice: once before reading TAV
            // and once after. If both values are the same we can be confident
            // that TBA didn't overflow on us.
            //
            // We retry until we get a reading that meets this criteria.
            //
            // This approach works with both count down and count up configured
            // timers.
            //
            // Note that this is slightly suboptimal for situtations in which
            // the reload value for your timer fits within 32 bits: in such
            // scenarios you don't even need to read TBV. But, since this _is_
            // a 64-bit wide timer, we assume that this is not a common use
            // case.
            let (upper, lower) = loop {
                let upper = inner.tbv.read().bits();
                let lower = inner.tav.read().bits();

                if upper == inner.tbv.read().bits() {
                    break (upper, lower)
                }
            };

            (upper as u64) << 32 | (lower as u64)
        }
    };
}

tm4c_timer_trait_impl! {
    (tm4c123x::TIMER0,  _32bits, timer0)
    (tm4c123x::TIMER1,  _32bits, timer1)
    (tm4c123x::TIMER2,  _32bits, timer2)
    (tm4c123x::TIMER3,  _32bits, timer3)
    (tm4c123x::TIMER4,  _32bits, timer4)
    (tm4c123x::TIMER5,  _32bits, timer5)
    (tm4c123x::WTIMER0, _64bits, wtimer0)
    (tm4c123x::WTIMER1, _64bits, wtimer1)
    (tm4c123x::WTIMER2, _64bits, wtimer2)
    (tm4c123x::WTIMER3, _64bits, wtimer3)
    (tm4c123x::WTIMER4, _64bits, wtimer4)
    (tm4c123x::WTIMER5, _64bits, wtimer5)
}

// To dodge coherence (and also to ensure we're counting up).
pub struct Tm4cTimerToClockAdapter<T: Tm4cTimerTrait>(T);
impl<T: Tm4cTimerTrait> Tm4cTimerToClockAdapter<T> {
    pub fn new(mut timer: T) -> Self {
        timer.count_up();
        Self(timer)
    }

    #[allow(unused)] // TODO: remove after moving!
    pub fn free(self) -> T {
        self.0
    }
}

impl<T: Tm4cTimerTrait> Clock for Tm4cTimerToClockAdapter<T>
where
    <T as Tm4cTimerTrait>::RawReading: TimeInt + Hash,
{
    type T = <T as Tm4cTimerTrait>::RawReading;

    // We'd like to be able to provide maximum precision here but doing so
    // would require modeling the current system clock frequency in the
    // type system and deriving the scaling factor accordingly.
    //
    // Instead we fix on millisecond precision.
    const SCALING_FACTOR: Fraction = Fraction::new(1, 1_000);

    fn try_now(&self) -> Result<Instant<Self>, Error> {
        let ms: embedded_time::duration::Milliseconds<<T as Tm4cTimerTrait>::RawReading> =
            self.0.read().try_into().map_err(|_| Error::Unspecified)?;

        Ok(Instant::new(ms.0))
    }
}
