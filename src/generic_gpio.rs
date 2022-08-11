use core::marker::PhantomData;
use core::sync::atomic::Ordering;
use core::{sync::atomic::AtomicBool, fmt::Debug};


use embedded_hal::digital::v2::InputPin as EmInputPin;
use embedded_hal::digital::v2::OutputPin as EmOutputPin;

use lc3_traits::peripherals::gpio::{GpioPinArr, GpioMiscError};

// TODO: create a version of this that uses the embedded_hal 1.0 IoPin

pub enum OwnedOrMut<'a, T> {
    Owned(T),
    Mut(&'a mut T),
}

impl<'a, T> OwnedOrMut<'a, T> {
    fn as_mut(&mut self) -> &mut T {
        match self {
            OwnedOrMut::Owned(o) => o,
            OwnedOrMut::Mut(m) => m,
        }
    }

    fn as_ref(&self) -> &T {
        match self {
            OwnedOrMut::Owned(o) => o,
            OwnedOrMut::Mut(m) => m,
        }
    }
}

impl<'a, T> From<T> for OwnedOrMut<'a, T> {
    fn from(o: T) -> Self {
        Self::Owned(o)
    }
}

impl<'a, T> From<&'a mut T> for OwnedOrMut<'a, T> {
    fn from(borrowed: &'a mut T) -> Self {
        Self::Mut(borrowed)
    }
}

macro_rules! pin_proxy {
    ($(($self:ident))? $field:ident[$gp_pin:ident] as $binding:pat => $e:expr) => { {
        use lc3_gp::GpioPin::*;
        pin_proxy!(@arms: $gp_pin
            (G0 to g0),
            (G1 to g1),
            (G2 to g2),
            (G3 to g3),
            (G4 to g4),
            (G5 to g5),
            (G6 to g6),
            (G7 to g7),
            in ($($self.)?$field):
                ($binding => $e)
        )
    }};

    (@arms: $gp_pin:ident $(($gp:ident to $field:ident)),* $(,)? in ($self:expr): ($binding:pat => $e:expr)) => {
        match $gp_pin {
            $(
                $gp => {
                    let $binding = $self.$field;
                    $e
                }
            )*
        }
    };
}

// TODO: give this a better name...
pub trait InterruptSolution<G0, G1, G2, G3, G4, G5, G6, G7, Ctx>
where
    G0: Interrupts + IoPin<Ctx = Ctx>,
    G1: Interrupts + IoPin<Ctx = Ctx>,
    G2: Interrupts + IoPin<Ctx = Ctx>,
    G3: Interrupts + IoPin<Ctx = Ctx>,
    G4: Interrupts + IoPin<Ctx = Ctx>,
    G5: Interrupts + IoPin<Ctx = Ctx>,
    G6: Interrupts + IoPin<Ctx = Ctx>,
    G7: Interrupts + IoPin<Ctx = Ctx>,
{
    fn interrupt_pending(&self, pin: lc3_gp::GpioPin, pins: &Pins<G0, G1, G2, G3, G4, G5, G6, G7, Ctx>, ctx: &Ctx) -> bool;

    fn clear_interrupt(&mut self, pin: lc3_gp::GpioPin, pins: &mut Pins<G0, G1, G2, G3, G4, G5, G6, G7, Ctx>, ctx: &mut Ctx);
}

pub struct FlagBasedInterrupts<'i>(&'i GpioPinArr<AtomicBool>);

impl<'i, G0, G1, G2, G3, G4, G5, G6, G7, Ctx> InterruptSolution<G0, G1, G2, G3, G4, G5, G6, G7, Ctx> for FlagBasedInterrupts<'i>
where
    G0: Interrupts + IoPin<Ctx = Ctx>,
    G1: Interrupts + IoPin<Ctx = Ctx>,
    G2: Interrupts + IoPin<Ctx = Ctx>,
    G3: Interrupts + IoPin<Ctx = Ctx>,
    G4: Interrupts + IoPin<Ctx = Ctx>,
    G5: Interrupts + IoPin<Ctx = Ctx>,
    G6: Interrupts + IoPin<Ctx = Ctx>,
    G7: Interrupts + IoPin<Ctx = Ctx>,
{
    fn interrupt_pending(&self, pin: lc3_gp::GpioPin, _pins: &Pins<G0, G1, G2, G3, G4, G5, G6, G7, Ctx>, _ctx: &Ctx) -> bool {
        self.0[pin].load(Ordering::SeqCst)
    }

    fn clear_interrupt(&mut self, pin: lc3_gp::GpioPin, _pins: &mut Pins<G0, G1, G2, G3, G4, G5, G6, G7, Ctx>, _ctx: &mut Ctx) {
        self.0[pin].store(false, Ordering::SeqCst)
    }
}

pub struct SelfContainedInterruptSolution<G0, G1, G2, G3, G4, G5, G6, G7, Ctx>(PhantomData<(G0, G1, G2, G3, G4, G5, G6, G7, Ctx)>)
where
    G0: PollForInterrupts + IoPin<Ctx = Ctx>,
    G1: PollForInterrupts + IoPin<Ctx = Ctx>,
    G2: PollForInterrupts + IoPin<Ctx = Ctx>,
    G3: PollForInterrupts + IoPin<Ctx = Ctx>,
    G4: PollForInterrupts + IoPin<Ctx = Ctx>,
    G5: PollForInterrupts + IoPin<Ctx = Ctx>,
    G6: PollForInterrupts + IoPin<Ctx = Ctx>,
    G7: PollForInterrupts + IoPin<Ctx = Ctx>;

impl<A, B, C, D, E, F, G, H, CC> Default for SelfContainedInterruptSolution<A, B, C, D, E, F, G, H, CC>
where
    A: PollForInterrupts + Interrupts + IoPin<Ctx = CC>,
    B: PollForInterrupts + Interrupts + IoPin<Ctx = CC>,
    C: PollForInterrupts + Interrupts + IoPin<Ctx = CC>,
    D: PollForInterrupts + Interrupts + IoPin<Ctx = CC>,
    E: PollForInterrupts + Interrupts + IoPin<Ctx = CC>,
    F: PollForInterrupts + Interrupts + IoPin<Ctx = CC>,
    G: PollForInterrupts + Interrupts + IoPin<Ctx = CC>,
    H: PollForInterrupts + Interrupts + IoPin<Ctx = CC>,
{
    fn default() -> Self {
        Self(Default::default())
    }
}

pub trait PollForInterrupts: IoPin {
    fn interrupt_pending(p: &Self::Input, ctx: &Self::Ctx) -> bool;
    fn clear_interrupt(p: &mut Self::Input, ctx: &mut Self::Ctx);
}

impl<A, B, C, D, E, F, G, H, CC> InterruptSolution<A, B, C, D, E, F, G, H, CC> for SelfContainedInterruptSolution<A, B, C, D, E, F, G, H, CC>
where
    A: PollForInterrupts + Interrupts + IoPin<Ctx = CC>,
    B: PollForInterrupts + Interrupts + IoPin<Ctx = CC>,
    C: PollForInterrupts + Interrupts + IoPin<Ctx = CC>,
    D: PollForInterrupts + Interrupts + IoPin<Ctx = CC>,
    E: PollForInterrupts + Interrupts + IoPin<Ctx = CC>,
    F: PollForInterrupts + Interrupts + IoPin<Ctx = CC>,
    G: PollForInterrupts + Interrupts + IoPin<Ctx = CC>,
    H: PollForInterrupts + Interrupts + IoPin<Ctx = CC>,
{
    fn interrupt_pending(&self, pin: lc3_gp::GpioPin, pins: &Pins<A, B, C, D, E, F, G, H, CC>, ctx: &CC) -> bool {
        pin_proxy!(pins[pin] as ref p => p.interrupt_pending(ctx))
    }

    fn clear_interrupt(&mut self, pin: lc3_gp::GpioPin, pins: &mut Pins<A, B, C, D, E, F, G, H, CC>, ctx: &mut CC) {
        pin_proxy!(pins[pin] as ref mut p => p.clear_interrupt(ctx))
    }
}

pub struct Pins<G0, G1, G2, G3, G4, G5, G6, G7, Ctx>
where
    G0: Interrupts + IoPin<Ctx = Ctx>,
    G1: Interrupts + IoPin<Ctx = Ctx>,
    G2: Interrupts + IoPin<Ctx = Ctx>,
    G3: Interrupts + IoPin<Ctx = Ctx>,
    G4: Interrupts + IoPin<Ctx = Ctx>,
    G5: Interrupts + IoPin<Ctx = Ctx>,
    G6: Interrupts + IoPin<Ctx = Ctx>,
    G7: Interrupts + IoPin<Ctx = Ctx>,
{
    g0: Pin<G0>,
    g1: Pin<G1>,
    g2: Pin<G2>,
    g3: Pin<G3>,
    g4: Pin<G4>,
    g5: Pin<G5>,
    g6: Pin<G6>,
    g7: Pin<G7>,
}

pub struct Gpio<
    'c,
    G0: Interrupts + IoPin<Ctx = Ctx>,
    G1: Interrupts + IoPin<Ctx = Ctx>,
    G2: Interrupts + IoPin<Ctx = Ctx>,
    G3: Interrupts + IoPin<Ctx = Ctx>,
    G4: Interrupts + IoPin<Ctx = Ctx>,
    G5: Interrupts + IoPin<Ctx = Ctx>,
    G6: Interrupts + IoPin<Ctx = Ctx>,
    G7: Interrupts + IoPin<Ctx = Ctx>,
    I: InterruptSolution<G0, G1, G2, G3, G4, G5, G6, G7, Ctx>,
    Ctx = (),
> {
    pins: Pins<G0, G1, G2, G3, G4, G5, G6, G7, Ctx>,
    interrupt_solution: I,
    ctx: OwnedOrMut<'c, Ctx>,
}

impl<'c, 'i, A, B, C, D, E, F, G, H, CC> Gpio<'c, A, B, C, D, E, F, G, H, FlagBasedInterrupts<'i>, CC>
where
    A: Interrupts + IoPin<Ctx = CC>,
    B: Interrupts + IoPin<Ctx = CC>,
    C: Interrupts + IoPin<Ctx = CC>,
    D: Interrupts + IoPin<Ctx = CC>,
    E: Interrupts + IoPin<Ctx = CC>,
    F: Interrupts + IoPin<Ctx = CC>,
    G: Interrupts + IoPin<Ctx = CC>,
    H: Interrupts + IoPin<Ctx = CC>,
{
    #[allow(clippy::too_many_arguments)]
    pub fn new_with_context(
        g0: A::Disabled,
        g1: B::Disabled,
        g2: C::Disabled,
        g3: D::Disabled,
        g4: E::Disabled,
        g5: F::Disabled,
        g6: G::Disabled,
        g7: H::Disabled,
        interrupt_flags: &'i GpioPinArr<AtomicBool>,
        ctx: impl Into<OwnedOrMut<'c, CC>>,
    ) -> Self {
        use Pin::Disabled as D;
        Self {
            pins: Pins {
                g0: D(g0),
                g1: D(g1),
                g2: D(g2),
                g3: D(g3),
                g4: D(g4),
                g5: D(g5),
                g6: D(g6),
                g7: D(g7),
            },
            interrupt_solution: FlagBasedInterrupts(interrupt_flags),
            ctx: ctx.into(),
        }
    }
}

impl<'c, 'i, A, B, C, D, E, F, G, H> Gpio<'c, A, B, C, D, E, F, G, H, FlagBasedInterrupts<'i>, ()>
where
    A: Interrupts + IoPin<Ctx = ()>,
    B: Interrupts + IoPin<Ctx = ()>,
    C: Interrupts + IoPin<Ctx = ()>,
    D: Interrupts + IoPin<Ctx = ()>,
    E: Interrupts + IoPin<Ctx = ()>,
    F: Interrupts + IoPin<Ctx = ()>,
    G: Interrupts + IoPin<Ctx = ()>,
    H: Interrupts + IoPin<Ctx = ()>,
{
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        g0: A::Disabled,
        g1: B::Disabled,
        g2: C::Disabled,
        g3: D::Disabled,
        g4: E::Disabled,
        g5: F::Disabled,
        g6: G::Disabled,
        g7: H::Disabled,
        interrupt_flags: &'i GpioPinArr<AtomicBool>
    ) -> Self {
        Self::new_with_context(g0, g1, g2, g3, g4, g5, g6, g7, interrupt_flags, ())
    }
}

mod other_interrupt_solution {
    use super::*;

    impl<'c, A, B, C, D, E, F, G, H, I, CC> Gpio<'c, A, B, C, D, E, F, G, H, I, CC>
    where
        A: Interrupts + IoPin<Ctx = CC>,
        B: Interrupts + IoPin<Ctx = CC>,
        C: Interrupts + IoPin<Ctx = CC>,
        D: Interrupts + IoPin<Ctx = CC>,
        E: Interrupts + IoPin<Ctx = CC>,
        F: Interrupts + IoPin<Ctx = CC>,
        G: Interrupts + IoPin<Ctx = CC>,
        H: Interrupts + IoPin<Ctx = CC>,
        I: InterruptSolution<A, B, C, D, E, F, G, H, CC> + Default,
    {
        #[allow(clippy::too_many_arguments)]
        pub fn new_with_context(
            g0: A::Disabled,
            g1: B::Disabled,
            g2: C::Disabled,
            g3: D::Disabled,
            g4: E::Disabled,
            g5: F::Disabled,
            g6: G::Disabled,
            g7: H::Disabled,
            ctx: impl Into<OwnedOrMut<'c, CC>>,
        ) -> Self {
            use Pin::Disabled as D;
            Self {
                pins: Pins {
                    g0: D(g0),
                    g1: D(g1),
                    g2: D(g2),
                    g3: D(g3),
                    g4: D(g4),
                    g5: D(g5),
                    g6: D(g6),
                    g7: D(g7),
                },
                interrupt_solution: Default::default(),
                ctx: ctx.into(),
            }
        }
    }

    impl<'c, A, B, C, D, E, F, G, H, I> Gpio<'c, A, B, C, D, E, F, G, H, I, ()>
    where
        A: Interrupts + IoPin<Ctx = ()>,
        B: Interrupts + IoPin<Ctx = ()>,
        C: Interrupts + IoPin<Ctx = ()>,
        D: Interrupts + IoPin<Ctx = ()>,
        E: Interrupts + IoPin<Ctx = ()>,
        F: Interrupts + IoPin<Ctx = ()>,
        G: Interrupts + IoPin<Ctx = ()>,
        H: Interrupts + IoPin<Ctx = ()>,
        I: InterruptSolution<A, B, C, D, E, F, G, H, ()> + Default,
    {
        #[allow(clippy::too_many_arguments)]
        pub fn new(
            g0: A::Disabled,
            g1: B::Disabled,
            g2: C::Disabled,
            g3: D::Disabled,
            g4: E::Disabled,
            g5: F::Disabled,
            g6: G::Disabled,
            g7: H::Disabled,
        ) -> Self {
            Self::new_with_context(g0, g1, g2, g3, g4, g5, g6, g7, ())
        }
    }
}

#[derive(Debug)]
enum Pin<P: Interrupts + IoPin> {
    Disabled(P::Disabled),
    Input(P::Input),
    Interrupt(P::Input),
    Output(P::Output),

    Transitioning,
}

/// $self: &mut T
/// $pat should accept either ref mut or value
/// $e should return value
macro_rules! transform {
    (($self:ident, $new_state:ident) => {
        $(
            $($variants:ident)|+ $(as $value_binding:ident)? => $next:tt in $e:expr,
        )+

        $(@else => $else:expr $(,)?)?
    }) => {
        // The idea is that this is correct by construction; it can see the match
        // pattern so it will use the right pattern after `mem::replac`ing.
        match ($self, $new_state) {
            $(

                ($(__ref_binding @ Self::$variants(_))|+, lc3_gp::GpioState::$next) => {
                    #[allow(unused_mut)]
                    {
                        let __self: &mut Pin<_> = __ref_binding;
                        let mut __tmp: Pin<_> = core::mem::replace(__self, Self::Transitioning);

                        // Expose as `$self` if no value binding is given:
                        $(#[cfg(all(disable, $value_binding))])?
                        let mut $self = __tmp;

                        // If a value binding was given, refine `$self`
                        // further and expose that:
                        transform!(
                            (@cond_bind: __tmp)
                            ($($variants)+)
                            ($($value_binding)?)
                        );

                        // Expand the expression:
                        #[warn(unused_mut)]
                        let res = {
                            $e
                        };

                        #[allow(unreachable_code, unused)]
                        { *__self = Self::$next(res); }

                        Ok(())
                    }
                }
            )+

            // (Self::Transitioning, _) => unreachable!(),
            (Self::Transitioning, _) => unsafe { ::core::hint::unreachable_unchecked() },

            $(_ => $else)?
        }
    };

    ((@cond_bind: $rhs:ident) ($($variants:ident)+) ($value_binding:ident)) => {
        let mut $value_binding = if let $(Self::$variants(x))|+ = $rhs {
            x
        } else {
            unsafe { ::core::hint::unreachable_unchecked() }
        };
    };
    ((@cond_bind: $rhs:ident) ($($variants:ident)+) ()) => { };
}

impl<P: Interrupts + IoPin> Pin<P> {
    fn set_state(&mut self, new_state: lc3_gp::GpioState, ctx: &mut P::Ctx) -> Result<(), lc3_gp::GpioMiscError> {
        let errf = <GpioMiscError>::from_source::<P::Error>;

        let this = self;
        transform! {
            (this, new_state) => {
                // No transition (assuming this optimizes away...)
                Disabled  as dis => Disabled  in dis,
                Output    as out => Output    in out,
                Input     as inp => Input     in inp,
                Interrupt as inp => Interrupt in inp,

                // To disabled:
                Output    as out => Disabled in P::output_to_disabled(out, ctx).map_err(errf)?,
                Input     as inp => Disabled in P::input_to_disabled(inp, ctx).map_err(errf)?,
                Interrupt as inp => Disabled in {
                    P::disable_interrupts(&mut inp, ctx).map_err(errf)?;
                    P::input_to_disabled(inp, ctx).map_err(errf)?
                },

                // To output:
                Disabled | Input | Interrupt => Output in {
                    let borrow = &mut this;
                    transform! ((borrow, new_state) => {
                        Disabled   as dis  => Output in P::disabled_to_output(dis, ctx).map_err(errf)?,
                        Input      as inp  => Output in P::input_to_output(inp, ctx).map_err(errf)?,
                        Interrupt  as inp  => Output in {
                            P::disable_interrupts(&mut inp, ctx).map_err(errf)?;
                            P::input_to_output(inp, ctx).map_err(errf)?
                        },
                        @else => unsafe { ::core::hint::unreachable_unchecked() },
                    })?;

                    // Pins are always to be low when switched to output mode.
                    if let Self::Output(mut out) = this {
                        EmOutputPin::set_low(&mut out).map_err(GpioMiscError::from_source)?;
                        out
                    } else { unsafe { ::core::hint::unreachable_unchecked() } }
                },

                // To input:
                Disabled  as dis => Input in P::disabled_to_input(dis, ctx).map_err(errf)?,
                Output    as out => Input in P::output_to_input(out, ctx).map_err(errf)?,
                Interrupt as inp => Input in {
                    P::disable_interrupts(&mut inp, ctx).map_err(errf)?;
                    inp
                },

                // To interrupt:
                Disabled | Input | Output => Interrupt in {
                    let borrow = &mut this;
                    transform! ((borrow, new_state) => {
                        Disabled as dis => Interrupt in P::disabled_to_input(dis, ctx).map_err(errf)?,
                        Input    as inp => Interrupt in inp,
                        Output   as out => Interrupt in P::output_to_input(out, ctx).map_err(errf)?,
                        @else => unsafe { ::core::hint::unreachable_unchecked() },
                    })?;

                    // Now that we've switched to Input mode, enabled interrupts"
                    if let Self::Interrupt(mut inp) = this {
                        P::enable_interrupts(&mut inp, ctx).map_err(errf)?;
                        inp
                    } else { unsafe { ::core::hint::unreachable_unchecked() } }
                },
            }
        }
    }

    fn get_state(&self) -> lc3_gp::GpioState {
        use Pin::*;
        use lc3_gp::GpioState as G;
        match self {
            Disabled(_) => G::Disabled,
            Input(_) => G::Input,
            Interrupt(_) => G::Interrupt,
            Output(_) => G::Output,
            Transitioning => unreachable!(),
        }
    }

    fn read(&self) -> Result<bool, lc3_gp::GpioReadError> {
        use Pin::*;
        use lc3_gp::GpioReadError as Err;
        match self {
            Input(inp) | Interrupt(inp) => {
                EmInputPin::is_high(inp).map_err(GpioMiscError::from_source).map_err(Into::into)
            },
            Disabled(_) => Err(Err::IsDisabled),
            Output(_) => Err(Err::IsInOutputMode),
            Transitioning => unreachable!(), // TODO: unchecked!
        }
    }

    fn write(&mut self, bit: bool) -> Result<(), lc3_gp::GpioWriteError> {
        use Pin::*;
        use lc3_gp::GpioWriteError as Err;
        match self {
            Output(out) => {
                EmOutputPin::set_state(out, bit.into()).map_err(GpioMiscError::from_source).map_err(Into::into)
            },
            Disabled(_) => Err(Err::IsDisabled),
            Input(_) => Err(Err::IsInInputMode),
            Interrupt(_) => Err(Err::IsInInterruptMode),
            Transitioning => unreachable!(), // TODO: unchecked!
        }
    }
}

impl<P: PollForInterrupts + Interrupts + IoPin> Pin<P> {
    #[inline(always)]
    fn interrupt_pending(&self, ctx: &P::Ctx) -> bool {
        match self {
            Pin::Interrupt(p) => <P as PollForInterrupts>::interrupt_pending(p, ctx),
            _ => false,
        }
    }

    #[inline(always)]
    fn clear_interrupt(&mut self, ctx: &mut P::Ctx) {
        match self {
            Pin::Interrupt(p) => <P as PollForInterrupts>::clear_interrupt(p, ctx),
            // _ => unreachable!("uh oh"),
            _ => {},
        }
    }
}

// Workaround to not pass the bound on `Error` along to users of `IoPin`.
//
// See: https://stackoverflow.com/a/69386814
// An example: https://play.rust-lang.org/?version=stable&mode=debug&edition=2018&gist=709222486f15d4f46290d387b9d92652
pub trait EmbeddedHalV2InputPinWithErrorImplingDebug: EmInputPin<Error = Self::BoundedErrorType> {
    type BoundedErrorType: Debug;
}
impl<I: EmInputPin> EmbeddedHalV2InputPinWithErrorImplingDebug for I
where
    <I as EmInputPin>::Error: Debug,
{
    type BoundedErrorType = I::Error;
}
pub trait EmbeddedHalV2OutputPinWithErrorImplingDebug: EmOutputPin<Error = Self::BoundedErrorType> {
    type BoundedErrorType: Debug;
}
impl<O: EmOutputPin> EmbeddedHalV2OutputPinWithErrorImplingDebug for O
where
    <O as EmOutputPin>::Error: Debug,
{
    type BoundedErrorType = O::Error;
}

pub trait IoPin
where
    // This is unfortunate; we want these to be requirements of implementing the trait
    // not WF requirements for users of this trait...
    // <Self::Input as embedded_hal::digital::v2::InputPin>::Error: Debug,
    // <Self::Output as embedded_hal::digital::v2::OutputPin>::Error: Debug,
{
    type Ctx;

    type Disabled;
    type Input: EmInputPin + EmbeddedHalV2InputPinWithErrorImplingDebug;
    type Output: EmOutputPin + EmbeddedHalV2OutputPinWithErrorImplingDebug;

    type Error: Debug;
    // TODO: can't require Into... gpio error types, bc of coherence; we don't
    // expect users to be able to modify their HAL crates (they can't impl our
    // trait or std's trait for a foreign type)

    fn disabled_to_input(d: Self::Disabled, ctx: &mut Self::Ctx) -> Result<Self::Input, Self::Error>;
    fn disabled_to_output(d: Self::Disabled, ctx: &mut Self::Ctx) -> Result<Self::Output, Self::Error>;

    fn input_to_disabled(i: Self::Input, ctx: &mut Self::Ctx) -> Result<Self::Disabled, Self::Error>;
    fn input_to_output(i: Self::Input, ctx: &mut Self::Ctx) -> Result<Self::Output, Self::Error>;

    fn output_to_disabled(o: Self::Output, ctx: &mut Self::Ctx) -> Result<Self::Disabled, Self::Error>;
    fn output_to_input(o: Self::Output, ctx: &mut Self::Ctx) -> Result<Self::Input, Self::Error>;
}


pub trait Interrupts: IoPin {
    fn enable_interrupts(i: &mut Self::Input, ctx: &mut Self::Ctx) -> Result<(), Self::Error>;
    fn disable_interrupts(i: &mut Self::Input, ctx: &mut Self::Ctx) -> Result<(), Self::Error>;
}

// TODO: support the whole port optimization!

pub use crate::multi_bank;
#[macro_export]
macro_rules! multi_bank {
    (
        for banks {
            $(
                $(#![$($mod_meta:tt)*])*

                for pins {$(
                    $(#[$($meta:tt)*])*
                    $pin_struct:ident as $alias:ident,
                )*} as $gpio_alias:ident;
            )+
        }: $macro_name:path {
            $($tt:tt)*
        }
    ) => {
        $crate::multi_bank!(expand: $macro_name
            | with suffix {
                $($tt)*
            } for [$({
                $(#![$($mod_meta)*])*

                for pins {$(
                    $(#[$($meta)*])*
                    $pin_struct as $alias,
                )*} as $gpio_alias;

            })+]
        );
    };

    (expand: $next:path | with suffix { $($args:tt)* } for [ { $($first:tt)* } $($rest:tt)* ]) => {
        $next! {
            $($first)*
            $($args)*
        }

        $crate::multi_bank! {
            expand: $next | with suffix { $($args)* }
            for [ $($rest)* ]
        }
    };

    (expand: $next:path | with suffix { $($args:tt)* } for [ ]) => { };
}

#[doc(hidden)]
pub mod macro_support {
    pub use paste::paste;
}

pub use crate::io_pins_with_typestate;
#[macro_export]
macro_rules! io_pins_with_typestate {
    (
        $(#![$($mod_meta:tt)*])*

        for pins {$(
            $(#[$($meta:tt)*])*
            $pin_struct:ident as $alias:ident,
        )*} as $gpio_alias:ident;

        type Ctx = $ctx:ty;
        type Error = $err:ty;

        type Disabled = $disabled:ty;
        type Input = $input:ty;
        type Output = $output:ty;

        => disabled = |$to_d:pat, $c_d:pat_param| $to_disabled:expr
        => input    = |$to_i:pat, $c_i:pat_param| $to_input:expr
        => output   = |$to_o:pat, $c_o:pat_param| $to_output:expr

        // $(
        => enable interrupts  = |$int_en:pat,  $c_ie:pat_param| $int_enabled:expr
        => disable interrupts = |$int_dis:pat, $c_di:pat_param| $int_disabled:expr
        // )?

        // TODO: make optional, support the interrupt based approach too!
        => interrupts {
            check = |$inp_mode_chk:pat, $c_int_c:pat_param| $int_check:expr;
            reset = |$inp_mode_res:pat, $c_int_r:pat_param| $int_reset:expr;
        }
        // $(

        // )?
    ) => { $crate::generic_gpio::macro_support::paste! {
        $(#[$($mod_meta)*])*
        #[allow(clippy::unit_arg)]
        pub mod [< io_pins $gpio_alias:lower:snake >] {
            #[allow(unused)]
            use super::*;
            #[derive(Debug)]
            struct Private;

            $(
                // Not constructable!
                $(#[$($meta)*])*
                #[derive(Debug)]
                pub struct $alias(Private);

                impl $crate::generic_gpio::IoPin for $alias {
                    type Ctx = $ctx;

                    type Disabled = $pin_struct<$disabled>;
                    type Input = $pin_struct<$input>;
                    type Output = $pin_struct<$output>;

                    type Error = $err;

                    fn input_to_disabled(
                        $to_d: Self::Input, $c_d: &mut Self::Ctx,
                    ) -> Result<Self::Disabled, Self::Error> { $to_disabled }
                    fn output_to_disabled(
                        $to_d: Self::Output, $c_d: &mut Self::Ctx,
                    ) -> Result<Self::Disabled, Self::Error> { $to_disabled }

                    fn disabled_to_input(
                        $to_i: Self::Disabled, $c_i: &mut Self::Ctx,
                    ) -> Result<Self::Input, Self::Error> { $to_input }
                    fn output_to_input(
                        $to_i: Self::Output, $c_i: &mut Self::Ctx,
                    ) -> Result<Self::Input, Self::Error> { $to_input }

                    fn disabled_to_output(
                        $to_o: Self::Disabled, $c_o: &mut Self::Ctx,
                    ) -> Result<Self::Output, Self::Error> { $to_output }
                    fn input_to_output(
                        $to_o: Self::Input, $c_o: &mut Self::Ctx,
                    ) -> Result<Self::Output, Self::Error> { $to_output }
                }

                // TODO: split off into separate macro:
                // $(
                    impl $crate::generic_gpio::Interrupts for $alias {
                        fn enable_interrupts(
                            $int_en: &mut Self::Input, $c_ie: &mut Self::Ctx,
                        ) -> Result<(), Self::Error> {
                            $int_enabled
                        }
                        fn disable_interrupts(
                            $int_dis: &mut Self::Input, $c_di: &mut Self::Ctx,
                        ) -> Result<(), Self::Error> {
                            $int_disabled
                        }
                    }
                // )?

                // TODO: make optional
                    impl $crate::generic_gpio::PollForInterrupts for $alias {
                        fn interrupt_pending($inp_mode_chk: &Self::Input, $c_int_c: &Self::Ctx) -> bool {
                            $int_check
                        }

                        fn clear_interrupt($inp_mode_res: &mut Self::Input, $c_int_r: &mut Self::Ctx) {
                            $int_reset
                        }
                    }
            )*
        }

        pub use [< io_pins $gpio_alias:lower:snake >]::{
            $($alias),*
        };
        // TODO: support the other flag based config too!
        pub type $gpio_alias<'c> = $crate::generic_gpio::Gpio<
            'c,
            $($alias,)*
            $crate::generic_gpio::SelfContainedInterruptSolution<
                $($alias,)*
                $ctx,
            >,
            $ctx,
        >;

        // pub type $gpio_alias<'c, 'i> = $crate::generic_gpio::Gpio<
        //     'c, 'i,
        //     $($alias,)*
        //     $ctx,
        // >;
    }};
}

use lc3_traits::peripherals::gpio::{self as lc3_gp};

#[allow(clippy::toplevel_ref_arg)]
impl<'c, A, B, C, D, E, F, G, H, I, CC> lc3_gp::Gpio for Gpio<'c, A, B, C, D, E, F, G, H, I, CC>
where
    A: Interrupts + IoPin<Ctx = CC>,
    B: Interrupts + IoPin<Ctx = CC>,
    C: Interrupts + IoPin<Ctx = CC>,
    D: Interrupts + IoPin<Ctx = CC>,
    E: Interrupts + IoPin<Ctx = CC>,
    F: Interrupts + IoPin<Ctx = CC>,
    G: Interrupts + IoPin<Ctx = CC>,
    H: Interrupts + IoPin<Ctx = CC>,
    I: InterruptSolution<A, B, C, D, E, F, G, H, CC>,
{
    fn set_state(&mut self, pin: lc3_gp::GpioPin, state: lc3_gp::GpioState) -> Result<(), GpioMiscError> {
        pin_proxy!(
            (self) pins[pin] as ref mut p => {
                p.set_state(state, self.ctx.as_mut())
            }
        )
    }

    fn get_state(&self, pin: lc3_gp::GpioPin) -> lc3_gp::GpioState {
        pin_proxy!((self) pins[pin] as ref p => p.get_state())
    }

    // TODO: change ReadError to be more specific? not sure
    //
    // definitely also allow for "Other" errors...
    //
    // also don't let us return the pin, it doesn't make any sense to..
    fn read(&self, pin: lc3_gp::GpioPin) -> Result<bool, lc3_gp::GpioReadError> {
        pin_proxy!((self) pins[pin] as ref p => p.read())
    }


    fn write(&mut self, pin: lc3_gp::GpioPin, bit: bool) -> Result<(), lc3_gp::GpioWriteError> {
        pin_proxy!((self) pins[pin] as ref mut p => p.write(bit))
    }

    fn interrupt_occurred(&self, pin: lc3_gp::GpioPin) -> bool {
        debug_assert!(matches!(self.get_state(pin), lc3_gp::GpioState::Interrupt));

        self.interrupt_solution.interrupt_pending(pin, &self.pins, self.ctx.as_ref())
    }


    fn reset_interrupt_flag(&mut self, pin: lc3_gp::GpioPin) {
        self.interrupt_solution.clear_interrupt(pin, &mut self.pins, self.ctx.as_mut())
    }

}

// TODO: split off interrupt support so we can support impls that both don't need a separate ISR w/interrupt flags and those that do

// TODO: have the macro expose a way to do the `read_all`/`write_all` optimization?? not sure how though...
