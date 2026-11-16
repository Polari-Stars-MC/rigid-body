//! Electric machine models — the torque/speed behaviour of the motor families a
//! real machine shop would stock, reduced to the few parameters a physics
//! engine actually needs.
//!
//! Pure computation only — no access to `WorldHandle`, `RigidBody`, or Rapier
//! state. Every function returns `None` on an input outside its physical domain
//! rather than a NaN or a silently clamped value, matching
//! [docs/formula-numeric-domains.md](../../../docs/formula-numeric-domains.md).
//!
//! ## Why this module exists
//!
//! Until now a "motor" in this repo meant one of two things:
//!
//! - a joint **position servo** (`set_spring` / `world_joint_set_motor`) — a
//!   controller, with no notion of what is being controlled; or
//! - a **rotor** in the propeller sense (`rotor/`, blade-element and momentum
//!   theory) — aerodynamics, not electromagnetics.
//!
//! Neither can answer "how much torque does this motor make at this speed, and
//! how much current does it draw doing it". That is the question a machine model
//! has to answer, and it is a property of the *machine*, not of the joint.
//!
//! ## The families covered
//!
//! | family | torque–speed | speed control | typical use |
//! | --- | --- | --- | --- |
//! | DC brushed (separately excited) | linear droop from stall | voltage | cheap drives, winches |
//! | DC series-wound | rises with current, no finite stall torque | voltage | starters, traction (historic) |
//! | BLDC / PMSM | linear droop + torque ripple | inverter (FOC / six-step) | drones, robots, EVs |
//! | stepper | torque falls with speed, discrete steps | step pulses | 3D printers, CNC |
//! | induction (squirrel cage) | nearly flat to breakdown, then collapses | V/f or vector | industrial pumps, conveyors |
//! | servo (DC or AC) | flat torque to rated speed, then constant power | closed loop | machine tools |
//! | linear motor | same as its rotary twin, force instead of torque | inverter | gantries, maglev |
//! | piezoelectric / ultrasonic | quasi-static force, resonant drive | resonance | precision stages |
//! | shape-memory alloy | force from phase transformation, hysteretic | Joule heating | soft robots, latches |
//!
//! Each family gets the same treatment: a `torque_at_speed`-style function with
//! the parameters that matter for that machine, plus the loss terms that decide
//! whether it overheats. None of them integrate anything — the caller owns time.
//!
//! ## Conventions
//!
//! - Torque [N·m], angular velocity [rad/s], power [W], current [A],
//!   voltage [V], resistance [Ω], inductance [H], temperature [K].
//! - `omega` is the **mechanical** angular velocity at the output shaft unless a
//!   function name says `electrical`.
//! - Efficiency is reported in `(0, 1]`; a machine that would return `> 1` from
//!   inconsistent parameters returns `None` instead of inventing free energy.
//! - Thermal models are first-order lumped (single body, one time constant).
//!   They are honest about being first-order; see [`thermal_steady_state`].
//!
//! ## Parameter values
//!
//! Every numeric constant in this module is a **worked example**, chosen to be
//! physically plausible, not a datasheet claim. They are labelled as such and
//! are not used as defaults anywhere — a caller supplies its own machine. Do not
//! cite them as authoritative; `docs/evidence/` records only the invariants the
//! tests pin (monotonicity, energy balance, limit behaviour), never the values.

use std::f64::consts::PI;

use crate::math::finite_many;

// ---------------------------------------------------------------------------
// Shared helpers
// ---------------------------------------------------------------------------

/// Mechanical power `P = τ·ω` [W].
///
/// Returns `None` if either input is non-finite. Negative torque at positive
/// speed is regenerating (braking) and yields negative power — that is physical,
/// not an error.
pub fn mechanical_power(torque: f64, omega: f64) -> Option<f64> {
    finite_many(&[torque, omega]).then_some(torque * omega)
}

/// Electrical power `P = V·I` [W].
pub fn electrical_power(voltage: f64, current: f64) -> Option<f64> {
    finite_many(&[voltage, current]).then_some(voltage * current)
}

/// Copper loss `P = I²·R` [W] in a winding of resistance `r`.
///
/// This is the term that heats a motor and the reason continuous torque is
/// limited well below peak torque. `r` must be `>= 0`.
pub fn copper_loss(current: f64, resistance: f64) -> Option<f64> {
    if !finite_many(&[current, resistance]) || resistance < 0.0 {
        return None;
    }
    Some(current * current * resistance)
}

/// Back-EMF of a DC machine: `e = k_e·ω` [V].
///
/// `k_e` is the back-EMF constant [V·s/rad]. In SI, `k_e` and the torque
/// constant `k_t` [N·m/A] are numerically **equal** for an ideal machine — that
/// identity is what makes the torque–speed line a straight line, and
/// [`kt_from_ke`] exists to make the assumption explicit rather than implicit.
pub fn back_emf(ke: f64, omega: f64) -> Option<f64> {
    finite_many(&[ke, omega]).then_some(ke * omega)
}

/// The SI identity `k_t = k_e`, in the units this module uses.
///
/// `k_t` [N·m/A] equals `k_e` [V·s/rad] for an ideal DC machine (no loss, no
/// saturation). Passing `k_e` through this function documents that the caller is
/// relying on that identity rather than on a separately measured `k_t`.
pub fn kt_from_ke(ke: f64) -> Option<f64> {
    finite_many(&[ke]).then_some(ke)
}

/// Electrical time constant `τ = L/R` [s] of a winding.
///
/// Sets how fast current can change, i.e. how fast torque can change. `r` must
/// be strictly positive.
pub fn electrical_time_constant(inductance: f64, resistance: f64) -> Option<f64> {
    if !finite_many(&[inductance, resistance]) || inductance < 0.0 || resistance <= 0.0 {
        return None;
    }
    Some(inductance / resistance)
}

/// First-order thermal step: `ΔT` after `dt` for a lumped body of thermal
/// capacity `c` [J/K] dissipating `p_loss` [W] into an ambient at `t_ambient`
/// through thermal resistance `r_th` [K/W].
///
/// `dT/dt = (p_loss − (T − T_ambient)/r_th) / c`
///
/// This is deliberately first-order: a real motor has a winding time constant
/// (seconds) and a case time constant (minutes), and pretending one lumped node
/// captures both would be worse than saying so. Callers that need the second
/// node should chain two calls.
pub fn thermal_step(
    temperature: f64,
    ambient: f64,
    p_loss: f64,
    thermal_capacity: f64,
    thermal_resistance: f64,
    dt: f64,
) -> Option<f64> {
    if !finite_many(&[
        temperature,
        ambient,
        p_loss,
        thermal_capacity,
        thermal_resistance,
        dt,
    ]) {
        return None;
    }
    if thermal_capacity <= 0.0 || thermal_resistance <= 0.0 || dt < 0.0 {
        return None;
    }
    let dissipated = (temperature - ambient) / thermal_resistance;
    let rate = (p_loss - dissipated) / thermal_capacity;
    Some(temperature + rate * dt)
}

/// Steady-state temperature rise for a continuous loss `p_loss`.
///
/// `ΔT = p_loss·r_th`. This is the number that decides a motor's *continuous*
/// rating: exceed it and the winding cooks regardless of how good the peak
/// torque looks.
pub fn thermal_steady_state(p_loss: f64, thermal_resistance: f64, ambient: f64) -> Option<f64> {
    if !finite_many(&[p_loss, thermal_resistance, ambient]) || thermal_resistance <= 0.0 {
        return None;
    }
    Some(ambient + p_loss * thermal_resistance)
}

/// Temperature margin left before `limit`: `limit − temperature` [K].
///
/// Negative means the machine is already over its limit.
pub fn thermal_margin(temperature: f64, limit: f64) -> Option<f64> {
    finite_many(&[temperature, limit]).then_some(limit - temperature)
}

// ---------------------------------------------------------------------------
// Brushed DC machine (separately excited / permanent magnet)
// ---------------------------------------------------------------------------

/// A permanent-magnet or separately-excited brushed DC machine.
///
/// The whole steady-state behaviour follows from two numbers and Ohm's law:
///
/// ```text
/// V = I·R + k_e·ω            (armature circuit)
/// τ = k_t·I                  (torque)
/// ```
///
/// Eliminating `I` gives the torque–speed line
///
/// ```text
/// τ(ω) = k_t·(V − k_e·ω)/R
/// ```
///
/// which is a straight line from the **stall torque** `k_t·V/R` at `ω = 0` to
/// zero torque at the **no-load speed** `V/k_e`. That single line is why a DC
/// motor "droops": load it and it slows down.
#[derive(Clone, Copy, Debug)]
pub struct DcMotor {
    /// Terminal voltage [V].
    pub voltage: f64,
    /// Armature resistance [Ω].
    pub resistance: f64,
    /// Back-EMF / torque constant [V·s/rad] = [N·m/A].
    pub ke: f64,
    /// No-load (friction + iron) torque [N·m], `>= 0`.
    pub no_load_torque: f64,
}

impl DcMotor {
    /// Validates the parameter set. Returns `false` for any non-physical value
    /// (negative resistance, non-positive `ke`, negative friction torque).
    pub fn is_valid(&self) -> bool {
        finite_many(&[self.voltage, self.resistance, self.ke, self.no_load_torque])
            && self.resistance > 0.0
            && self.ke > 0.0
            && self.no_load_torque >= 0.0
    }

    /// Torque available at shaft speed `omega` [rad/s], including the no-load
    /// friction torque.
    ///
    /// Above the no-load speed this returns the (negative) braking torque the
    /// machine would produce if back-driven — physical for a separately excited
    /// machine, and the basis of regenerative braking.
    pub fn torque_at_speed(&self, omega: f64) -> Option<f64> {
        if !self.is_valid() || !finite_many(&[omega]) {
            return None;
        }
        let current = (self.voltage - self.ke * omega) / self.resistance;
        let torque = self.ke * current
            - self
                .no_load_torque
                .copysign(if omega >= 0.0 { 1.0 } else { -1.0 });
        Some(torque)
    }

    /// Current drawn at shaft speed `omega` [A].
    pub fn current_at_speed(&self, omega: f64) -> Option<f64> {
        if !self.is_valid() || !finite_many(&[omega]) {
            return None;
        }
        Some((self.voltage - self.ke * omega) / self.resistance)
    }

    /// Stall torque `k_t·V/R` [N·m] — the torque at zero speed, minus friction.
    ///
    /// A real machine cannot sustain this: the current is limited only by `R`, so
    /// stall is a thermal event, not a mechanical one.
    pub fn stall_torque(&self) -> Option<f64> {
        if !self.is_valid() {
            return None;
        }
        Some(self.ke * self.voltage / self.resistance - self.no_load_torque)
    }

    /// Stall current `V/R` [A].
    pub fn stall_current(&self) -> Option<f64> {
        if !self.is_valid() {
            return None;
        }
        Some(self.voltage / self.resistance)
    }

    /// No-load speed [rad/s] — the speed at which the **shaft** torque reaches
    /// zero, i.e. where the electromagnetic torque has fallen to exactly the
    /// friction torque.
    ///
    /// This is *not* `V/k_e`: that is where the electromagnetic torque vanishes,
    /// which is a faster speed than a real machine can reach because friction
    /// stops it first. Distinguishing the two matters — `V/k_e` is what the
    /// armature-circuit algebra gives you, and using it as "no-load speed" makes
    /// [`Self::torque_at_speed`] return a non-zero value there, which is how a
    /// torque–speed line ends up not crossing zero where it claims to.
    ///
    /// With `no_load_torque = 0` this reduces to `V/k_e`.
    pub fn no_load_speed(&self) -> Option<f64> {
        if !self.is_valid() {
            return None;
        }
        // Solve τ(ω) = 0 for ω, with τ(ω) = k_e·(V − k_e·ω)/R − τ_friction.
        let speed = (self.voltage - self.no_load_torque * self.resistance / self.ke) / self.ke;
        (speed >= 0.0).then_some(speed)
    }

    /// The speed at which the **electromagnetic** torque vanishes, `V/k_e` [rad/s].
    ///
    /// The armature-circuit "ideal" no-load speed. It is an upper bound on
    /// [`Self::no_load_speed`] and is exposed separately because callers doing
    /// circuit algebra (back-EMF budgets, field-weakening limits) want this
    /// quantity specifically, not the friction-limited one.
    pub fn ideal_no_load_speed(&self) -> Option<f64> {
        if !self.is_valid() {
            return None;
        }
        Some(self.voltage / self.ke)
    }

    /// Shaft speed at which the machine delivers exactly `torque` [rad/s].
    ///
    /// The inverse of [`Self::torque_at_speed`]; `None` if the requested torque
    /// exceeds stall torque (the machine simply cannot make it).
    pub fn speed_at_torque(&self, torque: f64) -> Option<f64> {
        if !self.is_valid() || !finite_many(&[torque]) {
            return None;
        }
        let electrical_torque = torque + self.no_load_torque;
        let current = electrical_torque / self.ke;
        let speed = (self.voltage - current * self.resistance) / self.ke;
        (speed >= 0.0).then_some(speed)
    }

    /// Shaft power at `omega` [W]. Negative means regenerating.
    pub fn shaft_power(&self, omega: f64) -> Option<f64> {
        mechanical_power(self.torque_at_speed(omega)?, omega)
    }

    /// Efficiency at `omega`, as `shaft_power / electrical_power` in `(0, 1]`.
    ///
    /// Returns `None` where the machine is not motoring (zero or negative shaft
    /// power) — reporting a "negative efficiency" there would be meaningless.
    pub fn efficiency(&self, omega: f64) -> Option<f64> {
        let torque = self.torque_at_speed(omega)?;
        let shaft = torque * omega;
        let electrical = self.voltage * self.current_at_speed(omega)?;
        if electrical <= 0.0 || shaft <= 0.0 {
            return None;
        }
        let eta = shaft / electrical;
        (eta > 0.0 && eta <= 1.0).then_some(eta)
    }
}

// ---------------------------------------------------------------------------
// Series-wound DC machine
// ---------------------------------------------------------------------------

/// A series-wound DC machine, where field and armature carry the same current.
///
/// Because the field flux is itself proportional to current, torque goes as
/// `I²` rather than `I`, and the torque–speed curve is a rectangular hyperbola:
///
/// ```text
/// τ(ω) = k / (ω + ω_0)
/// ```
///
/// Two consequences matter to a physics engine:
///
/// 1. **Stall torque is unbounded** in the model — the machine will happily
///    produce more torque the slower it turns. Real starters destroy themselves
///    this way; there is no finite stall number to clamp to, so the caller must
///    impose a current limit.
/// 2. **It runs away at no load.** With no torque demand, `ω → ∞`. Never attach a
///    series motor to a free joint without a speed cap.
#[derive(Clone, Copy, Debug)]
pub struct SeriesMotor {
    /// Lumped constant `k` in `τ = k/(ω + ω_0)` [N·m·rad/s].
    pub torque_constant: f64,
    /// Speed offset `ω_0` [rad/s], `> 0`, keeping the hyperbola finite at `ω = 0`.
    pub speed_offset: f64,
}

impl SeriesMotor {
    pub fn is_valid(&self) -> bool {
        finite_many(&[self.torque_constant, self.speed_offset])
            && self.torque_constant > 0.0
            && self.speed_offset > 0.0
    }

    /// Torque at `omega` [N·m]. Always positive and strictly decreasing; never
    /// reaches zero, which is exactly the runaway problem.
    pub fn torque_at_speed(&self, omega: f64) -> Option<f64> {
        if !self.is_valid() || !finite_many(&[omega]) {
            return None;
        }
        let denominator = omega.max(0.0) + self.speed_offset;
        Some(self.torque_constant / denominator)
    }

    /// Shaft speed at which torque falls to `torque` [rad/s].
    ///
    /// `None` for a non-positive torque: the curve has no such point, and
    /// returning a huge number instead would hide the runaway.
    pub fn speed_at_torque(&self, torque: f64) -> Option<f64> {
        if !self.is_valid() || !finite_many(&[torque]) || torque <= 0.0 {
            return None;
        }
        let speed = self.torque_constant / torque - self.speed_offset;
        (speed >= 0.0).then_some(speed)
    }

    /// The speed a caller must impose to keep the machine at or below `power`
    /// [W]: solving `τ(ω)·ω = P` gives `ω² + ω_0·ω − k/P·... ` — reported here as
    /// the positive root.
    ///
    /// This is the cap that makes a series motor usable in a simulation at all.
    pub fn speed_limit_for_power(&self, power: f64) -> Option<f64> {
        if !self.is_valid() || !finite_many(&[power]) || power <= 0.0 {
            return None;
        }
        // τ·ω = k·ω/(ω + ω_0) = P  =>  ω·(k − P) = P·ω_0  =>  ω = P·ω_0/(k − P)
        let denominator = self.torque_constant - power;
        if denominator <= 0.0 {
            // The machine cannot reach this power at any finite speed.
            return None;
        }
        Some(power * self.speed_offset / denominator)
    }
}

// ---------------------------------------------------------------------------
// Brushless DC / permanent-magnet synchronous machine
// ---------------------------------------------------------------------------

/// A brushless DC (trapezoidal) or PMSM (sinusoidal) machine driven by an
/// inverter.
///
/// Electrically this is the same machine as [`DcMotor`] — the linear droop from
/// stall to no-load speed is unchanged. What the inverter adds is a *voltage
/// limit* that shrinks as speed rises (field weakening), and what the magnetics
/// add is a **cogging / torque-ripple** term:
///
/// ```text
/// τ(ω, θ) = τ_droop(ω) + τ_ripple·sin(n·θ + φ)
/// ```
///
/// `n` is the cogging period count per revolution (slots × poles / gcd). The
/// ripple is what a position servo feels as "notchy" motion and what a
/// low-resolution encoder cannot distinguish from noise.
#[derive(Clone, Copy, Debug)]
pub struct BrushlessMotor {
    /// DC-bus voltage available to the inverter [V].
    pub bus_voltage: f64,
    /// Per-phase resistance [Ω].
    pub phase_resistance: f64,
    /// Back-EMF constant [V·s/rad] (line-to-line, mechanical rad/s).
    pub ke: f64,
    /// Peak cogging torque amplitude [N·m], `>= 0`.
    pub cogging_torque: f64,
    /// Cogging periods per mechanical revolution, `>= 1`.
    pub cogging_periods: u32,
    /// Speed above which field weakening starts [rad/s]; `0` disables it.
    pub base_speed: f64,
}

impl BrushlessMotor {
    pub fn is_valid(&self) -> bool {
        finite_many(&[
            self.bus_voltage,
            self.phase_resistance,
            self.ke,
            self.cogging_torque,
            self.base_speed,
        ]) && self.phase_resistance > 0.0
            && self.ke > 0.0
            && self.cogging_torque >= 0.0
            && self.cogging_periods >= 1
            && self.base_speed >= 0.0
    }

    /// The voltage the inverter can still apply at `omega`, after the back-EMF
    /// eats into the bus.
    ///
    /// `v_eff = max(bus − k_e·ω, 0)`; field weakening (when `base_speed > 0`)
    /// keeps `v_eff` from collapsing to zero immediately past the base speed by
    /// allowing a negative d-axis current, modelled here as a soft floor at 10%
    /// of bus voltage.
    pub fn effective_voltage(&self, omega: f64) -> Option<f64> {
        if !self.is_valid() || !finite_many(&[omega]) {
            return None;
        }
        let available = self.bus_voltage - self.ke * omega.abs();
        if self.base_speed > 0.0 && omega.abs() > self.base_speed {
            // Field weakening: hold a floor so the machine can still be driven,
            // at the cost of efficiency (the d-axis current is pure loss).
            return Some(available.max(0.1 * self.bus_voltage));
        }
        Some(available.max(0.0))
    }

    /// Average (ripple-free) torque at `omega` [N·m].
    pub fn torque_at_speed(&self, omega: f64) -> Option<f64> {
        let v_eff = self.effective_voltage(omega)?;
        Some(self.ke * v_eff / self.phase_resistance)
    }

    /// Instantaneous torque including cogging at mechanical angle `theta` [rad].
    ///
    /// This is the term that makes a brushless motor feel notchy; a caller
    /// simulating a holding position needs it, a caller simulating bulk motion
    /// usually does not.
    pub fn torque_at(&self, omega: f64, theta: f64) -> Option<f64> {
        if !finite_many(&[theta]) {
            return None;
        }
        let average = self.torque_at_speed(omega)?;
        let ripple = self.cogging_torque * (self.cogging_periods as f64 * theta + PI * 0.5).sin();
        Some(average + ripple)
    }

    /// No-load speed, where `effective_voltage` first hits zero (or the field
    /// weakening floor stops it) [rad/s].
    ///
    /// With field weakening enabled the machine has no finite no-load speed in
    /// this model, so `None` is returned rather than a fabricated large number.
    pub fn no_load_speed(&self) -> Option<f64> {
        if !self.is_valid() || self.base_speed > 0.0 {
            return None;
        }
        Some(self.bus_voltage / self.ke)
    }

    /// Copper loss in all three phases at `omega` [W].
    ///
    /// Current is `τ/k_e` and is shared by the phases; for a balanced three-phase
    /// machine the total is `3·I_phase²·R_phase`, and with `I_phase = I_dc/√3`
    /// this reduces to `I_dc²·R_phase`.
    pub fn copper_loss(&self, omega: f64) -> Option<f64> {
        let torque = self.torque_at_speed(omega)?;
        let current = torque / self.ke;
        Some(current * current * self.phase_resistance)
    }
}

// ---------------------------------------------------------------------------
// Stepper motor
// ---------------------------------------------------------------------------

/// A hybrid stepper motor, modelled by its pull-out torque curve.
///
/// A stepper produces torque only while the rotor lags the commanded field. That
/// lag is bounded by one full step, so available torque falls with speed — the
/// dominant fact about steppers and the reason they lose steps if accelerated
/// too hard:
///
/// ```text
/// τ_pullout(ω) = τ_hold / (1 + (ω/ω_corner)^2)^(1/2)
/// ```
///
/// Below `ω_corner` the machine is essentially at holding torque; above it,
/// torque falls as `1/ω` and therefore power plateaus.
#[derive(Clone, Copy, Debug)]
pub struct StepperMotor {
    /// Holding torque at standstill [N·m].
    pub holding_torque: f64,
    /// Corner speed where torque has dropped to `τ_hold/√2` [rad/s].
    pub corner_speed: f64,
    /// Full steps per revolution (200 for a 1.8° motor).
    pub steps_per_revolution: u32,
    /// Rotor + load inertia seen by the motor [kg·m²], for the step-response
    /// estimate. `0` disables [`Self::max_start_rate`].
    pub rotor_inertia: f64,
}

impl StepperMotor {
    pub fn is_valid(&self) -> bool {
        finite_many(&[self.holding_torque, self.corner_speed, self.rotor_inertia])
            && self.holding_torque > 0.0
            && self.corner_speed > 0.0
            && self.steps_per_revolution > 0
            && self.rotor_inertia >= 0.0
    }

    /// Angular size of one full step [rad].
    pub fn step_angle(&self) -> Option<f64> {
        if !self.is_valid() {
            return None;
        }
        Some(2.0 * PI / self.steps_per_revolution as f64)
    }

    /// Pull-out torque at `omega` [N·m].
    pub fn torque_at_speed(&self, omega: f64) -> Option<f64> {
        if !self.is_valid() || !finite_many(&[omega]) {
            return None;
        }
        let ratio = omega.abs() / self.corner_speed;
        Some(self.holding_torque / (1.0 + ratio * ratio).sqrt())
    }

    /// The speed at which pull-out torque falls to `torque` [rad/s].
    pub fn speed_at_torque(&self, torque: f64) -> Option<f64> {
        if !self.is_valid() || !finite_many(&[torque]) {
            return None;
        }
        if torque <= 0.0 || torque > self.holding_torque {
            return None;
        }
        let ratio = ((self.holding_torque / torque).powi(2) - 1.0)
            .max(0.0)
            .sqrt();
        Some(self.corner_speed * ratio)
    }

    /// Highest step rate at which the rotor can start without losing steps
    /// [steps/s].
    ///
    /// Starting from rest, the rotor must accelerate its own inertia through one
    /// step before the field moves on. Equating the torque available at the
    /// resulting speed to `J·α` gives a start rate; this is the classic reason a
    /// stepper needs a ramp rather than a step input.
    ///
    /// Returns `None` when `rotor_inertia` is zero (no inertia to accelerate).
    pub fn max_start_rate(&self) -> Option<f64> {
        if !self.is_valid() || self.rotor_inertia <= 0.0 {
            return None;
        }
        // Per step, energy τ_hold·Δθ goes into ½·J·ω² => ω = sqrt(2·τ·Δθ/J).
        let step = self.step_angle()?;
        let omega = (2.0 * self.holding_torque * step / self.rotor_inertia).sqrt();
        Some(omega / step)
    }

    /// Rotor position after `steps` commanded steps, in `[0, 2π)` [rad].
    ///
    /// Microstepping is `steps` counted in microsteps, so the caller chooses the
    /// resolution; the function only needs the step angle.
    pub fn position_after_steps(&self, steps: i64) -> Option<f64> {
        let step = self.step_angle()?;
        Some((steps as f64 * step).rem_euclid(2.0 * PI))
    }
}

// ---------------------------------------------------------------------------
// AC induction machine
// ---------------------------------------------------------------------------

/// A three-phase squirrel-cage induction machine.
///
/// Torque depends on **slip** `s = (ω_sync − ω)/ω_sync`, where
/// `ω_sync = 2π·f/p` is the synchronous speed for supply frequency `f` and pole
/// pairs `p`. The classic Kloss form is used:
///
/// ```text
/// τ(s) = 2·τ_max / (s/s_max + s_max/s)
/// ```
///
/// It rises steeply, peaks at `s_max` (breakdown), then collapses. Below the
/// breakdown slip the machine is stable and self-regulating — that is why
/// induction motors dominate industry — but it **cannot** produce torque at
/// synchronous speed, and it cannot start at full voltage under heavy load.
#[derive(Clone, Copy, Debug)]
pub struct InductionMotor {
    /// Supply frequency [Hz].
    pub frequency: f64,
    /// Pole pairs (1 = 2-pole, 2 = 4-pole, …).
    pub pole_pairs: u32,
    /// Breakdown (pull-out) torque [N·m].
    pub breakdown_torque: f64,
    /// Slip at which breakdown torque occurs, in `(0, 1]`.
    pub breakdown_slip: f64,
}

impl InductionMotor {
    pub fn is_valid(&self) -> bool {
        finite_many(&[self.frequency, self.breakdown_torque, self.breakdown_slip])
            && self.frequency > 0.0
            && self.pole_pairs > 0
            && self.breakdown_torque > 0.0
            && self.breakdown_slip > 0.0
            && self.breakdown_slip <= 1.0
    }

    /// Synchronous speed `2π·f/p` [rad/s] — the speed of the rotating field.
    pub fn synchronous_speed(&self) -> Option<f64> {
        if !self.is_valid() {
            return None;
        }
        Some(2.0 * PI * self.frequency / self.pole_pairs as f64)
    }

    /// Slip at shaft speed `omega`, in `(0, 1]` while motoring.
    ///
    /// Returns `None` above synchronous speed: the machine is then generating
    /// (negative slip), a different regime this model does not cover.
    pub fn slip(&self, omega: f64) -> Option<f64> {
        let sync = self.synchronous_speed()?;
        if !finite_many(&[omega]) || omega < 0.0 || omega >= sync {
            return None;
        }
        Some((sync - omega) / sync)
    }

    /// Torque at shaft speed `omega` [N·m].
    pub fn torque_at_speed(&self, omega: f64) -> Option<f64> {
        let s = self.slip(omega)?;
        self.torque_at_slip(s)
    }

    /// Torque at slip `s` [N·m], the Kloss form.
    ///
    /// `s` must be in `(0, 1]`; `s = 0` (synchronous) produces no torque, so it
    /// returns `None` rather than zero — the distinction matters to a caller
    /// checking whether it can start.
    pub fn torque_at_slip(&self, s: f64) -> Option<f64> {
        if !self.is_valid() || !finite_many(&[s]) || s <= 0.0 || s > 1.0 {
            return None;
        }
        let ratio = s / self.breakdown_slip;
        let denominator = ratio + 1.0 / ratio;
        if denominator <= 0.0 {
            return None;
        }
        Some(2.0 * self.breakdown_torque / denominator)
    }

    /// Starting torque at `omega = 0` [N·m] — the number that decides whether a
    /// load can be started at all.
    pub fn starting_torque(&self) -> Option<f64> {
        self.torque_at_speed(0.0)
    }

    /// Shaft speed at which the machine makes `torque` [rad/s], on the stable
    /// (low-slip) branch.
    ///
    /// The Kloss curve is double-valued; this returns the branch the machine
    /// actually settles on. `None` above breakdown torque.
    pub fn speed_at_torque(&self, torque: f64) -> Option<f64> {
        if !self.is_valid() || !finite_many(&[torque]) || torque <= 0.0 {
            return None;
        }
        if torque > self.breakdown_torque {
            return None;
        }
        // Solve ratio + 1/ratio = 2·τ_max/τ for ratio <= 1.
        let target = 2.0 * self.breakdown_torque / torque;
        let discriminant = target * target - 4.0;
        if discriminant < 0.0 {
            return None;
        }
        let ratio = (target - discriminant.sqrt()) / 2.0;
        let s = ratio * self.breakdown_slip;
        let sync = self.synchronous_speed()?;
        Some(sync * (1.0 - s))
    }
}

// ---------------------------------------------------------------------------
// Servo motor (the machine, not the controller)
// ---------------------------------------------------------------------------

/// The torque envelope of a servo motor: **constant torque** up to a rated
/// speed, **constant power** beyond it, up to a maximum speed.
///
/// This envelope is the whole point of a servo motor, and it is a property of the
/// machine plus its drive (the drive limits current below base speed and voltage
/// above it):
///
/// ```text
/// τ(ω) = τ_rated                                  for ω <= ω_rated
/// τ(ω) = τ_rated·ω_rated/ω                        for ω_rated < ω <= ω_max
/// τ(ω) = 0                                        for ω > ω_max
/// ```
///
/// Note that this is an *envelope*, not a force law: a servo asked for less
/// torque delivers less. [`Self::available_torque`] therefore takes the demand
/// into account.
#[derive(Clone, Copy, Debug)]
pub struct ServoMotor {
    /// Continuous (thermal) torque rating [N·m].
    pub rated_torque: f64,
    /// Peak torque available briefly [N·m], `>= rated_torque`.
    pub peak_torque: f64,
    /// Base speed where the constant-torque region ends [rad/s].
    pub rated_speed: f64,
    /// Maximum mechanical speed [rad/s].
    pub max_speed: f64,
}

impl ServoMotor {
    pub fn is_valid(&self) -> bool {
        finite_many(&[
            self.rated_torque,
            self.peak_torque,
            self.rated_speed,
            self.max_speed,
        ]) && self.rated_torque > 0.0
            && self.peak_torque >= self.rated_torque
            && self.rated_speed > 0.0
            && self.max_speed >= self.rated_speed
    }

    /// Continuous torque available at `omega` [N·m] — the envelope.
    pub fn torque_at_speed(&self, omega: f64) -> Option<f64> {
        if !self.is_valid() || !finite_many(&[omega]) {
            return None;
        }
        let w = omega.abs();
        if w <= self.rated_speed {
            Some(self.rated_torque)
        } else if w <= self.max_speed {
            Some(self.rated_torque * self.rated_speed / w)
        } else {
            Some(0.0)
        }
    }

    /// Peak torque available at `omega` [N·m], same envelope shape scaled up.
    pub fn peak_torque_at_speed(&self, omega: f64) -> Option<f64> {
        let continuous = self.torque_at_speed(omega)?;
        Some(continuous * (self.peak_torque / self.rated_torque))
    }

    /// Torque actually delivered for a `demand` at `omega` [N·m].
    ///
    /// Clamps the demand to the envelope and to `peak_torque`; a demand beyond
    /// what the machine can make is *saturated*, which is the behaviour a
    /// controller must cope with.
    pub fn available_torque(&self, omega: f64, demand: f64) -> Option<f64> {
        if !finite_many(&[demand]) {
            return None;
        }
        let envelope = self.peak_torque_at_speed(omega)?;
        Some(demand.clamp(-envelope, envelope))
    }

    /// Whether the machine is running inside its continuous (thermal) rating.
    ///
    /// `false` means the demand is only sustainable for the peak duration — the
    /// caller is responsible for tracking that duration.
    pub fn within_continuous_rating(&self, omega: f64, demand: f64) -> Option<bool> {
        let envelope = self.torque_at_speed(omega)?;
        Some(demand.abs() <= envelope)
    }
}

// ---------------------------------------------------------------------------
// Linear motor
// ---------------------------------------------------------------------------

/// A linear motor: the same machine as its rotary twin, with force instead of
/// torque and velocity instead of angular velocity.
///
/// A linear synchronous motor has the same constant-force / constant-power
/// envelope as a rotary servo, and a linear induction motor has the same slip
/// behaviour as an induction machine. This type models the synchronous case,
/// which is what a gantry or a maglev uses.
#[derive(Clone, Copy, Debug)]
pub struct LinearMotor {
    /// Continuous force rating [N].
    pub rated_force: f64,
    /// Peak force [N], `>= rated_force`.
    pub peak_force: f64,
    /// Base speed [m/s].
    pub rated_speed: f64,
    /// Maximum speed [m/s].
    pub max_speed: f64,
}

impl LinearMotor {
    pub fn is_valid(&self) -> bool {
        finite_many(&[
            self.rated_force,
            self.peak_force,
            self.rated_speed,
            self.max_speed,
        ]) && self.rated_force > 0.0
            && self.peak_force >= self.rated_force
            && self.rated_speed > 0.0
            && self.max_speed >= self.rated_speed
    }

    /// Continuous force available at velocity `v` [N].
    pub fn force_at_speed(&self, v: f64) -> Option<f64> {
        if !self.is_valid() || !finite_many(&[v]) {
            return None;
        }
        let speed = v.abs();
        if speed <= self.rated_speed {
            Some(self.rated_force)
        } else if speed <= self.max_speed {
            Some(self.rated_force * self.rated_speed / speed)
        } else {
            Some(0.0)
        }
    }

    /// Force actually delivered for a `demand` at velocity `v` [N].
    pub fn available_force(&self, v: f64, demand: f64) -> Option<f64> {
        if !finite_many(&[demand]) {
            return None;
        }
        let envelope = self.force_at_speed(v)? * (self.peak_force / self.rated_force);
        Some(demand.clamp(-envelope, envelope))
    }

    /// Mechanical power at velocity `v` for `demand` [W].
    pub fn power(&self, v: f64, demand: f64) -> Option<f64> {
        mechanical_power(self.available_force(v, demand)?, v)
    }
}

// ---------------------------------------------------------------------------
// Piezoelectric / ultrasonic actuator
// ---------------------------------------------------------------------------

/// A piezoelectric actuator, in either quasi-static (stack) or resonant
/// (ultrasonic motor) mode.
///
/// A stack expands by `ΔL = d₃₃·V·n` and blocks a force `F_block = k·ΔL`; an
/// ultrasonic motor instead converts a travelling wave into motion at a
/// resonance, trading force for speed. Both are characterised by a **blocking
/// force** (the most it can push) and a **free displacement** (the most it can
/// move unloaded), with the force–displacement trade-off being a straight line
/// between them — the same structure as the DC torque–speed line.
#[derive(Clone, Copy, Debug)]
pub struct PiezoActuator {
    /// Blocking force at maximum drive [N].
    pub blocking_force: f64,
    /// Free displacement (stroke) at maximum drive [m].
    pub free_displacement: f64,
    /// Mechanical resonance [Hz]; `0` for a quasi-static stack.
    pub resonance_hz: f64,
}

impl PiezoActuator {
    pub fn is_valid(&self) -> bool {
        finite_many(&[
            self.blocking_force,
            self.free_displacement,
            self.resonance_hz,
        ]) && self.blocking_force > 0.0
            && self.free_displacement > 0.0
            && self.resonance_hz >= 0.0
    }

    /// Force available while displaced by `displacement` [N].
    ///
    /// Linear trade-off: full blocking force at zero displacement, zero force at
    /// full stroke. Displacement beyond the stroke yields zero (the actuator has
    /// simply left the contact), not a negative force.
    pub fn force_at_displacement(&self, displacement: f64) -> Option<f64> {
        if !self.is_valid() || !finite_many(&[displacement]) {
            return None;
        }
        let ratio = (displacement.abs() / self.free_displacement).min(1.0);
        Some(self.blocking_force * (1.0 - ratio))
    }

    /// Effective stiffness `k = F_block / ΔL_free` [N/m].
    pub fn stiffness(&self) -> Option<f64> {
        if !self.is_valid() {
            return None;
        }
        Some(self.blocking_force / self.free_displacement)
    }

    /// Work done per full stroke against a constant `load_force` [J].
    ///
    /// The useful work peaks at half the blocking force — a classic
    /// impedance-matching result, and the reason a piezo is run well below its
    /// blocking force when driving a load.
    pub fn work_per_stroke(&self, load_force: f64) -> Option<f64> {
        if !finite_many(&[load_force]) || load_force < 0.0 {
            return None;
        }
        if load_force > self.blocking_force {
            return None;
        }
        // Travel until force falls to the load: x = ΔL·(1 − F/F_block).
        let travel = self.free_displacement * (1.0 - load_force / self.blocking_force);
        Some(load_force * travel)
    }

    /// The load force that maximises work per stroke [N] — half the blocking
    /// force.
    pub fn matched_load_force(&self) -> Option<f64> {
        if !self.is_valid() {
            return None;
        }
        Some(0.5 * self.blocking_force)
    }

    /// Maximum operating frequency [Hz]: the resonance for an ultrasonic motor,
    /// or a bandwidth set by the stack's capacitance for a quasi-static one.
    ///
    /// Returns `None` for a quasi-static actuator with no resonance specified —
    /// its bandwidth is a drive-electronics question, not a machine one.
    pub fn max_frequency(&self) -> Option<f64> {
        if !self.is_valid() || self.resonance_hz <= 0.0 {
            return None;
        }
        Some(self.resonance_hz)
    }
}

// ---------------------------------------------------------------------------
// Shape-memory-alloy actuator
// ---------------------------------------------------------------------------

/// A shape-memory-alloy (SMA) wire actuator.
///
/// Heating above the austenite finish temperature recovers a fixed strain and
/// generates a recovery stress; cooling lets it be stretched again. The behaviour
/// is **hysteretic and one-way** in the simple form modelled here: heat to
/// contract, force-cool to reset. Two consequences a physics engine must respect:
///
/// 1. It is a *thermal* actuator — the bandwidth is set by how fast you can heat
///    and cool, which is why SMA wires are thin.
/// 2. Its stroke is bounded by the recoverable strain (typically a few percent),
///    so it is a high-force / low-stroke device.
#[derive(Clone, Copy, Debug)]
pub struct ShapeMemoryActuator {
    /// Wire length at rest [m].
    pub rest_length: f64,
    /// Recoverable strain at full actuation (dimensionless, e.g. `0.04`) [-].
    pub recoverable_strain: f64,
    /// Recovery stress at full actuation [Pa].
    pub recovery_stress: f64,
    /// Wire cross-sectional area [m²].
    pub cross_section: f64,
    /// Austenite finish temperature [K] — above this the wire contracts.
    pub austenite_finish: f64,
}

impl ShapeMemoryActuator {
    pub fn is_valid(&self) -> bool {
        finite_many(&[
            self.rest_length,
            self.recoverable_strain,
            self.recovery_stress,
            self.cross_section,
            self.austenite_finish,
        ]) && self.rest_length > 0.0
            && self.recoverable_strain > 0.0
            && self.recoverable_strain < 1.0
            && self.recovery_stress > 0.0
            && self.cross_section > 0.0
    }

    /// Blocked (maximum) force `σ·A` [N].
    pub fn blocking_force(&self) -> Option<f64> {
        if !self.is_valid() {
            return None;
        }
        Some(self.recovery_stress * self.cross_section)
    }

    /// Free stroke `L·ε` [m] — how far the wire shortens when unloaded.
    pub fn free_stroke(&self) -> Option<f64> {
        if !self.is_valid() {
            return None;
        }
        Some(self.rest_length * self.recoverable_strain)
    }

    /// Actuation fraction in `[0, 1]` from temperature, as a smooth step across
    /// the transformation.
    ///
    /// A real SMA has separate austenite-start and finish temperatures and a
    /// wide hysteresis band; this uses a logistic centred on
    /// `austenite_finish` with a 10 K width, which captures "cold = soft, hot =
    /// contracted" without pretending to model the hysteresis loop. Callers
    /// needing the loop must track heating vs cooling themselves.
    pub fn actuation_fraction(&self, temperature: f64) -> Option<f64> {
        if !self.is_valid() || !finite_many(&[temperature]) {
            return None;
        }
        let width = 10.0;
        let z = (temperature - self.austenite_finish) / width;
        // Numerically safe logistic.
        Some(if z >= 0.0 {
            1.0 / (1.0 + (-z).exp())
        } else {
            let e = z.exp();
            e / (1.0 + e)
        })
    }

    /// Force at `temperature` for a blocked wire [N].
    pub fn blocked_force_at(&self, temperature: f64) -> Option<f64> {
        Some(self.blocking_force()? * self.actuation_fraction(temperature)?)
    }

    /// Stroke at `temperature` for a free wire [m].
    pub fn stroke_at(&self, temperature: f64) -> Option<f64> {
        Some(self.free_stroke()? * self.actuation_fraction(temperature)?)
    }

    /// Work available per actuation cycle against `load_force` [J].
    ///
    /// Same impedance-matching shape as the piezo: work is `F·x` along a linear
    /// force–stroke trade-off, peaking at half the blocking force.
    pub fn work_per_cycle(&self, load_force: f64) -> Option<f64> {
        if !finite_many(&[load_force]) || load_force < 0.0 {
            return None;
        }
        let blocking = self.blocking_force()?;
        if load_force > blocking {
            return None;
        }
        let stroke = self.free_stroke()?;
        let travel = stroke * (1.0 - load_force / blocking);
        Some(load_force * travel)
    }
}
