//! Physics of the robot body: an inverted pendulum on driven wheels.
//!
//! # The model
//!
//! The body pivots on the axle; the wheels roll on the floor without
//! slipping. The input is the **acceleration of the axle** `a` (forward
//! positive), which is what the wheels do. The motors themselves are left
//! out for now: this assumes they produce whatever torque that acceleration
//! needs, instantly. The motor-model step replaces that with "duty cycle in,
//! torque out" later.
//!
//! Two things act on the body when the axle accelerates:
//!
//! 1. **The pseudo-force.** Seen from the axle, an accelerating axle looks
//!    like a force `−m·a` pulling the centre of mass backwards (like being
//!    pushed into the seat when a car speeds up): a torque `−m·d·a·cos θ`.
//! 2. **The motor's reaction torque.** The motors are bolted to the body.
//!    To spin the wheels forward they push on the wheels, and the wheels
//!    push back on the body just as hard: `−τ`, also tipping the body back.
//!    `τ` is whatever the wheels need: their own spin-up (`I_w·a/r`) plus
//!    `r` times the floor's grip force, which accelerates the whole robot
//!    forward (`m·a`) and the centre of mass's swing (`m·d·(θ''·cos θ −
//!    θ'²·sin θ)`).
//!
//! Adding gravity and collecting the `θ''` terms:
//!
//! ```text
//! (I + m·d·r·cos θ)·θ'' = m·d·g·sin θ + m·d·r·θ'²·sin θ
//!                         − a·(m·d·cos θ + m·r + I_w/r)
//! ```
//!
//! - `θ` pitch, 0 = upright, positive = tilted forward (same sign as the
//!   firmware's pitch); `a` axle acceleration
//! - `m` total mass, `d` height of the centre of mass above the axle (both of
//!   the whole robot: the wheels' mass sits at the axle, so `m·d` is the
//!   same with or without them)
//! - `I` moment of inertia of the body about the axle, `r` wheel radius,
//!   `I_w` spin inertia of both wheels including the motors' gearing
//!
//! Gravity tips the robot over further; driving the wheels forward (`a > 0`)
//! pushes it back. That is the whole balancing problem: drive the wheels
//! under the falling centre of mass.
//!
//! A plain cart with a pendulum on it (the "cart-pole" of textbooks) is the
//! same without the reaction torque, i.e. `r = I_w = 0`. The first version
//! of this model was that, which underestimated how much driving the wheels
//! helps (by ~40 % for this robot).
//!
//! There is no floor: the model keeps rotating through 90° and beyond. Where
//! the real robot hits the ground depends on the chassis, and a controller
//! has lost long before that anyway.
//!
//! # How fast does it fall?
//!
//! With the axle held still (`a = 0`, the motors locking the wheels to the
//! floor) and a small angle (`sin θ ≈ θ`): `θ'' = θ / τ²` with
//! `τ = √((I + m·d·r) / (m·g·d))`. Starting at rest at `θ₀`, the solution is
//! `θ(t) = θ₀·cosh(t/τ)`: the tilt multiplies by e ≈ 2.7 every `τ` seconds.
//!
//! `m·d·r` is the reaction torque again: to hold the axle still, the floor
//! has to push it back while the centre of mass swings forward, and the
//! motors have to hold the wheels against that push.
//!
//! # Integration
//!
//! [`State::step`] advances time by a small `dt` with semi-implicit Euler:
//! update the speeds from the accelerations, then the positions from the
//! *new* speeds. This is as simple as plain Euler, but unlike plain Euler it
//! doesn't add energy to an oscillation, so a swinging pendulum doesn't
//! slowly swing higher. Its error shrinks with `dt`; at 0.1 ms it is far below
//! anything we can measure on the robot.

use std::f64::consts::PI;

/// Gravity in m/s².
pub const G: f64 = 9.81;

/// Mass properties of the robot.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Body {
    /// Total mass in kg.
    pub mass_kg: f64,
    /// Height of the centre of mass above the axle in m.
    pub com_height_m: f64,
    /// Moment of inertia of the body about the axle in kg·m².
    pub inertia_kg_m2: f64,
    /// Wheel radius in m.
    pub wheel_radius_m: f64,
    /// Spin inertia of both wheels about the axle in kg·m², including the
    /// motor rotors as seen through the gearbox.
    pub wheel_inertia_kg_m2: f64,
}

impl Body {
    /// The prototype.
    ///
    /// Measured 2026-11-11: 350 g, centre of mass 8 cm above the axle,
    /// 10 swings in 7 s hanging upside down (`I` ≈ 0.0034 kg·m²); 65 mm
    /// wheels.
    ///
    /// Estimated, not measured: wheel spin inertia 3·10⁻⁴ kg·m². Two ~25 g
    /// wheels are only ~4·10⁻⁵; the rest is the motor rotors, whose inertia
    /// counts with the gear ratio squared (TT motors are typically 1:48, so
    /// ×2300). The motor-model step can measure it from how fast a wheel
    /// spins up in the air.
    ///
    /// Gives a fall time constant `τ` ≈ 0.125 s.
    #[must_use]
    pub fn prototype() -> Self {
        Self {
            wheel_radius_m: 0.0325,
            wheel_inertia_kg_m2: 3e-4,
            ..Self::from_swing_test(0.350, 0.08, 0.70)
        }
    }

    /// Body from a swing test: hang the robot upside down from its axle (no
    /// floor contact), let it swing a little and time one full swing (there
    /// and back), `period_s`.
    ///
    /// A pendulum on a fixed pivot swings with `T = 2π·√(I / (m·g·d))`, so
    /// `I = m·g·d·T² / (4π²)`. Wheel radius and inertia are left at 0.
    #[must_use]
    pub fn from_swing_test(mass_kg: f64, com_height_m: f64, period_s: f64) -> Self {
        Self {
            mass_kg,
            com_height_m,
            inertia_kg_m2: mass_kg * G * com_height_m * period_s * period_s / (4.0 * PI * PI),
            wheel_radius_m: 0.0,
            wheel_inertia_kg_m2: 0.0,
        }
    }

    /// Fall time constant `τ = √((I + m·d·r) / (m·g·d))` in s: with the axle
    /// held still, a small tilt grows by a factor e every `τ` seconds.
    #[must_use]
    pub fn fall_time_constant_s(&self) -> f64 {
        let md = self.mass_kg * self.com_height_m;
        ((self.inertia_kg_m2 + md * self.wheel_radius_m) / (md * G)).sqrt()
    }

    /// Angular acceleration `θ''` in rad/s² in `state` with the axle
    /// accelerating at `axle_accel_m_s2` (the equation in the module docs).
    #[must_use]
    pub fn pitch_accel(&self, state: &State, axle_accel_m_s2: f64) -> f64 {
        let md = self.mass_kg * self.com_height_m;
        let r = self.wheel_radius_m;
        let (sin, cos) = state.pitch_rad.sin_cos();
        let torque = md * G * sin + md * r * state.pitch_rate_rad_s.powi(2) * sin
            - axle_accel_m_s2 * self.axle_accel_lever(cos);
        torque / (self.inertia_kg_m2 + md * r * cos)
    }

    /// The axle acceleration that keeps the robot at rest at `pitch_rad`
    /// (`θ'' = 0`). It has to keep speeding up to do so.
    #[must_use]
    pub fn holding_accel(&self, pitch_rad: f64) -> f64 {
        let (sin, cos) = pitch_rad.sin_cos();
        self.mass_kg * self.com_height_m * G * sin / self.axle_accel_lever(cos)
    }

    /// How much torque on the body each m/s² of axle acceleration gives:
    /// `m·d·cos θ + m·r + I_w/r` (pseudo-force + reaction torque). With no
    /// wheels (`r = 0`) the last terms vanish.
    fn axle_accel_lever(&self, cos: f64) -> f64 {
        let r = self.wheel_radius_m;
        let wheels = if r > 0.0 {
            self.mass_kg * r + self.wheel_inertia_kg_m2 / r
        } else {
            0.0
        };
        self.mass_kg * self.com_height_m * cos + wheels
    }
}

/// Everything that changes over time.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct State {
    /// Pitch in rad, 0 = upright, positive = tilted forward.
    pub pitch_rad: f64,
    /// Pitch rate in rad/s.
    pub pitch_rate_rad_s: f64,
    /// Axle position in m (forward positive) from where it started.
    pub axle_pos_m: f64,
    /// Axle speed in m/s.
    pub axle_vel_m_s: f64,
}

impl State {
    /// At rest, tilted forward by `deg` degrees.
    #[must_use]
    pub fn tilted_deg(deg: f64) -> Self {
        Self {
            pitch_rad: deg.to_radians(),
            ..Self::default()
        }
    }

    /// Pitch in degrees.
    #[must_use]
    pub fn pitch_deg(&self) -> f64 {
        self.pitch_rad.to_degrees()
    }

    /// The state `dt_s` seconds later, with the axle accelerating at
    /// `axle_accel_m_s2` meanwhile (semi-implicit Euler, see module docs).
    ///
    /// Keep `dt_s` small compared with the fall time constant (~0.1 s):
    /// 0.1 ms is plenty.
    #[must_use]
    pub fn step(self, body: &Body, axle_accel_m_s2: f64, dt_s: f64) -> Self {
        let pitch_rate_rad_s =
            self.pitch_rate_rad_s + body.pitch_accel(&self, axle_accel_m_s2) * dt_s;
        let axle_vel_m_s = self.axle_vel_m_s + axle_accel_m_s2 * dt_s;
        Self {
            pitch_rad: self.pitch_rad + pitch_rate_rad_s * dt_s,
            pitch_rate_rad_s,
            axle_pos_m: self.axle_pos_m + axle_vel_m_s * dt_s,
            axle_vel_m_s,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::{Body, G, State};

    const DT: f64 = 1e-4;

    /// Runs `steps` steps of `DT` with a constant axle acceleration.
    fn run(body: &Body, mut state: State, axle_accel: f64, steps: u32) -> State {
        for _ in 0..steps {
            state = state.step(body, axle_accel, DT);
        }
        state
    }

    fn assert_rel_close(actual: f64, expected: f64, rel_tol: f64) {
        assert!(
            (actual - expected).abs() <= rel_tol * expected.abs(),
            "{actual} is not within {rel_tol} of {expected}"
        );
    }

    #[test]
    fn prototype_numbers() {
        let body = Body::prototype();
        // I = m·g·d·T²/(4π²) = 0.35 · 9.81 · 0.08 · 0.49 / 39.48
        assert_rel_close(body.inertia_kg_m2, 0.003_41, 0.01);
        // τ = √((I + m·d·r)/(m·g·d)) = √((0.00341 + 0.00091) / 0.2747)
        assert_rel_close(body.fall_time_constant_s(), 0.1254, 0.002);
    }

    #[test]
    fn without_wheels_it_is_the_swing_test_pendulum() {
        // r = 0: no reaction torque, a pendulum on a fixed pivot. Its τ is
        // T/2π of the swing test, and its holding acceleration g·tan θ.
        let body = Body::from_swing_test(0.35, 0.08, 0.70);
        assert_rel_close(
            body.fall_time_constant_s(),
            0.70 / std::f64::consts::TAU,
            1e-9,
        );
        let pitch = 5.0_f64.to_radians();
        assert_rel_close(body.holding_accel(pitch), G * pitch.tan(), 1e-9);
    }

    #[test]
    fn exactly_upright_stays_upright() {
        let state = run(&Body::prototype(), State::default(), 0.0, 10_000);
        assert_eq!(state, State::default());
    }

    #[test]
    fn small_tilt_falls_like_cosh() {
        // From 1°, compare with θ₀·cosh(t/τ) every 50 ms for 0.35 s (to ~9°).
        let body = Body::prototype();
        let tau = body.fall_time_constant_s();
        let mut state = State::tilted_deg(1.0);
        for k in 1..=7 {
            state = run(&body, state, 0.0, 500);
            let t = f64::from(k) * 0.05;
            let expected_deg = (t / tau).cosh();
            // The small-angle formula is off by < 1 % up to ~10°.
            assert_rel_close(state.pitch_deg(), expected_deg, 0.01);
        }
    }

    #[test]
    fn one_degree_to_ten_takes_three_eighths_of_a_second() {
        let body = Body::prototype();
        let mut state = State::tilted_deg(1.0);
        let mut steps = 0_u32;
        while state.pitch_deg() < 10.0 {
            state = state.step(&body, 0.0, DT);
            steps += 1;
        }
        // Analytic (small angles): τ·acosh(10) ≈ 0.125 s · 2.99 ≈ 0.375 s.
        let t = f64::from(steps) * DT;
        assert_rel_close(t, body.fall_time_constant_s() * 10.0_f64.acosh(), 0.01);
    }

    #[test]
    fn tilted_backwards_falls_backwards() {
        let state = run(&Body::prototype(), State::tilted_deg(-1.0), 0.0, 2_000);
        assert!(state.pitch_deg() < -2.0, "{state:?}");
    }

    #[test]
    fn holding_accel_holds_the_tilt() {
        // The robot stays tilted, but has to keep speeding up to do so.
        let body = Body::prototype();
        let start = State::tilted_deg(5.0);
        let a = body.holding_accel(start.pitch_rad);
        let state = run(&body, start, a, 10_000); // 1 s
        assert_rel_close(state.pitch_deg(), 5.0, 1e-9);
        assert_rel_close(state.axle_vel_m_s, a, 1e-9);
        assert_rel_close(state.axle_pos_m, a / 2.0, 1e-3); // x = a·t²/2
    }

    #[test]
    fn the_reaction_torque_makes_holding_easier() {
        // Driven wheels push the body back themselves, so less acceleration
        // holds a tilt than for a cart-pole (g·tan θ): about 58 % here.
        let pitch = 5.0_f64.to_radians();
        let ratio = Body::prototype().holding_accel(pitch) / (G * pitch.tan());
        assert!((ratio - 0.576).abs() < 0.01, "{ratio}");
    }

    #[test]
    fn accelerating_harder_pushes_it_back_up() {
        let body = Body::prototype();
        let start = State::tilted_deg(5.0);
        let state = run(
            &body,
            start,
            2.0 * body.holding_accel(start.pitch_rad),
            1_000,
        );
        assert!(state.pitch_deg() < 5.0, "{state:?}");
    }
}
