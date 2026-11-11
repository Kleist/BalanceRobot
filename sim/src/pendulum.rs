//! Physics of the robot body: an inverted pendulum on a moving axle.
//!
//! # The model
//!
//! The body (everything except the wheels) pivots on the axle. The input is
//! the **acceleration of the axle** `a` (forward positive), which is what the
//! wheels do. The motors are left out for now: this assumes they can produce
//! any acceleration we ask for, instantly. The motor-model step replaces that
//! with "duty cycle in, torque out" later.
//!
//! Seen from the axle, an accelerating axle looks like a force `-m·a` pulling
//! the centre of mass backwards (like being pushed into the seat when a car
//! speeds up). Together with gravity, the torque about the axle is
//!
//! ```text
//! I·θ'' = m·d·(g·sin θ − a·cos θ)
//! ```
//!
//! - `θ` pitch, 0 = upright, positive = tilted forward (same sign as the
//!   firmware's pitch)
//! - `m` mass, `d` height of the centre of mass above the axle
//! - `I` moment of inertia of the body about the axle
//!
//! Gravity tips the robot over further (`sin θ` grows with `θ`); driving the
//! wheels forward (`a > 0`) pushes it back. That is the whole balancing
//! problem: drive the wheels under the falling centre of mass.
//!
//! There is no floor: the model keeps rotating through 90° and beyond. Where
//! the real robot hits the ground depends on the chassis, and a controller
//! has lost long before that anyway.
//!
//! # How fast does it fall?
//!
//! With the axle held still (`a = 0`) and a small angle (`sin θ ≈ θ`):
//! `θ'' = θ / τ²` with `τ = √(I / (m·g·d))`. Its solution, starting at rest
//! at `θ₀`, is `θ(t) = θ₀·cosh(t/τ)`: the tilt multiplies by e ≈ 2.7 every
//! `τ` seconds.
//!
//! Hanging upside down, the sign flips (`θ'' = −θ/τ²`) and the body swings
//! with period `T = 2π·τ`. So timing a few swings measures `τ` directly; see
//! [`Body::from_swing_test`].
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

/// Mass properties of the body (everything that pivots on the axle).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Body {
    /// Total mass in kg.
    pub mass_kg: f64,
    /// Height of the centre of mass above the axle in m.
    pub com_height_m: f64,
    /// Moment of inertia about the axle in kg·m².
    pub inertia_kg_m2: f64,
}

impl Body {
    /// The prototype, measured 2026-11-11: 350 g, centre of mass 8 cm above
    /// the axle, 10 swings in 7 s hanging upside down.
    ///
    /// Gives `I` ≈ 0.0034 kg·m² and a fall time constant `τ` ≈ 0.11 s.
    #[must_use]
    pub fn prototype() -> Self {
        Self::from_swing_test(0.350, 0.08, 0.70)
    }

    /// Body from a swing test: hang the robot upside down from its axle,
    /// let it swing a little and time one full swing (there and back),
    /// `period_s`.
    ///
    /// A pendulum swings with `T = 2π·√(I / (m·g·d))`, so
    /// `I = m·g·d·T² / (4π²)`.
    #[must_use]
    pub fn from_swing_test(mass_kg: f64, com_height_m: f64, period_s: f64) -> Self {
        Self {
            mass_kg,
            com_height_m,
            inertia_kg_m2: mass_kg * G * com_height_m * period_s * period_s / (4.0 * PI * PI),
        }
    }

    /// Fall time constant `τ = √(I / (m·g·d))` in s: with the axle held still,
    /// a small tilt grows by a factor e every `τ` seconds.
    #[must_use]
    pub fn fall_time_constant_s(&self) -> f64 {
        (self.inertia_kg_m2 / (self.mass_kg * G * self.com_height_m)).sqrt()
    }

    /// Angular acceleration `θ''` in rad/s² at pitch `pitch_rad` with the axle
    /// accelerating at `axle_accel_m_s2` (the equation in the module docs).
    #[must_use]
    pub fn pitch_accel(&self, pitch_rad: f64, axle_accel_m_s2: f64) -> f64 {
        self.mass_kg * self.com_height_m * (G * pitch_rad.sin() - axle_accel_m_s2 * pitch_rad.cos())
            / self.inertia_kg_m2
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
            self.pitch_rate_rad_s + body.pitch_accel(self.pitch_rad, axle_accel_m_s2) * dt_s;
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
    fn prototype_numbers_from_the_swing_test() {
        let body = Body::prototype();
        // I = m·g·d·T²/(4π²) = 0.35 · 9.81 · 0.08 · 0.49 / 39.48
        assert_rel_close(body.inertia_kg_m2, 0.003_41, 0.01);
        // τ = T / 2π: the hanging swing and the upright fall share the same τ.
        assert_rel_close(
            body.fall_time_constant_s(),
            0.70 / std::f64::consts::TAU,
            1e-9,
        );
    }

    #[test]
    fn exactly_upright_stays_upright() {
        let state = run(&Body::prototype(), State::default(), 0.0, 10_000);
        assert_eq!(state, State::default());
    }

    #[test]
    fn small_tilt_falls_like_cosh() {
        // From 1°, compare with θ₀·cosh(t/τ) every 50 ms for 0.3 s (to ~10°).
        let body = Body::prototype();
        let tau = body.fall_time_constant_s();
        let mut state = State::tilted_deg(1.0);
        for k in 1..=6 {
            state = run(&body, state, 0.0, 500);
            let t = f64::from(k) * 0.05;
            let expected_deg = (t / tau).cosh();
            // sin θ < θ makes the real fall slightly slower; at 10° that is < 1 %.
            assert_rel_close(state.pitch_deg(), expected_deg, 0.01);
        }
    }

    #[test]
    fn one_degree_to_ten_takes_a_third_of_a_second() {
        let body = Body::prototype();
        let mut state = State::tilted_deg(1.0);
        let mut steps = 0_u32;
        while state.pitch_deg() < 10.0 {
            state = state.step(&body, 0.0, DT);
            steps += 1;
        }
        // Analytic (small angles): τ·acosh(10) ≈ 0.111 s · 2.99 ≈ 0.333 s.
        let t = f64::from(steps) * DT;
        assert_rel_close(t, body.fall_time_constant_s() * 10.0_f64.acosh(), 0.01);
    }

    #[test]
    fn tilted_backwards_falls_backwards() {
        let state = run(&Body::prototype(), State::tilted_deg(-1.0), 0.0, 2_000);
        assert!(state.pitch_deg() < -2.0, "{state:?}");
    }

    #[test]
    fn accelerating_with_g_tan_theta_holds_the_tilt() {
        // Forward acceleration a = g·tan θ exactly cancels gravity's torque:
        // the robot stays tilted, but has to keep speeding up to do so.
        let body = Body::prototype();
        let start = State::tilted_deg(5.0);
        let a = G * start.pitch_rad.tan();
        let state = run(&body, start, a, 10_000); // 1 s
        assert_rel_close(state.pitch_deg(), 5.0, 1e-9);
        assert_rel_close(state.axle_vel_m_s, a, 1e-9);
        assert_rel_close(state.axle_pos_m, a / 2.0, 1e-3); // x = a·t²/2
    }

    #[test]
    fn accelerating_harder_pushes_it_back_up() {
        let body = Body::prototype();
        let start = State::tilted_deg(5.0);
        let state = run(&body, start, 2.0 * G * start.pitch_rad.tan(), 1_000);
        assert!(state.pitch_deg() < 5.0, "{state:?}");
    }

    #[test]
    fn hanging_down_swings_with_the_measured_period() {
        // θ = 180° is hanging: it should swing back and forth with T = 0.70 s.
        // Start 1° off at rest. It swings back (rate < 0), stops at the other
        // end, swings forward (rate > 0) and is back at the start when the
        // rate turns negative again: one full period.
        let body = Body::prototype();
        let mut state = State::tilted_deg(181.0);
        let mut steps = 0_u32;
        let mut swung_forward = false;
        loop {
            state = state.step(&body, 0.0, DT);
            steps += 1;
            if state.pitch_rate_rad_s > 0.0 {
                swung_forward = true;
            } else if swung_forward {
                break;
            }
        }
        assert_rel_close(f64::from(steps) * DT, 0.70, 0.01);
    }
}
