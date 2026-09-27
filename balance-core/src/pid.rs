//! PID controller: turns "how far off are we" into "how hard to push".
//!
//! For the robot, the measurement is the pitch angle, the setpoint is the
//! angle we want (≈ 0°, upright) and the output is the motor command.
//!
//! - **P** (proportional) pushes in proportion to the current error.
//! - **I** (integral) adds up the error over time, so a small constant
//!   error (a heavier side, a sloped floor) is eventually pushed away.
//! - **D** (derivative) reacts to how fast the measurement changes, which
//!   damps overshoot and oscillation.

/// A PID controller with output limits and anti-windup.
///
/// Two details matter on real hardware:
///
/// - **Derivative on measurement:** D uses the change of the *measurement*,
///   not of the error. When the setpoint jumps (e.g. a steering command), the
///   error jumps too, and differentiating it would give a huge one-sample
///   spike ("derivative kick"). The measurement itself moves smoothly.
/// - **Anti-windup:** when the output is at its limit (motors at full power),
///   the I term would keep growing without having any effect. Once the error
///   finally changes sign, that stored-up integral keeps the output pinned in
///   the wrong direction for a long time. So the I term only grows while
///   doing so doesn't push further into the limit.
///
/// The I term is stored in output units (`ki` already applied), so changing
/// `ki` while running doesn't make the output jump.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Pid {
    kp: f32,
    ki: f32,
    kd: f32,
    out_min: f32,
    out_max: f32,
    i_term: f32,
    prev_measurement: Option<f32>,
}

impl Pid {
    /// A controller with gains `kp`, `ki` (per second), `kd` (seconds) and
    /// output limited to `out_min..=out_max`.
    #[must_use]
    pub const fn new(kp: f32, ki: f32, kd: f32, out_min: f32, out_max: f32) -> Self {
        Self {
            kp,
            ki,
            kd,
            out_min,
            out_max,
            i_term: 0.0,
            prev_measurement: None,
        }
    }

    /// Runs one control step `dt_s` seconds after the previous one and
    /// returns the (limited) output.
    ///
    /// The first call has no previous measurement, so its D term is 0. With
    /// `dt_s <= 0` neither I nor D can be computed, so only P is applied.
    pub fn update(&mut self, setpoint: f32, measurement: f32, dt_s: f32) -> f32 {
        let error = setpoint - measurement;
        let p = self.kp * error;

        let (i_candidate, d) = match self.prev_measurement {
            Some(prev) if dt_s > 0.0 => (
                self.i_term + self.ki * error * dt_s,
                -self.kd * (measurement - prev) / dt_s,
            ),
            _ => (self.i_term, 0.0),
        };
        self.prev_measurement = Some(measurement);

        // Anti-windup: accept the new I term unless the output is already
        // saturated and the error would push it further past that limit.
        let unclamped = p + i_candidate + d;
        let winding_up =
            (unclamped > self.out_max && error > 0.0) || (unclamped < self.out_min && error < 0.0);
        if !winding_up {
            self.i_term = i_candidate;
        }

        (p + self.i_term + d).clamp(self.out_min, self.out_max)
    }

    /// Forgets the I term and the previous measurement, e.g. after the robot
    /// fell over and is picked up again.
    pub fn reset(&mut self) {
        self.i_term = 0.0;
        self.prev_measurement = None;
    }
}

#[cfg(test)]
#[allow(
    clippy::float_cmp,
    reason = "these outputs are exact products or limits, so exact comparison is intended"
)]
mod tests {
    use super::Pid;

    const DT: f32 = 0.01;

    /// Simulates `steps` steps of a simple plant: the measurement moves with
    /// the controller output plus a constant disturbance, `x += (u + d)·dt`.
    /// Returns the final measurement.
    fn simulate(pid: &mut Pid, setpoint: f32, start: f32, disturbance: f32, steps: u16) -> f32 {
        let mut x = start;
        for _ in 0..steps {
            let u = pid.update(setpoint, x, DT);
            x += (u + disturbance) * DT;
        }
        x
    }

    #[test]
    fn p_only_is_gain_times_error() {
        let mut pid = Pid::new(2.0, 0.0, 0.0, -100.0, 100.0);
        assert_eq!(pid.update(10.0, 4.0, DT), 12.0);
        assert_eq!(pid.update(0.0, 4.0, DT), -8.0);
    }

    #[test]
    fn output_is_limited() {
        let mut pid = Pid::new(10.0, 0.0, 0.0, -1.0, 1.0);
        assert_eq!(pid.update(5.0, 0.0, DT), 1.0);
        assert_eq!(pid.update(-5.0, 0.0, DT), -1.0);
    }

    #[test]
    fn integral_removes_steady_state_error() {
        // A constant disturbance pulls the measurement away. P alone settles
        // with an offset of disturbance / kp; adding I removes it.
        let mut p_only = Pid::new(2.0, 0.0, 0.0, -10.0, 10.0);
        let x = simulate(&mut p_only, 0.0, 0.0, 1.0, 2000);
        assert!(
            (x - 0.5).abs() < 0.01,
            "P only settles at d/kp = 0.5, got {x}"
        );

        let mut pi = Pid::new(2.0, 1.0, 0.0, -10.0, 10.0);
        let x = simulate(&mut pi, 0.0, 0.0, 1.0, 2000);
        assert!(x.abs() < 0.01, "PI settles at 0, got {x}");
    }

    #[test]
    fn derivative_ignores_setpoint_jumps() {
        let mut pid = Pid::new(0.0, 0.0, 1.0, -100.0, 100.0);
        assert_eq!(pid.update(0.0, 0.0, DT), 0.0, "no D on the first call");
        assert_eq!(pid.update(50.0, 0.0, DT), 0.0, "setpoint jump: no kick");
        // Measurement rises 0.1 in 0.01 s = 10/s, so D pushes back with -10.
        assert!((pid.update(50.0, 0.1, DT) + 10.0).abs() < 1e-3);
    }

    #[test]
    fn anti_windup_keeps_integral_bounded() {
        // Setpoint far out of reach for 10 s: the output sits at the limit.
        // Then the setpoint comes back below the measurement: the output must
        // leave the limit right away instead of unwinding a huge integral.
        let mut pid = Pid::new(1.0, 5.0, 0.0, -1.0, 1.0);
        for _ in 0..1000 {
            assert_eq!(pid.update(100.0, 0.0, DT), 1.0);
        }
        let out = pid.update(-2.0, 0.0, DT);
        assert!(out < 0.0, "should reverse immediately, got {out}");
    }

    #[test]
    fn zero_dt_applies_only_p() {
        let mut pid = Pid::new(1.0, 100.0, 100.0, -100.0, 100.0);
        pid.update(0.0, 0.0, DT);
        assert_eq!(pid.update(3.0, 1.0, 0.0), 2.0);
    }

    #[test]
    fn reset_clears_integral_and_derivative_history() {
        let mut pid = Pid::new(0.0, 1.0, 1.0, -100.0, 100.0);
        simulate(&mut pid, 1.0, 0.0, 0.0, 100);
        pid.reset();
        assert_eq!(pid.update(0.0, 5.0, DT), 0.0);
    }
}
