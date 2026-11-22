//! Tilt remote: turns the micro:bit's accelerometer reading into a speed and turn command.
//!
//! The micro:bit is held upright in front of you, LEDs facing you, USB edge up. Its
//! accelerometer axes (measured, see the hardware table in `CLAUDE.md`) then point:
//! +X to your left (button A), +Y down (edge connector), +Z away from you (the back).
//! At rest the accelerometer reads 1 g pointing *up*, so tilting the micro:bit turns
//! that vector in the sensor frame, and its direction gives two angles ([`Attitude`]):
//!
//! - **pitch**: tipping the top edge away from you. 0° upright, 90° lying flat LEDs up.
//! - **roll**: lowering the right (button B) side, like turning a steering wheel
//!   clockwise. 0° level.
//!
//! [`TiltRemote`] low-pass filters the readings, measures both angles against a zero
//! attitude (where you hold it when you start) and maps each offset onto −1..=1 with a
//! dead zone and a full-scale angle ([`shape`]): more pitch than the zero = forward
//! (`speed` > 0), right side lower = turn right (`turn` > 0, the robot's convention:
//! `left = cmd + turn`, `right = cmd − turn`).

/// The two tilt angles of the micro:bit, in degrees.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Attitude {
    /// Top edge tipped away from you; 0° upright, 90° flat with the LEDs up.
    pub pitch_deg: f32,
    /// Right (button B) side lower; 0° level, −90..=90.
    pub roll_deg: f32,
}

impl Attitude {
    /// Both angles from one accelerometer reading in the micro:bit's sensor axes.
    /// Only the direction matters, so any unit works (mg, g, raw counts).
    ///
    /// - pitch: in the plane of Y (down) and Z (away from you), the up vector reads
    ///   `ay = −cos θ`, `az = −sin θ` after tipping forward by θ, so `atan2(−az, −ay)`.
    ///   −180..=180.
    /// - roll: the angle of the X axis (pointing left) above horizontal is
    ///   `atan2(−ax, √(ay² + az²))`; lowering the right side raises X, which reads +ax,
    ///   hence `atan2(ax, …)`. It stays the same whatever the pitch.
    ///
    /// A zero vector (free fall) gives 0° for both.
    #[must_use]
    pub fn from_accel(accel: [f32; 3]) -> Self {
        let [ax, ay, az] = accel;
        // Without this, negating 0.0 gives −0.0 and atan2(−0.0, −0.0) is −180°.
        if ay == 0.0 && az == 0.0 {
            return Self {
                pitch_deg: 0.0,
                roll_deg: if ax == 0.0 { 0.0 } else { 90.0f32.copysign(ax) },
            };
        }
        Self {
            pitch_deg: libm::atan2f(-az, -ay).to_degrees(),
            roll_deg: libm::atan2f(ax, libm::hypotf(ay, az)).to_degrees(),
        }
    }
}

/// Settings for [`TiltRemote`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Config {
    /// Tilts smaller than this (from the zero attitude) give 0, so a hand that isn't
    /// perfectly still doesn't make the robot creep.
    pub dead_zone_deg: f32,
    /// Tilt (from the zero attitude) that gives full command, ±1.
    pub full_scale_deg: f32,
    /// Time constant of the low-pass filter on the accelerometer reading: smooths
    /// hand tremor and sensor noise, at the cost of this much lag.
    pub filter_tau_s: f32,
}

impl Config {
    /// Starting values: 5° dead zone, full scale at 30°, 0.1 s filter.
    pub const MICROBIT: Self = Self {
        dead_zone_deg: 5.0,
        full_scale_deg: 30.0,
        filter_tau_s: 0.1,
    };
}

/// What the remote asks the robot to do; both in −1..=1.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Command {
    /// Positive = forward.
    pub speed: f32,
    /// Positive = turn right.
    pub turn: f32,
}

/// Maps a tilt offset in degrees onto −1..=1: 0 inside the dead zone, then rising
/// linearly to ±1 at the full-scale angle, clamped beyond. The ramp starts at the dead
/// zone's edge, so the command has no jump there. `NaN` gives 0.
#[must_use]
pub fn shape(offset_deg: f32, dead_zone_deg: f32, full_scale_deg: f32) -> f32 {
    let span = full_scale_deg - dead_zone_deg;
    let past_dead_zone = offset_deg.abs() - dead_zone_deg;
    if past_dead_zone.is_nan() || past_dead_zone <= 0.0 || span <= 0.0 {
        return 0.0;
    }
    (past_dead_zone / span).min(1.0).copysign(offset_deg)
}

/// Wraps an angle difference into −180..=180, so a zero near ±180° doesn't make a small
/// tilt look like a full turn.
fn wrap_deg(deg: f32) -> f32 {
    deg - 360.0 * libm::roundf(deg / 360.0)
}

/// Filters accelerometer readings and turns them into a [`Command`] relative to a zero
/// attitude.
#[derive(Debug, Clone)]
pub struct TiltRemote {
    config: Config,
    /// Low-pass filtered reading; `None` until the first sample.
    filtered: Option<[f32; 3]>,
    /// Attitude that gives a zero command; upright and level until set.
    zero: Attitude,
}

impl TiltRemote {
    /// A remote with no reading yet and the zero at upright and level.
    #[must_use]
    pub fn new(config: Config) -> Self {
        Self {
            config,
            filtered: None,
            zero: Attitude {
                pitch_deg: 0.0,
                roll_deg: 0.0,
            },
        }
    }

    /// Feeds one accelerometer reading (sensor axes, any unit) taken `dt_s` after the
    /// previous one and returns the command for the filtered attitude.
    ///
    /// The filter is first-order: each reading moves the estimate by
    /// `dt / (tau + dt)` of the way towards it, so after a step it covers ~63 % in
    /// `tau`. The first reading is taken as it is.
    pub fn step(&mut self, accel: [f32; 3], dt_s: f32) -> Command {
        let filtered = match self.filtered {
            None => accel,
            Some(previous) => {
                let k = dt_s / (self.config.filter_tau_s + dt_s);
                core::array::from_fn(|i| previous[i] + k * (accel[i] - previous[i]))
            }
        };
        self.filtered = Some(filtered);
        self.command()
    }

    /// The filtered attitude, or `None` before the first reading.
    #[must_use]
    pub fn attitude(&self) -> Option<Attitude> {
        self.filtered.map(Attitude::from_accel)
    }

    /// Makes the current filtered attitude the zero point. Does nothing before the
    /// first reading.
    pub fn capture_zero(&mut self) {
        if let Some(attitude) = self.attitude() {
            self.zero = attitude;
        }
    }

    /// The zero attitude.
    #[must_use]
    pub fn zero(&self) -> Attitude {
        self.zero
    }

    /// The command for the filtered attitude; 0 before the first reading.
    #[must_use]
    pub fn command(&self) -> Command {
        let Some(attitude) = self.attitude() else {
            return Command::default();
        };
        let Config {
            dead_zone_deg,
            full_scale_deg,
            ..
        } = self.config;
        Command {
            speed: shape(
                wrap_deg(attitude.pitch_deg - self.zero.pitch_deg),
                dead_zone_deg,
                full_scale_deg,
            ),
            turn: shape(
                attitude.roll_deg - self.zero.roll_deg,
                dead_zone_deg,
                full_scale_deg,
            ),
        }
    }
}

/// Where to show a command as one dot on the 5x5 LED matrix: `(row, column)`, each
/// 0..=4. Row 0 is the top (USB edge), so full forward is the top row; column 0 is the
/// left (button A side), so full right turn is column 4. A zero command is the centre.
/// The dot moves one LED per 0.5 of command (rounded to the nearest LED).
#[must_use]
pub fn dot(command: Command) -> (usize, usize) {
    (led_index(-command.speed), led_index(command.turn))
}

/// Maps −1..=1 onto LED 0..=4 (−1 → 0, 0 → 2, 1 → 4); `NaN` gives the centre.
fn led_index(value: f32) -> usize {
    let value = if value.is_nan() {
        0.0
    } else {
        value.clamp(-1.0, 1.0)
    };
    #[allow(
        clippy::cast_possible_truncation,
        clippy::cast_sign_loss,
        reason = "value is clamped to -1..=1, so the rounded index is in 0..=4"
    )]
    let index = libm::roundf((value + 1.0) * 2.0) as usize;
    index
}

#[cfg(test)]
mod tests {
    use super::{Attitude, Command, Config, TiltRemote, dot, shape, wrap_deg};

    /// A 1000 mg reading for the micro:bit held at `pitch` (top tipped away) and
    /// `roll` (right side lower), both in degrees. Pitch first, then roll about the
    /// tipped forward axis.
    fn reading(pitch: f32, roll: f32) -> [f32; 3] {
        let (p, r) = (pitch.to_radians(), roll.to_radians());
        [
            1000.0 * libm::sinf(r),
            -1000.0 * libm::cosf(r) * libm::cosf(p),
            -1000.0 * libm::cosf(r) * libm::sinf(p),
        ]
    }

    fn assert_close(actual: f32, expected: f32) {
        assert!(
            (actual - expected).abs() < 1e-3,
            "expected {expected}, got {actual}"
        );
    }

    #[test]
    fn upright_is_level() {
        let a = Attitude::from_accel([0.0, -1000.0, 0.0]);
        assert_close(a.pitch_deg, 0.0);
        assert_close(a.roll_deg, 0.0);
    }

    #[test]
    fn measured_poses() {
        // From the E3 axes log: flat LEDs up, and lying on the A / B side.
        assert_close(Attitude::from_accel([0.0, 0.0, -1000.0]).pitch_deg, 90.0);
        assert_close(Attitude::from_accel([-1000.0, 0.0, 0.0]).roll_deg, -90.0);
        assert_close(Attitude::from_accel([1000.0, 0.0, 0.0]).roll_deg, 90.0);
    }

    #[test]
    fn pitch_and_roll_are_independent() {
        for pitch in [-30.0, 0.0, 45.0, 80.0] {
            for roll in [-40.0, 0.0, 20.0] {
                let a = Attitude::from_accel(reading(pitch, roll));
                assert_close(a.pitch_deg, pitch);
                assert_close(a.roll_deg, roll);
            }
        }
    }

    #[test]
    fn free_fall_is_zero() {
        let a = Attitude::from_accel([0.0, 0.0, 0.0]);
        assert_close(a.pitch_deg, 0.0);
        assert_close(a.roll_deg, 0.0);
    }

    #[test]
    fn shape_has_dead_zone_ramp_and_clamp() {
        assert_close(shape(0.0, 5.0, 30.0), 0.0);
        assert_close(shape(4.9, 5.0, 30.0), 0.0);
        assert_close(shape(-5.0, 5.0, 30.0), 0.0);
        assert_close(shape(17.5, 5.0, 30.0), 0.5);
        assert_close(shape(-17.5, 5.0, 30.0), -0.5);
        assert_close(shape(30.0, 5.0, 30.0), 1.0);
        assert_close(shape(90.0, 5.0, 30.0), 1.0);
        assert_close(shape(-90.0, 5.0, 30.0), -1.0);
    }

    #[test]
    fn shape_survives_bad_input() {
        assert_close(shape(f32::NAN, 5.0, 30.0), 0.0);
        assert_close(shape(20.0, 30.0, 5.0), 0.0); // full scale inside the dead zone
    }

    #[test]
    fn wrap_keeps_differences_small() {
        assert_close(wrap_deg(10.0), 10.0);
        assert_close(wrap_deg(350.0), -10.0);
        assert_close(wrap_deg(-350.0), 10.0);
    }

    /// A remote that has settled at `pitch` / `roll` and captured that as its zero.
    fn zeroed_at(pitch: f32, roll: f32) -> TiltRemote {
        let mut remote = TiltRemote::new(Config::MICROBIT);
        remote.step(reading(pitch, roll), 0.02);
        remote.capture_zero();
        remote
    }

    /// Feeds the same reading for 2 s (20 filter time constants) and returns the
    /// settled command.
    fn settle(remote: &mut TiltRemote, pitch: f32, roll: f32) -> Command {
        let mut command = Command::default();
        for _ in 0..100 {
            command = remote.step(reading(pitch, roll), 0.02);
        }
        command
    }

    #[test]
    fn zero_attitude_gives_no_command() {
        let mut remote = zeroed_at(45.0, 0.0);
        assert_eq!(settle(&mut remote, 45.0, 0.0), Command::default());
        assert_eq!(settle(&mut remote, 49.0, -4.0), Command::default());
    }

    #[test]
    fn tipping_further_forward_drives_forward() {
        let mut remote = zeroed_at(45.0, 0.0);
        let command = settle(&mut remote, 45.0 + 17.5, 0.0);
        assert_close(command.speed, 0.5);
        assert_close(command.turn, 0.0);
        assert_close(settle(&mut remote, 10.0, 0.0).speed, -1.0);
    }

    #[test]
    fn lowering_the_right_side_turns_right() {
        let mut remote = zeroed_at(45.0, 0.0);
        let command = settle(&mut remote, 45.0, 17.5);
        assert_close(command.turn, 0.5);
        assert_close(command.speed, 0.0);
        assert_close(settle(&mut remote, 45.0, -40.0).turn, -1.0);
    }

    #[test]
    fn filter_reaches_63_percent_after_tau() {
        let mut remote = zeroed_at(0.0, 0.0);
        // Step from upright to full forward along Z: 5 samples of 20 ms = tau.
        for _ in 0..5 {
            remote.step([0.0, -1000.0, -1000.0], 0.02);
        }
        let Some(attitude) = remote.attitude() else {
            panic!("no attitude after five readings");
        };
        // Filtered az ≈ −1000 · (1 − (1 − 0.02/0.12)^5) ≈ −599, ay stays −1000.
        let expected = libm::atan2f(599.0, 1000.0).to_degrees();
        assert!((attitude.pitch_deg - expected).abs() < 0.5);
    }

    #[test]
    fn no_reading_no_command() {
        let mut remote = TiltRemote::new(Config::MICROBIT);
        assert_eq!(remote.command(), Command::default());
        assert!(remote.attitude().is_none());
        remote.capture_zero(); // must not panic or change the zero
        assert_close(remote.zero().pitch_deg, 0.0);
    }

    #[test]
    fn dot_follows_the_command() {
        assert_eq!(dot(Command::default()), (2, 2));
        let forward = Command {
            speed: 1.0,
            turn: 0.0,
        };
        assert_eq!(dot(forward), (0, 2));
        let back_left = Command {
            speed: -1.0,
            turn: -1.0,
        };
        assert_eq!(dot(back_left), (4, 0));
        let half_right = Command {
            speed: 0.0,
            turn: 0.5,
        };
        assert_eq!(dot(half_right), (2, 3));
        let nan = Command {
            speed: f32::NAN,
            turn: 2.0,
        };
        assert_eq!(dot(nan), (2, 4));
    }
}
