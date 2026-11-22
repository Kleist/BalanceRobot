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
//! You drive only while holding **both buttons** (A+B), the [`Grip`]: the attitude at
//! the moment both are down becomes the reference, and the command comes from the tilt
//! relative to it. Let go of either button and the command is 0 whatever the tilt;
//! the next press captures a new reference, so you can start from any comfortable angle.
//!
//! [`TiltRemote`] low-pass filters the readings, runs the grip and maps each relative
//! angle onto −1..=1 with a dead zone and a full-scale angle ([`shape`]): more pitch
//! than the reference = forward
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
    /// Tilts smaller than this (from the grip's reference) give 0, so a hand that isn't
    /// perfectly still doesn't make the robot creep.
    pub dead_zone_deg: f32,
    /// Tilt (from the grip's reference) that gives full command, ±1.
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

/// Wraps an angle difference into −180..=180, so a reference near ±180° doesn't make a
/// small tilt look like a full turn.
fn wrap_deg(deg: f32) -> f32 {
    deg - 360.0 * libm::roundf(deg / 360.0)
}

/// What a [`Grip::step`] changed.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum GripChange {
    /// Same as before: still held, or still not held.
    None,
    /// Both buttons just went down; the attitude at that moment is the new reference.
    On {
        /// The captured reference.
        reference: Attitude,
    },
    /// At least one button was just let go.
    Off,
}

/// "Hold A+B to drive": tracks whether both buttons are held and the attitude at the
/// moment they were pressed (the reference).
///
/// The angles are plain differences from the reference. That is exact for a pure pitch
/// or a pure roll and close enough for the ±30° a hand tilts the remote.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Grip {
    /// `Some` while both buttons are held.
    reference: Option<Attitude>,
}

impl Grip {
    /// Not held.
    #[must_use]
    pub const fn new() -> Self {
        Self { reference: None }
    }

    /// Updates the grip with whether both (debounced) buttons are held now and the
    /// current attitude. On the press it captures `attitude` as the reference.
    pub fn step(&mut self, both_held: bool, attitude: Attitude) -> GripChange {
        match (self.reference, both_held) {
            (None, true) => {
                self.reference = Some(attitude);
                GripChange::On {
                    reference: attitude,
                }
            }
            (Some(_), false) => {
                self.reference = None;
                GripChange::Off
            }
            _ => GripChange::None,
        }
    }

    /// The reference while held, `None` otherwise.
    #[must_use]
    pub fn reference(&self) -> Option<Attitude> {
        self.reference
    }

    /// `attitude` relative to the reference while held (pitch wrapped to ±180°);
    /// 0° for both when not held.
    #[must_use]
    pub fn relative(&self, attitude: Attitude) -> Attitude {
        match self.reference {
            Some(reference) => Attitude {
                pitch_deg: wrap_deg(attitude.pitch_deg - reference.pitch_deg),
                roll_deg: attitude.roll_deg - reference.roll_deg,
            },
            None => Attitude {
                pitch_deg: 0.0,
                roll_deg: 0.0,
            },
        }
    }
}

/// What [`TiltRemote::step`] produced from one reading.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Update {
    /// The filtered attitude.
    pub attitude: Attitude,
    /// The tilt relative to the grip's reference; 0 when not held.
    pub relative: Attitude,
    /// The command from `relative`; 0 when not held.
    pub command: Command,
    /// Whether the grip just started or ended.
    pub grip: GripChange,
}

/// Filters accelerometer readings, runs the [`Grip`] and turns the relative tilt into a
/// [`Command`].
#[derive(Debug, Clone)]
pub struct TiltRemote {
    config: Config,
    /// Low-pass filtered reading; `None` until the first sample.
    filtered: Option<[f32; 3]>,
    grip: Grip,
}

impl TiltRemote {
    /// A remote with no reading yet, not gripped.
    #[must_use]
    pub fn new(config: Config) -> Self {
        Self {
            config,
            filtered: None,
            grip: Grip::new(),
        }
    }

    /// Feeds one accelerometer reading (sensor axes, any unit) taken `dt_s` after the
    /// previous one, and whether both buttons are held.
    ///
    /// The filter is first-order: each reading moves the estimate by
    /// `dt / (tau + dt)` of the way towards it, so after a step it covers ~63 % in
    /// `tau`. The first reading is taken as it is. It filters the reading, not the
    /// command, so the reference a press captures is already smoothed, and letting go
    /// drops the command to 0 at once (nothing to reset).
    pub fn step(&mut self, accel: [f32; 3], dt_s: f32, both_held: bool) -> Update {
        let filtered = match self.filtered {
            None => accel,
            Some(previous) => {
                let k = dt_s / (self.config.filter_tau_s + dt_s);
                core::array::from_fn(|i| previous[i] + k * (accel[i] - previous[i]))
            }
        };
        self.filtered = Some(filtered);
        let attitude = Attitude::from_accel(filtered);
        let grip = self.grip.step(both_held, attitude);
        let relative = self.grip.relative(attitude);
        let Config {
            dead_zone_deg,
            full_scale_deg,
            ..
        } = self.config;
        Update {
            attitude,
            relative,
            command: Command {
                speed: shape(relative.pitch_deg, dead_zone_deg, full_scale_deg),
                turn: shape(relative.roll_deg, dead_zone_deg, full_scale_deg),
            },
            grip,
        }
    }

    /// The grip (held or not, and its reference).
    #[must_use]
    pub fn grip(&self) -> &Grip {
        &self.grip
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
    use super::{
        Attitude, Command, Config, Grip, GripChange, TiltRemote, Update, dot, shape, wrap_deg,
    };

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

    fn attitude(pitch_deg: f32, roll_deg: f32) -> Attitude {
        Attitude {
            pitch_deg,
            roll_deg,
        }
    }

    #[test]
    fn grip_captures_the_reference_on_the_press() {
        let mut grip = Grip::new();
        assert_eq!(grip.step(false, attitude(45.0, 3.0)), GripChange::None);
        assert_eq!(grip.relative(attitude(70.0, 20.0)), attitude(0.0, 0.0));

        let change = grip.step(true, attitude(45.0, 3.0));
        assert_eq!(
            change,
            GripChange::On {
                reference: attitude(45.0, 3.0)
            }
        );
        assert_eq!(grip.relative(attitude(45.0, 3.0)), attitude(0.0, 0.0));

        // Held: the reference stays, the relative tilt follows.
        assert_eq!(grip.step(true, attitude(60.0, -7.0)), GripChange::None);
        assert_eq!(grip.relative(attitude(60.0, -7.0)), attitude(15.0, -10.0));
    }

    #[test]
    fn letting_go_reads_zero_and_the_next_press_rezeros() {
        let mut grip = Grip::new();
        grip.step(true, attitude(45.0, 0.0));
        assert_eq!(grip.step(false, attitude(80.0, 30.0)), GripChange::Off);
        assert_eq!(grip.relative(attitude(80.0, 30.0)), attitude(0.0, 0.0));
        assert_eq!(grip.reference(), None);

        grip.step(true, attitude(10.0, -5.0));
        assert_eq!(grip.reference(), Some(attitude(10.0, -5.0)));
        assert_eq!(grip.relative(attitude(20.0, -5.0)), attitude(10.0, 0.0));
    }

    #[test]
    fn grip_wraps_pitch_across_180() {
        let mut grip = Grip::new();
        grip.step(true, attitude(175.0, 0.0));
        assert_close(grip.relative(attitude(-175.0, 0.0)).pitch_deg, 10.0);
    }

    /// Feeds the same reading for 2 s (20 filter time constants) and returns the
    /// last update.
    fn settle(remote: &mut TiltRemote, pitch: f32, roll: f32, held: bool) -> Update {
        let mut update = remote.step(reading(pitch, roll), 0.02, held);
        for _ in 0..100 {
            update = remote.step(reading(pitch, roll), 0.02, held);
        }
        update
    }

    /// A remote that has settled at `pitch` / `roll` and then gripped there.
    fn gripped_at(pitch: f32, roll: f32) -> TiltRemote {
        let mut remote = TiltRemote::new(Config::MICROBIT);
        settle(&mut remote, pitch, roll, false);
        remote.step(reading(pitch, roll), 0.02, true);
        remote
    }

    #[test]
    fn not_gripped_gives_no_command() {
        let mut remote = TiltRemote::new(Config::MICROBIT);
        let update = settle(&mut remote, 80.0, 30.0, false);
        assert_eq!(update.command, Command::default());
        assert_eq!(update.grip, GripChange::None);
        assert_close(update.attitude.pitch_deg, 80.0);
    }

    #[test]
    fn the_reference_gives_no_command() {
        let mut remote = gripped_at(45.0, 0.0);
        assert_eq!(
            settle(&mut remote, 45.0, 0.0, true).command,
            Command::default()
        );
        assert_eq!(
            settle(&mut remote, 49.0, -4.0, true).command,
            Command::default()
        );
    }

    #[test]
    fn tipping_further_forward_drives_forward() {
        let mut remote = gripped_at(45.0, 0.0);
        let command = settle(&mut remote, 45.0 + 17.5, 0.0, true).command;
        assert_close(command.speed, 0.5);
        assert_close(command.turn, 0.0);
        assert_close(settle(&mut remote, 10.0, 0.0, true).command.speed, -1.0);
    }

    #[test]
    fn lowering_the_right_side_turns_right() {
        let mut remote = gripped_at(45.0, 0.0);
        let command = settle(&mut remote, 45.0, 17.5, true).command;
        assert_close(command.turn, 0.5);
        assert_close(command.speed, 0.0);
        assert_close(settle(&mut remote, 45.0, -40.0, true).command.turn, -1.0);
    }

    #[test]
    fn letting_go_drops_the_command_at_once() {
        let mut remote = gripped_at(45.0, 0.0);
        settle(&mut remote, 80.0, 0.0, true);
        let update = remote.step(reading(80.0, 0.0), 0.02, false);
        assert_eq!(update.grip, GripChange::Off);
        assert_eq!(update.command, Command::default());
        assert!(remote.grip().reference().is_none());
    }

    #[test]
    fn the_reference_is_the_filtered_attitude() {
        // Settled upright, then a jolt on the press: the reference only moves by the
        // filter's share of the jolt (k = 0.02 / 0.12).
        let mut remote = TiltRemote::new(Config::MICROBIT);
        settle(&mut remote, 0.0, 0.0, false);
        let update = remote.step(reading(60.0, 0.0), 0.02, true);
        let GripChange::On { reference } = update.grip else {
            panic!("expected the grip to start, got {:?}", update.grip);
        };
        assert!(reference.pitch_deg > 0.0 && reference.pitch_deg < 15.0);
    }

    #[test]
    fn filter_reaches_63_percent_after_tau() {
        let mut remote = TiltRemote::new(Config::MICROBIT);
        remote.step(reading(0.0, 0.0), 0.02, false);
        // Step from upright to full forward along Z: 5 samples of 20 ms = tau.
        let mut update = remote.step([0.0, -1000.0, -1000.0], 0.02, false);
        for _ in 0..4 {
            update = remote.step([0.0, -1000.0, -1000.0], 0.02, false);
        }
        // Filtered az ≈ −1000 · (1 − (1 − 0.02/0.12)^5) ≈ −599, ay stays −1000.
        let expected = libm::atan2f(599.0, 1000.0).to_degrees();
        assert!((update.attitude.pitch_deg - expected).abs() < 0.5);
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
