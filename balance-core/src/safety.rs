//! Safety cut-offs: when the motors must stay off, whatever the controller says.
//!
//! Two independent reasons to switch the motors off:
//!
//! - **Not upright.** Past about 45° the robot can't recover, and a
//!   controller that keeps driving just spins the wheels on its side (or runs
//!   into the furniture). The cut-off uses *hysteresis*: it trips when
//!   `|pitch| > fall_deg`, but re-arms only once `|pitch| < rearm_deg`, a much
//!   smaller angle. With a single threshold, a pitch hovering around 45° would
//!   switch the motors on and off many times a second. Re-arming near upright
//!   also means a person has to stand the robot up on purpose before it drives
//!   again. The state starts *not upright*: the motors stay off at power-up
//!   until the robot has been held upright once.
//! - **Battery low.** Li-ion cells are damaged by deep discharge, and below
//!   about 3.2 V per cell (6.4 V for the 2S pack) there's little energy left
//!   anyway. Below `warn_v` the state becomes [`Battery::Low`] (just a warning,
//!   the motors keep running); below `off_v` it becomes [`Battery::Empty`] and
//!   the motors stay off. The voltage sags while the motors draw current and
//!   recovers when they stop, so a reading must stay below the threshold for
//!   `battery_hold_s` before it counts (a short sag when the motors start
//!   shouldn't stop the robot). Both battery states are *latched* until reboot:
//!   once the pack is low it doesn't become healthy again, and a voltage that
//!   recovers only because the motors stopped would otherwise switch them back
//!   on, sag, switch off, and so on.
//!
//! The state machine is pure: [`Safety::update`] takes the filter pitch and the
//! battery voltage and returns the new [`Status`] only when it changed, so the
//! caller can log each change once.

/// Thresholds for the cut-offs.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Limits {
    /// Motors off when `|pitch|` is above this, in degrees.
    pub fall_deg: f32,
    /// After a fall, motors allowed again only once `|pitch|` is below this.
    pub rearm_deg: f32,
    /// Warn when the battery stays below this voltage, in volts.
    pub warn_v: f32,
    /// Motors off for good when the battery stays below this voltage.
    pub off_v: f32,
    /// How long the battery has to stay below a threshold before it counts, in
    /// seconds.
    pub battery_hold_s: f32,
}

impl Limits {
    /// The prototype: 2S Li-ion pack (6.4-8.4 V).
    pub const PROTOTYPE: Self = Self {
        fall_deg: 45.0,
        rearm_deg: 5.0,
        warn_v: 6.6,
        off_v: 6.4,
        battery_hold_s: 1.0,
    };
}

/// Battery state. Ordered from good to bad; it only ever gets worse.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum Battery {
    /// Above the warning threshold.
    Ok,
    /// Below `warn_v`: motors still allowed, but charge soon.
    Low,
    /// Below `off_v`: motors off until reboot.
    Empty,
}

/// What the cut-offs currently say.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Status {
    /// The robot is (close enough to) upright: not fallen since it was last
    /// held upright.
    pub upright: bool,
    /// The battery state.
    pub battery: Battery,
}

impl Status {
    /// Whether the motors may run.
    #[must_use]
    pub fn motors_allowed(&self) -> bool {
        self.upright && self.battery != Battery::Empty
    }
}

/// The cut-off state machine.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Safety {
    limits: Limits,
    status: Status,
    /// How long the battery has been below `warn_v` / `off_v` without a break.
    below_warn_s: f32,
    below_off_s: f32,
}

impl Safety {
    /// Starts not upright (motors off until the robot is held upright) with a
    /// healthy battery.
    #[must_use]
    pub fn new(limits: Limits) -> Self {
        Self {
            limits,
            status: Status {
                upright: false,
                battery: Battery::Ok,
            },
            below_warn_s: 0.0,
            below_off_s: 0.0,
        }
    }

    /// The current status.
    #[must_use]
    pub fn status(&self) -> Status {
        self.status
    }

    /// Updates the cut-offs with the latest pitch (degrees, 0 = balanced) and
    /// battery voltage, `dt_s` seconds after the previous update. Returns the
    /// new status if it changed, `None` otherwise.
    ///
    /// A NaN pitch counts as fallen: if the estimate is broken, the motors
    /// should stop.
    pub fn update(&mut self, pitch_deg: f32, battery_v: f32, dt_s: f32) -> Option<Status> {
        let before = self.status;
        let tilt = pitch_deg.abs();

        // NaN compares false with everything, so check it explicitly: it trips too.
        if tilt.is_nan() || tilt > self.limits.fall_deg {
            self.status.upright = false;
        } else if tilt < self.limits.rearm_deg {
            self.status.upright = true;
        }

        self.below_warn_s =
            Self::time_below(self.below_warn_s, battery_v, self.limits.warn_v, dt_s);
        self.below_off_s = Self::time_below(self.below_off_s, battery_v, self.limits.off_v, dt_s);
        let battery = if self.below_off_s >= self.limits.battery_hold_s {
            Battery::Empty
        } else if self.below_warn_s >= self.limits.battery_hold_s {
            Battery::Low
        } else {
            Battery::Ok
        };
        // Latched: only ever gets worse.
        self.status.battery = self.status.battery.max(battery);

        (self.status != before).then_some(self.status)
    }

    /// Time spent below `threshold` without a break: grows by `dt_s` while
    /// below, restarts at 0 as soon as a reading is at or above it.
    fn time_below(time_s: f32, value: f32, threshold: f32, dt_s: f32) -> f32 {
        if value < threshold {
            time_s + dt_s
        } else {
            0.0
        }
    }
}

#[cfg(test)]
mod tests {
    use super::{Battery, Limits, Safety, Status};

    const DT: f32 = 0.002;
    const FULL: f32 = 8.0;

    /// A safety state that has been held upright once, so the motors are on.
    fn armed() -> Safety {
        let mut s = Safety::new(Limits::PROTOTYPE);
        let _ = s.update(0.0, FULL, DT);
        s
    }

    /// Feeds the same values for `seconds`; returns the last change, if any.
    fn hold(s: &mut Safety, pitch: f32, battery: f32, seconds: f32) -> Option<Status> {
        let mut last = None;
        let mut t = 0.0;
        while t < seconds {
            if let Some(status) = s.update(pitch, battery, DT) {
                last = Some(status);
            }
            t += DT;
        }
        last
    }

    #[test]
    fn starts_off_until_held_upright() {
        let mut s = Safety::new(Limits::PROTOTYPE);
        assert!(!s.status().motors_allowed());
        // Tilted 20°: between the thresholds, still not armed.
        assert_eq!(s.update(20.0, FULL, DT), None);
        assert!(!s.status().motors_allowed());
        let status = s.update(2.0, FULL, DT);
        assert!(status.is_some_and(|st| st.motors_allowed()));
    }

    #[test]
    fn trips_past_the_fall_angle_either_way() {
        let mut s = armed();
        assert_eq!(s.update(44.0, FULL, DT), None);
        let status = s.update(46.0, FULL, DT);
        assert!(status.is_some_and(|st| !st.upright && !st.motors_allowed()));

        let mut s = armed();
        assert!(s.update(-46.0, FULL, DT).is_some());
        assert!(!s.status().motors_allowed());
    }

    #[test]
    fn rearms_only_near_upright() {
        let mut s = armed();
        let _ = s.update(60.0, FULL, DT);
        // Back below 45° isn't enough: hysteresis.
        assert_eq!(s.update(30.0, FULL, DT), None);
        assert_eq!(s.update(6.0, FULL, DT), None);
        assert!(!s.status().motors_allowed());
        assert!(s.update(-4.0, FULL, DT).is_some());
        assert!(s.status().motors_allowed());
    }

    #[test]
    fn nan_pitch_counts_as_fallen() {
        let mut s = armed();
        assert!(s.update(f32::NAN, FULL, DT).is_some());
        assert!(!s.status().motors_allowed());
        // And NaN doesn't re-arm either.
        assert_eq!(s.update(f32::NAN, FULL, DT), None);
    }

    #[test]
    fn short_battery_sag_is_ignored() {
        let mut s = armed();
        assert_eq!(hold(&mut s, 0.0, 6.0, 0.5), None);
        // Recovers: the timers restart.
        assert_eq!(hold(&mut s, 0.0, FULL, 0.1), None);
        assert_eq!(hold(&mut s, 0.0, 6.0, 0.5), None);
        assert_eq!(s.status().battery, Battery::Ok);
    }

    #[test]
    fn warns_when_low_and_keeps_the_motors_on() {
        let mut s = armed();
        let status = hold(&mut s, 0.0, 6.5, 1.1);
        assert_eq!(status.map(|st| st.battery), Some(Battery::Low));
        assert!(s.status().motors_allowed());
    }

    #[test]
    fn cuts_off_when_empty_and_stays_off() {
        let mut s = armed();
        let status = hold(&mut s, 0.0, 6.3, 1.1);
        assert_eq!(status.map(|st| st.battery), Some(Battery::Empty));
        assert!(!s.status().motors_allowed());
        // Latched: a recovering voltage doesn't switch the motors back on.
        assert_eq!(hold(&mut s, 0.0, FULL, 2.0), None);
        assert!(!s.status().motors_allowed());
    }

    #[test]
    fn low_then_empty() {
        let mut s = armed();
        let _ = hold(&mut s, 0.0, 6.5, 1.1);
        assert_eq!(s.status().battery, Battery::Low);
        let status = hold(&mut s, 0.0, 6.3, 1.1);
        assert_eq!(status.map(|st| st.battery), Some(Battery::Empty));
    }
}
