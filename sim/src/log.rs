//! The firmware's IMU log, produced from simulated samples.
//!
//! [`Pipeline`] does what the firmware does with each sample: gyro bias
//! calibration at startup, then [`Controller::step`] from `balance-core`,
//! the very code the firmware runs. [`Pipeline::row`] formats the result
//! exactly like a firmware CSV line, so the same tools can read a real and a
//! simulated log.

use balance_core::{
    control::{Config, Controller},
    gyro::{BiasCalibrator, CalibrationError, GyroBias},
    imu::RawSample,
};

/// Loop period in s (500 Hz), the firmware's `DT_S`.
pub const DT_S: f32 = 0.002;
/// Loop period in µs.
pub const DT_US: u64 = 2000;
/// The controller settings the firmware uses.
const CONFIG: Config = Config::PROTOTYPE;
/// Still samples needed for the gyro calibration.
pub const CALIBRATION_SAMPLES: u32 = CONFIG.calibration_samples;

/// The `#` lines and the column header the firmware prints before the CSV.
#[must_use]
pub fn header(bias: GyroBias) -> String {
    let GyroBias([bx, by, bz]) = bias;
    format!(
        "# gyro bias (deg/s): x={bx:.3} y={by:.3} z={bz:.3}\n\
         # ICM-42670-P raw samples; accel G4 = 8192 LSB/g, gyro Deg500 = 65.5 LSB/(deg/s)\n\
         {:>10},{:>6},{:>6},{:>6},{:>6},{:>6},{:>6},{:>9},{:>7},{:>7},{:>7},{:>14},{:>16},{:>9},{:>6},{:>6}",
        "t_us",
        "ax",
        "ay",
        "az",
        "gx",
        "gy",
        "gz",
        "pitch_deg",
        "gx_dps",
        "gy_dps",
        "gz_dps",
        "gyro_pitch_deg",
        "filter_pitch_deg",
        "motor_cmd",
        "duty_l",
        "duty_r"
    )
}

/// The firmware's startup gyro calibration: the bias of samples taken while
/// the robot is still.
///
/// # Errors
///
/// The calibrator's error when there are fewer than
/// [`CALIBRATION_SAMPLES`] or the robot moved.
pub fn calibrate_bias(
    samples: impl IntoIterator<Item = RawSample>,
) -> Result<GyroBias, CalibrationError> {
    let mut calibrator = BiasCalibrator::new();
    for sample in samples {
        calibrator.add(CONFIG.gyro_dps(sample.gyro));
    }
    calibrator.finish(
        CONFIG.calibration_samples,
        CONFIG.calibration_max_spread_dps,
    )
}

/// The firmware's per-sample processing after calibration.
#[derive(Debug, Clone, Copy)]
pub struct Pipeline {
    bias: GyroBias,
    controller: Controller,
}

impl Pipeline {
    /// Calibrates the gyro bias from samples taken while the robot is still,
    /// like the firmware at startup.
    ///
    /// # Errors
    ///
    /// The calibrator's error when there are fewer than
    /// [`CALIBRATION_SAMPLES`] or the robot moved.
    pub fn calibrate(
        samples: impl IntoIterator<Item = RawSample>,
    ) -> Result<Self, CalibrationError> {
        let bias = calibrate_bias(samples)?;
        Ok(Self {
            bias,
            controller: Controller::new(CONFIG, bias),
        })
    }

    /// The gyro bias found by [`calibrate`](Self::calibrate), in °/s.
    #[must_use]
    pub const fn bias(&self) -> GyroBias {
        self.bias
    }

    /// Processes one sample taken at `t_us` and returns its CSV line (without
    /// a newline), formatted like the firmware's. `duty_pct` is the duty
    /// applied to the `[left, right]` motor in signed percent (the firmware
    /// logs what it actually sent, which can differ from the command).
    pub fn row(&mut self, t_us: u64, raw: &RawSample, duty_pct: [i8; 2]) -> String {
        let out = self.controller.step(raw, DT_S);
        let [ax, ay, az] = raw.accel;
        let [gx, gy, gz] = raw.gyro;
        let [rx, ry, rz] = out.rates_dps;
        let [duty_l, duty_r] = duty_pct;
        format!(
            "{t_us:>10},{ax:>6},{ay:>6},{az:>6},{gx:>6},{gy:>6},{gz:>6},{:>9.1},{rx:>7.2},{ry:>7.2},{rz:>7.2},{:>14.1},{:>16.1},{:>9.3},{duty_l:>6},{duty_r:>6}",
            out.accel_pitch_deg, out.gyro_pitch_deg, out.filter_pitch_deg, out.motor_cmd
        )
    }
}

#[cfg(test)]
mod tests {
    use balance_core::{
        gyro::{CalibrationError, GyroBias},
        imu::RawSample,
    };

    use super::{CALIBRATION_SAMPLES, Pipeline, header};

    /// The chip exactly level (1 g on X = up), which is 3.8° forward of the
    /// balance point.
    const UPRIGHT: RawSample = RawSample {
        accel: [8192, 0, 0],
        gyro: [0, 131, 0], // 2 °/s bias on Y
    };

    fn calibrated() -> Pipeline {
        Pipeline::calibrate((0..CALIBRATION_SAMPLES).map(|_| UPRIGHT)).unwrap()
    }

    fn fields(line: &str) -> Vec<&str> {
        line.split(',').map(str::trim).collect()
    }

    #[test]
    fn header_matches_the_firmware() {
        let text = header(GyroBias([0.1, -0.25, 0.0]));
        let lines: Vec<_> = text.lines().collect();
        assert_eq!(lines[0], "# gyro bias (deg/s): x=0.100 y=-0.250 z=0.000");
        assert!(lines[1].starts_with("# ICM-42670-P raw samples"));
        assert_eq!(
            fields(lines[2]),
            [
                "t_us",
                "ax",
                "ay",
                "az",
                "gx",
                "gy",
                "gz",
                "pitch_deg",
                "gx_dps",
                "gy_dps",
                "gz_dps",
                "gyro_pitch_deg",
                "filter_pitch_deg",
                "motor_cmd",
                "duty_l",
                "duty_r"
            ]
        );
        // Fixed-width columns: the header lines up with the rows.
        assert_eq!(lines[2].len(), calibrated().row(0, &UPRIGHT, [0, 0]).len());
    }

    #[test]
    fn calibration_removes_the_gyro_bias() {
        let mut pipeline = calibrated();
        let GyroBias([_, by, _]) = pipeline.bias();
        assert!((by - 2.0).abs() < 1e-3, "{by}");
        let row = pipeline.row(2000, &UPRIGHT, [-12, 34]);
        // Pitch 3.8: level is 3.8° forward of the balance point (−3.8°).
        // motor_cmd 0.456 = kp 0.12 × 3.8° (no I or D yet in the first step).
        assert_eq!(
            fields(&row),
            [
                "2000", "8192", "0", "0", "0", "131", "0", "3.8", "0.00", "0.00", "0.00", "3.8",
                "3.8", "0.456", "-12", "34"
            ]
        );
    }

    #[test]
    fn calibration_rejects_too_few_samples() {
        let result = Pipeline::calibrate([UPRIGHT; 10]);
        assert!(matches!(result, Err(CalibrationError::TooFewSamples)));
    }

    #[test]
    fn calibration_rejects_movement() {
        let moving = RawSample {
            gyro: [0, 655, 0], // 10 °/s
            ..UPRIGHT
        };
        let samples = (0..CALIBRATION_SAMPLES).map(|i| if i == 500 { moving } else { UPRIGHT });
        assert!(matches!(
            Pipeline::calibrate(samples),
            Err(CalibrationError::Moved { axis: 1, .. })
        ));
    }

    #[test]
    fn a_gyro_rate_integrates_into_gyro_pitch() {
        // Tipping forward at 10 °/s (−655 LSB on Y, on top of the 2 °/s bias)
        // for 100 samples of 2 ms = 2°, starting from 3.8°.
        let mut pipeline = calibrated();
        let turning = RawSample {
            gyro: [0, 131 - 655, 0],
            ..UPRIGHT
        };
        let mut row = String::new();
        for i in 0..100 {
            row = pipeline.row(i * 2000, &turning, [0, 0]);
        }
        assert_eq!(fields(&row)[11], "5.8");
    }
}
