//! The firmware's IMU log, produced from simulated samples.
//!
//! [`Pipeline`] does what the firmware's control loop does with each sample,
//! using the same `balance-core` functions: gyro bias calibration at startup,
//! accel pitch, bias-corrected rates, gyro-only pitch and the complementary
//! filter. [`Pipeline::row`] formats the result exactly like a firmware CSV
//! line, so the same tools can read a real and a simulated log.
//!
//! The constants and the axis mapping are copies of the firmware's
//! (`firmware/src/main.rs`); keep them in sync until the control-step PR
//! moves them into `balance-core`.

use balance_core::{
    attitude::{ComplementaryFilter, accel_pitch_deg},
    gyro::{BiasCalibrator, CalibrationError, GyroBias},
    imu::RawSample,
};

/// Loop period in s (500 Hz), the firmware's `DT_S`.
pub const DT_S: f32 = 0.002;
/// Loop period in µs.
pub const DT_US: u64 = 2000;
/// The firmware's `FILTER_TAU_S`.
const FILTER_TAU_S: f32 = 0.5;
/// The firmware's `CALIBRATION_SAMPLES`.
pub const CALIBRATION_SAMPLES: u32 = 1000;
/// The firmware's `CALIBRATION_MAX_SPREAD_DPS`.
const CALIBRATION_MAX_SPREAD_DPS: f32 = 2.0;
/// Gyro LSB per °/s at ±500 °/s (`GyroRange::Deg500.scale_factor()`).
const GYRO_SCALE: f32 = 65.5;

/// The firmware's `gyro_dps`.
fn gyro_dps(g: [i16; 3]) -> [f32; 3] {
    g.map(|v| f32::from(v) / GYRO_SCALE)
}

/// The firmware's `robot_axes`: `(forward, up)` = IMU `(x, z)`.
fn robot_axes(a: [i16; 3]) -> (f32, f32) {
    (f32::from(a[0]), f32::from(a[2]))
}

/// The firmware's `robot_pitch_rate`: gyro Y.
const fn robot_pitch_rate(rates_dps: [f32; 3]) -> f32 {
    rates_dps[1]
}

/// The `#` lines and the column header the firmware prints before the CSV.
#[must_use]
pub fn header(bias: GyroBias) -> String {
    let GyroBias([bx, by, bz]) = bias;
    format!(
        "# gyro bias (deg/s): x={bx:.3} y={by:.3} z={bz:.3}\n\
         # ICM-42670-P raw samples; accel G4 = 8192 LSB/g, gyro Deg500 = 65.5 LSB/(deg/s)\n\
         {:>10},{:>6},{:>6},{:>6},{:>6},{:>6},{:>6},{:>9},{:>7},{:>7},{:>7},{:>14},{:>16}",
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
        "filter_pitch_deg"
    )
}

/// The firmware's per-sample processing after calibration.
#[derive(Debug, Clone, Copy)]
pub struct Pipeline {
    bias: GyroBias,
    gyro_pitch: Option<f32>,
    filter: ComplementaryFilter,
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
        let mut calibrator = BiasCalibrator::new();
        for sample in samples {
            calibrator.add(gyro_dps(sample.gyro));
        }
        let bias = calibrator.finish(CALIBRATION_SAMPLES, CALIBRATION_MAX_SPREAD_DPS)?;
        Ok(Self {
            bias,
            gyro_pitch: None,
            filter: ComplementaryFilter::new(FILTER_TAU_S),
        })
    }

    /// The gyro bias found by [`calibrate`](Self::calibrate), in °/s.
    #[must_use]
    pub const fn bias(&self) -> GyroBias {
        self.bias
    }

    /// Processes one sample taken at `t_us` and returns its CSV line (without
    /// a newline), formatted like the firmware's.
    pub fn row(&mut self, t_us: u64, raw: &RawSample) -> String {
        let (forward, up) = robot_axes(raw.accel);
        let pitch = accel_pitch_deg(forward, up);
        let rates = self.bias.apply(gyro_dps(raw.gyro));
        let gyro_pitch = self.gyro_pitch.get_or_insert(pitch);
        *gyro_pitch += robot_pitch_rate(rates) * DT_S;
        let filter_pitch = self.filter.update(pitch, robot_pitch_rate(rates), DT_S);

        let [ax, ay, az] = raw.accel;
        let [gx, gy, gz] = raw.gyro;
        let [rx, ry, rz] = rates;
        format!(
            "{t_us:>10},{ax:>6},{ay:>6},{az:>6},{gx:>6},{gy:>6},{gz:>6},{pitch:>9.1},{rx:>7.2},{ry:>7.2},{rz:>7.2},{gyro_pitch:>14.1},{filter_pitch:>16.1}",
            gyro_pitch = *gyro_pitch
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

    const UPRIGHT: RawSample = RawSample {
        accel: [0, 0, 8192],
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
                "filter_pitch_deg"
            ]
        );
        // Fixed-width columns: the header lines up with the rows.
        assert_eq!(lines[2].len(), calibrated().row(0, &UPRIGHT).len());
    }

    #[test]
    fn calibration_removes_the_gyro_bias() {
        let mut pipeline = calibrated();
        let GyroBias([_, by, _]) = pipeline.bias();
        assert!((by - 2.0).abs() < 1e-3, "{by}");
        let row = pipeline.row(2000, &UPRIGHT);
        // "-0.0": accel_pitch_deg is atan2(-forward, up), which gives negative
        // zero for forward = 0. The firmware prints it the same way.
        assert_eq!(
            fields(&row),
            [
                "2000", "0", "0", "8192", "0", "131", "0", "-0.0", "0.00", "0.00", "0.00", "0.0",
                "-0.0"
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
        // 10 °/s (on top of the 2 °/s bias) for 100 samples of 2 ms = 2°.
        let mut pipeline = calibrated();
        let turning = RawSample {
            gyro: [0, 131 + 655, 0],
            ..UPRIGHT
        };
        let mut row = String::new();
        for i in 0..100 {
            row = pipeline.row(i * 2000, &turning);
        }
        assert_eq!(fields(&row)[11], "2.0");
    }
}
