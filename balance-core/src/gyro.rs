//! Gyro bias (offset) calibration.
//!
//! A MEMS gyro never reads exactly zero, even when perfectly still: each axis
//! has a small constant offset, the *bias* (typically a fraction of a °/s,
//! and it changes with temperature and between power-ups). That looks
//! harmless, but the pitch estimate *integrates* the rate over time, and
//! integrating a constant gives a line: a 0.5 °/s bias becomes 30° of drift
//! after one minute.
//!
//! The fix: at startup, while the robot is held still, average many samples
//! per axis. The true rate is zero, so the average is the bias. Subtract it
//! from every later sample.
//!
//! Units are whatever the caller passes (°/s or raw LSB), as long as
//! calibration and correction use the same unit.

/// Why a calibration was rejected.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CalibrationError {
    /// Fewer samples than requested: the average would be too noisy.
    TooFewSamples,
    /// The readings on some axis spread more than allowed, so the robot was
    /// probably moved. The average would then include real rotation, not
    /// just the bias, so the result can't be trusted.
    Moved,
}

/// Collects gyro samples taken while the robot is still, to estimate the
/// bias.
///
/// Keeps only a running sum, min, max and count per axis, so it needs no
/// buffer (no allocation in `no_std`) however many samples are added.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BiasCalibrator {
    sum: [f32; 3],
    min: [f32; 3],
    max: [f32; 3],
    count: u32,
}

impl BiasCalibrator {
    /// Starts an empty calibration.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            sum: [0.0; 3],
            // Start min/max at the opposite extremes so the first sample
            // replaces both.
            min: [f32::INFINITY; 3],
            max: [f32::NEG_INFINITY; 3],
            count: 0,
        }
    }

    /// Adds one `[x, y, z]` rate sample.
    pub fn add(&mut self, sample: [f32; 3]) {
        for (axis, &value) in sample.iter().enumerate() {
            self.sum[axis] += value;
            self.min[axis] = self.min[axis].min(value);
            self.max[axis] = self.max[axis].max(value);
        }
        self.count += 1;
    }

    /// Ends the calibration and returns the bias (mean per axis).
    ///
    /// - `min_samples`: fewer samples than this gives
    ///   [`CalibrationError::TooFewSamples`] (at least one is always needed).
    /// - `max_spread`: if max − min on any axis exceeds this, the robot moved
    ///   and the result is [`CalibrationError::Moved`]. Pick it a bit above
    ///   the gyro's noise at rest.
    ///
    /// # Errors
    ///
    /// See [`CalibrationError`].
    pub fn finish(self, min_samples: u32, max_spread: f32) -> Result<GyroBias, CalibrationError> {
        if self.count == 0 || self.count < min_samples {
            return Err(CalibrationError::TooFewSamples);
        }
        for axis in 0..3 {
            if self.max[axis] - self.min[axis] > max_spread {
                return Err(CalibrationError::Moved);
            }
        }
        #[allow(
            clippy::cast_precision_loss,
            reason = "counts above 2^24 (hours of samples) are not realistic"
        )]
        let n = self.count as f32;
        Ok(GyroBias(self.sum.map(|s| s / n)))
    }
}

impl Default for BiasCalibrator {
    fn default() -> Self {
        Self::new()
    }
}

/// Measured gyro bias per axis `[x, y, z]`, from [`BiasCalibrator::finish`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct GyroBias(pub [f32; 3]);

impl GyroBias {
    /// Removes the bias from a raw `[x, y, z]` sample: `raw − bias`.
    #[must_use]
    pub fn apply(self, raw: [f32; 3]) -> [f32; 3] {
        let bias = self.0;
        [raw[0] - bias[0], raw[1] - bias[1], raw[2] - bias[2]]
    }
}

#[cfg(test)]
mod tests {
    use super::{BiasCalibrator, CalibrationError, GyroBias};

    fn assert_close(actual: [f32; 3], expected: [f32; 3]) {
        for axis in 0..3 {
            assert!(
                (actual[axis] - expected[axis]).abs() < 1e-4,
                "expected {expected:?}, got {actual:?}"
            );
        }
    }

    fn calibrate(samples: &[[f32; 3]]) -> BiasCalibrator {
        let mut cal = BiasCalibrator::default();
        for &s in samples {
            cal.add(s);
        }
        cal
    }

    /// Noisy readings of a still gyro: `bias` ± 0.1, alternating.
    fn still_samples(bias: [f32; 3]) -> [[f32; 3]; 100] {
        core::array::from_fn(|i| {
            let noise = if i % 2 == 0 { 0.1 } else { -0.1 };
            bias.map(|b| b + noise)
        })
    }

    #[test]
    fn constant_samples_give_exact_bias() {
        let cal = calibrate(&[[0.5, -1.25, 2.0]; 10]);
        assert_eq!(cal.finish(10, 0.0), Ok(GyroBias([0.5, -1.25, 2.0])));
    }

    #[test]
    fn noisy_but_still_gives_mean() {
        let cal = calibrate(&still_samples([0.3, -0.7, 1.1]));
        let bias = cal.finish(100, 0.5).unwrap();
        assert_close(bias.0, [0.3, -0.7, 1.1]);
    }

    #[test]
    fn too_few_samples() {
        let cal = calibrate(&[[0.0; 3]; 9]);
        assert_eq!(cal.finish(10, 1.0), Err(CalibrationError::TooFewSamples));
    }

    #[test]
    fn no_samples_is_too_few_even_if_none_required() {
        assert_eq!(
            BiasCalibrator::new().finish(0, 1.0),
            Err(CalibrationError::TooFewSamples)
        );
    }

    #[test]
    fn spike_on_one_axis_means_moved() {
        let mut samples = still_samples([0.3, -0.7, 1.1]);
        samples[50][1] += 20.0; // someone bumped the robot
        let cal = calibrate(&samples);
        assert_eq!(cal.finish(100, 0.5), Err(CalibrationError::Moved));
    }

    #[test]
    fn apply_subtracts_per_axis() {
        let bias = GyroBias([1.0, -2.0, 0.5]);
        assert_close(bias.apply([3.0, 3.0, 3.0]), [2.0, 5.0, 2.5]);
    }

    #[test]
    fn corrected_calibration_samples_average_to_zero() {
        let samples = still_samples([0.3, -0.7, 1.1]);
        let bias = calibrate(&samples).finish(100, 0.5).unwrap();
        let mut sum = [0.0_f32; 3];
        for &s in &samples {
            let corrected = bias.apply(s);
            for axis in 0..3 {
                sum[axis] += corrected[axis];
            }
        }
        assert_close(sum.map(|s| s / 100.0), [0.0; 3]);
    }
}
