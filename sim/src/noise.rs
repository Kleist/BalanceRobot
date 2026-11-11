//! Reproducible Gaussian random numbers for sensor noise.
//!
//! Written out here instead of using the `rand` crates: it is two small,
//! well-known algorithms, and a fixed seed gives the same "noise" on every
//! run and every machine, so a test that passes once always passes.
//!
//! - **`SplitMix64`** turns a counter into well-mixed 64-bit numbers (good
//!   enough for simulation, not for cryptography).
//! - **Box-Muller** turns two uniform numbers into two independent normally
//!   distributed ones (mean 0, standard deviation 1).

use std::f64::consts::TAU;

/// Normally distributed numbers with mean 0 and standard deviation 1.
#[derive(Debug, Clone)]
pub struct Gaussian {
    state: u64,
    /// Box-Muller makes two numbers at a time; the second waits here.
    spare: Option<f64>,
}

impl Gaussian {
    /// A generator starting from `seed`.
    #[must_use]
    pub const fn new(seed: u64) -> Self {
        Self {
            state: seed,
            spare: None,
        }
    }

    /// The next number.
    pub fn sample(&mut self) -> f64 {
        if let Some(z) = self.spare.take() {
            return z;
        }
        // u1 in (0, 1], so ln(u1) is finite.
        let u1 = self.uniform();
        let u2 = self.uniform();
        let r = (-2.0 * u1.ln()).sqrt();
        let (sin, cos) = (TAU * u2).sin_cos();
        self.spare = Some(r * sin);
        r * cos
    }

    /// `SplitMix64`: step a counter by a large odd constant, then scramble it.
    const fn next_u64(&mut self) -> u64 {
        self.state = self.state.wrapping_add(0x9E37_79B9_7F4A_7C15);
        let mut z = self.state;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        z ^ (z >> 31)
    }

    /// Uniform in (0, 1]: the top 53 bits (an f64 holds exactly 53) plus 1,
    /// divided by 2⁵³.
    #[expect(
        clippy::cast_precision_loss,
        reason = "53-bit integers convert to f64 exactly"
    )]
    fn uniform(&mut self) -> f64 {
        ((self.next_u64() >> 11) + 1) as f64 / (1_u64 << 53) as f64
    }
}

#[cfg(test)]
mod tests {
    use super::Gaussian;

    #[test]
    fn mean_zero_standard_deviation_one() {
        let mut g = Gaussian::new(3);
        let n = 100_000;
        let values: Vec<f64> = (0..n).map(|_| g.sample()).collect();
        let mean = values.iter().sum::<f64>() / f64::from(n);
        let var = values.iter().map(|v| (v - mean).powi(2)).sum::<f64>() / f64::from(n);
        assert!(mean.abs() < 0.02, "{mean}");
        assert!((var.sqrt() - 1.0).abs() < 0.01, "{var}");
        // About 68 % within ±1σ, as for a normal distribution.
        let within: u32 = values.iter().map(|v| u32::from(v.abs() < 1.0)).sum();
        let share = f64::from(within) / f64::from(n);
        assert!((share - 0.6827).abs() < 0.01, "{share}");
    }

    #[test]
    fn different_seeds_differ() {
        assert!((Gaussian::new(1).sample() - Gaussian::new(2).sample()).abs() > 0.0);
    }
}
