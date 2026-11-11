//! Mean and standard deviation of the raw columns of an IMU log, real or
//! simulated, read from stdin.
//!
//! Lie the robot still, log for a minute, save the serial output to a file,
//! then: `cargo run -p sim --example log_stats < still.log`. The standard
//! deviations are the noise per sample, the gyro means the bias (in LSB):
//! the numbers that go into `ImuParams`.
//!
//! Lines starting with `#`, the column header and lines that don't parse
//! (e.g. other monitor output) are skipped.

use std::io::BufRead;

const COLUMNS: [&str; 6] = ["ax", "ay", "az", "gx", "gy", "gz"];

fn main() {
    let mut sum = [0.0_f64; 6];
    let mut sum_sq = [0.0_f64; 6];
    let mut n = 0_u32;
    for line in std::io::stdin().lock().lines().map_while(Result::ok) {
        let values: Option<Vec<f64>> = line
            .split(',')
            .skip(1) // t_us
            .take(6)
            .map(|v| v.trim().parse().ok())
            .collect();
        let Some(values) = values.filter(|v| v.len() == 6) else {
            continue;
        };
        for (i, v) in values.iter().enumerate() {
            sum[i] += v;
            sum_sq[i] += v * v;
        }
        n += 1;
    }
    if n < 2 {
        eprintln!("no data lines found");
        std::process::exit(1);
    }
    let n_f = f64::from(n);
    println!("{n} samples");
    println!("column     mean      std  (LSB)");
    for (i, name) in COLUMNS.iter().enumerate() {
        let mean = sum[i] / n_f;
        let std = ((sum_sq[i] / n_f - mean * mean) * n_f / (n_f - 1.0))
            .max(0.0)
            .sqrt();
        println!("{name:>6} {mean:>8.1} {std:>8.2}");
    }
}
