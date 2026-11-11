//! Simulation of the balance robot, run on the host (`std`).
//!
//! The goal is to try out the controller on a model before trying it on the
//! real robot, where a bad gain means a crash. This crate grows step by step:
//! physics first ([`pendulum`]), then the IMU ([`imu`]) and the firmware's log
//! ([`log`]), later a motor model and the closed loop running the same
//! `balance-core` code as the firmware.

pub mod imu;
pub mod log;
pub mod noise;
pub mod pendulum;
