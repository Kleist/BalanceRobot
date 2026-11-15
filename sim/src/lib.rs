//! Simulation of the balance robot, run on the host (`std`).
//!
//! The goal is to try out the controller on a model before trying it on the
//! real robot, where a bad gain means a crash. This crate grows step by step:
//! physics first ([`pendulum`]), then the IMU ([`imu`]) and the firmware's log
//! ([`log`]), a DC motor with the PWM deadband and battery voltage
//! ([`motor`]) and the closed loop
//! ([`closed_loop`]) running the same `balance-core` controller as the
//! firmware, optionally with its deadband compensation and battery scaling
//! (`balance_core::drive`).

pub mod closed_loop;
pub mod imu;
pub mod log;
pub mod motor;
pub mod noise;
pub mod pendulum;
