# BalanceRobot

Two-wheel self-balancing robot. Firmware in Rust (no_std) on ESP32-C3, later controlled from an Android phone over BLE.

## How we work (learning project)

This is a **learning project**. The goal is to understand each piece, not just to get a working robot fast.

- Take **one small step at a time**. Each step should be small enough to understand fully and to verify on the hardware.
- **One step per PR.** Stop after each step and wait for the user before starting the next one. Do not jump ahead or bundle future steps "while we're at it".
- Explain the *why*: new concepts (Embassy tasks, I2C, PWM, filters, PID, ...), crate choices and non-obvious lines of code.
- Prefer minimal code over generated boilerplate; if a generator is used, walk through what it produced.
- Each step ends with a concrete way to check it on the robot (what to flash, what to look for in the log or on the hardware).
- Keep the bring-up plan below up to date: mark steps done, split steps that turn out too big.

## Prototype

This repo starts by reviving an **old prototype chassis** as a test bed. Code developed here should carry over to a later rebuild with 37 mm 6V 300RPM geared DC motors with hall encoders.

## Current prototype hardware

| Part | Details |
|---|---|
| MCU | **ESP32-C3-DevKit-RUST-1 v1.2a** (ESP32-C3-MINI-1), replacing the old RF-Nano (ATmega328P + nRF24) |
| IMU | Onboard **ICM-42670-P** @ I2C 0x68 (preferred). Also available: GY-521 / MPU-6050 module on the chassis, also 0x68 by default |
| Other onboard | SHTC3 temp/humidity @ 0x70, WS2812 RGB LED on GPIO2, LED on GPIO7 |
| Motor driver | L298N dual H-bridge module, onboard 5V regulator enabled (CON5 jumper fitted). ENA/ENB jumpers removed (enable pins wired out for PWM) |
| Motors | 2x yellow "TT" gear motors, 65 mm wheels, **no encoders** |
| Battery | 2x 18650 in series (2S, 6.4-8.4 V), feeds L298N motor supply |

## Board I2C bus (fixed by board design)

- SDA = GPIO10, SCL = GPIO8
- ICM-42670-P at 0x68, SHTC3 at 0x70
- ESP32-C3 has only one I2C controller. If the GY-521 is used, tie its AD0 to 3V3 to move it to 0x69.

## Pinout plan

Full wiring (every connection, with what is wired so far and a diagram): [`docs/wiring.md`](docs/wiring.md). Keep the two in sync.

| Function | GPIO |
|---|---|
| ENA (PWM, LEDC) | 0 |
| IN1 | 1 |
| IN2 | 3 |
| ENB (PWM, LEDC) | 6 |
| IN3 | 20 |
| IN4 | 21 |
| Battery voltage sense (ADC1) | 4 (divider 100k / 33k, ~2.1 V at 8.4 V) |
| Spare (GY-521 INT if used) | 5 |

Avoid: GPIO2 (strapping + RGB LED), GPIO7 (LED), GPIO9 (BOOT), GPIO18/19 (USB; used for flashing and USB-Serial-JTAG logging), GPIO8/10 (I2C).
GPIO20/21 are UART0; log via USB-Serial-JTAG, not UART0. Build `esp-println` with the `jtag-serial` feature only (its default `auto` can fall back to UART0 and toggle IN3/IN4).
The ROM bootloader always prints on UART0 TX (GPIO21 = IN4) at reset; harmless as long as ENB (GPIO6) stays low during boot.

## Power notes

- **Never** connect the 2S pack to the board's BAT+ (single Li-ion cell only).
- During development: power the board over USB, share **GND only** with the L298N.
- Untethered: L298N 5V output -> board 5V pin. TODO: check the esp-rs/esp-rust-board schematic for whether 5V and VBUS are diode-isolated before connecting both.
- Add 100-470 uF bulk capacitance near the board; motor noise/brownout resets are likely.
- L298N drops ~2 V, so motors see ~5-6 V (fine for TT motors, but torque sags as the battery drains).
- L298N logic inputs accept 3.3 V (V_IH min 2.3 V).

## Project layout & checks

- `balance-core/` - all logic that doesn't touch hardware (filters, PID, motor mapping, battery math). `#![no_std]`, no hardware deps, unit-tested on the host. Put as much code here as possible.
- `firmware/` - thin hardware glue for the ESP32-C3, a separate cargo workspace because it only builds for the RISC-V target (its `.cargo/config.toml` sets the target, linker script and `espflash` runner). `cd firmware && cargo run --release` builds, flashes and opens the log monitor; flashing needs the user in the `dialout` group.
- Lint policy lives in `[workspace.lints]` in the root `Cargo.toml` (clippy pedantic, no `unsafe`, no `unwrap`/`expect`/`panic` outside tests), mirrored in `firmware/Cargo.toml`; keep the two in sync. CI treats warnings as errors.
- Toolchain is pinned in `rust-toolchain.toml`.
- Coverage is **reported, not enforced**: CI posts a PR comment comparing base vs PR. Decide per PR whether a drop is acceptable.

Run the same checks as CI locally:

```sh
cargo fmt --all --check
cargo clippy --workspace --all-targets -- -D warnings
cargo test --workspace
cargo build -p balance-core --target riscv32imc-unknown-none-elf   # still no_std?
RUSTDOCFLAGS="-D warnings" cargo doc --workspace --no-deps
cargo llvm-cov --workspace --open    # coverage report (cargo install cargo-llvm-cov)

cd firmware
cargo fmt --check
cargo clippy --release -- -D warnings
cargo build --release
```

## Software stack

- `esp-hal` + Embassy (no_std), async, via `esp-rtos`. Two executors: thread mode for setup, logging and the LED; an `InterruptExecutor` (software interrupt 1, priority 2) for the 500 Hz control loop, so it preempts everything else. Data the control task takes must be `Send` (LEDC channels aren't, so the task creates them itself)
- LEDC for motor PWM on both channels (needs esp-hal's `unstable` feature; the C3 has no MCPWM). 1 kHz, 10-bit. Measured deadband in "Known limitations" below
- `icm42670` crate for the onboard IMU (embedded-hal), used only for setup (WHO_AM_I, power, ranges): v0.2 reads each register byte in its own I2C transaction, which is slow and tears values. After setup the firmware takes the bus back (`free()`) and burst-reads all 12 data bytes from 0x0B in one transaction; decoding is `balance_core::imu::RawSample`
- IMU log format (firmware -> serial): `#` comment/header lines, then CSV `t_us,ax,ay,az,gx,gy,gz,pitch_deg,gx_dps,gy_dps,gz_dps,gyro_pitch_deg,filter_pitch_deg`, each column right-aligned to a fixed width with spaces: raw LSB, then the accelerometer-only pitch in degrees (positive = tilted forward), then the bias-corrected gyro rates in °/s, then the pitch from integrating the gyro alone (starts at the accel pitch, drifts), then the complementary-filter pitch. Sampled at 500 Hz, logged at 10 lines/s; `# loop max us: busy N of 2000, late ..., imu ..., battery adc ..., log send ...; dropped N` once a second (longest time per part since the last such line; `late` = how long after its scheduled time an iteration started). The sample loop doesn't print: it queues raw numbers (`LogMsg`) for the `log_writer` task, which formats them and writes via the async USB-Serial-JTAG driver; `println!` only before the loop and in `halt`. Before the CSV header, the startup gyro calibration logs `# gyro bias (deg/s): ...` (it retries while the robot is moving); accel ±4 g = 8192 LSB/g, gyro ±500 °/s = 65.5 LSB/(°/s)
- The onboard IMU interrupt is probably not broken out: drive the control loop from an Embassy ticker at ~500 Hz and poll the IMU
- BLE later. **ESP32-C3 is BLE only (no Classic BT / SPP)**, so the Android app must use GATT

## Bring-up plan

Small steps, one PR each, in order. Tick steps off as they are completed; insert new steps anywhere.

- [x] Create this CLAUDE.md
- [x] CI + `balance-core` skeleton: fmt, clippy, unit tests, no_std check, docs, coverage report
- [x] Minimal firmware crate (no Embassy yet) + full firmware build in CI; understand each file
- [x] Install the toolchain and `espflash` locally, flash the skeleton
- [x] See a "hello" log over USB-Serial-JTAG
- [x] Blink the LED on GPIO7 (blocking delay)
- [x] Read raw accelerometer + gyro values from the IMU and log them as CSV at ~100 Hz (done before the Embassy blink and I2C scan)
- [x] Same blink with Embassy (async task + timer)
- [x] Accel pitch math in `balance-core` (`attitude::accel_pitch_deg`), unit tested
- [x] Pitch angle from the accelerometer only (see how noisy it is)
- [x] Gyro bias calibration math in `balance-core` (`gyro::BiasCalibrator`), unit tested
- [x] Calibrate the gyro bias at startup
- [x] Pitch angle by integrating the gyro only (see the drift): ~0.4 °/min at rest after calibration
- [x] Burst-read the IMU (one I2C transaction per sample) so it keeps up with the control loop
- [x] Complementary filter math in `balance-core` (`attitude::ComplementaryFilter`), unit tested
- [x] Complementary filter combining both, fixed-rate loop (~500 Hz ticker)
- [x] Measure what each 500 Hz iteration spends its time on (IMU read, battery ADC, logging): the loop overran (max ~3000 us of 2000 us). Max per part: IMU burst read ~490 us, battery ADC (16 readings) ~730 us, battery println ~360 us, CSV println ~1700-2100 us, stats println ~330 us. Printing costs ~11-14 us per byte, partly soft-float formatting (the C3 has no FPU), so IMU + one CSV line already overruns
- [x] Logging off the control path: the loop pushes raw numbers into a queue (`embassy_sync::Channel`); a log task formats and writes them with the async USB-Serial-JTAG driver. Measure how late the ticks start
- [x] Run the control loop on a higher-priority interrupt executor, so it preempts the log task's formatting (which has no `.await`). Tick lateness: up to ~350 us with the log task on the same executor, ~100-120 us on the interrupt executor (mostly constant wake-up overhead; jitter ~20 us). Busy ~1300 of 2000 us
- [ ] Spread the battery ADC readings over iterations (one per iteration, averaged), instead of 16 at once (~730 us)
- [x] One motor, one direction, full speed (wheels off the ground)
- [x] Both motors, both directions, full speed (no PWM yet)
- [x] PWM speed control via LEDC
- [x] Measure the PWM deadband of both motors
- [x] Battery voltage via ADC on GPIO4
- [x] PID controller math in `balance-core` (`pid::Pid`: output limits, anti-windup, derivative on measurement), unit tested
- [ ] Simulation: host-only `sim/` crate (std, root workspace) with an inverted-pendulum-on-wheels physics model, no controller yet; unit test that an upright robot falls at the rate the physics predicts
- [ ] Simulation: IMU sensor model (noise, bias, quantisation in the firmware's LSB units), output CSV in the firmware log format
- [ ] Control step in `balance-core` (e.g. `Controller::step(imu_sample, dt) -> motor_cmd`) so firmware and simulator run the same code
- [ ] Simulation: close the loop (complementary filter + PID at 500 Hz) and tune P, D, I in simulation
- [ ] Simulation: motor model (PWM deadband, saturation, battery sag, gear backlash) and a "stays upright" test in CI; refine parameters with numbers measured on the robot
- [ ] P-only tilt loop, then add D, then I; tuning
- [ ] BLE: advertise and show up on the phone
- [ ] BLE GATT control (steering / setpoint) from Android

## Known limitations of the prototype

- No encoders: can do tilt control only, no speed/position hold, so expect drift. Partial mitigation: feed back the integrated motor command.
- TT motor gear backlash will make tuning harder.
- Large PWM deadband with hysteresis (1 kHz PWM, 2S pack, wheels off the ground, 2 % steps; measured 2026-11-08). A standing wheel needs 30-36 % duty to start; a turning wheel keeps going down to 12-14 %:

  | Motor | Direction | Starts at | Stops at |
  |---|---|---|---|
  | Left | Forward | 36 % | 14 % |
  | Left | Backward | 32 % | 13 % |
  | Right | Forward | 30 % | 12 % |
  | Right | Backward | 34 % | 14 % |
