# BalanceRobot

Two-wheel self-balancing robot. Firmware in Rust (no_std) on ESP32-C3, later controlled from a hand-held **BBC micro:bit v2** tilt remote over BLE (also Rust, Embassy).

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
| Front | **The closed side of the body** is the front: forward tilt, forward motor command and positive pitch all point that way |
| IMU mounting | Board stands upright: **+X up, +Z backward (towards the open side), +Y left** (axle = Y). Forward = −az, so pitch = `atan2(az, ax)`, pitch rate = `+gy`. Balance point (held still by hand, re-measured 2026-11-22: ax ≈ 8135, az ≈ −600) at **−4.2°** accel pitch, subtracted as `balance_pitch_deg` (was +3.8°, 8° off, see the axes step) |
| Other onboard | SHTC3 temp/humidity @ 0x70, WS2812 RGB LED on GPIO2, LED on GPIO7 |
| Motor driver | L298N dual H-bridge module, onboard 5V regulator enabled (CON5 jumper fitted). ENA/ENB jumpers removed (enable pins wired out for PWM) |
| Motors | 2x yellow "TT" gear motors, 65 mm wheels, **no encoders** |
| Battery | 2x 18650 in series (2S, 6.4-8.4 V), feeds L298N motor supply |
| Remote | **BBC micro:bit v2** (nRF52833, BLE 5): LSM303AGR accelerometer/magnetometer on the internal I2C, 5x5 LED matrix, buttons A/B, onboard USB debug probe (CMSIS-DAP: flashing and defmt/RTT logging via `probe-rs`). Powered by USB or 2xAAA. Hold **A+B** to drive: the attitude at the moment both are pressed is the zero point, tilting away from it forward/back = speed, left/right = turn. Not both held = no input (zero). **Accel axes** (measured 2026-11-21, raw LSM303AGR registers via `lsm303agr`, +1000 mg on the axis pointing up): **+X towards button A, +Y towards the edge connector, +Z out of the back** (LEDs up: z ≈ −1000). Seen from the LED side that is a left-handed set (X × Y points out of the LED side, but +Z points out of the back); the sign checks don't depend on it |
| Body (measured 2026-11-11) | 350 g total, centre of mass 8 cm above the axle, swings with T = 0.70 s hanging upside down -> I ~ 0.0034 kg m^2 about the axle. On its wheels with the axle held still by the motors, fall time constant tau = sqrt((I + m d r)/(m g d)) ~ 0.125 s (1 deg -> 10 deg in ~0.375 s); T/2pi ~ 0.11 s is the fixed-pivot value. Wheels can't be weighed separately; wheel + reflected motor spin inertia estimated at 3e-4 kg m^2 (not measured) |

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

- `balance-core/` - all logic that doesn't touch hardware (filters, PID, motor mapping, battery math). Command → duty in `drive` (`Drive::PROTOTYPE`: deadband compensation `duty = sign·(stop + |cmd|·(1 − stop))`, stop 8 %, dead zone 0.005, then battery scaling `× 7.4 V / v_battery`, clamped). `#![no_std]`, no hardware deps, unit-tested on the host. Put as much code here as possible.
- `sim/` - host-only (`std`) simulation of the robot, in the root workspace, depends on `balance-core`. So far: body physics (`sim::pendulum`, inverted pendulum on driven wheels with the axle acceleration as input, incl. the motors' reaction torque on the body), IMU model (`sim::imu`: specific force at the chip incl. tangential/centripetal terms, mounting tilt, bias, per-axis Gaussian noise, quantisation to LSB, real axes +X up / +Z backward; `ImuParams::prototype()` measured: chip 12.7 cm above / 1.5 cm behind the axle, noise and gyro bias from a still log), the firmware's log pipeline + CSV format (`sim::log`, runs the same `balance_core::control::Controller` as the firmware), a first-order DC motor (`sim::motor`: command sets a wheel speed, v_max 0.41 m/s measured, lag 50 ms estimated; `Motor::prototype()` adds the 100 Hz PWM deadband with hysteresis (`Deadband::PROTOTYPE`: start 19 %, stop 8 %, averaged over both motors; a wheel slower than 5 mm/s counts as standing), `Motor::ideal()` is linear; `at_battery(v)` scales the duty by v / 7.4 V, the reference voltage at which speed and deadband hold) and the closed loop (`sim::closed_loop::Sim`: 0.1 ms physics, 500 Hz controller, 1 ms command latency; `with_drive(Drive)` puts `balance_core::drive` between controller and motor). Examples: `fall` (pitch table), `imu_log [fall]` (simulated firmware log), `log_stats < file.log` (mean/std of the raw columns of a real or simulated log), `balance [--ideal] [KP KI KD [START_DEG]]` (closed-loop trace, deadband motor unless `--ideal`), `tune` (grid search for kp/ki/kd/kv/kx on the deadband motor + `Drive::PROTOTYPE`, must survive motor lag 15-50 ms with up to 20 ms dead time and a ±1° balance-point error, ranked by the distance from the start; `tune -- --check KP KI KD KV [KX]` stress-tests one set on 82 harder runs and reports drift, pitch, saturation and distance from the start; `--release`), `drive` (no compensation vs. compensation vs. battery scaling at 6.4-8.4 V, 20 runs each; `--release`), `compare [OPTIONS] [LOG] < robot.log` (reads a robot log, splits it into attempts from `# safety: motors on` to `motors off` via `sim::robot_log`, simulates each from the same start pitch and battery voltage with `Drive::PROTOTYPE`, prints both traces side by side and a summary: time to fall, RMS/max pitch, oscillation period from zero crossings, mean |cmd|, % saturated; `--kv`, `--offset-deg` (balance-point error), `--vmax`, `--lag-ms`, `--start-duty`, `--stop-duty`, `--ideal`, `--gains` to fit the model; `--synthetic START_DEG` checks the tool on a simulated log; `--release`).
- `firmware/` - thin hardware glue for the ESP32-C3, a separate cargo workspace because it only builds for the RISC-V target (its `.cargo/config.toml` sets the target, linker script and `espflash` runner). The code is a library (`src/lib.rs`, `run(spawner, Experiment)`) plus **one binary per experiment** in `src/bin`, so switching experiments never needs a code edit. From `firmware/`, each alias (in `.cargo/config.toml`) builds, flashes and opens the log monitor:

  | Command | Experiment | Motors |
  |---|---|---|
  | `cargo imu-log` (also plain `cargo run --release`) | IMU + controller log | off |
  | `cargo deadband-test` | deadband ramps, one motor at a time | wheels in the air |
  | `cargo motor-check` | both motors +0.5 / stop / −0.5 | wheels in the air |
  | `cargo sign-check` | controller with P-only gains | wheels in the air |
  | `cargo balance` | controller with the balance gains | on the floor, hand ready |

  Save a log with e.g. `cargo balance | tee ../balance-4.log`. The log starts with `# experiment: ...`. Flashing needs the user in the `dialout` group. A new experiment = a variant of `Experiment` + a 20-line file in `src/bin` + an alias.
- `remote/` (from step E1) - micro:bit v2 firmware, its own cargo workspace like `firmware/` (target `thumbv7em-none-eabihf`, `probe-rs run --chip nRF52833_xxAA` runner, `defmt` + RTT logging). `cd remote && cargo run --release` flashes and shows the log; needs `cargo install probe-rs-tools` and the probe-rs udev rule. Keeps the same lint block. Shared logic (tilt → command, packet format) lives in `balance-core`, so `balance-core` must also build for `thumbv7em-none-eabihf`.
- Lint policy lives in `[workspace.lints]` in the root `Cargo.toml` (clippy pedantic, no `unsafe`, no `unwrap`/`expect`/`panic` outside tests), mirrored in `firmware/Cargo.toml` and `remote/Cargo.toml`; keep the three in sync. CI treats warnings as errors.
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

# from step E1 (micro:bit remote):
cargo build -p balance-core --target thumbv7em-none-eabihf
cd remote
cargo fmt --check
cargo clippy --release -- -D warnings
cargo build --release
```

## Software stack

- `esp-hal` + Embassy (no_std), async, via `esp-rtos`. Two executors: thread mode for setup, logging and the LED; an `InterruptExecutor` (software interrupt 1, priority 2) for the 500 Hz control loop, so it preempts everything else. Data the control task takes must be `Send` (LEDC channels aren't, so the task creates them itself)
- LEDC for motor PWM on both channels (needs esp-hal's `unstable` feature; the C3 has no MCPWM). 100 Hz, 10-bit (the lowest the timer divider allows is ~78 Hz at 10-bit). Measured deadband in "Known limitations" below
- `icm42670` crate for the onboard IMU (embedded-hal), used only for setup (WHO_AM_I, power, ranges): v0.2 reads each register byte in its own I2C transaction, which is slow and tears values. After setup the firmware takes the bus back (`free()`) and burst-reads all 12 data bytes from 0x0B in one transaction; decoding is `balance_core::imu::RawSample`
- IMU log format (firmware -> serial): `#` comment/header lines, then CSV `t_us,ax,ay,az,gx,gy,gz,pitch_deg,gx_dps,gy_dps,gz_dps,gyro_pitch_deg,filter_pitch_deg,motor_cmd,duty_l,duty_r,speed_est,pos_est`, each column right-aligned to a fixed width with spaces: raw LSB, then the accelerometer-only pitch in degrees (positive = tilted forward), then the bias-corrected gyro rates in °/s, then the pitch from integrating the gyro alone (starts at the accel pitch, drifts), then the complementary-filter pitch, then the controller's motor command (−1..1, positive = forward), then the duty actually applied to the left / right motor in signed percent (what `Motor` last sent: 0 while the command isn't applied; the motor check/test sequences show up here too), then the controller's wheel-speed estimate (fraction of full speed, see Control pipeline), then its position estimate (integral of the speed estimate since the motors came on, in full-speed seconds ≈ 0.41 m). New columns go at the end, so existing column indices stay valid. Sampled at 500 Hz, logged at 50 lines/s (`LOG_EVERY_N_SAMPLES` 10, for comparing with the simulation); `# loop max us: busy N of 2000, late ..., imu ..., battery adc ..., log send ...; dropped N` once a second (longest time per part since the last such line; `late` = how long after its scheduled time an iteration started). The sample loop doesn't print: it queues raw numbers (`LogMsg`) for the `log_writer` task, which formats them and writes via the async USB-Serial-JTAG driver; `println!` only before the loop and in `halt`. Before the CSV header, the startup gyro calibration logs `# gyro bias (deg/s): ...` (it retries while the robot is moving); accel ±4 g = 8192 LSB/g, gyro ±500 °/s = 65.5 LSB/(°/s)
- Control pipeline: `balance_core::control::Controller` (settings in `Config::PROTOTYPE`: IMU axes, balance point, filter τ, calibration limits, PID gains kp 0.09 / ki 0.5 / kd 0.003 per degree and speed feedback kv 6, position feedback kx 4, see D2; speed estimate = the command through a 50 ms lag, `speed_tau_s`; position estimate = its integral, lean from it clamped to 3°). Sign convention: pitch and motor command positive = forward; positive gains mean "tilted forward -> drive forward". The I term is essential: DC motors act like speed sources, and then tilt-only P or PD is unstable for any gains (one slow pole always stays unstable: the robot runs away until the motors saturate); ∫pitch carries the wheel-speed information that stabilises it
- The onboard IMU interrupt is probably not broken out: drive the control loop from an Embassy ticker at ~500 Hz and poll the IMU
- BLE (planned, group E). **ESP32-C3 is BLE only (no Classic BT / SPP)**, so GATT. Robot = **peripheral** (advertises a custom GATT service with one write-without-response characteristic; testable from a phone with nRF Connect), micro:bit = **central** (scans for the service UUID, connects, writes ~20-50 packets/s). Host stack **TrouBLE** (`trouble-host`) on both sides: on the ESP32-C3 over `esp-radio`'s BLE controller (needs a heap, `esp-alloc`; its interrupts may cost the control loop time: measure `late`/`busy`), on the micro:bit over `embassy-nrf` + `nrf-sdc` (Nordic SoftDevice Controller, has the central role), driving the LEDs, buttons and the LSM303AGR (`lsm303agr` crate) directly. Not `microbit-bsp`: its latest release (0.4, 2025) is on `embassy-nrf` 0.4, but `nrf-sdc` 0.4 / `trouble-host` 0.8 need `embassy-nrf` 0.11 (they agree on `bt-hci` 0.10, `embassy-sync` 0.8, `embassy-time` 0.5). Pin versions that agree on `bt-hci` / `embassy-*` in the step that adds them
- Remote packet (`balance_core::link`, shared by both sides): `version u8, seq u8, speed i8, turn i8, flags u8` (flag `active` = A+B held). The zero point lives on the micro:bit (it captures the attitude when A+B are pressed), so speed and turn are already relative to it and 0 while not active. Robot side: not active, link watchdog (no valid packet for ~250 ms) or disconnected → speed and turn targets 0, **not** motors off, which would make a balancing robot fall. No remote kill switch: only the robot's own safety cut-offs turn the motors off. Targets rate-limited. Speed target enters the controller through the speed feedback: P and I act on `pitch + kv·(speed_est − target_speed)`, so the robot settles at the target speed; turn is differential (`left = cmd + turn`, `right = cmd − turn`)

## Bring-up plan

Small steps, one PR each, in order. Tick steps off as they are completed; insert new steps anywhere. The open steps are grouped (A-D) towards a first balance attempt as early as possible: only the simulation work that attempt needs comes before it.

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
- [x] Spread the battery ADC readings over iterations (one non-blocking `read_oneshot` per iteration, averaged over 16 = 64 ms), instead of 16 at once (~730 us)
- [x] One motor, one direction, full speed (wheels off the ground)
- [x] Both motors, both directions, full speed (no PWM yet)
- [x] PWM speed control via LEDC
- [x] Measure the PWM deadband of both motors
- [x] Battery voltage via ADC on GPIO4
- [x] PID controller math in `balance-core` (`pid::Pid`: output limits, anti-windup, derivative on measurement), unit tested
- [x] Simulation: host-only `sim/` crate (std, root workspace) with an inverted-pendulum-on-wheels physics model (axle acceleration as input, motors ideal), no controller yet; unit test that an upright robot falls at the rate the physics predicts
- [x] Simulation: IMU sensor model (noise, bias, quantisation in the firmware's LSB units), output CSV in the firmware log format
- [x] Measure the IMU noise/bias (still log + `log_stats`) and put them into `ImuParams::prototype()`: noise per sample is white, accel [13, 20, 18] LSB, gyro [8, 6, 6] LSB (≈ 0.09 °/s on Y); gyro bias [−10, −15, −50] LSB; accel bias not measurable in one position, the balance offset (now +3.8°, see the axes fix below) is modelled as a mounting tilt
- [x] Measure the IMU position and put it into `ImuParams::prototype()`: 12.7 cm above the axle, 1.5 cm behind (towards the open side). That is close to the centre of percussion (15.4 cm for a fall with the axle held still, after the reaction-torque fix below; 12.2 cm before it), so at the start of a fall the accel pitch shows only ~1/5 of the tilt: during fast motion only the gyro sees it
- [x] Control step in `balance-core` (`control::Controller::step(&RawSample, dt) -> Output`: axis mapping, bias, accel/gyro/filter pitch, PID -> motor command) so firmware and simulator run the same code. Settings in `control::Config::PROTOTYPE` (gains 0 for now); the firmware computes the command but doesn't apply it yet
- [x] Simulation physics fix: include the motors' reaction torque on the body (the wheels are driven from the body, not pushed like a cart). Makes the axle acceleration ~40 % more effective (holding a tilt needs ~58 % of g·tan θ) and the fall slower (τ 0.125 s instead of 0.111 s)
- [x] Simulation: close the loop (complementary filter + PID at 500 Hz) and tune P, D, I in simulation. Ideal first-order motor (v_max 0.41 m/s measured: 10 turns in 5 s at full duty, wheels in the air). Findings: P and PD can't balance (PD holds ~1.5 s, then runs away); PID kp 0.12, ki 2.0, kd 0.008 settles from 5° in ~1 s (~2 Hz wobble) and drifts a few cm per 10 s; survives motor speed ±40 % and lag ×0.5/×2. The commands it needs are small (mostly < 0.2), far inside the real deadband (12-36 %), so the motor model and deadband compensation are essential before trying it on the robot
- [x] Check the IMU axes and the balance point with the robot upright (balanced = 0°, forward = positive). First result +Z forward, pitch rate −gy, balance point −3.8°; **corrected in the C2 sign check**: that check took the open side as the front, opposite to the motors' forward (A1), so tilting forward drove the wheels backward. With the closed side as the front: +Z backward, forward = −az, pitch rate = +gy, balance point +3.8°. **Balance point re-measured 2026-11-22** (robot held still at its balance point, `cargo imu-log`): az ≈ −600 → −4.2° accel pitch, i.e. 8° from the 2026-11-11 hand measurement (az +540, possibly held off-balance or something on the chassis moved). The +3.8° value made every balance attempt run away forward: in simulation kv 6 tolerates ±4° (steady drift at offset/kv of full speed), 8° needs more than full speed and falls in ~0.6 s

**A. Drive the motors from code (firmware, wheels in the air)**

- [x] Signed motor command: `Motor::set(-1.0..=1.0)` via `balance_core::clamp_duty` (direction pins + PWM duty; 0 = stop). Check: a fixed +0.5 turns both wheels the way that drives the robot forward; record left/right and polarity in `docs/wiring.md`
- [x] Deadband at lower PWM frequency: rerun the existing deadband test (`MOTOR_TEST_ENABLED`) at e.g. 100 Hz and 250 Hz next to 1 kHz, noting the battery voltage. Pick the frequency with the smallest start duty; update `MOTOR_PWM_FREQUENCY` and the deadband table in "Known limitations"
- [x] Safety cut-offs (`balance_core::safety::Safety`, unit tested): motors off when fallen (|filter pitch| > 45°, re-arm only when held within 5° again; off at power-up until held upright once) and on low battery (warn < 6.6 V, off < 6.4 V; each needs 1 s below the threshold so a sag under load doesn't count, and both latch until reboot). Firmware logs each change as `# safety: ...` and gates the motor test / direction check with it (and the controller's command, see the sign check in C). Check: tilt past 45° and see the log report the cut-off. Verified on the robot: every balance log shows `# safety: motors off` at ±45° and `motors on` again within 5°

**B. Deadband in simulation and in the controller**

- [x] Simulation: deadband in `sim::motor` with the measured start/stop duties (hysteresis: a standing wheel needs the start duty, a turning wheel keeps going down to the stop duty). Expected: the tuned PID no longer balances, or limit-cycles. Found: it **stays up** (0.5-10° starts, 5 noise seeds, even with a 40 % start / 14 % stop deadband), in one of two modes, both tested in `closed_loop`: (1) usually a steady roll at 2-7 cm/s (0.2-0.6 m in 10 s): the wheels never stand still, so only the low stop duty matters and the I term supplies that offset; (2) from small tilts sometimes a stick-slip limit cycle, period ~0.4 s (2.5 Hz), pitch ±0.3°, the wheels stopping and restarting every cycle, until it switches to (1). Caveats: the deadband was measured with the wheels in the air (on the floor the load raises it), and breaking loose is modelled as instant. Deadband compensation (next step) should reduce the drift and the stick-slip rather than be needed to stay up
- [x] Deadband compensation in `balance-core` (`drive::Compensation`: maps the command onto the duty range where a turning wheel moves, `sign·(stop + |cmd|·(1 − stop))`, stop 8 %; dead zone 0.005 around 0), applied after `Controller::step`. CI test: stays upright for 10 s from 2° and 10° and at 6.4 / 8.4 V (`closed_loop::compensated_stays_upright_for_10_s`). Gains unchanged. Findings (`sim --example drive`, 4 starts × 5 seeds, none fell): mean drift in 10 s 0.27 m without → 0.18 m with compensation (ideal motor: 0.17 m), max 0.60 → 0.50 m: it makes the motor look ideal while turning. But it doesn't touch the start duty, so when the robot settles near zero speed the wheels stand and stick-slip: 4 of 20 runs (1 without), worst pitch RMS 0.38° (0.21° without, still small). A start kick (≥ start duty for 10-50 ms when the command leaves 0, reverses or rises out of a small "standing" range, guessed from the command since there's no encoder) didn't help; a higher offset (12-15 %) made the stick-slip worse. So the start kick was dropped; the remaining drift (~0.2 m/10 s) is the tilt-only controller's own, as with the ideal motor. Rerun after the IMU axes fix (C2; only the noise lands differently): drift 0.31 → 0.20 m (ideal 0.21 m), stick-slip in 7 of 20 runs (0 without), worst pitch RMS 0.37° (0.11° without). Wired into the firmware in C3 (`apply_control`)
- [x] Battery-voltage scaling of the command (same wheel speed per command at 7 V and 8.4 V), math in `balance-core` (`drive::scale_for_battery`: duty × 7.4 V / v_battery, clamped; applied after the deadband compensation, since the friction a duty overcomes scales with the voltage too; a bad reading ≤ 0 V or NaN means no scaling); simulated with `Motor::at_battery`. A turning wheel now settles at the same speed for the same command at 6.4-8.4 V (unit test). In closed loop the PID barely notices (it tolerates ±40 % motor speed): without scaling, drift 0.16-0.20 m and worst pitch RMS 0.29-0.44° from 8.4 V to 6.4 V; with scaling 0.18 m and 0.37-0.38° at every voltage. Its value is that later tuning on the robot holds over a whole charge. The deadband duties were measured at an unknown voltage, so 7.4 V as their reference is a convention; re-measure at a known voltage. Wired into the firmware in C3 (`apply_control`)

**C. First balance attempt on the robot**

- [x] Add `motor_cmd` (and the applied duty) as CSV columns in the firmware log and in `sim::log`; update the log-format description in "Software stack". Verified on the robot: present in every balance log (now also `speed_est`)
- [x] Sign check, wheels in the air: controller output applied to the motors with small gains; tilting forward spins the wheels forward, the cut-off stops them past 45°. Firmware: `CONTROL_ENABLED` (off by default) applies `motor_cmd` to both motors while the safety cut-offs allow it; `CONTROL_GAINS` = `SIGN_CHECK_GAINS` (P only, kp 0.03/°: wheels start at ~6-7° of tilt, full power only at 33°). First try (7.6 V): tilted forward, the wheels turned **backward** (log: filter pitch −17°, motor_cmd −0.51): the IMU axes had the front on the open side; fixed in `robot_axes` / `robot_pitch_rate` and the sim's IMU model. After the fix the robot balanced, so the signs are right
- [x] First balance attempt with the simulated PID gains (P-only and PD won't balance, see the simulation step), on the floor, hand ready. Save the log. Firmware: `BALANCE_GAINS` (`Config::PROTOTYPE.gains`), the command goes through `Drive::PROTOTYPE` (deadband compensation + battery scaling) before `Motor::set`, the PID is reset (`Controller::reset_pid`) while the motors are off so the I term can't wind up while the robot is held or lying, log at 50 lines/s. Result (`balance-1.log`, 7.75 V): **it stood** (9.8 s and 9.5 s, one attempt 1.8 s), but shook hard (~8 Hz, saturating) and drifted; `dropped 0`, busy ~1300 of 2000 µs. Analysis and re-tune in C4
- [x] Compare the robot log with the simulation of the same start; adjust the sim motor parameters (lag, wheel inertia, deadband) until they agree, then re-tune in simulation and on the robot. Tool: `cargo run --release -p sim --example compare -- robot.log` (see the sim description); on a simulated log the catch from 3° matches within ~0.1° for the first ~0.6 s, then the stick-slip phase depends on the noise, so compare the summaries (period, RMS pitch, mean |cmd|), not the later samples. **First log (`balance-1.log`, 7.75 V, kd 0.008)**: 3 attempts, it stood 9.8 s and 9.5 s (one fell after 1.8 s), but shook: ~8 Hz, RMS pitch ~5°, ±150 °/s, mean |cmd| ~0.6, saturated ~30 % (sim: ~2 Hz, 0.5°, 0.04, 0 %). kd 0.008 × 150 °/s alone is more than full command: the D term drives a fast oscillation. More motor lag doesn't reproduce it (≥ 160 ms gives a slow ~2 Hz fall); a *faster* motor (lag 15-30 ms) plus a little dead time does (new `--dead-ms`, `Sim::with_dead_time_ticks`: lag 30 ms + 20-40 ms dead time shakes at ~6 Hz, saturated ~50 %, and falls). Re-tune: kd halved to 0.004 (in `Config::PROTOTYPE`) stays up and calm on the nominal model and with lag 15-30 ms + 20 ms dead time (RMS ≤ 0.66°, never saturated); lower kp or ki didn't help further; 40 ms of dead time defeats every gain set tried. The fit is not unique (lag and dead time trade off); next robot log will tell. Also seen: mean pitch ~0 but mean command +0.05..0.1 for seconds, i.e. the I term keeps it rolling forward (drift): look at the balance point once the shaking is gone. **Second log (`balance-2.log`, kd 0.004)**: the 8 Hz shaking is gone, but a new failure: pitch steady at 0..+0.8° while the command ramps 0.1 → 1.0 in ~1 s (= ki × the small offset), saturates, and the robot falls forward. The I term is the only thing that knows about the speed, so any offset in the measured pitch makes it add speed until the motors saturate. The sim never showed it because its balance point was exact: 0.25° of error is enough to run away (`compare --offset-deg -0.5` with the old gains reproduces the log). **Third log (`balance-3.log`, PR #54, balance point trimmed 3.8 → 3.0)**: still forward, faster ramp (1.1-1.6 /s), fell after 1-2 s; one constant offset doesn't fit both logs, so trimming can't fix it. → speed feedback (D2). Also seen in both: every attempt re-arms while the robot is still being swung upright (|rate| 120-225 °/s), so it starts saturated. **Fifth log (`balance-5.log`, kv 6, balance point −4.2°)**: **it balances.** Attempt 2 stood 20 s: 16 s calm at 0.5-0.9° RMS pitch (per second), never saturated, then recovered from pushes of 6° and 9° until a 20° one; attempt 1 stood 5 s, recovered from a 14° disturbance and fell only when it drove onto a laptop. Drift 0.5-4 cm/s (mean `speed_est` −0.01..−0.1), i.e. the balance point is now right to within ~0.1-0.5° (kv·v). Robot vs sim in the calm phase: RMS ~0.75° vs 0.68°, mean |cmd| 0.09 vs 0.05, wobble ~4 Hz vs ~1.5 Hz (robot faster: backlash or a faster motor; not a problem). `dropped 0`, busy ≤ 1393 µs, no I2C errors, battery 7.78 → 7.76 V (7.33 V at the worst moment)

**D. After it balances**

- [ ] Simulation: gear backlash in the motor model, if the log comparison shows it matters (limit cycle around upright). Not needed so far: the robot wobbles at ~4 Hz (sim ~1.5 Hz) but only ±1°
- [x] Experiments as launchable binaries (no constants to edit): `cargo imu-log`, `deadband-test`, `motor-check`, `sign-check`, `balance` (see `firmware/` above). Replaces `CONTROL_ENABLED`, `MOTOR_TEST_ENABLED`, `MOTOR_CHECK_ENABLED` and `BALANCE_GAINS`
- [x] Speed feedback (moved up from "drift" after the second balance attempt; needed to stay up at all, not just for drift): `Controller` estimates the wheel speed as the command through the motor's lag (`speed_est`, fraction of full speed; no encoders) and the PID's P and I act on `pitch + kv · speed` (setpoint `−kv · speed`, D still on the pitch). A balance-point error then gives a steady drift (pitch + kv·speed = offset) instead of a runaway. Re-tuned in simulation (`tune`): without kv no gain set survives a ±1° balance error; kp 0.09, ki 0.5, kd 0.003, kv 6 survives all 82 stress runs (±1.5° error, lag 15-80 ms, up to 30 ms dead time, ±30 % speed, 6.6 V), drifting ≤ 0.1 m/s per degree of error, and so do its neighbours; kd 0.005 or kv 10 already fail. kv is positive (same sign as the pitch) and needs kp·kv < 1 (0.54). Cost: more stick-slip wobble near standstill (up to ~±1.3-2° instead of ~0.5°). Logged as `speed_est`. First robot try (`balance-4.log`) fell forward after 1.6-7 s, saturating, but with the balance point still 8° off (+3.8° instead of −4.2°; `compare --offset-deg -8` falls in ~0.6 s for any gains tried). Still to do: try on the robot with −4.2° (expect it to stand without falling forward and drift slowly; check `speed_est` stays well below 1). **Works on the robot** (`balance-5.log`, see C4)
- [x] Re-arm only when the robot is held still: the safety cut-off re-armed within 5° even while the robot was swung upright, so every attempt started saturated. `Safety::update` now also takes the pitch rate: re-arming (and the first arm at power-up) needs |pitch| < 5° **and** |pitch rate| < 20 °/s without a break for 0.2 s (`Limits::rearm_rate_dps`, `rearm_hold_s`); tripping stays immediate, and once armed the rate doesn't matter. Evidence: at every re-arm in `balance-4/5.log` the rate was 40-230 °/s; held still by hand it is ≤ 3 °/s. Check on the robot (`cargo balance | tee ../balance-6.log`): swinging it upright doesn't switch the motors on; held upright and still, `# safety: motors on` appears after ~0.2 s, and the first rows after it show small |gy_dps| and no saturated command. Verified on the robot (`balance-6.log`): armed once, at pitch −0.0° and ~5 °/s, first commands 0.02-0.05 instead of saturated; the attempt then stood ~13 s
- [x] Position hold: also feed back the integrated speed estimate (lean against the distance travelled), if the steady drift is too much; tried in simulation first. Done in simulation: `Controller` integrates `speed_est` into `pos_est` (full-speed seconds since the motors came on; `reset_pid` zeroes it) and P/I act on `pitch + kv·speed + kx·position`; the lean it asks for is clamped to 3° (`max_position_lean_deg`, and the estimate stops growing there). Why it works: the I term only settles when its input averages 0; without kx a balance error ε then needs a steady speed −ε/kv (drift), with kx the robot can stand still at position −ε/kx. kx 4 (in `Config::PROTOTYPE`): with a 1° error it stops ~0.10 m from the start (kx 0: 0.68 m after 10 s, 1.37 m after 20 s, still going); `tune --check`: 0 falls in 82 stress runs, worst 0.17 m from the start (kx 0: 1.04 m), but more rocking around the spot (worst pitch over the last 5 s 1.7° mean vs 0.3°). Grid (`tune`): best per kx 0.52 / 0.30 / 0.19 / 0.11 / 0.06 / 0.03 m for kx 0 / 1 / 2 / 4 / 8 / 16, but kx ≥ 6 makes neighbouring gain sets fall (kx 4: only ki ±30 %, 1 of 82 each). Logged as `pos_est`. The estimate is the command's integral, not the real distance (deadband, battery, floor), so the hold is approximate. Still to do: try on the robot. Verified on the robot (`balance-7.log`): one 27 s attempt (still standing when the log ends); a push at ~3 s (−24°, up to 380 °/s) drove `pos_est` to its 0.75 clamp (~31 cm), then it rolled back: ~25 cm at 5 s, 15 at 10 s, 9 at 15 s, 5 at 20 s, 4 at 25 s (estimate, i.e. the integrated command, not measured distance)
- [ ] Battery under load: check sag and brownouts while balancing; add bulk capacitance if needed (see "Power notes")

**E. micro:bit remote over BLE (in parallel with C/D; only E12 needs a balancing robot)**

- [x] Remote skeleton: `remote/` crate (own workspace, `thumbv7em-none-eabihf`), `probe-rs`, "hello" over defmt/RTT; CI builds it. Check: `cd remote && cargo run --release` prints hello. Verified (probe-rs 0.32): hello, then a tick from a 64 M-cycle busy loop (`cortex_m::asm::delay`) every ~7.1 s instead of 1 s ("at least n cycles"; probably flash wait states with the cache off). The runner picks the micro:bit's probe with `--probe 0d28:0204`, since probe-rs also lists the ESP32-C3's USB JTAG when the robot is plugged in
- [x] Embassy on the micro:bit: blink one LED of the matrix from a task, log button A/B presses. Check: LED blinks, presses show in the log. Verified: centre LED (row 3 P0.15, column 3 P0.31) blinks from an Embassy `Timer` (RTC1), A (P0.14) and B (P0.23) logged as press/release with 20 ms debounce. Needs `embassy-nrf`'s `nfc-pins-as-gpio`: P0.09/P0.10 are edge pins and UICR already says GPIO, otherwise it warns at every start
- [x] Raw accelerometer: LSM303AGR over the internal I2C, log mg at ~50 Hz; tilt it to find the axes (like the robot's axes step) and record them in the hardware table. Done: internal I2C (SCL P0.08, SDA P0.16) at 400 kHz, accel at 0x19, 12-bit high-resolution, 50 Hz, ±2 g (1 mg/LSB), block data update on; `Ticker` every 20 ms. TWIM's DMA reads only RAM and the driver writes register addresses from constants in flash, so `Twim::new` needs a RAM `tx_ram_buffer` (8 bytes in a `StaticCell`; an empty one fails with `RAMBufferTooSmall`). At rest |a| ≈ 990-1000 mg; axes in the hardware table
- [x] Grip, hold A+B to drive (E3.5): `balance_core::remote::Grip` state machine, unit tested: "both held" (from the debounced buttons, 20 ms as in E2); on its rising edge it captures the reference pitch/roll from the accel; while held it reports the tilt relative to that reference; letting go of either button ends it (relative tilt 0); pressing again captures a new zero (subtracting angles is fine at these tilts). Firmware logs `grip on` (with the reference) / `grip off` and the relative tilt. Check: hold the micro:bit at any angle and press A+B, the relative tilt reads ~0; tilt and it follows; release and it reads 0 whatever the tilt. Done: `Grip::step(both_held, attitude) -> GripChange` (`On { reference }` / `Off` / `None`), `relative(attitude)`; `TiltRemote::step(accel, dt, both_held) -> Update` runs it. The button tasks publish their debounced state (press and release, 20 ms) in an `AtomicBool` each, read by the tilt task every sample. Verified on the micro:bit: `grip on` with the reference, relative tilt follows, `grip off` reads 0 at any tilt, the next press re-zeros
- [x] Tilt → command math in `balance-core` (`remote`: from the grip's relative tilt, dead zone ~5°, full scale ~30°, clamp, light low-pass, filter reset on grip off), unit tested; the micro:bit shows the command as a dot on the 5x5 matrix. Check: press A+B at any angle and the dot sits in the centre; tilt and it moves; release and it jumps back to the centre. Done (before E3.5, then moved onto the grip): `Attitude::from_accel` (pitch `atan2(−az, −ay)`: 0° upright facing you, 90° flat LEDs up; roll `atan2(ax, √(ay²+az²))`, right side lower = positive, independent of pitch), `shape` (5° dead zone, ramp to ±1 at 30°), `Config::MICROBIT`, `dot` (forward = top row, right = right column). The 0.1 s low-pass is on the accel vector, not the command: the reference is captured from the smoothed attitude and letting go drops the command to 0 at once, so no filter reset is needed. One dot needs no row scanning (one row high, one column low). `balance-core` built for `thumbv7em-none-eabihf` in CI. Verified on the micro:bit: dot centred at the press, follows the tilt, back to the centre on release
- [ ] Packet format in `balance-core` (`link`: encode/decode, version, seq, flags = `active`), unit tested; `thumbv7em` no_std build in CI
- [ ] Robot advertises (`esp-radio` + `trouble-host`, heap added): visible in nRF Connect on a phone. Check `# loop max us` with the radio on: are `late` / `busy` still fine at 500 Hz?
- [ ] Robot GATT service: write characteristic, decode packets, log `# remote: ...`; link watchdog in `balance_core::link`, unit tested. Check: write bytes from nRF Connect and see them logged; stop writing and see the timeout
- [ ] micro:bit as central: scan for the robot's service UUID, connect, write a fixed packet once a second; LEDs show scanning / connected. Check: the robot logs the packets; the remote reconnects after a robot reset
- [ ] Live tilt commands at 20-50 Hz, `active` from the grip; the robot logs `active`, speed and turn (not applied yet) and packets per second. Check: commands are 0 with `active` 0 unless A+B are held; pressed at an odd angle they start at 0; rate and gaps in the log; range (walk away until it drops)
- [ ] Steering, wheels in the air: differential turn in `balance_core::drive` (`left = cmd + turn`, `right = cmd − turn`, clamped), unit tested; the robot applies `turn`. Check: hold A+B and tilt the micro:bit left, the wheels spin opposite ways; let go and the turn part goes to 0 (with the robot held level, the wheels stop)
- [ ] Speed setpoint, simulation first: target speed in `Controller` (P/I on `pitch + kv·(speed_est − target)`, rate-limited); sim example stepping the target 0 → 0.2 → 0. Check in sim: stays up and follows the target
- [ ] Drive it (needs D2 done, i.e. the robot balances): apply the speed target on the robot. Check: tilt forward and it rolls forward, level the remote or release A+B and it stops; on link loss it stops but stays up

## Known limitations of the prototype

- No encoders: can do tilt control only, no speed/position hold, so expect drift. Partial mitigation: feed back the integrated motor command (bring-up plan, group D).
- TT motor gear backlash will make tuning harder.
- Large PWM deadband with hysteresis, wheels off the ground, 2 % steps. A standing wheel needs a minimum duty to start; a turning wheel keeps going down to a lower one. Measured 2026-11-08 and 2026-11-15 (battery voltage mostly not noted), so the numbers are only good to about ±10 points: the same 1 kHz setup gave left backward 32 % one day and 20 % the next. At 7.6 V the wheels did not start at 1 kHz even at 40 %. A lower PWM frequency helps: 100 Hz starts lower and stops lower than 1 kHz in every run, so the firmware uses 100 Hz. At 250 Hz the start duties were 20-25 % and the stop duty 10 % for all four combinations. The 100 Hz values are the later of two runs (the first gave 20-30 % to start):

  | Motor | Direction | Starts at, 100 Hz | Stops at, 100 Hz | Starts at, 1 kHz (11-08) | Stops at, 1 kHz (11-08) |
  |---|---|---|---|---|---|
  | Left | Forward | 16 % | 8 % | 36 % | 14 % |
  | Left | Backward | 16 % | 8 % | 32 % | 13 % |
  | Right | Forward | 22 % | 8 % | 30 % | 12 % |
  | Right | Backward | 20 % | 8 % | 34 % | 14 % |

  Still to do: 100 / 250 / 1000 Hz back to back on one charge with the voltage noted, and the stop duties at 1 kHz today.
