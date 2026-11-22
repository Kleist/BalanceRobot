# BalanceRobot

Two-wheel self-balancing robot. Firmware in Rust (no_std) on ESP32-C3, later controlled from a hand-held **BBC micro:bit v2** tilt remote over BLE (also Rust, Embassy).

## How we work (learning project)

This is a **learning project**. The goal is to understand each piece, not just to get a working robot fast.

- Take **one small step at a time**. Each step should be small enough to understand fully and to verify on the hardware.
- **One step per PR.** Stop after each step and wait for the user before starting the next one. Do not jump ahead or bundle future steps "while we're at it".
- Explain the *why*: new concepts (Embassy tasks, I2C, PWM, filters, PID, ...), crate choices and non-obvious lines of code.
- Prefer minimal code over generated boilerplate; if a generator is used, walk through what it produced.
- Each step ends with a concrete way to check it on the robot (what to flash, what to look for in the log or on the hardware).
- Keep [`PLAN.md`](PLAN.md) up to date: tick steps when done and move them with their findings to [`HISTORY.md`](HISTORY.md); split steps that turn out too big.

## Prototype

This repo starts by reviving an **old prototype chassis** as a test bed. The finished robot keeps the TT motors, the ESP32-C3 and the IMU on a custom board in a printed body (see "Product" in [`PLAN.md`](PLAN.md)), so code developed here carries over.

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
  | `cargo ble-link` | BLE link to the remote: packets per second, gaps and `active` changes logged, not applied. IMU and controller run, but no CSV lines (only `#` lines) | off |

  Save a log with e.g. `cargo balance | tee ../balance-4.log`. The log starts with `# experiment: ...`. Flashing needs the user in the `dialout` group. A new experiment = a variant of `Experiment` + a 20-line file in `src/bin` + an alias.
- `remote/` (from step E1) - micro:bit v2 firmware, its own cargo workspace like `firmware/` (target `thumbv7em-none-eabihf`, `probe-rs run --chip nRF52833_xxAA` runner, `defmt` + RTT logging). `cd remote && cargo run --release` flashes and shows the log; needs `cargo install probe-rs-tools` and the probe-rs udev rule. Keeps the same lint block. Shared logic (tilt → command, packet format) lives in `balance-core`, so `balance-core` must also build for `thumbv7em-none-eabihf`.
- `web/` (from step I1) - phone remote and browser simulator (group I), its own cargo workspace like `firmware/` and `remote/` (target `wasm32-unknown-unknown`). A binary crate built by **trunk** (`cargo install trunk --locked`): `trunk build` runs `cargo build`, then `wasm-bindgen` (the JS glue) and `wasm-opt`, and writes a static site to `web/dist/` from `web/index.html`. `cd web && trunk serve --open` serves it on http://127.0.0.1:8080 and rebuilds on every change. From step I2 the page is the simulator: `App` (exported to JS with `#[wasm_bindgen]`) holds a `sim::closed_loop::Sim` (prototype motor + `Drive::PROTOTYPE`, let go at 5°); the page's only JavaScript calls `App::frame(now)` from `requestAnimationFrame`, `clock::Clock` turns frame times into whole 2 ms ticks (real time at any refresh rate, at most 100 ms per frame), `view` draws it on a canvas from the side (camera follows the robot, ground marks every 5 cm, readout). `cargo test` in `web/` runs natively (clock, and the page's robot stands up from 5°). `.github/workflows/web.yml` builds it and pushes the result to the `gh-pages` branch, which GitHub Pages serves (repo Settings → Pages → Deploy from a branch → `gh-pages`, `/ (root)`): `main` at https://kleist.github.io/BalanceRobot/, each PR as a preview at `.../BalanceRobot/pr-preview/pr-<number>/` (linked in a bot comment on the PR, removed when the PR closes).
- Lint policy lives in `[workspace.lints]` in the root `Cargo.toml` (clippy pedantic, no `unsafe`, no `unwrap`/`expect`/`panic` outside tests), mirrored in `firmware/Cargo.toml`, `remote/Cargo.toml` and `web/Cargo.toml`; keep the four in sync. CI treats warnings as errors.
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

# from step I1 (web app):
cd web
cargo fmt --check
cargo clippy --target wasm32-unknown-unknown --release -- -D warnings
cargo test
trunk build --release
```

## Software stack

- `esp-hal` + Embassy (no_std), async, via `esp-rtos`. Two executors: thread mode for setup, logging and the LED; an `InterruptExecutor` (software interrupt 1, priority 2) for the 500 Hz control loop, so it preempts everything else. Data the control task takes must be `Send` (LEDC channels aren't, so the task creates them itself)
- LEDC for motor PWM on both channels (needs esp-hal's `unstable` feature; the C3 has no MCPWM). 100 Hz, 10-bit (the lowest the timer divider allows is ~78 Hz at 10-bit). Measured deadband in "Known limitations" below
- `icm42670` crate for the onboard IMU (embedded-hal), used only for setup (WHO_AM_I, power, ranges): v0.2 reads each register byte in its own I2C transaction, which is slow and tears values. After setup the firmware takes the bus back (`free()`) and burst-reads all 12 data bytes from 0x0B in one transaction; decoding is `balance_core::imu::RawSample`
- IMU log format (firmware -> serial): `#` comment/header lines, then CSV `t_us,ax,ay,az,gx,gy,gz,pitch_deg,gx_dps,gy_dps,gz_dps,gyro_pitch_deg,filter_pitch_deg,motor_cmd,duty_l,duty_r,speed_est,pos_est`, each column right-aligned to a fixed width with spaces: raw LSB, then the accelerometer-only pitch in degrees (positive = tilted forward), then the bias-corrected gyro rates in °/s, then the pitch from integrating the gyro alone (starts at the accel pitch, drifts), then the complementary-filter pitch, then the controller's motor command (−1..1, positive = forward), then the duty actually applied to the left / right motor in signed percent (what `Motor` last sent: 0 while the command isn't applied; the motor check/test sequences show up here too), then the controller's wheel-speed estimate (fraction of full speed, see Control pipeline), then its position estimate (integral of the speed estimate since the motors came on, in full-speed seconds ≈ 0.41 m). New columns go at the end, so existing column indices stay valid. Sampled at 500 Hz, logged at 50 lines/s (`LOG_EVERY_N_SAMPLES` 10, for comparing with the simulation); `# loop max us: busy N of 2000, late ..., imu ..., battery adc ..., log send ...; dropped N` once a second (longest time per part since the last such line; `late` = how long after its scheduled time an iteration started). The sample loop doesn't print: it queues raw numbers (`LogMsg`) for the `log_writer` task, which formats them and writes via the async USB-Serial-JTAG driver; `println!` only before the loop and in `halt`. Before the CSV header, the startup gyro calibration logs `# gyro bias (deg/s): ...` (it retries while the robot is moving); accel ±4 g = 8192 LSB/g, gyro ±500 °/s = 65.5 LSB/(°/s)
- Control pipeline: `balance_core::control::Controller` (settings in `Config::PROTOTYPE`: IMU axes, balance point, filter τ, calibration limits, PID gains kp 0.09 / ki 0.5 / kd 0.003 per degree and speed feedback kv 6, position feedback kx 4, see D2; speed estimate = the command through a 50 ms lag, `speed_tau_s`; position estimate = its integral, lean from it clamped to 3°). Sign convention: pitch and motor command positive = forward; positive gains mean "tilted forward -> drive forward". The I term is essential: DC motors act like speed sources, and then tilt-only P or PD is unstable for any gains (one slow pole always stays unstable: the robot runs away until the motors saturate); ∫pitch carries the wheel-speed information that stabilises it
- The onboard IMU interrupt is probably not broken out: drive the control loop from an Embassy ticker at ~500 Hz and poll the IMU
- BLE (group E, in progress). **ESP32-C3 is BLE only (no Classic BT / SPP)**, so GATT. Robot = **peripheral** (advertises a custom GATT service with one write-without-response characteristic; testable from a phone with nRF Connect), micro:bit = **central** (scans for the service UUID, connects, writes ~20-50 packets/s). Host stack **TrouBLE** (`trouble-host`) on both sides: on the ESP32-C3 over `esp-radio`'s BLE controller (needs a heap, `esp-alloc`; its interrupts may cost the control loop time: measure `late`/`busy`), on the micro:bit over `embassy-nrf` + `nrf-sdc` (Nordic SoftDevice Controller, has the central role), driving the LEDs, buttons and the LSM303AGR (`lsm303agr` crate) directly. Not `microbit-bsp`: its latest release (0.4, 2025) is on `embassy-nrf` 0.4, but `nrf-sdc` 0.4 / `trouble-host` 0.8 need `embassy-nrf` 0.11 (they agree on `bt-hci` 0.10, `embassy-sync` 0.8, `embassy-time` 0.5). Pin versions that agree on `bt-hci` / `embassy-*` in the step that adds them. Robot side (E6): `esp-radio` **=1.0.0-beta.1** (the release for `esp-hal` 1.2; pinned exactly, beta.2 was yanked; features `ble`, `unstable`), `trouble-host` 0.8 (peripheral, default packet pool), `bt-hci` 0.10 (`ExternalController` over `esp-radio`'s `BleConnector`, which implements `bt-hci-transport` 0.1's `Transport`, the trait `bt-hci` 0.10 uses; `esp-radio` also pulls an unused `bt-hci` 0.9), heap from `esp-alloc` (64 KiB reclaimed bootloader RAM + 36 KiB), code in `firmware/src/ble.rs`, only in the experiments that use it (`Experiment::uses_ble`). `esp-rtos` needs its `esp-radio` feature (a preemptive scheduler for the controller's threads), which is crate-wide: it costs **every** experiment ~100 µs of `late` and `busy` and ~75 µs of `imu` (cause not known), radio or not; the BT interrupts are at priority 1, below the control loop. Remote side (E8): `nrf-sdc` 0.4 (**`multirole`**: the `central`-only library lacks extended scanning, which `trouble-host`'s runner references, so it doesn't link) + `nrf-mpsl` 0.4 with **`critical-section-impl`** instead of cortex-m's `critical-section-single-core` (MPSL's radio interrupts at priority 0 must never be masked); MPSL owns RTC0, TIMER0, TEMP and PPI 17-31; every other interrupt (GPIOTE, RTC1 time driver, TWIM, RNG) is set to priority 2 by hand, since embassy-nrf drivers leave the reset default 0; code in `remote/src/ble.rs`
- Remote packet (`balance_core::link`, shared by both sides): `version u8, seq u8, speed i8, turn i8, flags u8` (flag `active` = A+B held). The micro:bit sends one per accelerometer sample (50/s) through a `Signal` (newest only, so a stalled radio skips samples instead of delaying them), connection interval 20 ms (trouble-host's default 80 ms would bunch them); the robot logs `# remote: N packets/s, lost (seq gaps), longest gap; latest command` once a second (`link::LinkStats`). The zero point lives on the micro:bit (it captures the attitude when A+B are pressed), so speed and turn are already relative to it and 0 while not active. Robot side: not active, link watchdog (no valid packet for ~250 ms) or disconnected → speed and turn targets 0, **not** motors off, which would make a balancing robot fall. No remote kill switch: only the robot's own safety cut-offs turn the motors off. Targets rate-limited. Speed target enters the controller through the speed feedback: P and I act on `pitch + kv·(speed_est − target_speed)`, so the robot settles at the target speed; turn is differential (`left = cmd + turn`, `right = cmd − turn`)

## Plan and history

- [`PLAN.md`](PLAN.md): the open steps, in groups (D, E on the prototype; F robot board, G chassis, H firmware on the robot board, I phone web app), and the product decisions for the finished robot.
- [`HISTORY.md`](HISTORY.md): every completed step with its findings and measurements. Look there before re-measuring or re-deciding something.
- When a step is done, tick it in `PLAN.md` and move it with its findings to `HISTORY.md`.

## Known limitations of the prototype

- No encoders: can do tilt control only, no speed/position hold, so expect drift. Partial mitigation: feed back the integrated motor command (done in group D, see `HISTORY.md`).
- TT motor gear backlash will make tuning harder.
- Large PWM deadband with hysteresis, wheels off the ground, 2 % steps. A standing wheel needs a minimum duty to start; a turning wheel keeps going down to a lower one. Measured 2026-11-08 and 2026-11-15 (battery voltage mostly not noted), so the numbers are only good to about ±10 points: the same 1 kHz setup gave left backward 32 % one day and 20 % the next. At 7.6 V the wheels did not start at 1 kHz even at 40 %. A lower PWM frequency helps: 100 Hz starts lower and stops lower than 1 kHz in every run, so the firmware uses 100 Hz. At 250 Hz the start duties were 20-25 % and the stop duty 10 % for all four combinations. The 100 Hz values are the later of two runs (the first gave 20-30 % to start):

  | Motor | Direction | Starts at, 100 Hz | Stops at, 100 Hz | Starts at, 1 kHz (11-08) | Stops at, 1 kHz (11-08) |
  |---|---|---|---|---|---|
  | Left | Forward | 16 % | 8 % | 36 % | 14 % |
  | Left | Backward | 16 % | 8 % | 32 % | 13 % |
  | Right | Forward | 22 % | 8 % | 30 % | 12 % |
  | Right | Backward | 20 % | 8 % | 34 % | 14 % |

  Still to do: 100 / 250 / 1000 Hz back to back on one charge with the voltage noted, and the stop duties at 1 kHz today.
