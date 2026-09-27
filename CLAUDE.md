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
- `firmware/` (coming in step 3) - thin hardware glue for the ESP32-C3, a separate cargo workspace because it only builds for the RISC-V target.
- Lint policy lives in `[workspace.lints]` in the root `Cargo.toml` (clippy pedantic, no `unsafe`, no `unwrap`/`expect`/`panic` outside tests). CI treats warnings as errors.
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
```

## Software stack

- `esp-hal` + Embassy (no_std), async
- LEDC for motor PWM on both channels
- `icm42670` crate for the onboard IMU (embedded-hal)
- The onboard IMU interrupt is probably not broken out: drive the control loop from an Embassy ticker at ~500 Hz and poll the IMU
- BLE later. **ESP32-C3 is BLE only (no Classic BT / SPP)**, so the Android app must use GATT

## Bring-up plan

Small steps, one PR each. Mark steps done as they are completed.

1. ~~Create this CLAUDE.md~~ (done)
2. ~~CI + `balance-core` skeleton: fmt, clippy, unit tests, no_std check, docs, coverage report~~ (done)
3. Minimal firmware crate (no Embassy yet) + full firmware build in CI; understand each file
4. Install the toolchain and `espflash` locally, flash the skeleton
5. See a "hello" log over USB-Serial-JTAG
6. Blink the LED on GPIO7 (blocking delay)
7. Same blink with Embassy (async task + timer)
8. I2C bus scan: find ICM-42670-P (0x68) and SHTC3 (0x70)
9. Read raw accelerometer values from the IMU and print them
10. Pitch angle from the accelerometer only (see how noisy it is)
11. Read the gyro, calibrate its bias at startup
12. Pitch angle by integrating the gyro only (see the drift)
13. Complementary filter combining both, fixed-rate loop (~500 Hz ticker)
14. One motor, one direction, full speed (wheels off the ground)
15. Both directions, then PWM speed control via LEDC
16. Both motors: verify directions, measure the PWM deadband
17. Battery voltage via ADC on GPIO4
18. P-only tilt loop, then add D, then I; tuning
19. BLE: advertise and show up on the phone
20. BLE GATT control (steering / setpoint) from Android

## Known limitations of the prototype

- No encoders: can do tilt control only, no speed/position hold, so expect drift. Partial mitigation: feed back the integrated motor command.
- TT motor gear backlash will make tuning harder.
