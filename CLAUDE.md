# BalanceRobot

Two-wheel self-balancing robot. Firmware in Rust (no_std) on ESP32-C3, later controlled from an Android phone over BLE.

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
GPIO20/21 are UART0; log via USB-Serial-JTAG, not UART0.

## Power notes

- **Never** connect the 2S pack to the board's BAT+ (single Li-ion cell only).
- During development: power the board over USB, share **GND only** with the L298N.
- Untethered: L298N 5V output -> board 5V pin. TODO: check the esp-rs/esp-rust-board schematic for whether 5V and VBUS are diode-isolated before connecting both.
- Add 100-470 uF bulk capacitance near the board; motor noise/brownout resets are likely.
- L298N drops ~2 V, so motors see ~5-6 V (fine for TT motors, but torque sags as the battery drains).
- L298N logic inputs accept 3.3 V (V_IH min 2.3 V).

## Software stack

- `esp-hal` + Embassy (no_std), async
- LEDC for motor PWM on both channels
- `icm42670` crate for the onboard IMU (embedded-hal)
- The onboard IMU interrupt is probably not broken out: drive the control loop from an Embassy ticker at ~500 Hz and poll the IMU
- BLE later. **ESP32-C3 is BLE only (no Classic BT / SPP)**, so the Android app must use GATT

## Project layout & commands

Generated with `esp-generate` 1.4 (`esp32c3`, `unstable-hal`, `embassy`, `esp-backtrace`, `log`, `ci`).

- `src/bin/main.rs` - firmware entry point (Embassy main, task spawning)
- `src/lib.rs` - put reusable, hardware-independent modules here (filter, PID, motor mapping) so they carry over to the encoder rebuild
- `.cargo/config.toml` - target `riscv32imc-unknown-none-elf`, `build-std = ["core"]`, `cargo run` = `espflash flash --monitor`
- `.cargo/esp-config.toml` - runtime config (`ESP_LOG` level)
- Toolchain: stable Rust >= 1.95 with `rust-src` (pinned via `rust-toolchain.toml`)

```sh
cargo build --release                               # build
cargo run --release                                 # flash + monitor (needs espflash, board on USB)
cargo fmt --all -- --check                          # CI: format
cargo clippy --all-features --workspace -- -D warnings  # CI: lint
```

`esp-println` is built with `jtag-serial` only (no `auto`), so logs never go to UART0 on GPIO20/21, which drive IN3/IN4.
Note the ROM bootloader still prints on UART0 TX (GPIO21 = IN4) at reset; harmless as long as ENB (GPIO6) stays low during boot.

## Bring-up plan

1. ~~Project scaffold, logging over USB-Serial-JTAG, blink LED~~ (done)
2. Read IMU, calibrate gyro bias at startup
3. Pitch angle via complementary filter
4. Motors open-loop: verify directions, measure PWM deadband
5. PID tilt loop, tuning
6. BLE GATT control (steering / setpoint) from Android

## Known limitations of the prototype

- No encoders: can do tilt control only, no speed/position hold, so expect drift. Partial mitigation: feed back the integrated motor command.
- TT motor gear backlash will make tuning harder.
