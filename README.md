# BalanceRobot

A two-wheel self-balancing robot, driven by tilting a hand-held remote. Everything is written in Rust: the robot's firmware on an ESP32-C3, a simulator that runs the same control code on a PC, and the remote on a BBC micro:bit v2.

It is a learning project: built one small, understood step at a time, with each step checked on the hardware or in simulation. The robot balances today on an old prototype chassis; the micro:bit remote over Bluetooth Low Energy is in progress.

## How it works

- An IMU (ICM-42670-P) is read 500 times a second. A complementary filter combines the accelerometer and the gyro into the robot's pitch.
- A PID controller turns the pitch into a motor command, with feedback on the estimated wheel speed and distance so the robot neither runs away nor drifts off. There are no wheel encoders: speed and position are estimated from the commands themselves.
- The command is corrected for the motors' deadband and the battery voltage, then drives two TT gear motors.
- Safety cut-offs switch the motors off when the robot has fallen or the battery is low.
- The remote (planned): hold A+B on the micro:bit and tilt it to drive; let go and the robot stops, but keeps balancing.

## Repository layout

| Path | What it is |
|---|---|
| `balance-core/` | All logic that doesn't touch hardware: filters, PID, controller, motor mapping, safety, remote and packet maths. `no_std`, unit-tested on the PC, shared by the firmware, the remote and the simulator |
| `firmware/` | ESP32-C3 firmware (`esp-hal` + Embassy). One binary per experiment: `cargo imu-log`, `motor-check`, `sign-check`, `balance`, ... |
| `remote/` | micro:bit v2 firmware (Embassy on nRF52833): buttons, accelerometer, tilt to command |
| `web/` | Phone remote and browser simulator (WebAssembly, built with trunk), published at https://kleist.github.io/BalanceRobot/ |
| `sim/` | PC simulation: pendulum physics, IMU and motor models, the closed loop, tuning and log-comparison tools |
| `docs/wiring.md` | How the prototype is wired, pin by pin |
| `balance-*.log` | Logs of real balance attempts, used to fit the simulation |
| [`PLAN.md`](PLAN.md) | What's next: the remote, then a custom board, a 3D-printed body, and a phone remote with a simulator in the browser |
| [`HISTORY.md`](HISTORY.md) | Every completed step, with what was measured and learned |
| [`CLAUDE.md`](CLAUDE.md) | Working notes: hardware details, power notes, software stack, how to run the checks |

## Hardware (prototype)

| Part | Details |
|---|---|
| MCU | ESP32-C3-DevKit-RUST-1 (onboard ICM-42670-P IMU) |
| Motor driver | L298N dual H-bridge |
| Motors | 2x TT gear motors, 65 mm wheels, no encoders |
| Battery | 2x 18650 in series (6.4-8.4 V) |
| Remote | BBC micro:bit v2 |

[`PLAN.md`](PLAN.md) describes the finished version: one JLCPCB-assembled board with USB-C charging, a DRV8833 motor driver and a speaker, in a printed body built to survive falls.

## Getting started

The toolchain is pinned in `rust-toolchain.toml`.

```sh
# Logic and simulation, on the PC
cargo test --workspace
cargo run --release -p sim --example balance     # simulated balance run, printed as a trace

# Robot firmware (needs espflash and the ESP32-C3 on USB)
cd firmware
cargo balance                                    # build, flash and show the log

# micro:bit remote (needs probe-rs)
cd ../remote
cargo run --release
```

See "Project layout & checks" in [`CLAUDE.md`](CLAUDE.md) for every experiment and the full set of checks CI runs.

## License

See [LICENSE](LICENSE).
