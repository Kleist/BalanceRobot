//! The packet the micro:bit remote sends to the robot over BLE, shared by both sides so
//! they can't disagree on the format.
//!
//! Five bytes, one BLE write each:
//!
//! | Byte | Field | Meaning |
//! |---|---|---|
//! | 0 | `version` | [`VERSION`]; the robot ignores packets of any other version |
//! | 1 | `seq` | counts up by one per packet, wrapping 255 → 0, so the robot can see gaps |
//! | 2 | `speed` | `i8`, −127..=127 = −1..=1 (positive = forward) |
//! | 3 | `turn` | `i8`, −127..=127 = −1..=1 (positive = turn right) |
//! | 4 | `flags` | bit 0 `active` (A+B held); the other bits must be 0 |
//!
//! The commands are already relative to the grip's reference (the micro:bit captures
//! it), and the robot treats a packet that isn't `active` as speed and turn 0, whatever
//! the two bytes say. Small integers instead of `f32`: 2 bytes instead of 8, and no
//! question of byte order or NaN on the wire. 1/127 of full scale is far finer than a
//! hand can tilt.
//!
//! Decoding is strict: a wrong length, an unknown version or a reserved flag bit is an
//! error, so a garbled or foreign write is dropped instead of steering the robot.

use crate::remote::Command;

/// Packet format version; bump it when the layout changes.
pub const VERSION: u8 = 1;
/// Length of an encoded packet in bytes.
pub const LEN: usize = 5;

/// Flag bit: both buttons held, the commands are live.
const FLAG_ACTIVE: u8 = 0b0000_0001;

/// One remote packet, as values.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Packet {
    /// Sequence number, wrapping.
    pub seq: u8,
    /// Speed, −127..=127 (−128 decodes like −127).
    pub speed: i8,
    /// Turn, −127..=127 (−128 decodes like −127).
    pub turn: i8,
    /// Both buttons held.
    pub active: bool,
}

/// Why [`Packet::decode`] rejected some bytes.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DecodeError {
    /// Not [`LEN`] bytes; holds the length received.
    WrongLength(usize),
    /// Another packet version; holds the version received.
    UnknownVersion(u8),
    /// A flag bit other than `active` is set; holds the flags byte.
    ReservedFlags(u8),
}

impl Packet {
    /// A packet for `command`. Not `active` sends 0 for both, so the bytes say the same
    /// as the flag.
    #[must_use]
    pub fn new(seq: u8, command: Command, active: bool) -> Self {
        let (speed, turn) = if active {
            (to_wire(command.speed), to_wire(command.turn))
        } else {
            (0, 0)
        };
        Self {
            seq,
            speed,
            turn,
            active,
        }
    }

    /// The bytes to send. `cast_unsigned` keeps the bits of the `i8` (two's
    /// complement: −1 is 0xFF), it doesn't clamp.
    #[must_use]
    pub fn encode(&self) -> [u8; LEN] {
        [
            VERSION,
            self.seq,
            self.speed.cast_unsigned(),
            self.turn.cast_unsigned(),
            if self.active { FLAG_ACTIVE } else { 0 },
        ]
    }

    /// Reads a packet from received bytes; see [`DecodeError`] for what it rejects.
    ///
    /// # Errors
    ///
    /// When the bytes aren't a valid packet of this version.
    pub fn decode(bytes: &[u8]) -> Result<Self, DecodeError> {
        let &[version, seq, speed, turn, flags] = bytes else {
            return Err(DecodeError::WrongLength(bytes.len()));
        };
        if version != VERSION {
            return Err(DecodeError::UnknownVersion(version));
        }
        if flags & !FLAG_ACTIVE != 0 {
            return Err(DecodeError::ReservedFlags(flags));
        }
        Ok(Self {
            seq,
            speed: speed.cast_signed(),
            turn: turn.cast_signed(),
            active: flags & FLAG_ACTIVE != 0,
        })
    }

    /// The command this packet asks for, each −1..=1; 0 for both when not `active`.
    #[must_use]
    pub fn command(&self) -> Command {
        if !self.active {
            return Command::default();
        }
        Command {
            speed: from_wire(self.speed),
            turn: from_wire(self.turn),
        }
    }
}

/// −1..=1 → −127..=127, rounded to the nearest step; clamped, `NaN` → 0.
fn to_wire(value: f32) -> i8 {
    let value = if value.is_nan() {
        0.0
    } else {
        value.clamp(-1.0, 1.0)
    };
    #[allow(
        clippy::cast_possible_truncation,
        reason = "value is clamped to -1..=1, so the rounded result is in -127..=127"
    )]
    let wire = libm::roundf(value * 127.0) as i8;
    wire
}

/// −127..=127 → −1..=1; −128 (never sent) gives −1.
fn from_wire(value: i8) -> f32 {
    (f32::from(value) / 127.0).max(-1.0)
}

#[cfg(test)]
mod tests {
    use super::{DecodeError, LEN, Packet, VERSION};
    use crate::remote::Command;

    fn command(speed: f32, turn: f32) -> Command {
        Command { speed, turn }
    }

    fn assert_close(actual: f32, expected: f32) {
        assert!(
            (actual - expected).abs() < 1e-6,
            "expected {expected}, got {actual}"
        );
    }

    #[test]
    fn byte_layout() {
        let packet = Packet::new(200, command(1.0, -0.5), true);
        // -0.5 · 127 = -63.5, rounded away from zero to -64 = 0xC0.
        assert_eq!(packet.encode(), [VERSION, 200, 127, 0xC0, 0b1]);
        assert_eq!(packet.encode().len(), LEN);
    }

    #[test]
    fn round_trip() {
        for active in [false, true] {
            for (speed, turn) in [(0, 0), (127, -127), (-1, 1), (64, -64)] {
                let packet = Packet {
                    seq: 7,
                    speed,
                    turn,
                    active,
                };
                assert_eq!(Packet::decode(&packet.encode()), Ok(packet));
            }
        }
    }

    #[test]
    fn command_survives_the_wire_to_within_one_step() {
        for (speed, turn) in [(0.0, 0.0), (1.0, -1.0), (0.3, -0.77), (-0.004, 0.996)] {
            let sent = Packet::new(0, command(speed, turn), true);
            let Ok(received) = Packet::decode(&sent.encode()) else {
                panic!("own packet rejected: {sent:?}");
            };
            let got = received.command();
            assert!((got.speed - speed).abs() <= 0.5 / 127.0 + 1e-6);
            assert!((got.turn - turn).abs() <= 0.5 / 127.0 + 1e-6);
        }
    }

    #[test]
    fn out_of_range_and_nan_are_clamped() {
        let packet = Packet::new(0, command(3.0, f32::NAN), true);
        assert_eq!((packet.speed, packet.turn), (127, 0));
        let packet = Packet::new(0, command(-3.0, f32::NEG_INFINITY), true);
        assert_eq!((packet.speed, packet.turn), (-127, -127));
    }

    #[test]
    fn inactive_means_no_command() {
        // The remote sends zeros when not active...
        let packet = Packet::new(1, command(0.8, -0.4), false);
        assert_eq!((packet.speed, packet.turn, packet.active), (0, 0, false));
        // ...and the robot ignores the bytes anyway if a sender doesn't.
        let Ok(received) = Packet::decode(&[VERSION, 1, 100, 50, 0]) else {
            panic!("valid packet rejected");
        };
        assert_eq!(received.command(), Command::default());
    }

    #[test]
    fn minus_128_reads_as_full_reverse() {
        let Ok(received) = Packet::decode(&[VERSION, 0, 0x80, 0x80, 1]) else {
            panic!("valid packet rejected");
        };
        assert_close(received.command().speed, -1.0);
        assert_close(received.command().turn, -1.0);
    }

    #[test]
    fn rejects_wrong_length() {
        assert_eq!(Packet::decode(&[]), Err(DecodeError::WrongLength(0)));
        assert_eq!(
            Packet::decode(&[VERSION, 0, 0, 0]),
            Err(DecodeError::WrongLength(4))
        );
        assert_eq!(
            Packet::decode(&[VERSION, 0, 0, 0, 1, 0]),
            Err(DecodeError::WrongLength(6))
        );
    }

    #[test]
    fn rejects_other_versions() {
        assert_eq!(
            Packet::decode(&[VERSION + 1, 0, 0, 0, 1]),
            Err(DecodeError::UnknownVersion(VERSION + 1))
        );
        assert_eq!(
            Packet::decode(&[0, 0, 0, 0, 1]),
            Err(DecodeError::UnknownVersion(0))
        );
    }

    #[test]
    fn rejects_reserved_flags() {
        assert_eq!(
            Packet::decode(&[VERSION, 0, 0, 0, 0b10]),
            Err(DecodeError::ReservedFlags(0b10))
        );
        assert_eq!(
            Packet::decode(&[VERSION, 0, 0, 0, 0xFF]),
            Err(DecodeError::ReservedFlags(0xFF))
        );
    }
}
