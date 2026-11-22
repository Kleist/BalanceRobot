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
//!
//! On the robot, packets arrive as BLE writes to one characteristic of a custom GATT
//! service ([`SERVICE_UUID`], [`PACKET_UUID`]). The [`Watchdog`] turns them into the
//! command the robot follows: the last valid packet's, or 0 once none has arrived for
//! [`TIMEOUT_US`], after a disconnect, or while it isn't `active`. 0 means "stand still"
//! (keep balancing), never "motors off": a balancing robot without motors falls.

use crate::remote::Command;

/// Packet format version; bump it when the layout changes.
pub const VERSION: u8 = 1;
/// Length of an encoded packet in bytes.
pub const LEN: usize = 5;

/// The robot's GATT service, as written in the usual UUID notation
/// (`55c4fb07-f880-4bd5-804a-c5eac902a078`, randomly generated). Advertised by the robot,
/// so the remote can find it by this instead of by name.
pub const SERVICE_UUID: u128 = 0x55c4_fb07_f880_4bd5_804a_c5ea_c902_a078;
/// The characteristic the remote writes packets to
/// (`b7b46705-9fb9-498c-9e95-cb4368dc792b`), in [`SERVICE_UUID`].
pub const PACKET_UUID: u128 = 0xb7b4_6705_9fb9_498c_9e95_cb43_68dc_792b;

/// A UUID's 16 bytes in the order BLE sends them: least significant byte first, the
/// reverse of how it is written.
#[must_use]
pub const fn uuid_bytes(uuid: u128) -> [u8; 16] {
    uuid.to_le_bytes()
}

/// AD types that carry a list of 128-bit service UUIDs: "incomplete" (more exist than
/// fit) and "complete".
const AD_TYPE_UUIDS_128: [u8; 2] = [0x06, 0x07];

/// Whether advertising data lists the service `uuid`. Advertising data (and a scan
/// response) is a sequence of *AD structures*: a length byte (counting the type byte and
/// the data), a type byte, then the data. The robot puts its service UUID in a "complete
/// list of 128-bit UUIDs" structure, so the remote can pick it out of everything it hears
/// without connecting. Malformed data (a length running past the end) ends the search.
#[must_use]
pub fn advertises_service(data: &[u8], uuid: u128) -> bool {
    let wanted = uuid_bytes(uuid);
    let mut rest = data;
    while let [len, tail @ ..] = rest {
        let len = usize::from(*len);
        if len == 0 || len > tail.len() {
            return false; // 0 = padding up to 31 bytes; too long = malformed
        }
        let (structure, next) = tail.split_at(len);
        if let [ad_type, uuids @ ..] = structure
            && AD_TYPE_UUIDS_128.contains(ad_type)
            && uuids.as_chunks::<16>().0.contains(&wanted)
        {
            return true;
        }
        rest = next;
    }
    false
}

/// With no valid packet for this long, the robot stops following the remote (speed and
/// turn 0). The remote sends 20-50 packets per second, so 250 ms is 5-12 lost in a row:
/// long enough not to trip on a short radio hiccup, short enough that a robot driving at
/// full speed (~0.4 m/s) rolls only ~10 cm on after the link drops.
pub const TIMEOUT_US: u64 = 250_000;

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

/// Turns the stream of packets into the command the robot follows, and notices when it
/// stops. Time is passed in as microseconds since any fixed start (the firmware's
/// `Instant`), so this stays independent of the hardware clock.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Watchdog {
    timeout_us: u64,
    /// The last valid packet and when it arrived; `None` before the first one, after a
    /// timeout and after a disconnect.
    last: Option<(u64, Packet)>,
}

impl Watchdog {
    /// No packet yet, so no command.
    #[must_use]
    pub const fn new(timeout_us: u64) -> Self {
        Self {
            timeout_us,
            last: None,
        }
    }

    /// A valid packet arrived at `now_us`.
    pub fn on_packet(&mut self, now_us: u64, packet: Packet) {
        self.last = Some((now_us, packet));
    }

    /// The remote disconnected: forget its last packet at once, without waiting for the
    /// timeout.
    pub fn on_disconnect(&mut self) {
        self.last = None;
    }

    /// Checks the timeout. Returns `true` once, at the first call after the last packet
    /// has become too old, and forgets that packet; `false` otherwise (also while there
    /// is no packet to time out). Call it regularly to log the moment the link is lost.
    pub fn poll(&mut self, now_us: u64) -> bool {
        if self.live_packet(now_us).is_none() && self.last.is_some() {
            self.last = None;
            true
        } else {
            false
        }
    }

    /// The last packet, if it is younger than the timeout.
    #[must_use]
    pub fn live_packet(&self, now_us: u64) -> Option<Packet> {
        let (at_us, packet) = self.last?;
        // `saturating_sub`: a timestamp from "the future" (shouldn't happen) counts as new.
        (now_us.saturating_sub(at_us) < self.timeout_us).then_some(packet)
    }

    /// What the robot should do now: the last packet's command while it is live (0 if it
    /// isn't `active`), otherwise 0. Checks the age itself, so it is right even if
    /// [`poll`](Self::poll) wasn't called.
    #[must_use]
    pub fn command(&self, now_us: u64) -> Command {
        self.live_packet(now_us)
            .map_or_else(Command::default, |packet| packet.command())
    }
}

/// Counts the packets that arrive, for a once-a-second line in the robot's log: how
/// many came, how many the remote numbered but never arrived (gaps in `seq`), and the
/// longest time between two. Time in microseconds, as for [`Watchdog`].
///
/// A lost packet shows up as a jump in `seq`: from 7 to 10 means 8 and 9 are missing.
/// BLE itself retransmits until a packet gets through, so within one connection a gap
/// means the remote skipped samples (it sends only the newest when the radio falls
/// behind), not radio loss. Radio loss shows up as long gaps instead. The same `seq`
/// twice would count as 255 lost; the remote never sends that.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct LinkStats {
    /// Arrival time and `seq` of the last packet; `None` before the first and after a
    /// disconnect.
    last: Option<(u64, u8)>,
    summary: LinkSummary,
}

/// What [`LinkStats::take`] reports, since the previous `take`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct LinkSummary {
    /// Packets that arrived.
    pub received: u32,
    /// Packets missing between them, from the `seq` gaps.
    pub lost: u32,
    /// Longest time between two packets, in µs (0 with fewer than two).
    pub longest_gap_us: u64,
}

impl LinkStats {
    /// Nothing counted yet.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            last: None,
            summary: LinkSummary {
                received: 0,
                lost: 0,
                longest_gap_us: 0,
            },
        }
    }

    /// A valid packet numbered `seq` arrived at `now_us`.
    pub fn on_packet(&mut self, now_us: u64, seq: u8) {
        if let Some((at_us, last_seq)) = self.last {
            // Wrapping: from 255 to 0 is the next one, nothing lost.
            let missing = seq.wrapping_sub(last_seq).wrapping_sub(1);
            self.summary.lost += u32::from(missing);
            let gap = now_us.saturating_sub(at_us);
            self.summary.longest_gap_us = self.summary.longest_gap_us.max(gap);
        }
        self.last = Some((now_us, seq));
        self.summary.received += 1;
    }

    /// The remote disconnected: the next packet starts afresh, so neither the time nor
    /// the `seq` jump across the reconnect counts.
    pub fn on_disconnect(&mut self) {
        self.last = None;
    }

    /// The counts since the last call, which start again from 0. The last packet is
    /// kept, so a gap across two reports still counts (in the second).
    pub fn take(&mut self) -> LinkSummary {
        core::mem::take(&mut self.summary)
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
    use super::{
        DecodeError, LEN, LinkStats, LinkSummary, PACKET_UUID, Packet, SERVICE_UUID, TIMEOUT_US,
        VERSION, Watchdog, advertises_service, uuid_bytes,
    };
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

    /// Advertising data made of `parts` in a zero-padded buffer, and how many bytes are
    /// used. 64 bytes: a legacy advertisement holds 31, extended advertising more.
    fn ad(parts: &[&[u8]]) -> ([u8; 64], usize) {
        let mut bytes = [0; 64];
        let mut len = 0;
        for part in parts {
            bytes[len..len + part.len()].copy_from_slice(part);
            len += part.len();
        }
        (bytes, len)
    }

    /// The robot's advertising data: flags, then the complete list of 128-bit UUIDs.
    fn robot_advertising() -> ([u8; 64], usize) {
        // Flags: LE general discoverable, no BR/EDR.
        ad(&[&[2, 0x01, 0x06], &[17, 0x07], &uuid_bytes(SERVICE_UUID)])
    }

    #[test]
    fn finds_the_service_in_the_robots_advertising() {
        let (bytes, len) = robot_advertising();
        assert!(advertises_service(&bytes[..len], SERVICE_UUID));
        // Zero padding after the data ends the search, it doesn't break it.
        assert!(advertises_service(&bytes, SERVICE_UUID));
    }

    #[test]
    fn other_uuids_and_names_dont_match() {
        let (bytes, len) = robot_advertising();
        assert!(!advertises_service(&bytes[..len], PACKET_UUID));
        // The name in the scan response: complete local name "BalanceRobot".
        let mut name = [0u8; 14];
        name[0] = 13;
        name[1] = 0x09;
        name[2..].copy_from_slice(b"BalanceRobot");
        assert!(!advertises_service(&name, SERVICE_UUID));
        assert!(!advertises_service(&[], SERVICE_UUID));
    }

    #[test]
    fn the_uuid_as_another_type_doesnt_count() {
        // Same 16 bytes, but as manufacturer data (0xff), not a service list.
        let (bytes, len) = ad(&[&[17, 0xff], &uuid_bytes(SERVICE_UUID)]);
        assert!(!advertises_service(&bytes[..len], SERVICE_UUID));
    }

    #[test]
    fn finds_it_among_several_uuids() {
        // An incomplete list of two (34 bytes: only fits extended advertising).
        let (bytes, len) = ad(&[
            &[1 + 32, 0x06],
            &uuid_bytes(PACKET_UUID),
            &uuid_bytes(SERVICE_UUID),
        ]);
        assert!(advertises_service(&bytes[..len], SERVICE_UUID));
    }

    #[test]
    fn a_length_past_the_end_stops_the_search() {
        // Claims 29 bytes of UUIDs, but only 16 follow.
        let (bytes, len) = ad(&[&[30, 0x07], &uuid_bytes(SERVICE_UUID)]);
        assert!(!advertises_service(&bytes[..len], SERVICE_UUID));
    }

    #[test]
    fn uuids_go_out_least_significant_byte_first() {
        let bytes = uuid_bytes(SERVICE_UUID);
        assert_eq!(bytes[0], 0x78); // last pair of "...c902a078"
        assert_eq!(bytes[15], 0x55); // first pair of "55c4fb07-..."
        assert_ne!(SERVICE_UUID, PACKET_UUID);
    }

    fn live(seq: u8, speed: f32) -> Packet {
        Packet::new(seq, command(speed, 0.0), true)
    }

    #[test]
    fn no_packet_no_command() {
        let mut watchdog = Watchdog::new(TIMEOUT_US);
        assert_eq!(watchdog.command(0), Command::default());
        assert_eq!(watchdog.live_packet(0), None);
        assert!(!watchdog.poll(1_000_000)); // nothing to time out
    }

    #[test]
    fn follows_the_last_packet() {
        let mut watchdog = Watchdog::new(TIMEOUT_US);
        watchdog.on_packet(1_000, live(1, 0.5));
        watchdog.on_packet(21_000, live(2, -0.25));
        let got = watchdog.command(30_000);
        assert!((got.speed + 0.25).abs() <= 0.5 / 127.0);
        assert_eq!(watchdog.live_packet(30_000).map(|p| p.seq), Some(2));
    }

    #[test]
    fn inactive_packet_keeps_the_link_but_gives_no_command() {
        let mut watchdog = Watchdog::new(TIMEOUT_US);
        watchdog.on_packet(0, Packet::new(5, command(0.9, 0.9), false));
        assert_eq!(watchdog.command(10), Command::default());
        assert!(watchdog.live_packet(10).is_some());
    }

    #[test]
    fn times_out_once() {
        let mut watchdog = Watchdog::new(TIMEOUT_US);
        watchdog.on_packet(1_000, live(1, 1.0));
        // Just inside the timeout: still live.
        assert!(!watchdog.poll(1_000 + TIMEOUT_US - 1));
        assert!(watchdog.command(1_000 + TIMEOUT_US - 1).speed > 0.99);
        // At the timeout: command 0 even before `poll` is called...
        assert_eq!(watchdog.command(1_000 + TIMEOUT_US), Command::default());
        // ...and `poll` reports it exactly once.
        assert!(watchdog.poll(1_000 + TIMEOUT_US));
        assert!(!watchdog.poll(1_000 + TIMEOUT_US + 50_000));
        // A new packet brings it back.
        watchdog.on_packet(2_000_000, live(9, 1.0));
        assert!(watchdog.command(2_000_001).speed > 0.99);
    }

    #[test]
    fn disconnect_stops_at_once_without_a_timeout() {
        let mut watchdog = Watchdog::new(TIMEOUT_US);
        watchdog.on_packet(0, live(1, 1.0));
        watchdog.on_disconnect();
        assert_eq!(watchdog.command(1), Command::default());
        assert!(!watchdog.poll(1_000_000));
    }

    #[test]
    fn a_packet_from_the_future_counts_as_new() {
        let mut watchdog = Watchdog::new(TIMEOUT_US);
        watchdog.on_packet(5_000, live(1, 1.0));
        assert!(watchdog.live_packet(4_000).is_some());
    }

    #[test]
    fn stats_count_packets_and_the_longest_gap() {
        let mut stats = LinkStats::new();
        stats.on_packet(1_000, 10);
        stats.on_packet(21_000, 11);
        stats.on_packet(61_000, 12);
        assert_eq!(
            stats.take(),
            LinkSummary {
                received: 3,
                lost: 0,
                longest_gap_us: 40_000
            }
        );
    }

    #[test]
    fn stats_count_seq_gaps_as_lost_and_wrap() {
        let mut stats = LinkStats::new();
        stats.on_packet(0, 7);
        stats.on_packet(20, 10); // 8 and 9 missing
        stats.on_packet(40, 255);
        stats.on_packet(60, 0); // 255 -> 0 is the next one
        stats.on_packet(80, 2); // 1 missing
        let summary = stats.take();
        assert_eq!(summary.received, 5);
        assert_eq!(summary.lost, 2 + 244 + 1);
    }

    #[test]
    fn take_starts_again_but_a_gap_across_two_reports_counts() {
        let mut stats = LinkStats::new();
        stats.on_packet(0, 1);
        stats.on_packet(20_000, 2);
        let _ = stats.take();
        assert_eq!(stats.take(), LinkSummary::default());
        stats.on_packet(500_000, 4);
        assert_eq!(
            stats.take(),
            LinkSummary {
                received: 1,
                lost: 1,
                longest_gap_us: 480_000
            }
        );
    }

    #[test]
    fn stats_ignore_the_jump_across_a_reconnect() {
        let mut stats = LinkStats::new();
        stats.on_packet(0, 100);
        stats.on_disconnect();
        stats.on_packet(5_000_000, 3);
        assert_eq!(
            stats.take(),
            LinkSummary {
                received: 2,
                lost: 0,
                longest_gap_us: 0
            }
        );
    }
}
