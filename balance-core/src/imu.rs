//! Decoding raw ICM-42670-P sensor data.
//!
//! The six data registers sit next to each other in the chip's register map:
//! accel X, Y, Z, then gyro X, Y, Z, each a signed 16-bit value stored as two
//! bytes, high byte first ("big-endian"). Reading all 12 bytes in one I2C
//! transaction (a *burst read*) is fast and keeps the high and low byte of
//! each value together, so no value can be torn between two samples.

/// Address of the first data register (`ACCEL_DATA_X1`). The chip increments
/// the register address after each byte, so a burst read from here returns
/// [`DATA_LEN`] bytes: accel X/Y/Z, then gyro X/Y/Z.
pub const DATA_START_REGISTER: u8 = 0x0B;

/// Bytes in one burst read: 6 values × 2 bytes.
pub const DATA_LEN: usize = 12;

/// One accel + gyro sample in raw sensor units (LSB), axes `[x, y, z]`.
///
/// The chip reports `-32768` on an axis while it has no valid data (e.g.
/// right after power-up).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RawSample {
    /// Acceleration per axis, in LSB (8192 per g at ±4 g).
    pub accel: [i16; 3],
    /// Rotation rate per axis, in LSB (65.5 per °/s at ±500 °/s).
    pub gyro: [i16; 3],
}

impl RawSample {
    /// Decodes the bytes of a burst read starting at [`DATA_START_REGISTER`].
    #[must_use]
    pub fn from_burst(bytes: &[u8; DATA_LEN]) -> Self {
        let value = |i: usize| i16::from_be_bytes([bytes[2 * i], bytes[2 * i + 1]]);
        Self {
            accel: [value(0), value(1), value(2)],
            gyro: [value(3), value(4), value(5)],
        }
    }
}

#[cfg(test)]
mod tests {
    use super::RawSample;

    #[test]
    fn decodes_axes_in_order_high_byte_first() {
        let bytes = [
            0x00, 0x01, 0x00, 0x02, 0x00, 0x03, // accel x, y, z
            0x01, 0x00, 0x02, 0x00, 0x03, 0x00, // gyro x, y, z
        ];
        let sample = RawSample::from_burst(&bytes);
        assert_eq!(sample.accel, [1, 2, 3]);
        assert_eq!(sample.gyro, [256, 512, 768]);
    }

    #[test]
    fn decodes_negative_values_as_twos_complement() {
        let mut bytes = [0; 12];
        bytes[0..2].copy_from_slice(&[0xFF, 0xFA]); // accel x = -6
        bytes[6..8].copy_from_slice(&[0x80, 0x00]); // gyro x = -32768
        bytes[10..12].copy_from_slice(&[0x7F, 0xFF]); // gyro z = 32767
        let sample = RawSample::from_burst(&bytes);
        assert_eq!(sample.accel, [-6, 0, 0]);
        assert_eq!(sample.gyro, [-32768, 0, 32767]);
    }

    #[test]
    fn one_g_on_z_at_4g_range() {
        // 8192 LSB = 0x2000: what the accelerometer reads lying flat at ±4 g.
        let mut bytes = [0; 12];
        bytes[4..6].copy_from_slice(&[0x20, 0x00]);
        assert_eq!(RawSample::from_burst(&bytes).accel, [0, 0, 8192]);
    }
}
