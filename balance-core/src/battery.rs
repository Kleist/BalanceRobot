//! Battery voltage from an ADC reading of a resistor divider.
//!
//! The 2S pack (up to 8.4 V) is far above what the ADC can measure (about
//! 2.5 V on the ESP32-C3), so two resistors divide it down first:
//! battery + → `top` → ADC pin → `bottom` → GND. The same current flows
//! through both, so the pin sees the fraction `bottom / (top + bottom)` of
//! the battery voltage.

/// A resistor divider between the battery and an ADC pin.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Divider {
    /// Resistor from battery + to the ADC pin, in ohms.
    pub top_ohm: f32,
    /// Resistor from the ADC pin to GND, in ohms.
    pub bottom_ohm: f32,
}

impl Divider {
    /// Battery voltage in volts from the voltage measured at the pin, in mV.
    ///
    /// Undoes the division: `battery = pin × (top + bottom) / bottom`. A
    /// `bottom_ohm` of zero is meaningless (the pin would always read 0 V) and
    /// gives infinity or NaN.
    #[must_use]
    pub fn battery_v(&self, pin_mv: u16) -> f32 {
        let ratio = (self.top_ohm + self.bottom_ohm) / self.bottom_ohm;
        f32::from(pin_mv) / 1000.0 * ratio
    }
}

#[cfg(test)]
mod tests {
    use super::Divider;

    /// The robot's divider: 100 kΩ / 33 kΩ.
    const ROBOT: Divider = Divider {
        top_ohm: 100_000.0,
        bottom_ohm: 33_000.0,
    };

    fn assert_close(actual: f32, expected: f32) {
        assert!(
            (actual - expected).abs() < 1e-3,
            "expected {expected}, got {actual}"
        );
    }

    #[test]
    fn full_pack_reads_about_2_1_v_at_the_pin() {
        // 8.4 V × 33 / 133 = 2084 mV at the pin.
        assert_close(ROBOT.battery_v(2084), 8.399);
    }

    #[test]
    fn zero_at_the_pin_is_zero_volts() {
        assert_close(ROBOT.battery_v(0), 0.0);
    }

    #[test]
    fn equal_resistors_double_the_pin_voltage() {
        let half = Divider {
            top_ohm: 10_000.0,
            bottom_ohm: 10_000.0,
        };
        assert_close(half.battery_v(1500), 3.0);
    }
}
