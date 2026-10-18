//! Faults a test can inject. They act only on what the controller sees (sensors) and on what it commands
//! (inverter); the motor physics stay healthy. Random faults draw from their own seeded stream, so switching one on
//! never changes the sensor noise sequence.

/// Everything that can be broken; [`Faults::NONE`] is healthy.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Faults {
    /// Sensor path.
    pub sensor: SensorFaults,
    /// Inverter path.
    pub inverter: InverterFaults,
}

impl Faults {
    /// No faults.
    pub const NONE: Faults = Faults {
        sensor: SensorFaults::NONE,
        inverter: InverterFaults::NONE,
    };
}

impl Default for Faults {
    fn default() -> Self {
        Faults::NONE
    }
}

/// Sensor faults.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SensorFaults {
    /// Phase current channel frozen at its last reading.
    pub current_stuck: [bool; 3],
    /// Extra phase current offset (drift), A.
    pub current_offset_a: [f32; 3],
    /// Phase current gain, 1.0 healthy.
    pub current_gain: [f32; 3],
    /// Chance per sample and phase that the reading is a random ADC code.
    pub current_glitch_probability: f32,
    /// Extra DC link voltage offset, V.
    pub vdc_offset_v: f32,
    /// Chance per sample that the encoder loses one count (the error accumulates, like real missed edges).
    pub encoder_missed_count_probability: f32,
    /// Hall lines frozen at this pattern (bit 0 = A, 1 = B, 2 = C).
    pub hall_stuck: Option<u8>,
    /// Chance per sample that one hall line reads flipped.
    pub hall_glitch_probability: f32,
}

impl SensorFaults {
    /// No sensor faults.
    pub const NONE: SensorFaults = SensorFaults {
        current_stuck: [false; 3],
        current_offset_a: [0.0; 3],
        current_gain: [1.0; 3],
        current_glitch_probability: 0.0,
        vdc_offset_v: 0.0,
        encoder_missed_count_probability: 0.0,
        hall_stuck: None,
        hall_glitch_probability: 0.0,
    };
}

/// Inverter faults. Ignored while the drive is [`Drive::Off`](crate::Drive::Off).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct InverterFaults {
    /// Per leg a, b, c.
    pub legs: [LegFault; 3],
    /// Chance per step and healthy leg that its pulse is missed (duty 0 for that step).
    pub missed_pulse_probability: f32,
}

impl InverterFaults {
    /// No inverter faults.
    pub const NONE: InverterFaults = InverterFaults {
        legs: [LegFault::Healthy; 3],
        missed_pulse_probability: 0.0,
    };
}

/// State of one inverter leg.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum LegFault {
    /// Switches as commanded.
    #[default]
    Healthy,
    /// Low side always on (duty 0).
    StuckLow,
    /// High side always on (duty 1).
    StuckHigh,
}
