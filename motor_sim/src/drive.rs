//! Layer 1, drive: what the controller writes. Register-like PWM timers, ADC triggers and shunt topologies come
//! with MS-14; [`Drive`] stays the convenient way to command a step.

/// What the controller applies for one step. Voltage commands go through the same duty stage as
/// [`Drive::Duty`] (min-max zero sequence, i.e. SVPWM), so they saturate at the inverter hexagon like the real
/// thing.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Drive {
    /// High-side duty of legs a, b, c, 0..=1 (clamped).
    Duty([f32; 3]),
    /// Stationary-frame voltage, V (amplitude-invariant Clarke, so |v| up to Vdc/√3 is linear).
    AlphaBeta {
        /// α voltage, V.
        v_alpha_v: f32,
        /// β voltage, V.
        v_beta_v: f32,
    },
    /// Phase-to-neutral voltages a, b, c, V; any common part is ignored (isolated star point).
    PhaseVolts([f32; 3]),
    /// All switches off. L1 treats the phases as open: the currents drop to zero at once (no diode conduction).
    Off,
}
