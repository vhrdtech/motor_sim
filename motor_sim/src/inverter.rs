//! Average inverter: what the controller commands, turned into leg duties and then into phase voltages.

use crate::faults::{InverterFaults, LegFault};
use crate::rng::Rng;

const SQRT3_2: f32 = 0.866_025_4;
const INV_SQRT3: f32 = 0.577_350_3;

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

/// Leg duties for `drive`, before faults. `None` for [`Drive::Off`].
pub(crate) fn duties(drive: &Drive, vdc_v: f32) -> Option<[f32; 3]> {
    let v = match *drive {
        Drive::Off => return None,
        Drive::Duty(d) => return Some(d.map(clamp_duty)),
        Drive::AlphaBeta {
            v_alpha_v,
            v_beta_v,
        } => [
            v_alpha_v,
            -0.5 * v_alpha_v + SQRT3_2 * v_beta_v,
            -0.5 * v_alpha_v - SQRT3_2 * v_beta_v,
        ],
        Drive::PhaseVolts(v) => v,
    };
    let max = v[0].max(v[1]).max(v[2]);
    let min = v[0].min(v[1]).min(v[2]);
    let zero_seq = -0.5 * (max + min);
    Some(v.map(|x| clamp_duty(0.5 + (x + zero_seq) / vdc_v)))
}

/// Clamp to 0..=1; NaN becomes 0.
fn clamp_duty(d: f32) -> f32 {
    if d.is_nan() { 0.0 } else { d.clamp(0.0, 1.0) }
}

/// Apply inverter faults to the leg duties. Draws one random number per leg every call.
pub(crate) fn apply_faults(d: [f32; 3], f: &InverterFaults, rng: &mut Rng) -> [f32; 3] {
    let mut out = d;
    for (k, x) in out.iter_mut().enumerate() {
        let missed = rng.chance(f.missed_pulse_probability);
        *x = match f.legs[k] {
            LegFault::Healthy if missed => 0.0,
            LegFault::Healthy => *x,
            LegFault::StuckLow => 0.0,
            LegFault::StuckHigh => 1.0,
        };
    }
    out
}

/// Phase-to-neutral voltages (V) of a balanced star from leg duties.
pub(crate) fn phase_volts(d: [f32; 3], vdc_v: f32) -> [f32; 3] {
    let mean = (d[0] + d[1] + d[2]) / 3.0;
    d.map(|x| vdc_v * (x - mean))
}

/// Amplitude-invariant Clarke transform of three phase quantities.
pub(crate) fn clarke(x: [f32; 3]) -> (f32, f32) {
    ((2.0 * x[0] - x[1] - x[2]) / 3.0, (x[1] - x[2]) * INV_SQRT3)
}

/// Inverse Clarke transform.
pub(crate) fn inv_clarke(alpha: f32, beta: f32) -> [f32; 3] {
    [
        alpha,
        -0.5 * alpha + SQRT3_2 * beta,
        -0.5 * alpha - SQRT3_2 * beta,
    ]
}
