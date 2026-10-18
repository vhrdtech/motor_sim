//! What the electromagnetic engine needs from a machine: its map between flux and current, as a magnetic two-port.

use super::frame::Frame;

/// Open-circuit flux linkage ψm(θ) of the magnets in the machine's frame and its first two derivatives by the
/// electrical angle, V·s (per rad, per rad²).
#[derive(Clone, Copy, Debug, PartialEq, Default)]
pub struct Magnet {
    /// ψm, V·s.
    pub psi: [f32; 2],
    /// ∂ψm/∂θe, V·s/rad.
    pub d1: [f32; 2],
    /// ∂²ψm/∂θe², V·s/rad².
    pub d2: [f32; 2],
}

/// The current map linearised at one point: the current, the incremental inverse inductance and the angle slope.
#[derive(Clone, Copy, Debug, PartialEq, Default)]
pub struct Linearised {
    /// Current, A.
    pub i: [f32; 2],
    /// Γ = ∂i/∂x, row-major, 1/H (the inverse of the incremental inductance, cross-coupling included).
    pub di_dx: [[f32; 2]; 2],
    /// ∂i/∂θe at constant flux, A/rad.
    pub di_dtheta: [f32; 2],
}

/// A machine for the electromagnetic engine: winding resistance, pole pairs and a map between flux linkage and
/// current in its [`Frame`].
///
/// The engine's state is the flux relative to the magnets, `x = ψ − ψm(θ)` (V·s, phase peak), which keeps the
/// small winding flux apart from the large magnet flux in f32. The machine maps it to the current
/// `i = I(x, θ)`, and the field energy `W̃(x, θ)` with `i = ∂W̃/∂x` gives the torque by co-energy, so the energy
/// balance holds by construction. Energies here are per two-axis unit: the physical energy is 1.5·W̃
/// (amplitude-invariant transform).
pub trait Machine {
    /// The frame the map is stored in.
    type Frame: Frame;
    /// False when nothing depends on the rotor angle (a linear dq machine): the engine skips the angle terms.
    const ANGLE_DEPENDENT: bool = true;

    /// Pole pairs.
    fn pole_pairs(&self) -> u32;
    /// Phase resistance, Ω. Must be > 0.
    fn rs_ohm(&self) -> f32;
    /// Magnet flux and its angle derivatives at electrical angle `theta_e` (rad).
    fn magnet(&self, theta_e: f32) -> Magnet;
    /// Current (A) from the flux relative to the magnets `x` (V·s).
    fn current(&self, x: [f32; 2], theta_e: f32) -> [f32; 2];
    /// Current, Γ = ∂i/∂x and ∂i/∂θ at `x`.
    fn linearise(&self, x: [f32; 2], theta_e: f32) -> Linearised;
    /// Field energy W̃(x, θ), J per two-axis unit (physical: 1.5·W̃).
    fn field_energy(&self, x: [f32; 2], theta_e: f32) -> f32;
    /// ∂W̃/∂θe at constant `x`, J/rad per two-axis unit: the cogging and reluctance part of the torque.
    fn field_energy_dtheta(&self, x: [f32; 2], theta_e: f32) -> f32;

    /// The flux relative to the magnets that carries current `i` (A): the forward map. By default a Newton solve
    /// on [`Machine::linearise`] from `x = 0`.
    fn flux(&self, i: [f32; 2], theta_e: f32) -> [f32; 2] {
        let mut x = [0.0f32; 2];
        for _ in 0..12 {
            let l = self.linearise(x, theta_e);
            let r = [i[0] - l.i[0], i[1] - l.i[1]];
            let [[a, b], [c, d]] = l.di_dx;
            let det = a * d - b * c;
            if det == 0.0 || !det.is_finite() {
                break;
            }
            let dx = [(d * r[0] - b * r[1]) / det, (a * r[1] - c * r[0]) / det];
            x = [x[0] + dx[0], x[1] + dx[1]];
            if libm::fabsf(dx[0]) + libm::fabsf(dx[1])
                <= 1e-7 * (libm::fabsf(x[0]) + libm::fabsf(x[1]))
            {
                break;
            }
        }
        x
    }
}

/// Electromagnetic torque (N·m) from the co-energy: (3/2)·p·([rotating] ψ×i + i·∂ψm/∂θ − ∂W̃/∂θ|x).
#[inline]
pub(crate) fn torque<M: Machine>(m: &M, x: [f32; 2], i: [f32; 2], theta_e: f32) -> f32 {
    let mag = m.magnet(theta_e);
    let mut t = 0.0;
    if <M::Frame as Frame>::ROTATING {
        t += (x[0] + mag.psi[0]) * i[1] - (x[1] + mag.psi[1]) * i[0];
    }
    if M::ANGLE_DEPENDENT {
        t += i[0] * mag.d1[0] + i[1] * mag.d1[1] - m.field_energy_dtheta(x, theta_e);
    }
    1.5 * m.pole_pairs() as f32 * t
}
