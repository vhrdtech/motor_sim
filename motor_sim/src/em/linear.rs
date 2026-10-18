//! Linear machines: constant inductances, sinusoidal magnet flux.

use super::frame::{AlphaBeta, Dq};
use super::machine::{Linearised, Machine, Magnet};
use crate::params::PmsmParams;

/// MS-1's machine: dq, diagonal L (Ld, Lq), magnet flux λ along d.
impl Machine for PmsmParams {
    type Frame = Dq;
    const ANGLE_DEPENDENT: bool = false;

    fn pole_pairs(&self) -> u32 {
        self.pole_pairs
    }
    fn rs_ohm(&self) -> f32 {
        self.rs_ohm
    }
    #[inline]
    fn magnet(&self, _theta_e: f32) -> Magnet {
        Magnet {
            psi: [self.flux_linkage_wb, 0.0],
            ..Magnet::default()
        }
    }
    #[inline]
    fn current(&self, x: [f32; 2], _theta_e: f32) -> [f32; 2] {
        // The same products as `linearise`, so i − Γ·x is exactly zero there.
        [(1.0 / self.ld_h) * x[0], (1.0 / self.lq_h) * x[1]]
    }
    #[inline]
    fn linearise(&self, x: [f32; 2], theta_e: f32) -> Linearised {
        Linearised {
            i: self.current(x, theta_e),
            di_dx: [[1.0 / self.ld_h, 0.0], [0.0, 1.0 / self.lq_h]],
            di_dtheta: [0.0; 2],
        }
    }
    fn field_energy(&self, x: [f32; 2], _theta_e: f32) -> f32 {
        0.5 * (x[0] * x[0] / self.ld_h + x[1] * x[1] / self.lq_h)
    }
    fn field_energy_dtheta(&self, _x: [f32; 2], _theta_e: f32) -> f32 {
        0.0
    }
    #[inline]
    fn flux(&self, i: [f32; 2], _theta_e: f32) -> [f32; 2] {
        [self.ld_h * i[0], self.lq_h * i[1]]
    }
}

/// A dq machine with a constant full inductance matrix: `ψd = Ld·id + Ldq·iq + λ`, `ψq = Ldq·id + Lq·iq`. The mutual
/// term `Ldq` is the linearised cross-saturation of an IPM; with `Ldq = 0` this is [`PmsmParams`].
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct LinearDq {
    /// Pole pairs.
    pub pole_pairs: u32,
    /// Phase resistance, Ω. Must be > 0.
    pub rs_ohm: f32,
    /// d-axis inductance, H.
    pub ld_h: f32,
    /// q-axis inductance, H.
    pub lq_h: f32,
    /// Mutual dq inductance, H; `Ldq² < Ld·Lq`.
    pub ldq_h: f32,
    /// Permanent magnet flux linkage, Wb (phase peak).
    pub flux_linkage_wb: f32,
}

impl LinearDq {
    /// The machine of `p` with a mutual inductance `ldq_h` (H).
    pub fn from_pmsm(p: &PmsmParams, ldq_h: f32) -> Self {
        LinearDq {
            pole_pairs: p.pole_pairs,
            rs_ohm: p.rs_ohm,
            ld_h: p.ld_h,
            lq_h: p.lq_h,
            ldq_h,
            flux_linkage_wb: p.flux_linkage_wb,
        }
    }

    /// Γ = L⁻¹.
    fn gamma(&self) -> [[f32; 2]; 2] {
        let det = self.ld_h * self.lq_h - self.ldq_h * self.ldq_h;
        let (a, b) = (self.lq_h / det, -self.ldq_h / det);
        [[a, b], [b, self.ld_h / det]]
    }
}

impl Machine for LinearDq {
    type Frame = Dq;
    const ANGLE_DEPENDENT: bool = false;

    fn pole_pairs(&self) -> u32 {
        self.pole_pairs
    }
    fn rs_ohm(&self) -> f32 {
        self.rs_ohm
    }
    fn magnet(&self, _theta_e: f32) -> Magnet {
        Magnet {
            psi: [self.flux_linkage_wb, 0.0],
            ..Magnet::default()
        }
    }
    fn current(&self, x: [f32; 2], _theta_e: f32) -> [f32; 2] {
        mul(self.gamma(), x)
    }
    fn linearise(&self, x: [f32; 2], _theta_e: f32) -> Linearised {
        let g = self.gamma();
        Linearised {
            i: mul(g, x),
            di_dx: g,
            di_dtheta: [0.0; 2],
        }
    }
    fn field_energy(&self, x: [f32; 2], theta_e: f32) -> f32 {
        let i = self.current(x, theta_e);
        0.5 * (x[0] * i[0] + x[1] * i[1])
    }
    fn field_energy_dtheta(&self, _x: [f32; 2], _theta_e: f32) -> f32 {
        0.0
    }
    fn flux(&self, i: [f32; 2], _theta_e: f32) -> [f32; 2] {
        mul([[self.ld_h, self.ldq_h], [self.ldq_h, self.lq_h]], i)
    }
}

/// [`PmsmParams`] in the stationary frame: `ψαβ = L(θ)·i + λ·(cos θ, sin θ)` with the saliency rotating at 2θ.
/// Physically the same machine as in dq; the engine solves it to second order in the step (the inductance and the
/// back-EMF turn during the step), which makes it the check that the engine is generic over its frame.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct LinearAlphaBeta(pub PmsmParams);

impl LinearAlphaBeta {
    /// Γ(θ) = L(θ)⁻¹ and its derivative by θ.
    fn gamma(&self, theta_e: f32) -> ([[f32; 2]; 2], [[f32; 2]; 2]) {
        let p = &self.0;
        let (gd, gq) = (1.0 / p.ld_h, 1.0 / p.lq_h);
        let (sum, dif) = (0.5 * (gd + gq), 0.5 * (gd - gq));
        let (s2, c2) = libm::sincosf(2.0 * theta_e);
        (
            [[sum + dif * c2, dif * s2], [dif * s2, sum - dif * c2]],
            [
                [-2.0 * dif * s2, 2.0 * dif * c2],
                [2.0 * dif * c2, 2.0 * dif * s2],
            ],
        )
    }
}

impl Machine for LinearAlphaBeta {
    type Frame = AlphaBeta;

    fn pole_pairs(&self) -> u32 {
        self.0.pole_pairs
    }
    fn rs_ohm(&self) -> f32 {
        self.0.rs_ohm
    }
    fn magnet(&self, theta_e: f32) -> Magnet {
        let (s, c) = libm::sincosf(theta_e);
        let l = self.0.flux_linkage_wb;
        Magnet {
            psi: [l * c, l * s],
            d1: [-l * s, l * c],
            d2: [-l * c, -l * s],
        }
    }
    fn current(&self, x: [f32; 2], theta_e: f32) -> [f32; 2] {
        mul(self.gamma(theta_e).0, x)
    }
    fn linearise(&self, x: [f32; 2], theta_e: f32) -> Linearised {
        let (g, dg) = self.gamma(theta_e);
        Linearised {
            i: mul(g, x),
            di_dx: g,
            di_dtheta: mul(dg, x),
        }
    }
    fn field_energy(&self, x: [f32; 2], theta_e: f32) -> f32 {
        let i = self.current(x, theta_e);
        0.5 * (x[0] * i[0] + x[1] * i[1])
    }
    fn field_energy_dtheta(&self, x: [f32; 2], theta_e: f32) -> f32 {
        let d = mul(self.gamma(theta_e).1, x);
        0.5 * (x[0] * d[0] + x[1] * d[1])
    }
    fn flux(&self, i: [f32; 2], theta_e: f32) -> [f32; 2] {
        let p = &self.0;
        let (sum, dif) = (0.5 * (p.ld_h + p.lq_h), 0.5 * (p.ld_h - p.lq_h));
        let (s2, c2) = libm::sincosf(2.0 * theta_e);
        mul([[sum + dif * c2, dif * s2], [dif * s2, sum - dif * c2]], i)
    }
}

#[inline]
pub(crate) fn mul(m: [[f32; 2]; 2], v: [f32; 2]) -> [f32; 2] {
    [
        m[0][0] * v[0] + m[0][1] * v[1],
        m[1][0] * v[0] + m[1][1] * v[1],
    ]
}
