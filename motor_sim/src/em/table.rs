//! Machines from tables: the inverse flux map i = I(x, θ) and the field energy on uniform grids.

use core::marker::PhantomData;

use super::frame::{Dq, Frame};
use super::machine::{Linearised, Machine, Magnet};
use crate::grid::{Grid2, Grid3};

/// A table over the flux `x` (two axes, V·s) and maybe the electrical angle: [`Grid2`] ignores the angle, a
/// [`Grid3`] has it as its third (normally periodic) axis.
pub trait Field {
    /// Value and gradient (∂/∂x0, ∂/∂x1, ∂/∂θe) at `x`, `theta_e`.
    fn sample(&self, x: [f32; 2], theta_e: f32) -> (f32, [f32; 3]);
    /// True if the table depends on the angle.
    const ANGLE_DEPENDENT: bool;
}

impl Field for Grid2<'_> {
    const ANGLE_DEPENDENT: bool = false;
    #[inline]
    fn sample(&self, x: [f32; 2], _theta_e: f32) -> (f32, [f32; 3]) {
        let (v, g) = self.eval_grad(x[0], x[1]);
        (v, [g[0], g[1], 0.0])
    }
}

impl Field for Grid3<'_> {
    const ANGLE_DEPENDENT: bool = true;
    #[inline]
    fn sample(&self, x: [f32; 2], theta_e: f32) -> (f32, [f32; 3]) {
        self.eval_grad(x[0], x[1], theta_e)
    }
}

/// A machine from tables (saturation, cross-saturation, harmonics, cogging), stored in frame `F` ([`Dq`] by
/// default): the two current components and the field energy over the flux relative to a constant magnet flux.
///
/// The tables should come from one field-energy function (`i = ∂W̃/∂x`) so torque and energy agree; MDL builds
/// them offline from FEM or analytic generators. The engine linearises the current tables by their in-cell
/// gradient; flux beyond the table extrapolates the edge cells. The angle derivative of the magnet flux is zero:
/// everything angle-dependent lives in the tables.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MapMachine<G, F = Dq> {
    /// Pole pairs.
    pub pole_pairs: u32,
    /// Phase resistance, Ω. Must be > 0.
    pub rs_ohm: f32,
    /// Constant magnet flux the tables' flux axes are relative to, in the frame, V·s (λ along d for a PMSM).
    pub magnet_wb: [f32; 2],
    /// Current components, A, over (x0, x1[, θe]).
    pub current_a: [G; 2],
    /// Field energy W̃ with `i = ∂W̃/∂x`, J per two-axis unit (physical: 1.5·W̃).
    pub energy_j: G,
    frame: PhantomData<F>,
}

impl<G: Field, F: Frame> MapMachine<G, F> {
    /// A machine from its tables.
    pub fn new(
        pole_pairs: u32,
        rs_ohm: f32,
        magnet_wb: [f32; 2],
        current_a: [G; 2],
        energy_j: G,
    ) -> Self {
        MapMachine {
            pole_pairs,
            rs_ohm,
            magnet_wb,
            current_a,
            energy_j,
            frame: PhantomData,
        }
    }
}

impl<G: Field, F: Frame> Machine for MapMachine<G, F> {
    type Frame = F;
    const ANGLE_DEPENDENT: bool = G::ANGLE_DEPENDENT;

    fn pole_pairs(&self) -> u32 {
        self.pole_pairs
    }
    fn rs_ohm(&self) -> f32 {
        self.rs_ohm
    }
    fn magnet(&self, _theta_e: f32) -> Magnet {
        Magnet {
            psi: self.magnet_wb,
            ..Magnet::default()
        }
    }
    fn current(&self, x: [f32; 2], theta_e: f32) -> [f32; 2] {
        [
            self.current_a[0].sample(x, theta_e).0,
            self.current_a[1].sample(x, theta_e).0,
        ]
    }
    fn linearise(&self, x: [f32; 2], theta_e: f32) -> Linearised {
        let (i0, g0) = self.current_a[0].sample(x, theta_e);
        let (i1, g1) = self.current_a[1].sample(x, theta_e);
        Linearised {
            i: [i0, i1],
            di_dx: [[g0[0], g0[1]], [g1[0], g1[1]]],
            di_dtheta: [g0[2], g1[2]],
        }
    }
    fn field_energy(&self, x: [f32; 2], theta_e: f32) -> f32 {
        self.energy_j.sample(x, theta_e).0
    }
    fn field_energy_dtheta(&self, x: [f32; 2], theta_e: f32) -> f32 {
        self.energy_j.sample(x, theta_e).1[2]
    }
}
