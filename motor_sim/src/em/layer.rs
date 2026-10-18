//! The electromagnetics layer: flux state, one exponential step per base substep, current and torque out.

use super::{Frame, Machine, advance, torque};
use crate::mechanics::RotorMotion;
use crate::power::WindingVolts;

/// What the electromagnetics hands on for one base substep (plain data).
#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct EmStep {
    /// αβ current at the start and the end of the substep, A.
    pub i_ab: [[f32; 2]; 2],
    /// Electromagnetic torque averaged over the substep, N·m.
    pub torque_nm: f32,
    /// Energy the winding took from the inverter, J (trapezoid of v·i).
    pub energy_in_j: f32,
    /// Energy lost in the winding resistance, J.
    pub copper_j: f32,
}

/// State of the electromagnetics layer.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct Electromagnetics {
    /// Flux linkage minus the magnet flux, in the machine's frame, V·s.
    flux: [f32; 2],
    /// The machine may have changed: re-derive the flux from the currents before the next step, so the currents
    /// stay continuous.
    stale: bool,
}

impl Electromagnetics {
    /// No current at rotor angle 0.
    pub fn new<M: Machine>(m: &M) -> Self {
        Electromagnetics {
            flux: m.flux([0.0; 2], 0.0),
            stale: false,
        }
    }

    pub fn flux(&self) -> [f32; 2] {
        self.flux
    }

    pub fn mark_stale(&mut self) {
        self.stale = true;
    }

    /// After a change of the machine: the flux that carries the dq current `i_dq` (A) at `theta_e`.
    pub fn sync<M: Machine>(&mut self, m: &M, i_dq: [f32; 2], theta_e: f32) {
        if self.stale {
            let i = <M::Frame as Frame>::from_dq(i_dq, theta_e);
            self.flux = m.flux(i, theta_e);
            self.stale = false;
        }
    }

    /// One base substep of `h` seconds.
    #[inline]
    pub fn step<M: Machine>(&mut self, m: &M, v: &WindingVolts, r: &RotorMotion, h: f32) -> EmStep {
        let (theta0, omega_e) = (r.theta_e, r.omega_e);
        let theta1 = theta0 + omega_e * h;
        let x0 = self.flux;
        let (x1, i0, i1) = if v.on {
            let lin = m.linearise(x0, theta0);
            let x1 = advance(m, x0, &lin, v.v_ab, theta0, omega_e, h);
            (x1, lin.i, m.current(x1, theta1))
        } else {
            // Phases open: the currents drop to zero at once.
            (m.flux([0.0; 2], theta1), m.current(x0, theta0), [0.0; 2])
        };
        self.flux = x1;
        let ab0 = <M::Frame as Frame>::to_ab(i0, theta0);
        let ab1 = <M::Frame as Frame>::to_ab(i1, theta1);
        let va = v.v_ab;
        EmStep {
            i_ab: [ab0, ab1],
            torque_nm: 0.5 * (torque(m, x0, i0, theta0) + torque(m, x1, i1, theta1)),
            energy_in_j: 1.5 * h * (va[0] * (ab0[0] + ab1[0]) + va[1] * (ab0[1] + ab1[1])) / 2.0,
            copper_j: 0.75
                * m.rs_ohm()
                * h
                * (i0[0] * i0[0] + i0[1] * i0[1] + i1[0] * i1[0] + i1[1] * i1[1]),
        }
    }

    /// Current in the machine's frame (A) and torque (N·m) at `theta_e`.
    pub fn outputs<M: Machine>(&self, m: &M, theta_e: f32) -> ([f32; 2], f32) {
        let i = m.current(self.flux, theta_e);
        (i, torque(m, self.flux, i, theta_e))
    }

    /// Energy in the winding field, J.
    pub fn field_energy_j<M: Machine>(&self, m: &M, theta_e: f32) -> f64 {
        1.5 * m.field_energy(self.flux, theta_e) as f64
    }
}
