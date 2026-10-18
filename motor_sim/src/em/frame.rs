//! Reference frames of the electromagnetic engine. The frame is a property of the machine's map, not of the engine.

/// Two-axis reference frame a [`Machine`](crate::Machine) keeps its flux and currents in. Both frames here describe
/// a three-phase winding in star without a neutral (no zero sequence); amplitude-invariant transforms.
pub trait Frame {
    /// True for a frame turning with the rotor ([`Dq`]): the flux equation gets the rotation term −ω·J·ψ.
    const ROTATING: bool;
    /// Stationary αβ quantity into this frame at electrical angle `theta_e` (rad).
    fn from_ab(ab: [f32; 2], theta_e: f32) -> [f32; 2];
    /// Quantity in this frame to αβ.
    fn to_ab(x: [f32; 2], theta_e: f32) -> [f32; 2];
    /// dq quantity into this frame.
    fn from_dq(dq: [f32; 2], theta_e: f32) -> [f32; 2];
    /// Quantity in this frame to dq.
    fn to_dq(x: [f32; 2], theta_e: f32) -> [f32; 2];
}

/// Rotor frame: d along the magnet, q 90° electrical ahead. Maps of rotating-field machines (SPM, IPM, SynRM, BLDC,
/// hybrid steppers) are smooth here and depend on the angle only through harmonics.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub struct Dq;

/// Stationary frame (Clarke of the phase quantities). For maps that are naturally per phase (SRM, faulted or
/// unbalanced windings) and to check the engine against the rotor frame.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub struct AlphaBeta;

impl Frame for Dq {
    const ROTATING: bool = true;
    fn from_ab(ab: [f32; 2], theta_e: f32) -> [f32; 2] {
        let (d, q) = park(ab[0], ab[1], theta_e);
        [d, q]
    }
    fn to_ab(x: [f32; 2], theta_e: f32) -> [f32; 2] {
        let (a, b) = inv_park(x[0], x[1], theta_e);
        [a, b]
    }
    fn from_dq(dq: [f32; 2], _theta_e: f32) -> [f32; 2] {
        dq
    }
    fn to_dq(x: [f32; 2], _theta_e: f32) -> [f32; 2] {
        x
    }
}

impl Frame for AlphaBeta {
    const ROTATING: bool = false;
    fn from_ab(ab: [f32; 2], _theta_e: f32) -> [f32; 2] {
        ab
    }
    fn to_ab(x: [f32; 2], _theta_e: f32) -> [f32; 2] {
        x
    }
    fn from_dq(dq: [f32; 2], theta_e: f32) -> [f32; 2] {
        Dq::to_ab(dq, theta_e)
    }
    fn to_dq(x: [f32; 2], theta_e: f32) -> [f32; 2] {
        Dq::from_ab(x, theta_e)
    }
}

/// Park transform: αβ → dq at electrical angle `theta_e` (rad).
pub(crate) fn park(alpha: f32, beta: f32, theta_e: f32) -> (f32, f32) {
    let (s, c) = libm::sincosf(theta_e);
    (alpha * c + beta * s, -alpha * s + beta * c)
}

/// Inverse Park transform: dq → αβ.
pub(crate) fn inv_park(d: f32, q: f32, theta_e: f32) -> (f32, f32) {
    let (s, c) = libm::sincosf(theta_e);
    (d * c - q * s, d * s + q * c)
}
