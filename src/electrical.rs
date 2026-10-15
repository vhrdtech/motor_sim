//! dq PMSM electrical equations, solved exactly over one step with the electrical speed held constant.
//!
//! ```text
//! Ld·di_d/dt = v_d − R·i_d + ω·Lq·i_q
//! Lq·di_q/dt = v_q − R·i_q − ω·Ld·i_d − ω·λ
//! ```
//!
//! With ω frozen this is a linear time-invariant 2×2 system. The inverter holds a constant αβ voltage over the
//! step, which turns into a dq voltage rotating at −ω; the forced response to that sinusoid, the constant back-EMF
//! term and the homogeneous part `e^{A·h}` all have closed forms, so the step is exact for any `h` and stable for
//! any motor (an RK4 step goes unstable once `h·R/L` passes ~2.8). `tests/discretisation.rs` compares both against
//! analytic step responses.

use crate::params::PmsmParams;

#[derive(Clone, Copy)]
struct C {
    re: f32,
    im: f32,
}

impl C {
    fn mul(self, o: C) -> C {
        C {
            re: self.re * o.re - self.im * o.im,
            im: self.re * o.im + self.im * o.re,
        }
    }
    fn div(self, o: C) -> C {
        let n = o.re * o.re + o.im * o.im;
        C {
            re: (self.re * o.re + self.im * o.im) / n,
            im: (self.im * o.re - self.re * o.im) / n,
        }
    }
    fn sub(self, o: C) -> C {
        C {
            re: self.re - o.re,
            im: self.im - o.im,
        }
    }
    fn real(re: f32) -> C {
        C { re, im: 0.0 }
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

/// Electromagnetic torque, N·m.
pub(crate) fn torque(p: &PmsmParams, i_d: f32, i_q: f32) -> f32 {
    1.5 * p.pole_pairs as f32 * (p.flux_linkage_wb * i_q + (p.ld_h - p.lq_h) * i_d * i_q)
}

/// dq currents (A) after `h` seconds from `i_dq`, with the αβ voltage `v_ab` (V) held, rotor at electrical angle
/// `theta_e` (rad) at the start and turning at a constant `omega_e` (rad/s).
pub(crate) fn step_exact(
    p: &PmsmParams,
    i_dq: (f32, f32),
    v_ab: (f32, f32),
    theta_e: f32,
    omega_e: f32,
    h: f32,
) -> (f32, f32) {
    let (r, ld, lq, w) = (p.rs_ohm, p.ld_h, p.lq_h, omega_e);
    let a11 = -r / ld;
    let a12 = w * lq / ld;
    let a21 = -w * ld / lq;
    let a22 = -r / lq;
    let det_a = a11 * a22 - a12 * a21;

    // Voltage in dq: u(t) = a·cos(ωt) + b·sin(ωt) with a = Park(θ0)·v, b = (a_q, −a_d).
    let (vd0, vq0) = park(v_ab.0, v_ab.1, theta_e);

    // Forced response to the rotating voltage: x_p(t) = Re(Z·e^{−jωt}), Z = −(A + jωI)⁻¹·B·(a + j·b).
    let w1 = C {
        re: vd0 / ld,
        im: vq0 / ld,
    };
    let w2 = C {
        re: vq0 / lq,
        im: -vd0 / lq,
    };
    let m11 = C { re: a11, im: w };
    let m22 = C { re: a22, im: w };
    let det_m = m11.mul(m22).sub(C::real(a12 * a21));
    // inv(M) = [[m22, −a12], [−a21, m11]] / det_m; Z = −inv(M)·w
    let z1 = C::real(0.0).sub(m22.mul(w1).sub(C::real(a12).mul(w2)).div(det_m));
    let z2 = C::real(0.0).sub(m11.mul(w2).sub(C::real(a21).mul(w1)).div(det_m));

    // Constant back-EMF term c = (0, −ω·λ/Lq): steady part x_c = −A⁻¹·c.
    let c2 = -w * p.flux_linkage_wb / lq;
    let xc = (a12 * c2 / det_a, -a11 * c2 / det_a);

    // e^{A·h} = e^{m·h}·(f0·I + f1·(A − m·I)), m = tr(A)/2, Δ = m² − det(A).
    let m = 0.5 * (a11 + a22);
    let half_diff = 0.5 * (a11 - a22);
    let delta = half_diff * half_diff + a12 * a21;
    let dh2 = delta * h * h;
    let (f0, f1) = if libm::fabsf(dh2) < 1e-4 {
        (
            1.0 + dh2 / 2.0 + dh2 * dh2 / 24.0,
            h * (1.0 + dh2 / 6.0 + dh2 * dh2 / 120.0),
        )
    } else if delta < 0.0 {
        let s = libm::sqrtf(-delta);
        let (sn, cs) = libm::sincosf(s * h);
        (cs, sn / s)
    } else {
        let s = libm::sqrtf(delta);
        (libm::coshf(s * h), libm::sinhf(s * h) / s)
    };
    let emh = libm::expf(m * h);
    let e11 = emh * (f0 + f1 * (a11 - m));
    let e12 = emh * f1 * a12;
    let e21 = emh * f1 * a21;
    let e22 = emh * (f0 + f1 * (a22 - m));

    let (sw, cw) = libm::sincosf(w * h);
    // x_p(h) = P·cos(ωh) + Q·sin(ωh) with Z = P + jQ
    let xp_h = (z1.re * cw + z1.im * sw, z2.re * cw + z2.im * sw);
    let y0 = (i_dq.0 - z1.re - xc.0, i_dq.1 - z2.re - xc.1);
    (
        e11 * y0.0 + e12 * y0.1 + xp_h.0 + xc.0,
        e21 * y0.0 + e22 * y0.1 + xp_h.1 + xc.1,
    )
}
