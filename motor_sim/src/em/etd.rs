//! The flux step: exponential (Rosenbrock-Euler) step around the local incremental inductance, in closed form.
//!
//! In the machine's frame (rotating at ω_f: ω for dq, 0 for αβ), with the state `x = ψ − ψm(θ)`:
//!
//! ```text
//! dx/dt = v − R·i − ω_f·J·(x + ψm) − ω·∂ψm/∂θ,   i = I(x, θ),   J = [[0, −1], [1, 0]]
//! ```
//!
//! Linearised at the start of the step, `i ≈ i0 + Γ·(x − x0) + ∂i/∂θ·ω·t` and the magnet terms to first order in
//! `ω·t`, this is `dx/dt = A·x + u(t) + c + d·t` with `A = −R·Γ − ω_f·J` (full 2×2, cross-coupling included). The
//! inverter holds a constant αβ voltage over the step, which in the frame is a sinusoid at −ω_f. With ω frozen the
//! step is a linear time-invariant system: the homogeneous part `e^{A·h}`, the forced response to the sinusoid,
//! the constant and the ramp all have closed forms. For a linear map that is the exact solution for any `h` and
//! stable for any motor (an RK4 step goes unstable once `h·R/L` passes ~2.8); for a saturated map it is the
//! exponential Rosenbrock-Euler method, second order. `tests/discretisation.rs` and `tests/engine.rs` compare it
//! against analytic step responses and fine RK4 references. The new current comes from the map at the new flux.

use super::frame::Frame;
use super::machine::{Linearised, Machine};

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

/// Flux relative to the magnets (V·s) after `h` seconds from `x0`, with the map linearised at `x0` as `lin`, the αβ
/// voltage `v_ab` (V) held, rotor at electrical angle `theta_e` (rad) at the start turning at a constant `omega_e`
/// (rad/s).
#[inline]
pub(crate) fn advance<M: Machine>(
    m: &M,
    x0: [f32; 2],
    lin: &Linearised,
    v_ab: [f32; 2],
    theta_e: f32,
    omega_e: f32,
    h: f32,
) -> [f32; 2] {
    let r = m.rs_ohm();
    let wf = if <M::Frame as Frame>::ROTATING {
        omega_e
    } else {
        0.0
    };
    let [[g11, g12], [g21, g22]] = lin.di_dx;
    let a11 = -r * g11;
    let a12 = -r * g12 + wf;
    let a21 = -r * g21 - wf;
    let a22 = -r * g22;
    let det_a = a11 * a22 - a12 * a21;

    // Constant forcing c (x-independent part of −R·i and the magnet terms) and ramp d.
    let mag = m.magnet(theta_e);
    let off = [
        lin.i[0] - (g11 * x0[0] + g12 * x0[1]),
        lin.i[1] - (g21 * x0[0] + g22 * x0[1]),
    ];
    // −ω_f·J·ψm = (ω_f·ψm_q, −ω_f·ψm_d)
    let mut c = [-r * off[0] + wf * mag.psi[1], -r * off[1] - wf * mag.psi[0]];
    let mut d = [0.0f32; 2];
    if M::ANGLE_DEPENDENT {
        let w = omega_e;
        c[0] -= w * mag.d1[0];
        c[1] -= w * mag.d1[1];
        d = [
            w * (-r * lin.di_dtheta[0] + wf * mag.d1[1] - w * mag.d2[0]),
            w * (-r * lin.di_dtheta[1] - wf * mag.d1[0] - w * mag.d2[1]),
        ];
    }

    // Voltage in the frame: u(t) = a·cos(ω_f t) + b·sin(ω_f t) with a = frame(θ0)·v, b = (a_q, −a_d).
    let a = <M::Frame as Frame>::from_ab(v_ab, theta_e);

    // Forced response to the rotating voltage: x_p(t) = Re(Z·e^{−jω_f t}), Z = −(A + jω_f·I)⁻¹·(a + j·b).
    let w1 = C { re: a[0], im: a[1] };
    let w2 = C {
        re: a[1],
        im: -a[0],
    };
    let m11 = C { re: a11, im: wf };
    let m22 = C { re: a22, im: wf };
    let det_m = m11.mul(m22).sub(C::real(a12 * a21));
    // inv(M) = [[m22, −a12], [−a21, m11]] / det_m; Z = −inv(M)·w
    let z1 = C::real(0.0).sub(m22.mul(w1).sub(C::real(a12).mul(w2)).div(det_m));
    let z2 = C::real(0.0).sub(m11.mul(w2).sub(C::real(a21).mul(w1)).div(det_m));

    // Constant and ramp: x_c(t) = p0 + p1·t with p1 = −A⁻¹·d, p0 = A⁻¹·(p1 − c).
    let inv_a = |v: [f32; 2]| {
        [
            (a22 * v[0] - a12 * v[1]) / det_a,
            (a11 * v[1] - a21 * v[0]) / det_a,
        ]
    };
    let p1 = if M::ANGLE_DEPENDENT {
        let n = inv_a(d);
        [-n[0], -n[1]]
    } else {
        [0.0; 2]
    };
    let p0 = inv_a([p1[0] - c[0], p1[1] - c[1]]);

    // e^{A·h} = e^{m·h}·(f0·I + f1·(A − m·I)), m = tr(A)/2, Δ = m² − det(A). For short steps e^{A·h} and the
    // rotation are close to the identity, so both are formed minus the identity (expm1, half angles) and the step
    // is an increment of x0: f32 rounding then scales with the change over the step, not with the state, and does
    // not pile up over many short steps.
    let mm = 0.5 * (a11 + a22);
    let half_diff = 0.5 * (a11 - a22);
    let delta = half_diff * half_diff + a12 * a21;
    let dh2 = delta * h * h;
    // (f0 − 1, f1)
    let (f0m1, f1) = if libm::fabsf(dh2) < 1e-4 {
        (
            dh2 / 2.0 + dh2 * dh2 / 24.0,
            h * (1.0 + dh2 / 6.0 + dh2 * dh2 / 120.0),
        )
    } else if delta < 0.0 {
        let s = libm::sqrtf(-delta);
        let (sn, cs) = libm::sincosf(0.5 * s * h);
        (-2.0 * sn * sn, 2.0 * sn * cs / s)
    } else {
        let s = libm::sqrtf(delta);
        let (sh, ch) = (libm::sinhf(0.5 * s * h), libm::coshf(0.5 * s * h));
        (2.0 * sh * sh, 2.0 * sh * ch / s)
    };
    let em1 = libm::expm1f(mm * h);
    let emh = 1.0 + em1;
    // E − I: e^{m·h}·f0 − 1 = em1 + (f0 − 1) + em1·(f0 − 1)
    let d0 = em1 + f0m1 + em1 * f0m1;
    let e11 = d0 + emh * f1 * (a11 - mm);
    let e12 = emh * f1 * a12;
    let e21 = emh * f1 * a21;
    let e22 = d0 + emh * f1 * (a22 - mm);

    let (sh, ch) = libm::sincosf(0.5 * wf * h);
    let (sw, cwm1) = (2.0 * sh * ch, -2.0 * sh * sh);
    // x_p(h) − x_p(0) = P·(cos(ω_f h) − 1) + Q·sin(ω_f h) with Z = P + jQ
    let dxp = [z1.re * cwm1 + z1.im * sw, z2.re * cwm1 + z2.im * sw];
    let y0 = [x0[0] - z1.re - p0[0], x0[1] - z2.re - p0[1]];
    [
        x0[0] + (e11 * y0[0] + e12 * y0[1] + dxp[0] + p1[0] * h),
        x0[1] + (e21 * y0[0] + e22 * y0[1] + dxp[1] + p1[1] * h),
    ]
}
