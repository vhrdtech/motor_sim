//! The electrical step against analytic step responses, next to an RK4 integrator of the same equations. This is
//! the comparison behind the choice of the exact per-step solution (see `src/electrical.rs`).

use motor_sim::{Drive, Load, Motor, MotorParams, PmsmParams, presets};

const DT: f32 = 1.0 / 25_000.0;

/// RK4 on the dq equations with ω frozen and the αβ voltage held (so the dq voltage rotates), `n` substeps.
fn rk4(
    p: &PmsmParams,
    i: (f64, f64),
    v_ab: (f64, f64),
    theta0: f64,
    w: f64,
    h: f64,
    n: u32,
) -> (f64, f64) {
    let (r, ld, lq, lam) = (
        p.rs_ohm as f64,
        p.ld_h as f64,
        p.lq_h as f64,
        p.flux_linkage_wb as f64,
    );
    let f = |t: f64, x: (f64, f64)| {
        let th = theta0 + w * t;
        let (s, c) = th.sin_cos();
        let vd = v_ab.0 * c + v_ab.1 * s;
        let vq = -v_ab.0 * s + v_ab.1 * c;
        (
            (vd - r * x.0 + w * lq * x.1) / ld,
            (vq - r * x.1 - w * ld * x.0 - w * lam) / lq,
        )
    };
    let hs = h / n as f64;
    let mut x = i;
    for k in 0..n {
        let t = k as f64 * hs;
        let k1 = f(t, x);
        let k2 = f(t + hs / 2.0, (x.0 + hs / 2.0 * k1.0, x.1 + hs / 2.0 * k1.1));
        let k3 = f(t + hs / 2.0, (x.0 + hs / 2.0 * k2.0, x.1 + hs / 2.0 * k2.1));
        let k4 = f(t + hs, (x.0 + hs * k3.0, x.1 + hs * k3.1));
        x = (
            x.0 + hs / 6.0 * (k1.0 + 2.0 * k2.0 + 2.0 * k3.0 + k4.0),
            x.1 + hs / 6.0 * (k1.1 + 2.0 * k2.1 + 2.0 * k3.1 + k4.1),
        );
    }
    x
}

/// Motor spinning at `omega_m` (held by a dyno) with zero current; returns it with the drive not applied yet.
fn spinning(params: MotorParams, omega_m: f32) -> Motor {
    let mut m = Motor::new(params, 1);
    m.set_load(Load::Speed {
        omega_m_rad_s: omega_m,
    });
    m.step(&Drive::Off, DT);
    m
}

/// Run `steps` PWM periods of a constant αβ voltage through the motor and through RK4 with `n` substeps, both
/// from the same start; returns (motor αβ currents, RK4 αβ currents) after each step.
fn run_both(
    m: &mut Motor,
    v_ab: (f32, f32),
    steps: usize,
    n: u32,
) -> Vec<((f64, f64), (f64, f64))> {
    let p = m.params().pmsm;
    let w = (p.pole_pairs as f32 * m.truth().omega_m) as f64;
    let mut x = (0.0, 0.0);
    let mut out = vec![];
    for _ in 0..steps {
        let th = m.truth().theta_e as f64;
        x = rk4(&p, x, (v_ab.0 as f64, v_ab.1 as f64), th, w, DT as f64, n);
        m.step(
            &Drive::AlphaBeta {
                v_alpha_v: v_ab.0,
                v_beta_v: v_ab.1,
            },
            DT,
        );
        let th1 = th + w * DT as f64;
        let rk_ab = (
            x.0 * th1.cos() - x.1 * th1.sin(),
            x.0 * th1.sin() + x.1 * th1.cos(),
        );
        let t = m.truth();
        out.push(((t.i_alpha as f64, t.i_beta as f64), rk_ab));
    }
    out
}

#[test]
fn locked_rotor_step_matches_rl_response() {
    let params = presets::vesc_hobby_outrunner();
    let p = params.pmsm;
    let mut m = Motor::new(params, 1);
    m.set_load(Load::Locked);
    let v = 0.3f32;
    let tau = (p.ld_h / p.rs_ohm) as f64;
    let res = run_both(&mut m, (v, 0.0), 50, 1);
    let (mut err_exact, mut err_rk4) = (0.0f64, 0.0f64);
    for (k, (exact, rk)) in res.iter().enumerate() {
        let t = (k + 1) as f64 * DT as f64;
        let analytic = v as f64 / p.rs_ohm as f64 * (1.0 - (-t / tau).exp());
        err_exact = err_exact.max((exact.0 - analytic).abs() + exact.1.abs());
        err_rk4 = err_rk4.max((rk.0 - analytic).abs() + rk.1.abs());
    }
    eprintln!(
        "locked rotor, h/τ = {:.3}: exact {err_exact:.2e} A, RK4×1 {err_rk4:.2e} A",
        DT as f64 / tau
    );
    let i_final = v / p.rs_ohm;
    assert!(err_exact < 2e-6 * i_final as f64);
}

/// SPM at constant speed, αβ voltage step from zero current. In complex αβ: L·di/dt = v − R·i − jωλ·e^{jθ(t)},
/// so i(t) = v/R·(1 − e^{−t/τ}) + K·(e^{jθ(t)} − e^{jθ0}·e^{−t/τ}) with K = −jωλ/(R + jωL).
#[test]
fn spm_at_speed_step_matches_analytic() {
    let params = presets::vesc_hobby_outrunner();
    let p = params.pmsm;
    let omega_m = 600.0; // ≈ 5700 rpm, ωe = 4200 rad/s: ωe·h = 0.17 rad per PWM period
    let th0 = spinning(params, omega_m).truth().theta_e as f64;
    let (r, l, lam) = (p.rs_ohm as f64, p.ld_h as f64, p.flux_linkage_wb as f64);
    let w = p.pole_pairs as f64 * omega_m as f64;
    let v = (2.0f32, 1.0f32);
    for n in [1u32, 2, 4] {
        let mut m = spinning(params, omega_m);
        let res = run_both(&mut m, v, 100, n);
        let (mut err_exact, mut err_rk4, mut peak) = (0.0f64, 0.0f64, 0.0f64);
        for (k, (exact, rk)) in res.iter().enumerate() {
            let t = (k + 1) as f64 * DT as f64;
            let decay = (-t * r / l).exp();
            // K = −jωλ/(R + jωL)
            let den = r * r + w * w * l * l;
            let kre = -w * lam * w * l / den;
            let kim = -w * lam * r / den;
            let (s, c) = (th0 + w * t).sin_cos();
            let (s0, c0) = th0.sin_cos();
            let ere = c - c0 * decay;
            let eim = s - s0 * decay;
            let a = (
                v.0 as f64 / r * (1.0 - decay) + kre * ere - kim * eim,
                v.1 as f64 / r * (1.0 - decay) + kre * eim + kim * ere,
            );
            peak = peak.max(a.0.hypot(a.1));
            err_exact = err_exact.max((exact.0 - a.0).hypot(exact.1 - a.1));
            err_rk4 = err_rk4.max((rk.0 - a.0).hypot(rk.1 - a.1));
        }
        eprintln!(
            "SPM at ωe·h = {:.2} rad, peak {peak:.1} A: exact {err_exact:.2e} A, RK4×{n} {err_rk4:.2e} A",
            w * DT as f64
        );
        // The exact step sits at the f32 rounding floor; RK4 (here in f64) only gets there with substeps.
        assert!(err_exact < 2e-6 * peak, "exact error {err_exact} A");
        if n == 1 {
            assert!(err_exact < err_rk4);
        }
    }
}

/// IPM (Ld ≠ Lq) at speed against a fine RK4 reference (2000 substeps per step, f64).
#[test]
fn ipm_at_speed_matches_fine_reference() {
    let params = presets::ipm();
    let omega_m = 1000.0; // ωe = 4000 rad/s
    let v = (-10.0f32, 20.0f32);
    let mut reference = spinning(params, omega_m);
    let reference = run_both(&mut reference, v, 100, 2000);
    for n in [1u32, 4] {
        let mut m = spinning(params, omega_m);
        let res = run_both(&mut m, v, 100, n);
        let (mut err_exact, mut err_rk4, mut peak) = (0.0f64, 0.0f64, 0.0f64);
        for ((exact, rk), (_, fine)) in res.iter().zip(&reference) {
            peak = peak.max(fine.0.hypot(fine.1));
            err_exact = err_exact.max((exact.0 - fine.0).hypot(exact.1 - fine.1));
            err_rk4 = err_rk4.max((rk.0 - fine.0).hypot(rk.1 - fine.1));
        }
        eprintln!("IPM, peak {peak:.1} A: exact {err_exact:.2e} A, RK4×{n} {err_rk4:.2e} A");
        // f32 rounding of the steady-state terms (~λ/Ld) accumulates to ~1e-5 of the peak over 100 steps; the same
        // formula in f64 matches RK4 to 1e-13.
        assert!(err_exact < 2e-5 * peak, "exact error {err_exact} A");
    }
}

/// A low-inductance motor where one PWM period is several electrical time constants: RK4 with one substep blows
/// up, the exact step does not.
#[test]
fn stiff_motor_stays_stable() {
    let mut params = presets::vesc_hobby_outrunner();
    params.pmsm.ld_h = 1e-6;
    params.pmsm.lq_h = 1e-6;
    params.pmsm.rs_ohm = 0.12; // h·R/L = 4.8
    let mut m = Motor::new(params, 1);
    m.set_load(Load::Locked);
    let res = run_both(&mut m, (0.6, 0.0), 50, 1);
    let (exact, rk) = res.last().unwrap();
    eprintln!(
        "stiff: exact {:.3} A, RK4×1 {:.3e} A, expected 5 A",
        exact.0, rk.0
    );
    assert!((exact.0 - 5.0).abs() < 1e-3);
    assert!(rk.0.abs() > 1e3 || rk.0.is_nan());
}
