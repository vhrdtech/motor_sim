//! The layered engine (MS-7): linear machines reproduce MS-1, a full inductance matrix and saturated flux maps
//! against fine RK4 references at step sizes from 1 to 50 µs, energy balance with maps, and steps independent of
//! the PWM period.

use motor_sim::{
    Axis, Carrier, Drive, Grid2, Grid3, LinearAlphaBeta, LinearDq, Load, Machine, MapMachine,
    Motor, MotorParams, Plant, PmsmParams, presets,
};

const DT: f32 = 1.0 / 25_000.0;

/// MS-1's electrical step (`src/electrical.rs` at 0.1.1): dq currents after `h` with the αβ voltage held and ω
/// frozen, in current coordinates with diagonal L. The reference the flux engine has to reproduce.
fn ms1_step(
    p: &PmsmParams,
    i: (f32, f32),
    v_ab: (f32, f32),
    th: f32,
    w: f32,
    h: f32,
) -> (f32, f32) {
    let (r, ld, lq) = (p.rs_ohm, p.ld_h, p.lq_h);
    let (a11, a12, a21, a22) = (-r / ld, w * lq / ld, -w * ld / lq, -r / lq);
    let det_a = a11 * a22 - a12 * a21;
    let (s, c) = th.sin_cos();
    let (vd0, vq0) = (v_ab.0 * c + v_ab.1 * s, -v_ab.0 * s + v_ab.1 * c);
    // Complex arithmetic on (re, im) pairs.
    let mul = |a: (f32, f32), b: (f32, f32)| (a.0 * b.0 - a.1 * b.1, a.0 * b.1 + a.1 * b.0);
    let div = |a: (f32, f32), b: (f32, f32)| {
        let n = b.0 * b.0 + b.1 * b.1;
        ((a.0 * b.0 + a.1 * b.1) / n, (a.1 * b.0 - a.0 * b.1) / n)
    };
    let (w1, w2) = ((vd0 / ld, vq0 / ld), (vq0 / lq, -vd0 / lq));
    let (m11, m22) = ((a11, w), (a22, w));
    let mm = mul(m11, m22);
    let det_m = (mm.0 - a12 * a21, mm.1);
    let t1 = mul(m22, w1);
    let z1 = div((-(t1.0 - a12 * w2.0), -(t1.1 - a12 * w2.1)), det_m);
    let t2 = mul(m11, w2);
    let z2 = div((-(t2.0 - a21 * w1.0), -(t2.1 - a21 * w1.1)), det_m);
    let c2 = -w * p.flux_linkage_wb / lq;
    let xc = (a12 * c2 / det_a, -a11 * c2 / det_a);
    let m = 0.5 * (a11 + a22);
    let hd = 0.5 * (a11 - a22);
    let delta = hd * hd + a12 * a21;
    let dh2 = delta * h * h;
    let (f0, f1) = if dh2.abs() < 1e-4 {
        (
            1.0 + dh2 / 2.0 + dh2 * dh2 / 24.0,
            h * (1.0 + dh2 / 6.0 + dh2 * dh2 / 120.0),
        )
    } else if delta < 0.0 {
        let s = (-delta).sqrt();
        ((s * h).cos(), (s * h).sin() / s)
    } else {
        let s = delta.sqrt();
        ((s * h).cosh(), (s * h).sinh() / s)
    };
    let emh = (m * h).exp();
    let (e11, e12, e21, e22) = (
        emh * (f0 + f1 * (a11 - m)),
        emh * f1 * a12,
        emh * f1 * a21,
        emh * (f0 + f1 * (a22 - m)),
    );
    let (sw, cw) = (w * h).sin_cos();
    let xp = (z1.0 * cw + z1.1 * sw, z2.0 * cw + z2.1 * sw);
    let y0 = (i.0 - z1.0 - xc.0, i.1 - z2.0 - xc.1);
    (
        e11 * y0.0 + e12 * y0.1 + xp.0 + xc.0,
        e21 * y0.0 + e22 * y0.1 + xp.1 + xc.1,
    )
}

/// The same in f64: the exact solution MS-1 rounds to f32.
fn ms1_step_f64(
    p: &PmsmParams,
    i: (f64, f64),
    v_ab: (f64, f64),
    th: f64,
    w: f64,
    h: f64,
) -> (f64, f64) {
    let (r, ld, lq) = (p.rs_ohm as f64, p.ld_h as f64, p.lq_h as f64);
    let (a11, a12, a21, a22) = (-r / ld, w * lq / ld, -w * ld / lq, -r / lq);
    let det_a = a11 * a22 - a12 * a21;
    let (s, c) = th.sin_cos();
    let (vd0, vq0) = (v_ab.0 * c + v_ab.1 * s, -v_ab.0 * s + v_ab.1 * c);
    // Complex arithmetic on (re, im) pairs.
    let mul = |a: (f64, f64), b: (f64, f64)| (a.0 * b.0 - a.1 * b.1, a.0 * b.1 + a.1 * b.0);
    let div = |a: (f64, f64), b: (f64, f64)| {
        let n = b.0 * b.0 + b.1 * b.1;
        ((a.0 * b.0 + a.1 * b.1) / n, (a.1 * b.0 - a.0 * b.1) / n)
    };
    let (w1, w2) = ((vd0 / ld, vq0 / ld), (vq0 / lq, -vd0 / lq));
    let (m11, m22) = ((a11, w), (a22, w));
    let mm = mul(m11, m22);
    let det_m = (mm.0 - a12 * a21, mm.1);
    let t1 = mul(m22, w1);
    let z1 = div((-(t1.0 - a12 * w2.0), -(t1.1 - a12 * w2.1)), det_m);
    let t2 = mul(m11, w2);
    let z2 = div((-(t2.0 - a21 * w1.0), -(t2.1 - a21 * w1.1)), det_m);
    let c2 = -w * p.flux_linkage_wb as f64 / lq;
    let xc = (a12 * c2 / det_a, -a11 * c2 / det_a);
    let m = 0.5 * (a11 + a22);
    let hd = 0.5 * (a11 - a22);
    let delta = hd * hd + a12 * a21;
    let dh2 = delta * h * h;
    let (f0, f1) = if dh2.abs() < 1e-4 {
        (
            1.0 + dh2 / 2.0 + dh2 * dh2 / 24.0,
            h * (1.0 + dh2 / 6.0 + dh2 * dh2 / 120.0),
        )
    } else if delta < 0.0 {
        let s = (-delta).sqrt();
        ((s * h).cos(), (s * h).sin() / s)
    } else {
        let s = delta.sqrt();
        ((s * h).cosh(), (s * h).sinh() / s)
    };
    let emh = (m * h).exp();
    let (e11, e12, e21, e22) = (
        emh * (f0 + f1 * (a11 - m)),
        emh * f1 * a12,
        emh * f1 * a21,
        emh * (f0 + f1 * (a22 - m)),
    );
    let (sw, cw) = (w * h).sin_cos();
    let xp = (z1.0 * cw + z1.1 * sw, z2.0 * cw + z2.1 * sw);
    let y0 = (i.0 - z1.0 - xc.0, i.1 - z2.0 - xc.1);
    (
        e11 * y0.0 + e12 * y0.1 + xp.0 + xc.0,
        e21 * y0.0 + e22 * y0.1 + xp.1 + xc.1,
    )
}

/// A slowly turning αβ voltage, different every step.
fn voltage(k: usize, amp: f32) -> (f32, f32) {
    let th = k as f32 * 0.013;
    (amp * th.cos(), amp * (0.7 * th).sin())
}

/// The flux engine against MS-1's closed form at frozen speed, both against that closed form in f64 (the exact
/// solution): the engine reproduces it to 1e-5 of the peak, at least as close as MS-1's own f32 evaluation.
#[test]
fn linear_engine_reproduces_ms1_electrical_step() {
    for (params, omega_m, amp) in [
        (presets::vesc_hobby_outrunner(), 600.0, 3.0),
        (presets::ipm(), 800.0, 25.0),
        (presets::industrial_spm(), 0.0, 10.0),
    ] {
        let mut m = Motor::new(params, 1);
        m.set_load(Load::Speed {
            omega_m_rad_s: omega_m,
        });
        m.step(&Drive::Off, DT); // up to speed
        let w = params.machine.pole_pairs as f32 * omega_m;
        let mut old = (0.0f32, 0.0f32);
        let mut exact = (0.0f64, 0.0f64);
        let (mut err, mut err_old, mut peak) = (0.0f64, 0.0f64, 0.0f64);
        for k in 0..2000 {
            let v = voltage(k, amp);
            let th = m.truth().theta_e;
            m.step(
                &Drive::AlphaBeta {
                    v_alpha_v: v.0,
                    v_beta_v: v.1,
                },
                DT,
            );
            let t = m.truth();
            // The voltage after the duty stage, as the motor applied it.
            let v = (t.v_alpha, t.v_beta);
            old = ms1_step(&params.machine, old, v, th, w, DT);
            exact = ms1_step_f64(
                &params.machine,
                exact,
                (v.0 as f64, v.1 as f64),
                th as f64,
                w as f64,
                DT as f64,
            );
            peak = peak.max(exact.0.hypot(exact.1));
            err = err.max((t.i_d as f64 - exact.0).hypot(t.i_q as f64 - exact.1));
            err_old = err_old.max((old.0 as f64 - exact.0).hypot(old.1 as f64 - exact.1));
        }
        eprintln!(
            "ω {omega_m}: peak {peak:.2} A, flux engine {:.2e}, MS-1 in f32 {:.2e} of peak",
            err / peak,
            err_old / peak
        );
        assert!(err < 1e-5 * peak, "{err} A of {peak} A");
        assert!(err <= err_old.max(1e-6 * peak));
    }
}

/// Run `n` steps of `dt` with the same drive function on a Motor and a Plant; the largest current and speed
/// differences relative to their peaks.
fn compare<M: Machine>(params: MotorParams, machine: M, dt: f32, n: usize) -> (f32, f32) {
    let mut a = Motor::new(params, 3);
    let mut b = Plant::new(params.with_machine(machine), 3);
    a.set_load(Load::Fan {
        k_nms2_per_rad2: 1e-5,
    });
    b.set_load(Load::Fan {
        k_nms2_per_rad2: 1e-5,
    });
    let (mut err, mut peak, mut werr, mut wpeak) = (0.0f32, 0.0f32, 0.0f32, 0.0f32);
    for k in 0..n {
        // q-axis voltage from the true angle, braking in the last quarter.
        let t = a.truth();
        let vq = if k < 3 * n / 4 { 8.0 } else { 0.5 };
        let th = t.theta_e + 0.5 * t.omega_e * dt;
        let d = Drive::AlphaBeta {
            v_alpha_v: -vq * th.sin(),
            v_beta_v: vq * th.cos(),
        };
        a.step(&d, dt);
        b.step(&d, dt);
        let (ta, tb) = (a.truth(), b.truth());
        peak = peak.max(ta.i_d.hypot(ta.i_q));
        err = err.max((ta.i_alpha - tb.i_alpha).hypot(ta.i_beta - tb.i_beta));
        wpeak = wpeak.max(ta.omega_m.abs());
        werr = werr.max((ta.omega_m - tb.omega_m).abs());
    }
    (err / peak, werr / wpeak)
}

/// Bilinear tables of a linear map are exact, so a [`MapMachine`] of them is MS-1 again.
fn linear_tables(p: &PmsmParams, n: u32, span_wb: f32) -> [Vec<f32>; 3] {
    let ax = Axis::new(-span_wb, span_wb, n).unwrap();
    let mut t = [vec![], vec![], vec![]];
    for a in 0..n as usize {
        for b in 0..n as usize {
            let (x0, x1) = (ax.node(a), ax.node(b));
            t[0].push(x0 / p.ld_h);
            t[1].push(x1 / p.lq_h);
            t[2].push(0.5 * (x0 * x0 / p.ld_h + x1 * x1 / p.lq_h));
        }
    }
    t
}

#[test]
fn linear_machines_reproduce_motor() {
    let params = presets::ipm();
    let n = 25_000; // 1 s: up to speed and braking
    let (e, w) = compare(params, LinearDq::from_pmsm(&params.machine, 0.0), DT, n);
    eprintln!("LinearDq, Ldq = 0: current {e:.2e}, speed {w:.2e} relative");
    assert!(e < 1e-5 && w < 1e-5);

    let span = 0.08;
    let tables = linear_tables(&params.machine, 9, span);
    let ax = Axis::new(-span, span, 9).unwrap();
    let g = |k: usize| Grid2::new(ax, ax, &tables[k]).unwrap();
    let map: MapMachine<Grid2> = MapMachine::new(
        params.machine.pole_pairs,
        params.machine.rs_ohm,
        [params.machine.flux_linkage_wb, 0.0],
        [g(0), g(1)],
        g(2),
    );
    let (e, w) = compare(params, map, DT, n);
    eprintln!("linear tables: current {e:.2e}, speed {w:.2e} relative");
    assert!(e < 1e-5 && w < 1e-5);

    // The stationary frame is second order in the step (inductance and back-EMF turn within it): close at 10 µs.
    let (e, w) = compare(params, LinearAlphaBeta(params.machine), 10e-6, 4 * n);
    eprintln!("LinearAlphaBeta at 10 µs: current {e:.2e}, speed {w:.2e} relative");
    assert!(e < 5e-3 && w < 1e-3);
}

/// RK4 in f64 on the dq flux equations with the αβ voltage `v` held and ω constant, `i_of(x, θ)` the map:
/// dx/dt = v_dq(t) − R·i − ω·J·(x + ψm). Returns x at multiples of `every` substeps.
#[allow(clippy::too_many_arguments)]
fn rk4_dq(
    i_of: &dyn Fn([f64; 2], f64) -> [f64; 2],
    r: f64,
    psi_m: [f64; 2],
    v: [f64; 2],
    th0: f64,
    w: f64,
    hs: f64,
    steps: usize,
    every: usize,
) -> Vec<[f64; 2]> {
    let f = |t: f64, x: [f64; 2]| {
        let th = th0 + w * t;
        let (s, c) = th.sin_cos();
        let (vd, vq) = (v[0] * c + v[1] * s, -v[0] * s + v[1] * c);
        let i = i_of(x, th);
        [
            vd - r * i[0] + w * (x[1] + psi_m[1]),
            vq - r * i[1] - w * (x[0] + psi_m[0]),
        ]
    };
    let add = |x: [f64; 2], k: [f64; 2], s: f64| [x[0] + s * k[0], x[1] + s * k[1]];
    let mut x = [0.0; 2];
    let mut out = vec![];
    for n in 0..steps {
        let t = n as f64 * hs;
        let k1 = f(t, x);
        let k2 = f(t + hs / 2.0, add(x, k1, hs / 2.0));
        let k3 = f(t + hs / 2.0, add(x, k2, hs / 2.0));
        let k4 = f(t + hs, add(x, k3, hs));
        x = [
            x[0] + hs / 6.0 * (k1[0] + 2.0 * k2[0] + 2.0 * k3[0] + k4[0]),
            x[1] + hs / 6.0 * (k1[1] + 2.0 * k2[1] + 2.0 * k3[1] + k4[1]),
        ];
        if (n + 1) % every == 0 {
            out.push(x);
        }
    }
    out
}

const STEPS_US: [u32; 8] = [1, 2, 5, 10, 20, 25, 40, 50];

/// Plant held at `omega_m` by a dyno, constant αβ voltage `v` for 2 ms at each step size; the largest dq current
/// error against `reference` (sampled every 200 µs) relative to its peak, per step size.
fn sweep<M: Machine + Copy>(
    params: &MotorParams,
    machine: M,
    omega_m: f32,
    v: [f32; 2],
    reference: impl Fn(f64) -> Vec<[f64; 2]>,
) -> Vec<(u32, f64)> {
    let spinning = || {
        let mut plant = Plant::new(params.with_machine(machine), 1);
        plant.set_load(Load::Speed {
            omega_m_rad_s: omega_m,
        });
        plant.step(&Drive::Off, 1e-6);
        plant
    };
    let refs = reference(spinning().truth().theta_e as f64);
    let mut out = vec![];
    for h_us in STEPS_US {
        let mut plant = spinning();
        let h = h_us as f32 * 1e-6;
        let per_sample = (200 / h_us) as usize;
        let (mut err, mut peak) = (0.0f64, 0.0f64);
        for k in 0..refs.len() * per_sample {
            plant.step(
                &Drive::AlphaBeta {
                    v_alpha_v: v[0],
                    v_beta_v: v[1],
                },
                h,
            );
            if (k + 1) % per_sample == 0 {
                let r = refs[(k + 1) / per_sample - 1];
                let t = plant.truth();
                peak = peak.max(r[0].hypot(r[1]));
                err = err.max((t.i_d as f64 - r[0]).hypot(t.i_q as f64 - r[1]));
            }
        }
        out.push((h_us, err / peak));
    }
    out
}

#[test]
fn constant_non_diagonal_inductance_against_rk4() {
    let params = presets::ipm();
    let ldq = 0.08e-3; // Ldq² = 0.07·Ld·Lq
    let machine = LinearDq::from_pmsm(&params.machine, ldq);
    let (ld, lq, ldq) = (machine.ld_h as f64, machine.lq_h as f64, ldq as f64);
    let det = ld * lq - ldq * ldq;
    let gamma = move |x: [f64; 2], _th: f64| {
        [
            (lq * x[0] - ldq * x[1]) / det,
            (ld * x[1] - ldq * x[0]) / det,
        ]
    };
    let omega_m = 1000.0; // ωe = 4000 rad/s: 0.2 rad per 50 µs step
    let w = 4.0 * omega_m as f64;
    let v = [-10.0f32, 20.0];
    let psi_m = [machine.flux_linkage_wb as f64, 0.0];
    let r = machine.rs_ohm as f64;
    let res = sweep(&params, machine, omega_m, v, |th0| {
        rk4_dq(
            &gamma,
            r,
            psi_m,
            [v[0] as f64, v[1] as f64],
            th0,
            w,
            0.01e-6,
            200_000,
            20_000,
        )
        .into_iter()
        .map(|x| gamma(x, 0.0))
        .collect()
    });
    for (h, e) in &res {
        eprintln!("LinearDq Ldq ≠ 0, h {h} µs: {e:.2e} of peak");
        // Exact for any step: only f32 rounding, which grows with the number of steps.
        assert!(*e < 2e-5 * 1e9, "h {h} µs: {e}");
    }

    // Torque includes the mutual term: 1.5·p·(λ·iq + (Ld − Lq)·id·iq + Ldq·(iq² − id²)).
    let mut plant = Plant::new(params.with_machine(machine), 1);
    plant.set_load(Load::Locked);
    for _ in 0..3000 {
        plant.step(
            &Drive::AlphaBeta {
                v_alpha_v: -1.0,
                v_beta_v: 1.5,
            },
            DT,
        );
    }
    let t = plant.truth();
    let p = &params.machine;
    let (id, iq) = (t.i_d, t.i_q);
    assert!((id + 1.0 / p.rs_ohm).abs() < 1e-3 && (iq - 1.5 / p.rs_ohm).abs() < 1e-3);
    let te = 1.5
        * p.pole_pairs as f32
        * (p.flux_linkage_wb * iq + (p.ld_h - p.lq_h) * id * iq + ldq as f32 * (iq * iq - id * id));
    assert!(
        (t.torque_e - te).abs() < 1e-4 * te.abs(),
        "{} vs {te}",
        t.torque_e
    );
}

/// A saturating map from one field energy: W̃ = ½(x_d²/Ld + x_q²/Lq) + k/4·|x|⁴ (cross-saturation through |x|²),
/// optionally with an angle ripple ε·cos(6θ) on 1/Ld (cogging-like harmonic). Returns (i_d, i_q, W̃).
fn saturating(p: &PmsmParams, x: [f64; 2], th: f64, ripple: f64) -> [f64; 3] {
    let k = 0.5 / (p.ld_h as f64 * 9e-4); // the current doubles relative to linear around |x| = 30 mWb
    let gd = (1.0 + ripple * (6.0 * th).cos()) / p.ld_h as f64;
    let gq = 1.0 / p.lq_h as f64;
    let s = x[0] * x[0] + x[1] * x[1];
    [
        gd * x[0] + k * s * x[0],
        gq * x[1] + k * s * x[1],
        0.5 * (gd * x[0] * x[0] + gq * x[1] * x[1]) + 0.25 * k * s * s,
    ]
}

const SPAN_WB: f32 = 0.06;
const NODES: u32 = 61;

fn tables_2d(p: &PmsmParams) -> [Vec<f32>; 3] {
    let ax = Axis::new(-SPAN_WB, SPAN_WB, NODES).unwrap();
    let mut t = [vec![], vec![], vec![]];
    for a in 0..NODES as usize {
        for b in 0..NODES as usize {
            let v = saturating(p, [ax.node(a) as f64, ax.node(b) as f64], 0.0, 0.0);
            for k in 0..3 {
                t[k].push(v[k] as f32);
            }
        }
    }
    t
}

const THETA_NODES: u32 = 30;

fn tables_3d(p: &PmsmParams, ripple: f64) -> [Vec<f32>; 3] {
    let ax = Axis::new(-SPAN_WB, SPAN_WB, NODES).unwrap();
    let az = theta_axis();
    let mut t = [vec![], vec![], vec![]];
    for a in 0..NODES as usize {
        for b in 0..NODES as usize {
            for c in 0..THETA_NODES as usize {
                let v = saturating(
                    p,
                    [ax.node(a) as f64, ax.node(b) as f64],
                    az.node(c) as f64,
                    ripple,
                );
                for k in 0..3 {
                    t[k].push(v[k] as f32);
                }
            }
        }
    }
    t
}

fn theta_axis() -> Axis {
    Axis::periodic(0.0, core::f32::consts::FRAC_PI_3, THETA_NODES).unwrap()
}

fn map_2d<'a>(p: &PmsmParams, t: &'a [Vec<f32>; 3]) -> MapMachine<Grid2<'a>> {
    let ax = Axis::new(-SPAN_WB, SPAN_WB, NODES).unwrap();
    let g = |k: usize| Grid2::new(ax, ax, &t[k]).unwrap();
    MapMachine::new(
        p.pole_pairs,
        p.rs_ohm,
        [p.flux_linkage_wb, 0.0],
        [g(0), g(1)],
        g(2),
    )
}

fn map_3d<'a>(p: &PmsmParams, t: &'a [Vec<f32>; 3]) -> MapMachine<Grid3<'a>> {
    let ax = Axis::new(-SPAN_WB, SPAN_WB, NODES).unwrap();
    let g = |k: usize| Grid3::new(ax, ax, theta_axis(), &t[k]).unwrap();
    MapMachine::new(
        p.pole_pairs,
        p.rs_ohm,
        [p.flux_linkage_wb, 0.0],
        [g(0), g(1)],
        g(2),
    )
}

#[test]
fn saturated_map_converges_against_rk4() {
    let params = presets::ipm();
    let p = params.machine;
    let tables = tables_2d(&p);
    let machine = map_2d(&p, &tables);
    let omega_m = 500.0;
    let w = 4.0 * omega_m as f64;
    let v = [-15.0f32, 25.0];
    // The reference integrates the same tables (so only the time stepping differs).
    let i_of = |x: [f64; 2], th: f64| {
        let i = machine.current([x[0] as f32, x[1] as f32], th as f32);
        [i[0] as f64, i[1] as f64]
    };
    let res = sweep(&params, machine, omega_m, v, |th0| {
        rk4_dq(
            &i_of,
            p.rs_ohm as f64,
            [p.flux_linkage_wb as f64, 0.0],
            [v[0] as f64, v[1] as f64],
            th0,
            w,
            0.01e-6,
            200_000,
            20_000,
        )
        .into_iter()
        .map(|x| i_of(x, 0.0))
        .collect()
    });
    for (h, e) in &res {
        eprintln!("saturated map, h {h} µs: {e:.2e} of peak");
    }
    // Second order: well under 1% at 50 µs, and the error falls with the step.
    let at = |h: u32| res.iter().find(|r| r.0 == h).unwrap().1;
    assert!(at(50) < 1e-2, "{}", at(50));
    assert!(at(10) < 1e-3, "{}", at(10));
    assert!(at(1) < 1e-4, "{}", at(1));
    assert!(at(10) < 0.25 * at(50));
}

fn dq_drive(t: &motor_sim::Truth, dt: f32, vq: f32) -> Drive {
    let th = t.theta_e + 0.5 * t.omega_e * dt;
    Drive::AlphaBeta {
        v_alpha_v: -vq * th.sin(),
        v_beta_v: vq * th.cos(),
    }
}

/// DC link energy = field + copper + friction + load + kinetic, with saturated maps (with and without angle
/// dependence), through acceleration and regenerative braking.
fn energy_balance<M: Machine>(params: &MotorParams, machine: M, tol: f64) {
    let mut plant = Plant::new(params.with_machine(machine), 1);
    plant.set_load(Load::Fan {
        k_nms2_per_rad2: 1e-5,
    });
    let w0 = plant.magnetic_energy_j();
    let check = |plant: &Plant<M>| {
        let t = plant.truth();
        let e_in = (t.energy_drawn_wh - t.energy_regen_wh) * 3600.0;
        let mech = t.friction_loss_j + t.load_work_j + plant.kinetic_energy_j();
        let e_out = plant.magnetic_energy_j() - w0 + t.copper_loss_j + mech;
        eprintln!(
            "in {e_in:.4} J, out {e_out:.4} J (field {:.4}, cu {:.3}, mech {mech:.3}), ω {}, i_q {}",
            plant.magnetic_energy_j() - w0,
            t.copper_loss_j,
            t.omega_m,
            t.i_q
        );
        assert!(mech > 0.05 * e_in);
        assert!((e_in - e_out).abs() < tol * e_in, "{e_in} vs {e_out}");
    };
    let dt = 10e-6;
    for _ in 0..30_000 {
        let d = dq_drive(plant.truth(), dt, 6.0);
        plant.step(&d, dt);
    }
    check(&plant);
    // A hard pulse deep into saturation: the field energy matters now.
    let mut x_max = 0.0f32;
    for _ in 0..1_000 {
        let d = dq_drive(plant.truth(), dt, 25.0);
        plant.step(&d, dt);
        x_max = x_max.max(plant.flux()[0].hypot(plant.flux()[1]));
    }
    eprintln!("pulse: |x| up to {x_max:.4} V·s");
    assert!(x_max > 0.02, "the pulse should reach saturation");
    check(&plant);
    for _ in 0..5_000 {
        let d = dq_drive(plant.truth(), dt, 0.5);
        plant.step(&d, dt);
    }
    assert!(
        plant.truth().energy_regen_wh > 0.0,
        "braking should regenerate"
    );
    check(&plant);
}

#[test]
fn energy_balance_with_maps() {
    let params = presets::ipm();
    let p = params.machine;
    let t2 = tables_2d(&p);
    energy_balance(&params, map_2d(&p, &t2), 2e-4);
    let t3 = tables_3d(&p, 0.05);
    energy_balance(&params, map_3d(&p, &t3), 1e-3);
}

/// Steps of 10 µs under a 25 kHz carrier (drive updated once per PWM period) end every period where per-period
/// steps without a carrier do, and both approach a switching-resolved run (0.2 µs steps under the carrier).
#[test]
fn step_independent_of_pwm_period() {
    let params = presets::industrial_spm();
    let run = |carrier: bool, h: f32| {
        let mut m = Motor::new(params, 1);
        m.set_load(Load::Viscous {
            b_nms_per_rad: 1e-3,
        });
        if carrier {
            m.set_carrier(Some(Carrier::center(25e3)));
        }
        let per = (DT / h).round() as usize;
        let mut out = vec![];
        for _ in 0..1000 {
            let d = dq_drive(m.truth(), DT, 12.0);
            for _ in 0..per {
                m.step(&d, h);
            }
            let t = m.truth();
            out.push((t.i_d, t.i_q, t.omega_m, t.time_s));
        }
        out
    };
    let period = run(false, DT);
    let quarter = run(true, 10e-6);
    let fine = run(true, 0.2e-6);
    let (mut err_q, mut err_p, mut err_pq, mut peak, mut werr) =
        (0.0f32, 0.0f32, 0.0f32, 0.0f32, 0.0f32);
    for ((a, b), f) in period.iter().zip(&quarter).zip(&fine) {
        assert!((a.3 - b.3).abs() < 1e-9);
        peak = peak.max(f.1.abs());
        err_pq = err_pq.max((a.0 - b.0).hypot(a.1 - b.1));
        err_p = err_p.max((a.0 - f.0).hypot(a.1 - f.1));
        err_q = err_q.max((b.0 - f.0).hypot(b.1 - f.1));
        werr = werr.max((a.2 - b.2).abs() / a.2.abs().max(1.0));
    }
    eprintln!(
        "peak i_q {peak:.2} A; at period ends: per-period vs 10 µs {err_pq:.2e} A; vs 0.2 µs carrier: per-period \
         {err_p:.2e} A, 10 µs {err_q:.2e} A; speed {werr:.2e}"
    );
    // The average model and the carrier deliver the same volt-seconds per period.
    assert!(err_pq < 1e-3 * peak, "{err_pq} A");
    assert!(err_p < 2e-3 * peak && err_q < 2e-3 * peak);
    assert!(werr < 1e-3);
}

/// Changing inductances between steps keeps the currents (as MS-1, whose state was the current): the flux state is
/// re-derived from them.
#[test]
fn params_change_keeps_currents_continuous() {
    let mut m = Motor::new(presets::ipm(), 1);
    m.set_load(Load::Locked);
    let d = Drive::AlphaBeta {
        v_alpha_v: 1.0,
        v_beta_v: 0.5,
    };
    for _ in 0..200 {
        m.step(&d, DT);
    }
    let before = (m.truth().i_d, m.truth().i_q);
    m.params_mut().machine.ld_h *= 2.0;
    m.params_mut().machine.lq_h *= 0.5;
    m.step(&d, 1e-9);
    let after = (m.truth().i_d, m.truth().i_q);
    assert!((after.0 - before.0).abs() < 1e-4 && (after.1 - before.1).abs() < 1e-4);
}

/// A snapshot restored and stepped with the same inputs repeats the run bit for bit: noise, random faults,
/// carrier, mechanics and the clock included.
#[test]
fn snapshot_restore_replays_bit_for_bit() {
    let mut m = Motor::new(presets::vesc_hobby_outrunner(), 9);
    m.set_load(Load::Inertia {
        inertia_kgm2: 1e-5,
        stiffness_nm_per_rad: 0.5,
        damping_nms_per_rad: 1e-4,
        viscous_nms_per_rad: 1e-5,
    });
    m.set_carrier(Some(Carrier::center(25e3)));
    m.faults_mut().inverter.missed_pulse_probability = 0.01;
    m.faults_mut().sensor.current_glitch_probability = 0.01;
    let drive = |k: usize, m: &Motor| dq_drive(m.truth(), 10e-6, 2.0 + (k % 7) as f32 * 0.1);
    for k in 0..3000 {
        let d = drive(k, &m);
        m.step(&d, 10e-6);
    }
    let snap = m.snapshot();
    let clock = m.clock();
    assert_eq!(clock.ticks, 3000);
    assert!((clock.time_s - 0.03).abs() < 1e-9 && clock.time_s == m.truth().time_s);
    let run = |m: &mut Motor| {
        (0..2000)
            .map(|k| {
                let d = drive(k, m);
                m.step(&d, 10e-6);
                (*m.truth(), *m.sensors())
            })
            .collect::<Vec<_>>()
    };
    let first = run(&mut m);
    m.restore(&snap);
    assert_eq!(m.clock(), clock);
    let second = run(&mut m);
    assert!(first == second, "replay differs");
    assert!(first.last().unwrap().0.omega_m > 10.0);
}
