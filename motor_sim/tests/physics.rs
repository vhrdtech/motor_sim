//! Closed-form checks of the plant: steady state, locked rotor, loads, energy balance.

use motor_sim::{Drive, Load, Motor, presets};

const DT: f32 = 1.0 / 25_000.0;

/// dq voltage turned into αβ at the mid-step angle, using the true rotor angle (an ideal FOC voltage stage).
fn dq(m: &Motor, v_d: f32, v_q: f32) -> Drive {
    let t = m.truth();
    let th = t.theta_e + 0.5 * t.omega_e * DT;
    let (s, c) = th.sin_cos();
    Drive::AlphaBeta {
        v_alpha_v: v_d * c - v_q * s,
        v_beta_v: v_d * s + v_q * c,
    }
}

/// Voltage-mode FOC with the d-axis coupling cancelled (v_d = −ωe·Lq·i_q), so i_d → 0 at steady state.
fn decoupled(m: &Motor, v_q: f32) -> Drive {
    let t = m.truth();
    dq(m, -t.omega_e * m.params().pmsm.lq_h * t.i_q, v_q)
}

fn run(m: &mut Motor, seconds: f32, mut drive: impl FnMut(&Motor) -> Drive) {
    for _ in 0..(seconds / DT) as usize {
        let d = drive(m);
        m.step(&d, DT);
    }
}

#[test]
fn steady_state_speed_and_torque_match_closed_form() {
    let params = presets::industrial_spm();
    let (p, mech) = (params.pmsm, params.mech);
    let mut m = Motor::new(params, 1);
    let t_load = 0.2;
    m.set_load(Load::Constant { torque_nm: t_load });
    let v_q = 20.0;
    run(&mut m, 0.3, |m| decoupled(m, v_q));

    // i_q = (v_q − ωe·λ)/R, kt·i_q = T_load + Tc + B·ω  →  ω = (kt·v_q/R − T_load − Tc)/(B + kt·p·λ/R)
    let kt = p.kt_nm_per_a();
    let pl = p.pole_pairs as f32 * p.flux_linkage_wb;
    let omega = (kt * v_q / p.rs_ohm - t_load - mech.coulomb_nm)
        / (mech.viscous_nms_per_rad + kt * pl / p.rs_ohm);
    let t = m.truth();
    eprintln!(
        "ω {} rad/s, closed form {omega}; i_d {} A, i_q {} A",
        t.omega_m, t.i_d, t.i_q
    );
    assert!((t.omega_m - omega).abs() < 1e-3 * omega);
    assert!(t.i_d.abs() < 0.02);
    let te = t_load + mech.coulomb_nm + mech.viscous_nms_per_rad * t.omega_m;
    assert!((t.torque_e - te).abs() < 1e-3 * te);
    assert!((t.torque_load - t_load).abs() < 1e-6);
}

#[test]
fn locked_rotor_current_and_reluctance_torque() {
    let params = presets::ipm();
    let p = params.pmsm;
    let mut m = Motor::new(params, 1);
    m.set_load(Load::Locked);
    // Rotor at θe = 0, so αβ = dq: v = R·i for i_d = −20 A, i_q = 30 A.
    let (id, iq) = (-20.0, 30.0);
    let drive = dq(&m, p.rs_ohm * id, p.rs_ohm * iq);
    let tau_q = p.lq_h / p.rs_ohm;
    run(&mut m, tau_q, |_| drive);
    let t = m.truth();
    let expect_q = iq * (1.0 - (-1.0f32).exp());
    assert!(
        (t.i_q - expect_q).abs() < 0.01 * iq.abs(),
        "i_q {} after one τq",
        t.i_q
    );
    run(&mut m, 20.0 * tau_q, |_| drive);
    let t = m.truth();
    assert!((t.i_d - id).abs() < 1e-3 * 20.0 && (t.i_q - iq).abs() < 1e-3 * 30.0);
    let te = 1.5 * p.pole_pairs as f32 * (p.flux_linkage_wb * iq + (p.ld_h - p.lq_h) * id * iq);
    assert!((t.torque_e - te).abs() < 1e-3 * te);
    assert_eq!(t.torque_load, t.torque_e);
    assert_eq!(t.omega_m, 0.0);
    assert_eq!(t.theta_m, 0.0);
    // Phase a carries i_α = i_d at θe = 0, the DC link only the copper loss.
    assert!((t.i_a - id).abs() < 0.05);
    let p_cu = 1.5 * p.rs_ohm * (id * id + iq * iq);
    assert!((t.p_dc - p_cu).abs() < 1e-3 * p_cu);
}

#[test]
fn fan_load_settles_at_quadratic_torque() {
    let params = presets::vesc_hobby_outrunner();
    let mech = params.mech;
    let mut m = Motor::new(params, 1);
    let k = 2e-7;
    m.set_load(Load::Fan { k_nms2_per_rad2: k });
    run(&mut m, 2.0, |m| decoupled(m, 4.0));
    let t = m.truth();
    let w = t.omega_m;
    let te = k * w * w + mech.viscous_nms_per_rad * w + mech.coulomb_nm;
    eprintln!("fan: ω {w} rad/s, torque {} vs {te} N·m", t.torque_e);
    assert!((t.torque_e - te).abs() < 2e-3 * te);
}

#[test]
fn second_inertia_follows_with_spring_twist() {
    let params = presets::industrial_spm();
    let mut m = Motor::new(params, 1);
    let (k, b2) = (5.0, 1e-4);
    m.set_load(Load::Inertia {
        inertia_kgm2: 1e-4,
        stiffness_nm_per_rad: k,
        damping_nms_per_rad: 1e-3,
        viscous_nms_per_rad: b2,
    });
    run(&mut m, 1.0, |m| decoupled(m, 15.0));
    let t = m.truth();
    assert!((t.omega_load - t.omega_m).abs() < 1e-3 * t.omega_m);
    let twist = (t.theta_m - t.theta_load) as f32;
    assert!((twist - b2 * t.omega_load / k).abs() < 0.01 * twist);
    assert!((t.torque_load - b2 * t.omega_load).abs() < 0.01 * t.torque_load);
}

#[test]
fn profile_and_speed_loads() {
    fn step_up(t: f64) -> f32 {
        if t < 0.1 { 0.0 } else { 0.5 }
    }
    let mut m = Motor::new(presets::industrial_spm(), 1);
    m.set_load(Load::Profile(step_up));
    run(&mut m, 0.2, |m| decoupled(m, 10.0));
    assert!((m.truth().torque_load - 0.5).abs() < 1e-6);

    m.set_load(Load::Speed {
        omega_m_rad_s: -50.0,
    });
    run(&mut m, 0.01, |_| Drive::Duty([0.5; 3]));
    let t = m.truth();
    assert_eq!(t.omega_m, -50.0);
    // Shorted windings brake: the dyno has to push.
    assert!(t.torque_e > 0.0 && t.torque_load > 0.0);
}

#[test]
fn coulomb_friction_holds_and_stops_the_rotor() {
    let mut m = Motor::new(presets::industrial_spm(), 1);
    // Torque below breakaway: no motion.
    let p = m.params().pmsm;
    let i_hold = 0.5 * m.params().mech.coulomb_nm / p.kt_nm_per_a();
    run(&mut m, 0.05, |m| dq(m, 0.0, p.rs_ohm * i_hold));
    assert_eq!(m.truth().omega_m, 0.0);
    // Spin up, then coast: friction brings it to rest and keeps it there.
    run(&mut m, 0.05, |m| decoupled(m, 10.0));
    assert!(m.truth().omega_m > 10.0);
    run(&mut m, 2.0, |_| Drive::Off);
    assert_eq!(m.truth().omega_m, 0.0);
    assert_eq!(m.truth().i_q, 0.0);
}

#[test]
fn duty_and_voltage_drives_agree() {
    let params = presets::industrial_spm();
    let mut a = Motor::new(params, 1);
    let mut b = Motor::new(params, 1);
    let vdc = params.inverter.vdc_v;
    for k in 0..2000 {
        let th = k as f32 * 0.01;
        let (s, c) = th.sin_cos();
        let v = [
            10.0 * c,
            10.0 * (c * -0.5 + s * 0.866_025_4),
            10.0 * (c * -0.5 - s * 0.866_025_4),
        ];
        a.step(&Drive::PhaseVolts(v), DT);
        b.step(&Drive::Duty(v.map(|x| 0.5 + x / vdc)), DT);
    }
    let (ta, tb) = (a.truth(), b.truth());
    assert!((ta.i_q - tb.i_q).abs() < 1e-3 && (ta.omega_m - tb.omega_m).abs() < 1e-3);
    // Beyond the hexagon the command saturates: duties stay in 0..=1.
    a.step(
        &Drive::AlphaBeta {
            v_alpha_v: 100.0,
            v_beta_v: 0.0,
        },
        DT,
    );
    assert_eq!(a.truth().duty, [1.0, 0.0, 0.0]);
    assert!((a.truth().v_alpha - 2.0 / 3.0 * vdc).abs() < 1e-3);
}

/// DC link energy = magnetic + copper + friction + load + kinetic, through acceleration and regenerative braking.
#[test]
fn energy_balance() {
    let params = presets::ipm();
    let mut m = Motor::new(params, 1);
    m.set_load(Load::Fan {
        k_nms2_per_rad2: 1e-5,
    });
    let check = |m: &Motor| {
        let t = m.truth();
        let e_in = (t.energy_drawn_wh - t.energy_regen_wh) * 3600.0;
        let mech = t.friction_loss_j + t.load_work_j + m.kinetic_energy_j();
        let e_out = m.magnetic_energy_j() + t.copper_loss_j + mech;
        eprintln!(
            "in {e_in:.4} J, out {e_out:.4} J (cu {:.3}, fric {:.3}, load {:.3}, kin {:.3}), ω {}",
            t.copper_loss_j,
            t.friction_loss_j,
            t.load_work_j,
            m.kinetic_energy_j(),
            t.omega_m
        );
        assert!(mech > 0.1 * e_in);
        assert!((e_in - e_out).abs() < 1e-4 * e_in);
        let ah = t.charge_drawn_ah - t.charge_regen_ah;
        assert!((ah * params.inverter.vdc_v as f64 * 3600.0 - e_in).abs() < 1e-6 * e_in);
    };
    run(&mut m, 0.2, |m| decoupled(m, 6.0));
    check(&m);
    run(&mut m, 0.05, |m| decoupled(m, 0.5));
    assert!(m.truth().energy_regen_wh > 0.0, "braking should regenerate");
    check(&m);
}
