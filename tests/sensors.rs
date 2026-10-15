//! Sensors, faults, determinism and the variable visitor.

use motor_sim::{Drive, LegFault, Load, Motor, Sensors, Source, Value, Var, presets};

const DT: f32 = 1.0 / 25_000.0;

fn spin(m: &mut Motor, steps: usize) -> Vec<Sensors> {
    (0..steps)
        .map(|k| {
            let th = k as f32 * 0.05;
            let (s, c) = th.sin_cos();
            m.step(
                &Drive::AlphaBeta {
                    v_alpha_v: 2.0 * c,
                    v_beta_v: 2.0 * s,
                },
                DT,
            );
            *m.sensors()
        })
        .collect()
}

#[test]
fn same_seed_same_run_other_seed_other_noise() {
    let p = presets::vesc_hobby_outrunner();
    let a = spin(&mut Motor::new(p, 7), 2000);
    let b = spin(&mut Motor::new(p, 7), 2000);
    let c = spin(&mut Motor::new(p, 8), 2000);
    assert_eq!(a, b);
    assert_ne!(a, c);
}

#[test]
fn faults_draw_from_their_own_stream() {
    let p = presets::vesc_hobby_outrunner();
    let mut healthy = Motor::new(p, 7);
    let mut glitchy = Motor::new(p, 7);
    glitchy.faults_mut().sensor.hall_glitch_probability = 0.5;
    glitchy.faults_mut().sensor.encoder_missed_count_probability = 0.5;
    let a = spin(&mut healthy, 500);
    let b = spin(&mut glitchy, 500);
    for (x, y) in a.iter().zip(&b) {
        assert_eq!(x.i_abc_a, y.i_abc_a);
        assert_eq!(x.vdc_v, y.vdc_v);
    }
    assert!(a.iter().zip(&b).any(|(x, y)| x.hall != y.hall));
    assert_ne!(
        a.last().unwrap().encoder_count,
        b.last().unwrap().encoder_count
    );
}

#[test]
fn clean_adc_reads_truth_within_half_an_lsb() {
    let mut p = presets::vesc_hobby_outrunner();
    p.sensors.current_noise_a = 0.0;
    p.sensors.current_offset_a = [0.0; 3];
    p.sensors.vdc_noise_v = 0.0;
    let mut m = Motor::new(p, 1);
    m.set_load(Load::Locked);
    let lsb_i = 2.0 * p.sensors.current_full_scale_a / 4096.0;
    let lsb_v = p.sensors.vdc_full_scale_v / 4096.0;
    spin(&mut m, 200);
    let (t, s) = (m.truth(), m.sensors());
    for (meas, truth) in s.i_abc_a.iter().zip([t.i_a, t.i_b, t.i_c]) {
        assert!(
            (meas - truth).abs() <= 0.5 * lsb_i + 1e-6,
            "{meas} vs {truth}"
        );
    }
    assert!((s.vdc_v - t.vdc).abs() <= 0.5 * lsb_v + 1e-6);
    // Offsets and saturation.
    m.params_mut().sensors.current_offset_a = [1.0, 0.0, 0.0];
    m.step(&Drive::Duty([1.0, 0.0, 0.0]), 0.01);
    let s = *m.sensors();
    assert!(s.i_abc_a[0] >= p.sensors.current_full_scale_a - lsb_i);
    assert!(s.i_abc_a[1] <= -p.sensors.current_full_scale_a + 2.0 * lsb_i);
}

#[test]
fn encoder_and_halls_follow_the_rotor() {
    let p = presets::industrial_spm();
    let mut m = Motor::new(p, 1);
    m.set_load(Load::Speed {
        omega_m_rad_s: 10.0,
    });
    let mut halls = vec![];
    for _ in 0..25_000 {
        m.step(&Drive::Off, DT);
        let (t, s) = (m.truth(), m.sensors());
        let expect =
            (t.theta_m * p.sensors.encoder_cpr as f64 / std::f64::consts::TAU).floor() as i64;
        assert_eq!(
            s.encoder_count as i64,
            expect.rem_euclid(p.sensors.encoder_cpr as i64)
        );
        if halls.last() != Some(&s.hall) {
            halls.push(s.hall);
        }
    }
    // 10 rad/s for 1 s ≈ 1.6 turns ≈ 6.4 electrical turns: the forward sequence repeats.
    let seq = [5u8, 1, 3, 2, 6, 4];
    let start = seq.iter().position(|h| *h == halls[0]).unwrap();
    for (k, h) in halls.iter().enumerate() {
        assert_eq!(*h, seq[(start + k) % 6]);
    }
    assert!(halls.len() > 30);
}

#[test]
fn sensor_faults() {
    let p = presets::vesc_hobby_outrunner();
    let mut m = Motor::new(p, 3);
    spin(&mut m, 100);
    let held = m.sensors().i_abc_a[1];
    m.faults_mut().sensor.current_stuck[1] = true;
    m.faults_mut().sensor.hall_stuck = Some(0b111);
    m.faults_mut().sensor.vdc_offset_v = 5.0;
    for s in spin(&mut m, 100) {
        assert_eq!(s.i_abc_a[1], held);
        assert_eq!(s.hall, 7);
        assert!((s.vdc_v - p.inverter.vdc_v - 5.0).abs() < 0.2);
    }
    m.faults_mut().sensor.current_glitch_probability = 1.0;
    let a = spin(&mut m, 50);
    assert!(a.windows(2).any(|w| w[0].i_abc_a[0] != w[1].i_abc_a[0]));
}

#[test]
fn inverter_faults() {
    let p = presets::industrial_spm();
    let mut m = Motor::new(p, 3);
    m.faults_mut().inverter.legs = [LegFault::StuckHigh, LegFault::Healthy, LegFault::StuckLow];
    m.step(&Drive::Duty([0.5; 3]), DT);
    assert_eq!(m.truth().duty, [1.0, 0.5, 0.0]);
    // Missed pulses: some steps see a leg at 0.
    m.faults_mut().inverter = motor_sim::InverterFaults {
        missed_pulse_probability: 0.3,
        ..motor_sim::InverterFaults::NONE
    };
    let mut missed = 0;
    for _ in 0..100 {
        m.step(&Drive::Duty([0.6; 3]), DT);
        missed += m.truth().duty.iter().filter(|d| **d == 0.0).count();
    }
    assert!((50..150).contains(&missed), "{missed} missed pulses");
    // The physics stay healthy: the inverter off clears the current regardless of faults.
    m.faults_mut().inverter.legs[0] = LegFault::StuckHigh;
    m.step(&Drive::Off, DT);
    assert_eq!((m.truth().i_d, m.truth().i_q), (0.0, 0.0));
}

#[test]
fn visitor_lists_every_variable_once() {
    let mut m = Motor::new(presets::ipm(), 1);
    spin(&mut m, 10);
    let mut vars: Vec<(Var, Value)> = vec![];
    m.visit(&mut |v: &Var, x| vars.push((*v, x)));
    let truth: Vec<_> = vars
        .iter()
        .filter(|(v, _)| v.source == Source::Truth)
        .collect();
    let sensors: Vec<_> = vars
        .iter()
        .filter(|(v, _)| v.source == Source::Sensor)
        .collect();
    assert_eq!(truth.len(), 36);
    assert_eq!(sensors.len(), 6);
    for group in [&truth, &sensors] {
        let mut names: Vec<_> = group.iter().map(|(v, _)| v.name).collect();
        names.sort();
        names.dedup();
        assert_eq!(names.len(), group.len());
    }
    for (v, x) in &vars {
        if matches!(x, Value::F32(_) | Value::F64(_)) && !v.name.starts_with("duty") {
            assert!(!v.unit.is_empty(), "{} has no unit", v.name);
        }
    }
    let i_q = vars.iter().find(|(v, _)| v.name == "i_q").unwrap();
    assert_eq!(i_q.1, Value::F32(m.truth().i_q));
}
