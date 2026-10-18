//! Cost bench for `motor_sim` (MS-8): ns per [`Motor::step`] per configuration (preset × load × dt × sensors),
//! and the `thumbv7em-none-eabihf` code size of `motor_sim` per cargo feature.
//!
//! `cargo run -p motor_bench --release` prints the table and checks every row against `baseline.tsv`, exiting
//! with an error if any configuration got more than [`REGRESSION_FACTOR`] slower; a row with no prior baseline
//! (a new configuration, or the first run on a new machine) is added rather than compared. Timings are this
//! thread's own CPU time (`CLOCK_THREAD_CPUTIME_ID`), not wall-clock, so time spent preempted by other load on a
//! busy dev box doesn't count against a run; a `REPEATS`-fold minimum further absorbs scheduler jitter. Neither
//! fixes a machine-wide CPU frequency drop under sustained heavy load (seen on gpd-omarchy with several
//! concurrent builds running): that shifts every row together, cross-run, by however much the clock dropped.
//! Treat a flagged regression with suspicion if the machine was busy during either run, and prefer re-baselining
//! (`--update`) once it is quiet over trusting the delta. Baselines are machine-specific either way: meaningful
//! only against another run on the same machine.
//! `cargo run -p motor_bench --release -- --update` re-measures and rewrites every row, after a deliberate,
//! reviewed change in cost.

use std::collections::BTreeMap;
use std::env;
use std::fmt::Write as _;
use std::fs;
use std::hint::black_box;
use std::path::{Path, PathBuf};
use std::process::Command;

use anyhow::{Context, bail, ensure};
use console::style;
use motor_sim::{
    Axis, Carrier, Drive, Grid2, Grid3, LinearAlphaBeta, LinearDq, Load, Machine, MapMachine,
    Motor, MotorParams, Plant, PmsmParams, presets,
};

/// A measurement over this factor of its baseline is a regression (1.20 = 20%).
const REGRESSION_FACTOR: f64 = 1.20;
const WARMUP_STEPS: u64 = 5_000;
const MEASURED_STEPS: u64 = 1_000_000;
/// Repeats per configuration; the reported value is the minimum, which is robust to scheduler noise from other
/// load on the machine (preemption only ever makes a run slower, never faster).
const REPEATS: u32 = 7;

const BASELINE_PATH: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/baseline.tsv");

/// This thread's own CPU time, ns. Unlike a wall clock, it does not advance while the thread is preempted, so a
/// busy dev box doesn't inflate the measurement.
#[cfg(unix)]
fn thread_cpu_time_ns() -> u64 {
    let mut ts = libc::timespec {
        tv_sec: 0,
        tv_nsec: 0,
    };
    // SAFETY: `ts` is a valid, live `timespec` for `clock_gettime` to write into.
    unsafe {
        libc::clock_gettime(libc::CLOCK_THREAD_CPUTIME_ID, &mut ts);
    }
    ts.tv_sec as u64 * 1_000_000_000 + ts.tv_nsec as u64
}

#[cfg(not(unix))]
compile_error!("motor_bench times Motor::step via CLOCK_THREAD_CPUTIME_ID (unix only)");

type PresetFn = fn() -> MotorParams;

struct TimeCase {
    name: String,
    preset: PresetFn,
    load: Load,
    dt_s: f32,
    sensors_on: bool,
}

fn time_cases() -> Vec<TimeCase> {
    let presets: [(&str, PresetFn); 3] = [
        ("vesc", presets::vesc_hobby_outrunner),
        ("spm", presets::industrial_spm),
        ("ipm", presets::ipm),
    ];
    let loads: [(&str, Load); 2] = [
        ("none", Load::None),
        (
            "fan",
            Load::Fan {
                k_nms2_per_rad2: 2e-7,
            },
        ),
    ];
    let dts: [(&str, f32); 2] = [("dt10", 10e-6), ("dt40", 40e-6)];
    let sensors: [(&str, bool); 2] = [("sensors_on", true), ("sensors_off", false)];

    let mut cases = Vec::with_capacity(presets.len() * loads.len() * dts.len() * sensors.len());
    for (pname, preset) in presets {
        for (lname, load) in loads {
            for (dname, dt_s) in dts {
                for (sname, sensors_on) in sensors {
                    cases.push(TimeCase {
                        name: format!("time.{pname}.{lname}.{dname}.{sname}"),
                        preset,
                        load,
                        dt_s,
                        sensors_on,
                    });
                }
            }
        }
    }
    cases
}

/// Minimum ns per [`Motor::step`] for one configuration over [`REPEATS`] trials.
fn ns_per_step(case: &TimeCase) -> f64 {
    let mut params = (case.preset)();
    if !case.sensors_on {
        // `Motor::step` always samples the sensors; "off" is the cheapest a controller could configure them
        // (no noise, no encoder) rather than a code path that is skipped.
        params.sensors.current_noise_a = 0.0;
        params.sensors.vdc_noise_v = 0.0;
        params.sensors.encoder_cpr = 0;
    }
    let mut motor = Motor::new(params, 42);
    motor.set_load(case.load);
    ns_per_plant_step(&mut motor, case.dt_s)
}

/// Minimum ns per [`Plant::step`] over [`REPEATS`] trials, each step driven from the plant's own angle like a
/// controller would (see the crate-level example in `motor_sim`), so the optimizer cannot fold the loop away.
fn ns_per_plant_step<M: Machine>(plant: &mut Plant<M>, dt_s: f32) -> f64 {
    let step = |plant: &mut Plant<M>| {
        let theta = plant.truth().theta_e;
        let drive = Drive::AlphaBeta {
            v_alpha_v: -3.0 * theta.sin(),
            v_beta_v: 3.0 * theta.cos(),
        };
        plant.step(&drive, dt_s);
        black_box(plant.truth().i_q);
    };

    for _ in 0..WARMUP_STEPS {
        step(plant);
    }
    let mut best = f64::INFINITY;
    for _ in 0..REPEATS {
        let start = thread_cpu_time_ns();
        for _ in 0..MEASURED_STEPS {
            step(plant);
        }
        let ns = (thread_cpu_time_ns() - start) as f64 / MEASURED_STEPS as f64;
        best = best.min(ns);
    }
    best
}

/// A saturating map from one field energy, W̃ = ½(x_d²/Ld + x_q²/Lq) + k/4·|x|⁴, optionally with a 6th-harmonic
/// ripple on 1/Ld: the current doubles relative to linear around |x| = 30 mWb. Returns (i_d, i_q, W̃).
fn saturating(p: &PmsmParams, x: [f32; 2], theta_e: f32, ripple: f32) -> [f32; 3] {
    let k = 0.5 / (p.ld_h * 9e-4);
    let gd = (1.0 + ripple * (6.0 * theta_e).cos()) / p.ld_h;
    let gq = 1.0 / p.lq_h;
    let s = x[0] * x[0] + x[1] * x[1];
    [
        gd * x[0] + k * s * x[0],
        gq * x[1] + k * s * x[1],
        0.5 * (gd * x[0] * x[0] + gq * x[1] * x[1]) + 0.25 * k * s * s,
    ]
}

const MAP_SPAN_WB: f32 = 0.06;
const MAP_NODES: u32 = 33;
const MAP_THETA_NODES: u32 = 30;

/// Tables of [`saturating`] over (x_d, x_q) and, with `theta_nodes` > 0, the electrical angle over 60°.
fn map_tables(p: &PmsmParams, theta_nodes: u32) -> [Vec<f32>; 3] {
    let ax = Axis::new(-MAP_SPAN_WB, MAP_SPAN_WB, MAP_NODES).expect("axis");
    let az = Axis::periodic(0.0, std::f32::consts::FRAC_PI_3, theta_nodes.max(2)).expect("axis");
    let mut t = [vec![], vec![], vec![]];
    for a in 0..ax.len() {
        for b in 0..ax.len() {
            for c in 0..theta_nodes.max(1) as usize {
                let (th, ripple) = if theta_nodes > 0 {
                    (az.node(c), 0.05)
                } else {
                    (0.0, 0.0)
                };
                let v = saturating(p, [ax.node(a), ax.node(b)], th, ripple);
                for k in 0..3 {
                    t[k].push(v[k]);
                }
            }
        }
    }
    t
}

/// Engine configurations beyond the presets' linear machine (MS-7), each at a 10 µs step with sensors on: the
/// full inductance matrix, the stationary frame, 2D and 3D flux maps, and the PWM carrier.
fn engine_rows(baseline: &BTreeMap<String, f64>, rows: &mut Vec<Row>) {
    let dt = 10e-6;
    let fan = Load::Fan {
        k_nms2_per_rad2: 2e-7,
    };
    let vesc = presets::vesc_hobby_outrunner();
    let ipm = presets::ipm();

    let mut plant = Plant::new(
        vesc.with_machine(LinearDq::from_pmsm(&vesc.machine, 1e-6)),
        42,
    );
    plant.set_load(fan);
    let ns = ns_per_plant_step(&mut plant, dt);
    report(
        "time.engine.vesc_lineardq.dt10".into(),
        ns,
        "ns",
        baseline,
        rows,
    );

    let mut plant = Plant::new(vesc.with_machine(LinearAlphaBeta(vesc.machine)), 42);
    plant.set_load(fan);
    let ns = ns_per_plant_step(&mut plant, dt);
    report(
        "time.engine.vesc_alphabeta.dt10".into(),
        ns,
        "ns",
        baseline,
        rows,
    );

    let mut motor = Motor::new(vesc, 42);
    motor.set_load(fan);
    motor.set_carrier(Some(Carrier::center(25e3)));
    let ns = ns_per_plant_step(&mut motor, dt);
    report(
        "time.engine.vesc_carrier25k.dt10".into(),
        ns,
        "ns",
        baseline,
        rows,
    );

    let p = ipm.machine;
    let ax = Axis::new(-MAP_SPAN_WB, MAP_SPAN_WB, MAP_NODES).expect("axis");
    let t2 = map_tables(&p, 0);
    let g2 = |k: usize| Grid2::new(ax, ax, &t2[k]).expect("grid");
    let machine: MapMachine<Grid2> = MapMachine::new(
        p.pole_pairs,
        p.rs_ohm,
        [p.flux_linkage_wb, 0.0],
        [g2(0), g2(1)],
        g2(2),
    );
    let mut plant = Plant::new(ipm.with_machine(machine), 42);
    plant.set_load(fan);
    let ns = ns_per_plant_step(&mut plant, dt);
    report(
        "time.engine.ipm_map2d.dt10".into(),
        ns,
        "ns",
        baseline,
        rows,
    );

    let az = Axis::periodic(0.0, std::f32::consts::FRAC_PI_3, MAP_THETA_NODES).expect("axis");
    let t3 = map_tables(&p, MAP_THETA_NODES);
    let g3 = |k: usize| Grid3::new(ax, ax, az, &t3[k]).expect("grid");
    let machine: MapMachine<Grid3> = MapMachine::new(
        p.pole_pairs,
        p.rs_ohm,
        [p.flux_linkage_wb, 0.0],
        [g3(0), g3(1)],
        g3(2),
    );
    let mut plant = Plant::new(ipm.with_machine(machine), 42);
    plant.set_load(fan);
    let ns = ns_per_plant_step(&mut plant, dt);
    report(
        "time.engine.ipm_map3d.dt10".into(),
        ns,
        "ns",
        baseline,
        rows,
    );
}

/// A cargo feature combination of `motor_sim` to measure code size for.
struct SizeCase {
    name: &'static str,
    cargo_args: &'static [&'static str],
}

const SIZE_CASES: &[SizeCase] = &[
    SizeCase {
        name: "codesize.default",
        cargo_args: &[],
    },
    SizeCase {
        name: "codesize.no_maps",
        cargo_args: &["--no-default-features"],
    },
];

/// `.text` bytes of `motor_sim`'s compiled rlib for `thumbv7em-none-eabihf`: not a linked, dead-code-eliminated
/// binary, but consistent run to run, which is what a regression check needs.
fn code_size_bytes(workspace_root: &Path, case: &SizeCase) -> anyhow::Result<f64> {
    let mut args = vec![
        "build",
        "-p",
        "motor_sim",
        "--release",
        "--target",
        "thumbv7em-none-eabihf",
    ];
    args.extend_from_slice(case.cargo_args);
    let status = Command::new("cargo")
        .args(&args)
        .current_dir(workspace_root)
        .status()
        .context("spawning cargo build")?;
    ensure!(status.success(), "cargo {} failed", args.join(" "));

    let rlib = workspace_root.join("target/thumbv7em-none-eabihf/release/libmotor_sim.rlib");
    let out = Command::new("size")
        .arg("-t")
        .arg(&rlib)
        .output()
        .context("spawning `size` (binutils); install it to run the code size bench")?;
    ensure!(out.status.success(), "size {} failed", rlib.display());
    let text = String::from_utf8_lossy(&out.stdout);
    let totals = text.lines().last().context("`size` produced no output")?;
    let bytes: f64 = totals
        .split_whitespace()
        .next()
        .context("parsing `size` output")?
        .parse()
        .context("parsing `size` output")?;
    Ok(bytes)
}

fn load_baseline() -> BTreeMap<String, f64> {
    let mut map = BTreeMap::new();
    let Ok(text) = fs::read_to_string(BASELINE_PATH) else {
        return map;
    };
    for line in text.lines() {
        let line = line.trim();
        if line.is_empty() || line.starts_with('#') {
            continue;
        }
        if let Some((key, value)) = line.split_once('\t')
            && let Ok(v) = value.trim().parse::<f64>()
        {
            map.insert(key.to_string(), v);
        }
    }
    map
}

fn write_baseline(map: &BTreeMap<String, f64>) -> anyhow::Result<()> {
    let mut out = String::from(
        "# motor_bench baseline (MS-8): ns per Motor::step, thread CPU time (time.*), and thumbv7em-none-eabihf\n\
         # .text bytes (codesize.*). Machine-specific either way: meaningful only against another run on this\n\
         # same machine, ideally not a busy one (see the crate doc comment). Regenerate with\n\
         # `cargo run -p motor_bench --release -- --update`.\n",
    );
    for (key, value) in map {
        writeln!(out, "{key}\t{value:.1}")?;
    }
    fs::write(BASELINE_PATH, out).with_context(|| format!("writing {BASELINE_PATH}"))?;
    Ok(())
}

/// One printed row; records whether it is a regression (over the baseline) or new (no prior baseline).
struct Row {
    key: String,
    value: f64,
    unit: &'static str,
    regressed: bool,
    is_new: bool,
}

fn report(
    key: String,
    value: f64,
    unit: &'static str,
    baseline: &BTreeMap<String, f64>,
    rows: &mut Vec<Row>,
) {
    let (regressed, is_new) = match baseline.get(&key) {
        Some(&base) => (value > base * REGRESSION_FACTOR, false),
        None => (false, true),
    };
    let status = match baseline.get(&key) {
        Some(&base) => {
            let delta_pct = (value / base - 1.0) * 100.0;
            if regressed {
                style(format!("+{delta_pct:.0}% over baseline — regression"))
                    .red()
                    .bold()
                    .to_string()
            } else {
                style(format!("{delta_pct:+.0}%")).green().to_string()
            }
        }
        None => style("new, no baseline yet").yellow().to_string(),
    };
    println!("  {key:<28} {value:>10.1} {unit:<3} {status}");
    rows.push(Row {
        key,
        value,
        unit,
        regressed,
        is_new,
    });
}

fn print_usage() {
    println!(
        "motor_bench {}\n\n\
         ns per Motor::step per configuration, and the thumbv7em-none-eabihf code size of motor_sim, checked\n\
         against {BASELINE_PATH}.\n\n\
         USAGE:\n    motor_bench [--update]\n\n\
         --update   re-measure and rewrite every row in baseline.tsv (after a deliberate, reviewed cost change)\n\
         --version  print version\n    --help     print this message",
        env!("MOTOR_BENCH_VERSION")
    );
}

fn main() -> anyhow::Result<()> {
    let args: Vec<String> = env::args().skip(1).collect();
    if args.iter().any(|a| a == "--help" || a == "-h") {
        print_usage();
        return Ok(());
    }
    if args.iter().any(|a| a == "--version" || a == "-V") {
        println!("motor_bench {}", env!("MOTOR_BENCH_VERSION"));
        return Ok(());
    }
    let update = args.iter().any(|a| a == "--update");
    if let Some(unknown) = args.iter().find(|a| a.as_str() != "--update") {
        bail!("unknown argument '{unknown}'; see --help");
    }

    let workspace_root: PathBuf = Path::new(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .context("motor_bench is expected directly under the workspace root")?
        .to_path_buf();
    let baseline = load_baseline();
    let mut rows = Vec::new();

    println!("ns per Motor::step, {MEASURED_STEPS} steps each ({WARMUP_STEPS} warmup):\n");
    for case in time_cases() {
        let ns = ns_per_step(&case);
        report(case.name, ns, "ns", &baseline, &mut rows);
    }
    engine_rows(&baseline, &mut rows);

    println!("\nthumbv7em-none-eabihf .text size of motor_sim, per cargo feature:\n");
    for case in SIZE_CASES {
        let bytes = code_size_bytes(&workspace_root, case)?;
        report(case.name.to_string(), bytes, "B", &baseline, &mut rows);
    }

    if update {
        let fresh: BTreeMap<String, f64> = rows.iter().map(|r| (r.key.clone(), r.value)).collect();
        write_baseline(&fresh)?;
        println!("\nBaseline rewritten: {BASELINE_PATH}");
        return Ok(());
    }

    let new_rows: Vec<&Row> = rows.iter().filter(|r| r.is_new).collect();
    if !new_rows.is_empty() {
        let mut grown = baseline.clone();
        for row in &new_rows {
            grown.insert(row.key.clone(), row.value);
        }
        write_baseline(&grown)?;
        println!(
            "\n{} new configuration(s) added to the baseline.",
            new_rows.len()
        );
    }

    let regressions: Vec<&Row> = rows.iter().filter(|r| r.regressed).collect();
    if !regressions.is_empty() {
        let pct = ((REGRESSION_FACTOR - 1.0) * 100.0) as u32;
        eprintln!(
            "\n{} configuration(s) over {pct}% slower than their baseline:",
            regressions.len()
        );
        for row in &regressions {
            eprintln!("  {} ({:.1} {})", row.key, row.value, row.unit);
        }
        bail!("cost regression");
    }

    Ok(())
}
