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
use motor_sim::{Drive, Load, Motor, MotorParams, presets};

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

/// Minimum ns per [`Motor::step`] for one configuration over [`REPEATS`] trials, each step driven from the
/// motor's own angle like a controller would (see the crate-level example in `motor_sim`), so the optimizer
/// cannot fold the loop away.
fn ns_per_step(case: &TimeCase) -> f64 {
    let mut params = (case.preset)();
    if !case.sensors_on {
        // `Motor::step` always samples the sensors; "off" is the cheapest a controller could configure them
        // (no noise, no encoder) rather than a code path that is skipped. See HANDOFF.md.
        params.sensors.current_noise_a = 0.0;
        params.sensors.vdc_noise_v = 0.0;
        params.sensors.encoder_cpr = 0;
    }
    let mut motor = Motor::new(params, 42);
    motor.set_load(case.load);

    let step = |motor: &mut Motor| {
        let theta = motor.truth().theta_e;
        let drive = Drive::AlphaBeta {
            v_alpha_v: -3.0 * theta.sin(),
            v_beta_v: 3.0 * theta.cos(),
        };
        motor.step(&drive, case.dt_s);
        black_box(motor.truth().i_q);
    };

    for _ in 0..WARMUP_STEPS {
        step(&mut motor);
    }
    let mut best = f64::INFINITY;
    for _ in 0..REPEATS {
        let start = thread_cpu_time_ns();
        for _ in 0..MEASURED_STEPS {
            step(&mut motor);
        }
        let ns = (thread_cpu_time_ns() - start) as f64 / MEASURED_STEPS as f64;
        best = best.min(ns);
    }
    best
}

/// A cargo feature combination of `motor_sim` to measure code size for. Today `motor_sim` has no features of its
/// own, so this is just the default build; MS-7's layers will add rows here as they land.
struct SizeCase {
    name: &'static str,
    cargo_args: &'static [&'static str],
}

const SIZE_CASES: &[SizeCase] = &[SizeCase {
    name: "codesize.default",
    cargo_args: &[],
}];

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
