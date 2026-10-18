//! `motor_cad`: import a motor from STEP, classify its parts and write the twin's scene; write the parametric test
//! motors. A thin CLI over the `motor_cad` library.

use anyhow::{Context, Result};
use clap::{CommandFactory, Parser, Subcommand};
use clap_complete::{ArgValueCompleter, CompleteEnv, CompletionCandidate};
use console::style;
use indicatif::{ProgressBar, ProgressStyle};
use motor_cad::{
    classify, generate, overrides::Overrides, report, scene::Scene, step_read, step_write,
};
use std::path::{Path, PathBuf};
use std::time::Duration;

#[derive(Parser)]
#[command(
    name = "motor_cad",
    version = env!("MOTOR_CAD_VERSION"),
    about = "Motor geometry from CAD: STEP import, part roles, the twin's scene (glTF)",
    after_help = "Shell completion: source <(COMPLETE=bash motor_cad) (or zsh, fish)"
)]
struct Cli {
    /// colour: auto (a terminal without NO_COLOR), always, never
    #[arg(long, global = true, default_value = "auto")]
    color: String,
    #[command(subcommand)]
    cmd: Cmd,
}

#[derive(Subcommand)]
enum Cmd {
    /// Read a STEP file, classify its parts, print the report; write the scene as .glb with --glb
    Import {
        /// the STEP file (an override file `<name>.motor.toml` beside it is used when present)
        #[arg(add = ArgValueCompleter::new(step_files))]
        step: PathBuf,
        /// write the scene here (`-` for `<name>.glb` beside the STEP file)
        #[arg(long)]
        glb: Option<PathBuf>,
        /// write the Markdown report here
        #[arg(long)]
        report: Option<PathBuf>,
    },
    /// Write an override file template beside a STEP file, one entry per part with the role found
    InitOverrides {
        #[arg(add = ArgValueCompleter::new(step_files))]
        step: PathBuf,
        /// replace an existing file
        #[arg(long)]
        force: bool,
    },
    /// Write the parametric test motors (outrunner, stepper, anonymised outrunner) as STEP
    Gen {
        /// directory
        #[arg(default_value = ".")]
        out: PathBuf,
    },
}

/// STEP files in the directory being completed, with their size.
fn step_files(current: &std::ffi::OsStr) -> Vec<CompletionCandidate> {
    let cur = current.to_string_lossy();
    let (dir, prefix) = match cur.rfind('/') {
        Some(i) => (cur[..=i].to_string(), cur[i + 1..].to_string()),
        None => (String::new(), cur.to_string()),
    };
    let Ok(rd) = std::fs::read_dir(if dir.is_empty() { "." } else { &dir }) else {
        return vec![];
    };
    let mut out = Vec::new();
    for e in rd.flatten() {
        let name = e.file_name().to_string_lossy().to_string();
        if !name.starts_with(&prefix) {
            continue;
        }
        let is_dir = e.file_type().is_ok_and(|t| t.is_dir());
        let ext = Path::new(&name)
            .extension()
            .map(|x| x.to_string_lossy().to_lowercase())
            .unwrap_or_default();
        if is_dir {
            out.push(
                CompletionCandidate::new(format!("{dir}{name}/")).help(Some("directory".into())),
            );
        } else if ext == "step" || ext == "stp" {
            let kb = e.metadata().map(|m| m.len() / 1024).unwrap_or(0);
            out.push(
                CompletionCandidate::new(format!("{dir}{name}"))
                    .help(Some(format!("STEP, {kb} kB").into())),
            );
        }
    }
    out
}

fn spinner(msg: &str) -> ProgressBar {
    let pb = ProgressBar::new_spinner();
    pb.set_style(ProgressStyle::with_template("{spinner} {msg} {elapsed}").unwrap());
    pb.set_message(msg.to_string());
    pb.enable_steady_tick(Duration::from_millis(100));
    pb
}

fn load(step: &Path) -> Result<(step_read::CadModel, classify::Classification, Overrides)> {
    let pb = spinner(&format!("reading {}", step.display()));
    let model =
        step_read::read_step_file(step).with_context(|| format!("importing {}", step.display()))?;
    pb.set_message(format!("classifying {} parts", model.parts.len()));
    let ov = Overrides::load_beside(step)?;
    let c = classify::classify(&model, &ov);
    pb.finish_and_clear();
    Ok((model, c, ov))
}

fn main() -> Result<()> {
    // answers shell completion requests (COMPLETE=<shell> set) and exits, otherwise does nothing
    CompleteEnv::with_factory(Cli::command).complete();
    let cli = Cli::parse();
    match cli.color.as_str() {
        "never" => console::set_colors_enabled(false),
        "always" => console::set_colors_enabled(true),
        _ => {}
    }
    match cli.cmd {
        Cmd::Import {
            step,
            glb,
            report: report_path,
        } => {
            let t = std::time::Instant::now();
            let (model, c, _) = load(&step)?;
            let title = step
                .file_name()
                .map(|n| n.to_string_lossy().to_string())
                .unwrap_or_default();
            let md = report::markdown(&title, &model, &c);
            for line in md.lines() {
                // colour the role column's confidence: green ≥ 0.8, yellow ≥ 0.5, red below
                if let Some(conf) = line
                    .split(" | ")
                    .nth(3)
                    .and_then(|x| x.trim().parse::<f64>().ok())
                {
                    let st = if conf >= 0.8 {
                        style(line).green()
                    } else if conf >= 0.5 {
                        style(line).yellow()
                    } else {
                        style(line).red()
                    };
                    println!("{st}");
                } else if line.starts_with('#') {
                    println!("{}", style(line).bold());
                } else {
                    println!("{line}");
                }
            }
            if let Some(p) = report_path {
                std::fs::write(&p, &md).with_context(|| format!("writing {}", p.display()))?;
            }
            if let Some(g) = glb {
                let g = if g.as_os_str() == "-" {
                    step.with_extension("glb")
                } else {
                    g
                };
                let scene = Scene::build(&title, &model, &c);
                std::fs::write(&g, scene.to_glb())
                    .with_context(|| format!("writing {}", g.display()))?;
                println!("{} {}", style("scene").bold(), g.display());
            }
            println!(
                "{}",
                style(format!("{:.2} s", t.elapsed().as_secs_f64())).dim()
            );
        }
        Cmd::InitOverrides { step, force } => {
            let path = Overrides::path_for(&step);
            if path.exists() && !force {
                anyhow::bail!("{} exists (--force replaces it)", path.display());
            }
            let (model, c, _) = load(&step)?;
            let mut s = format!(
                "# motor_cad overrides for {}: first match wins; `match` is a product name, an instance name or a path\n\
                 # A/B/Part, `*` matches anything; roles: shaft rotor magnet stator winding bearing housing fastener pcb wiring\n\
                 # sensor other. Uncomment what is wrong.\n\n# [axis]\n# origin = [{:.3}, {:.3}, {:.3}]\n# direction = [{:.4}, {:.4}, {:.4}]\n",
                step.display(),
                c.axis.origin[0] / model.unit_m,
                c.axis.origin[1] / model.unit_m,
                c.axis.origin[2] / model.unit_m,
                c.axis.dir[0],
                c.axis.dir[1],
                c.axis.dir[2]
            );
            let mut seen = std::collections::HashSet::new();
            for (p, v) in model.parts.iter().zip(&c.verdicts) {
                if !seen.insert(p.name.clone()) {
                    continue;
                }
                s.push_str(&format!(
                    "\n# found: {} ({:.2}){}\n# [[part]]\n# match = {:?}\n# role = \"{}\"\n",
                    v.role,
                    v.confidence,
                    if v.rotating { ", rotating" } else { "" },
                    p.name,
                    v.role
                ));
            }
            std::fs::write(&path, s).with_context(|| format!("writing {}", path.display()))?;
            println!("{} {}", style("wrote").bold(), path.display());
        }
        Cmd::Gen { out } => {
            std::fs::create_dir_all(&out)?;
            let o = generate::outrunner(&Default::default());
            let s = generate::stepper(&Default::default());
            let bare = generate::anonymised(&o, "Motor");
            for (name, doc) in [
                ("outrunner_5065.step", &o),
                ("stepper_nema17.step", &s),
                ("outrunner_bare.step", &bare),
            ] {
                let p = out.join(name);
                std::fs::write(&p, step_write::write_step(doc))
                    .with_context(|| format!("writing {}", p.display()))?;
                println!("{} {}", style("wrote").bold(), p.display());
            }
        }
    }
    Ok(())
}
