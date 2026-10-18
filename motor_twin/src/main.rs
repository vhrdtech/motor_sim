//! `motor_twin`: the motor's digital twin in Bevy, first slice (DEMO-2): a motor imported from CAD by `motor_cad`
//! (a STEP file, classified on load, or a scene `.glb` it wrote), the rotating group spinning kinematically at a set
//! speed, parts shown and hidden by role, an exploded view along the axis and a cutaway. `--screenshot` renders
//! headless to a PNG and exits. The state source (plant in process, wire_weaver) comes next.

mod view;

use bevy::app::ScheduleRunnerPlugin;
use bevy::prelude::*;
use bevy::window::ExitCondition;
use bevy::winit::WinitPlugin;
use bevy_egui::{EguiContexts, EguiPlugin, EguiPrimaryContextPass, egui};
use clap::Parser;
use motor_cad::role::Role;
use std::path::PathBuf;
use std::time::Duration;
use view::{Orbit, TwinState};

#[derive(Parser, Debug, Clone, Resource)]
#[command(name = "motor_twin", version = env!("MOTOR_TWIN_VERSION"), about = "The motor's digital twin: a motor from CAD, spinning")]
struct Args {
    /// a STEP file (classified on load, with `<name>.motor.toml` overrides beside it) or a scene .glb from motor_cad
    model: PathBuf,
    /// rotor speed, rpm
    #[arg(long, default_value_t = 600.0)]
    rpm: f32,
    /// exploded view along the axis, 0 (assembled) .. 1
    #[arg(long, default_value_t = 0.0)]
    explode: f32,
    /// cut away the half in front of the camera's start position
    #[arg(long)]
    cutaway: bool,
    /// hide these roles (comma separated: housing,fastener,...)
    #[arg(long, value_delimiter = ',')]
    hide: Vec<String>,
    /// show only these roles
    #[arg(long, value_delimiter = ',')]
    only: Vec<String>,
    /// camera yaw and pitch, degrees
    #[arg(long, default_value_t = 35.0, allow_hyphen_values = true)]
    yaw: f32,
    #[arg(long, default_value_t = 25.0, allow_hyphen_values = true)]
    pitch: f32,
    /// render headless to this PNG and exit
    #[arg(long)]
    screenshot: Option<PathBuf>,
    /// screenshot size, WIDTHxHEIGHT
    #[arg(long, default_value = "1600x1000")]
    size: String,
    /// frames to render before the screenshot (pipelines compile in the first ones)
    #[arg(long, default_value_t = 40)]
    frames: u32,
    /// rotor angle for the screenshot, degrees
    #[arg(long, default_value_t = 0.0)]
    angle: f32,
}

fn parse_roles(v: &[String]) -> Vec<Role> {
    v.iter()
        .filter_map(|s| Role::from_tag(s).or_else(|| (s == "other").then_some(Role::Other)))
        .collect()
}

fn main() -> AppExit {
    let args = Args::parse();
    let scene = match view::load(&args.model) {
        Ok(s) => s,
        Err(e) => {
            eprintln!("motor_twin: {e:#}");
            return AppExit::error();
        }
    };
    let mut state = TwinState::new(&scene);
    state.rpm = args.rpm;
    state.explode = args.explode;
    state.cutaway = args.cutaway;
    state.angle = args.angle.to_radians() as f64;
    let only = parse_roles(&args.only);
    let hide = parse_roles(&args.hide);
    for r in Role::ALL {
        let shown = (only.is_empty() || only.contains(&r)) && !hide.contains(&r);
        state.shown.insert(r, shown);
    }
    let orbit = Orbit::framing(
        &scene,
        args.yaw.to_radians(),
        args.pitch.to_radians(),
        args.explode,
    );

    let mut app = App::new();
    if let Some(out) = args.screenshot.clone() {
        let (w, h) = args
            .size
            .split_once('x')
            .and_then(|(w, h)| Some((w.parse().ok()?, h.parse().ok()?)))
            .unwrap_or((1600, 1000));
        state.rpm = 0.0; // a still frame at --angle
        app.add_plugins(
            DefaultPlugins
                .set(WindowPlugin {
                    primary_window: None,
                    exit_condition: ExitCondition::DontExit,
                    close_when_requested: false,
                    ..default()
                })
                .disable::<WinitPlugin>(),
        )
        .add_plugins(ScheduleRunnerPlugin::run_loop(Duration::from_secs_f64(
            1.0 / 60.0,
        )))
        .insert_resource(view::Headless {
            out,
            size: UVec2::new(w, h),
            frames: args.frames,
            taken: false,
        });
    } else {
        app.add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: format!("motor_twin: {}", scene.name),
                resolution: (1600, 1000).into(),
                ..default()
            }),
            ..default()
        }))
        .add_plugins(EguiPlugin::default())
        .add_systems(EguiPrimaryContextPass, ui)
        .add_systems(Update, view::orbit_input);
    }
    app.insert_resource(view::SceneRes(scene))
        .insert_resource(state)
        .insert_resource(orbit)
        .insert_resource(ClearColor(Color::srgb(0.11, 0.12, 0.14)))
        .insert_resource(GlobalAmbientLight {
            color: Color::srgb(0.85, 0.88, 1.0),
            brightness: 350.0,
            ..default()
        })
        .add_systems(Startup, view::setup)
        .add_systems(
            Update,
            (view::spin, view::apply_orbit, view::headless_capture),
        )
        .run()
}

/// The control panel.
fn ui(
    mut contexts: EguiContexts,
    mut state: ResMut<TwinState>,
    scene: Res<view::SceneRes>,
) -> Result {
    let ctx = contexts.ctx_mut()?;
    egui::Window::new("motor")
        .default_width(260.0)
        .default_pos([12.0, 12.0])
        .show(ctx, |ui| {
            let s = &scene.0;
            ui.heading(&s.name);
            ui.label(format!(
                "{:?}, {} slots, {} magnets",
                s.topology,
                s.slots.map(|x| x.to_string()).unwrap_or("?".into()),
                s.poles.map(|x| x.to_string()).unwrap_or("?".into())
            ));
            ui.separator();
            ui.add(
                egui::Slider::new(&mut state.rpm, 0.0..=20_000.0)
                    .text("rpm")
                    .logarithmic(true),
            );
            ui.horizontal(|ui| {
                ui.label("slow motion");
                for (label, f) in [("1×", 1.0), ("1/10", 0.1), ("1/100", 0.01)] {
                    ui.selectable_value(&mut state.slow_motion, f, label);
                }
            });
            ui.label(format!(
                "shown: {:.0} rpm, angle {:.0}°",
                state.rpm * state.slow_motion,
                state.angle.to_degrees().rem_euclid(360.0)
            ));
            ui.separator();
            ui.add(egui::Slider::new(&mut state.explode, 0.0..=1.0).text("explode"));
            ui.checkbox(&mut state.cutaway, "cutaway");
            ui.separator();
            ui.label("roles");
            let counts = state.counts.clone();
            for r in Role::ALL {
                let n = counts.get(&r).copied().unwrap_or(0);
                if n == 0 {
                    continue;
                }
                ui.horizontal(|ui| {
                    let mut on = state.shown.get(&r).copied().unwrap_or(true);
                    if ui.checkbox(&mut on, format!("{r} ({n})")).changed() {
                        state.shown.insert(r, on);
                    }
                    if ui.small_button("only").clicked() {
                        for q in Role::ALL {
                            state.shown.insert(q, q == r);
                        }
                    }
                });
            }
            if ui.button("show all").clicked() {
                for q in Role::ALL {
                    state.shown.insert(q, true);
                }
            }
            if ui.button("hide housing").clicked() {
                state.shown.insert(Role::Housing, false);
            }
            ui.separator();
            ui.small("drag: orbit, right drag: pan, wheel: zoom");
        });
    Ok(())
}
