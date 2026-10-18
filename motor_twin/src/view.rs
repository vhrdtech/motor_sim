//! The 3D view: the motor stood upright (its axis along Bevy's +Y through the origin), PBR materials by role,
//! the rotating group spun about the axis, explode and cutaway, an orbit camera, and the headless screenshot.

use bevy::asset::RenderAssetUsages;
use bevy::camera::RenderTarget;
use bevy::input::mouse::{AccumulatedMouseMotion, AccumulatedMouseScroll};
use bevy::light::CascadeShadowConfigBuilder;
use bevy::mesh::{Indices, PrimitiveTopology};
use bevy::prelude::*;
use bevy::render::render_resource::{TextureFormat, TextureUsages};
use bevy::render::view::screenshot::{Screenshot, ScreenshotCaptured, save_to_disk};
use motor_cad::role::Role;
use motor_cad::scene::{Scene, role_pbr};
use std::collections::HashMap;
use std::path::{Path, PathBuf};

#[derive(Resource)]
pub struct SceneRes(pub Scene);

/// Loads a STEP file (import and classify, overrides beside it) or a motor_cad `.glb`.
pub fn load(path: &Path) -> anyhow::Result<Scene> {
    let ext = path
        .extension()
        .map(|e| e.to_string_lossy().to_lowercase())
        .unwrap_or_default();
    let name = path
        .file_stem()
        .map(|s| s.to_string_lossy().to_string())
        .unwrap_or_default();
    if ext == "glb" {
        let bytes = anyhow::Context::with_context(std::fs::read(path), || {
            format!("reading {}", path.display())
        })?;
        return Ok(Scene::from_glb(&bytes)?);
    }
    let model = motor_cad::step_read::read_step_file(path)?;
    let ov = motor_cad::overrides::Overrides::load_beside(path)?;
    let c = motor_cad::classify::classify(&model, &ov);
    Ok(Scene::build(&name, &model, &c))
}

#[derive(Resource)]
pub struct TwinState {
    pub rpm: f32,
    pub slow_motion: f32,
    /// rad
    pub angle: f64,
    pub explode: f32,
    pub cutaway: bool,
    pub shown: HashMap<Role, bool>,
    pub counts: HashMap<Role, usize>,
}

impl TwinState {
    pub fn new(scene: &Scene) -> TwinState {
        let mut counts = HashMap::new();
        for p in &scene.parts {
            *counts.entry(p.role).or_insert(0) += 1;
        }
        TwinState {
            rpm: 600.0,
            slow_motion: 1.0,
            angle: 0.0,
            explode: 0.0,
            cutaway: false,
            shown: HashMap::new(),
            counts,
        }
    }
}

/// One part in the view.
#[derive(Component)]
pub struct PartView {
    role: Role,
    rotating: bool,
    /// centroid, view frame (m)
    centroid: Vec3,
    /// goes all the way round the axis: cut by triangles in a cutaway, else shown or hidden by its centroid
    around: bool,
    full: Handle<Mesh>,
    cut: Option<Handle<Mesh>>,
    /// explode direction and amount at explode = 1 (m along +Y)
    explode_by: f32,
}

#[derive(Resource)]
pub struct Orbit {
    pub yaw: f32,
    pub pitch: f32,
    pub dist: f32,
    pub target: Vec3,
    /// the cutaway removes the half on this side (horizontal unit vector)
    pub cut_normal: Vec3,
}

impl Orbit {
    /// Frames the motor: distance from its size.
    pub fn framing(scene: &Scene, yaw: f32, pitch: f32, explode: f32) -> Orbit {
        let (lo, hi) = view_bounds(scene);
        let size = (hi - lo).length().max(1e-3);
        let cut_normal = Vec3::new(yaw.sin(), 0.0, yaw.cos());
        let mut target = (lo + hi) / 2.0;
        // the exploded view stretches the motor along Y by about (1 + 2 explode)
        target.y += explode * 0.15 * (hi.y - lo.y);
        Orbit {
            yaw,
            pitch,
            dist: (1.15 + 1.1 * explode) * size,
            target,
            cut_normal,
        }
    }

    fn transform(&self) -> Transform {
        let dir = Vec3::new(
            self.pitch.cos() * self.yaw.sin(),
            self.pitch.sin(),
            self.pitch.cos() * self.yaw.cos(),
        );
        Transform::from_translation(self.target + dir * self.dist).looking_at(self.target, Vec3::Y)
    }
}

/// The CAD frame to the view frame: motor axis origin to 0, axis to +Y.
fn to_view(scene: &Scene) -> impl Fn([f32; 3]) -> Vec3 + '_ {
    let a = &scene.axis;
    let o = Vec3::new(a.origin[0] as f32, a.origin[1] as f32, a.origin[2] as f32);
    let d = Vec3::new(a.dir[0] as f32, a.dir[1] as f32, a.dir[2] as f32).normalize_or(Vec3::Z);
    let q = Quat::from_rotation_arc(d, Vec3::Y);
    move |p: [f32; 3]| q * (Vec3::from(p) - o)
}

fn view_rot(scene: &Scene) -> Quat {
    let a = &scene.axis;
    let d = Vec3::new(a.dir[0] as f32, a.dir[1] as f32, a.dir[2] as f32).normalize_or(Vec3::Z);
    Quat::from_rotation_arc(d, Vec3::Y)
}

fn view_bounds(scene: &Scene) -> (Vec3, Vec3) {
    let f = to_view(scene);
    let (mut lo, mut hi) = (Vec3::splat(f32::INFINITY), Vec3::splat(f32::NEG_INFINITY));
    // the motor body: leads and cables would pull the framing away
    let body: Vec<_> = scene
        .parts
        .iter()
        .filter(|p| p.role != Role::Wiring)
        .collect();
    let parts = if body.is_empty() {
        scene.parts.iter().collect()
    } else {
        body
    };
    for p in parts {
        for v in &p.mesh.positions {
            let v = f(*v);
            lo = lo.min(v);
            hi = hi.max(v);
        }
    }
    if lo.x > hi.x {
        (Vec3::ZERO, Vec3::ZERO)
    } else {
        (lo, hi)
    }
}

/// Bevy meshes of a part in the view frame: all of it, and the half behind the cut plane.
fn meshes(scene: &Scene, i: usize, cut_normal: Vec3) -> (Mesh, Mesh, Vec3, bool) {
    let f = to_view(scene);
    let q = view_rot(scene);
    let m = &scene.parts[i].mesh;
    let pos: Vec<Vec3> = m.positions.iter().map(|p| f(*p)).collect();
    let nor: Vec<[f32; 3]> = m
        .normals
        .iter()
        .map(|n| (q * Vec3::from(*n)).to_array())
        .collect();
    let centroid = pos.iter().copied().sum::<Vec3>() / pos.len().max(1) as f32;
    // coverage round the axis from area samples: flat faces have vertices only at their corners
    let mut bins = [false; 72];
    for p in m
        .surface_samples(3000)
        .iter()
        .map(|p| f([p[0] as f32, p[1] as f32, p[2] as f32]))
    {
        let a = p.z.atan2(p.x);
        bins[(((a + std::f32::consts::PI) / std::f32::consts::TAU) * 72.0) as usize % 72] = true;
    }
    let around = bins.iter().all(|b| *b);
    let build = |idx: Vec<u32>| {
        Mesh::new(
            PrimitiveTopology::TriangleList,
            RenderAssetUsages::default(),
        )
        .with_inserted_attribute(
            Mesh::ATTRIBUTE_POSITION,
            pos.iter().map(|v| v.to_array()).collect::<Vec<_>>(),
        )
        .with_inserted_attribute(Mesh::ATTRIBUTE_NORMAL, nor.clone())
        .with_inserted_indices(Indices::U32(idx))
    };
    let keep: Vec<u32> = m
        .indices
        .as_chunks::<3>()
        .0
        .iter()
        .filter(|t| {
            let c = (pos[t[0] as usize] + pos[t[1] as usize] + pos[t[2] as usize]) / 3.0;
            Vec3::new(c.x, 0.0, c.z).dot(cut_normal) <= 0.0
        })
        .flatten()
        .copied()
        .collect();
    (build(m.indices.clone()), build(keep), centroid, around)
}

/// Where a role goes in the exploded view, in lengths of the motor along +Y.
fn explode_shift(role: Role) -> f32 {
    match role {
        Role::Rotor | Role::Shaft => 0.9,
        Role::Magnet => 0.7,
        Role::Winding => 0.35,
        Role::Bearing => 0.2,
        Role::Stator | Role::Other => 0.0,
        Role::Pcb | Role::Wiring => -0.45,
        Role::Sensor => -0.55,
        Role::Housing => -0.65,
        Role::Fastener => -0.95,
    }
}

#[derive(Resource)]
pub struct Headless {
    pub out: PathBuf,
    pub size: UVec2,
    pub frames: u32,
    pub taken: bool,
}

#[derive(Resource)]
pub struct HeadlessTarget(Handle<Image>);

pub fn setup(
    mut commands: Commands,
    scene: Res<SceneRes>,
    orbit: Res<Orbit>,
    headless: Option<Res<Headless>>,
    mut mesh_assets: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut images: ResMut<Assets<Image>>,
) {
    let s = &scene.0;
    let (lo, hi) = view_bounds(s);
    let length = (hi.y - lo.y).max(1e-3);
    for (i, p) in s.parts.iter().enumerate() {
        let (full, cut, centroid, around) = meshes(s, i, orbit.cut_normal);
        let (metallic, roughness) = role_pbr(p.role);
        let [r, g, b] = p.colour;
        let mat = materials.add(StandardMaterial {
            base_color: Color::srgb(r, g, b),
            metallic,
            perceptual_roughness: roughness,
            reflectance: 0.5,
            double_sided: true,
            cull_mode: None,
            ..default()
        });
        let full = mesh_assets.add(full);
        let cut = around.then(|| mesh_assets.add(cut));
        let explode_by = explode_shift(p.role) * length + 0.5 * centroid.y;
        commands.spawn((
            Mesh3d(full.clone()),
            MeshMaterial3d(mat),
            Transform::IDENTITY,
            Name::new(p.name.clone()),
            PartView {
                role: p.role,
                rotating: p.rotating,
                centroid,
                around,
                full,
                cut,
                explode_by,
            },
        ));
    }
    // lights: a key light with shadows, a fill and a rim, plus the ambient
    let size = (hi - lo).length().max(1e-3);
    commands.spawn((
        DirectionalLight {
            illuminance: 9_000.0,
            shadow_maps_enabled: true,
            ..default()
        },
        CascadeShadowConfigBuilder {
            maximum_distance: 6.0 * size,
            first_cascade_far_bound: 2.0 * size,
            ..default()
        }
        .build(),
        Transform::from_xyz(1.0, 2.0, 1.5).looking_at(Vec3::ZERO, Vec3::Y),
    ));
    commands.spawn((
        DirectionalLight {
            illuminance: 2_500.0,
            ..default()
        },
        Transform::from_xyz(-1.5, 0.5, 1.0).looking_at(Vec3::ZERO, Vec3::Y),
    ));
    commands.spawn((
        DirectionalLight {
            illuminance: 3_000.0,
            ..default()
        },
        Transform::from_xyz(0.0, 1.0, -2.0).looking_at(Vec3::ZERO, Vec3::Y),
    ));

    // a simple sky for the metals to reflect: bright above, dark below
    let env = EnvironmentMapLight::hemispherical_gradient(
        &mut images,
        Color::srgb(0.95, 0.97, 1.0),
        Color::srgb(0.45, 0.46, 0.5),
        Color::srgb(0.08, 0.08, 0.08),
    )
    .with_intensity(1200.0);
    // Bevy's default near plane (0.1 m) is the size of a small motor
    let projection = Projection::Perspective(PerspectiveProjection {
        near: size * 0.005,
        far: size * 200.0,
        ..default()
    });
    let cam = (
        Camera3d::default(),
        projection,
        Msaa::Sample4,
        orbit.transform(),
        env,
    );
    match headless {
        Some(h) => {
            let mut img =
                Image::new_target_texture(h.size.x, h.size.y, TextureFormat::Rgba8UnormSrgb, None);
            img.texture_descriptor.usage |= TextureUsages::COPY_SRC;
            let handle = images.add(img);
            commands.spawn((cam, RenderTarget::Image(handle.clone().into())));
            commands.insert_resource(HeadlessTarget(handle));
        }
        None => {
            commands.spawn(cam);
        }
    }
}

/// Spins the rotating group, places parts for the exploded view, applies role visibility and the cutaway.
pub fn spin(
    time: Res<Time>,
    mut state: ResMut<TwinState>,
    mut q: Query<(&PartView, &mut Transform, &mut Visibility, &mut Mesh3d)>,
    orbit: Res<Orbit>,
) {
    let w = state.rpm as f64 / 60.0 * std::f64::consts::TAU * state.slow_motion as f64;
    state.angle = (state.angle + w * time.delta_secs_f64()).rem_euclid(std::f64::consts::TAU);
    let rot = Quat::from_rotation_y(state.angle as f32);
    for (p, mut t, mut vis, mut mesh) in &mut q {
        let off = Vec3::Y * state.explode * p.explode_by;
        let r = if p.rotating { rot } else { Quat::IDENTITY };
        *t = Transform {
            translation: off,
            rotation: r,
            scale: Vec3::ONE,
        };
        let mut shown = state.shown.get(&p.role).copied().unwrap_or(true);
        let want = match (&p.cut, state.cutaway) {
            (Some(cut), true) => cut,
            _ => &p.full,
        };
        if mesh.0 != *want {
            mesh.0 = want.clone();
        }
        if state.cutaway && !p.around {
            let c = r * p.centroid;
            shown &= Vec3::new(c.x, 0.0, c.z).dot(orbit.cut_normal) <= 0.0;
        }
        let v = if shown {
            Visibility::Inherited
        } else {
            Visibility::Hidden
        };
        if *vis != v {
            *vis = v;
        }
    }
}

pub fn orbit_input(
    buttons: Res<ButtonInput<MouseButton>>,
    motion: Res<AccumulatedMouseMotion>,
    scroll: Res<AccumulatedMouseScroll>,
    wants: Option<Res<bevy_egui::input::EguiWantsInput>>,
    mut orbit: ResMut<Orbit>,
) {
    if wants.is_some_and(|w| w.wants_any_pointer_input()) {
        return;
    }
    let d = motion.delta;
    if buttons.pressed(MouseButton::Left) {
        orbit.yaw -= d.x * 0.008;
        orbit.pitch = (orbit.pitch + d.y * 0.008).clamp(-1.5, 1.5);
    }
    if buttons.pressed(MouseButton::Right) || buttons.pressed(MouseButton::Middle) {
        let t = orbit.transform();
        let k = orbit.dist * 0.0015;
        orbit.target += (-t.right() * d.x + t.up() * d.y) * k;
    }
    if scroll.delta.y != 0.0 {
        orbit.dist *= (1.0 - scroll.delta.y * 0.1).clamp(0.5, 1.5);
    }
}

pub fn apply_orbit(orbit: Res<Orbit>, mut cams: Query<&mut Transform, With<Camera3d>>) {
    if orbit.is_changed() {
        for mut t in &mut cams {
            *t = orbit.transform();
        }
    }
}

/// Headless: after the warm-up frames, capture the image target, save it, exit.
pub fn headless_capture(
    mut commands: Commands,
    headless: Option<ResMut<Headless>>,
    target: Option<Res<HeadlessTarget>>,
    mut frame: Local<u32>,
) {
    let (Some(mut h), Some(target)) = (headless, target) else {
        return;
    };
    *frame += 1;
    if h.taken || *frame < h.frames {
        return;
    }
    h.taken = true;
    let mut save = save_to_disk(h.out.clone());
    let out = h.out.clone();
    commands.spawn(Screenshot::image(target.0.clone())).observe(
        move |cap: On<ScreenshotCaptured>, mut exit: MessageWriter<AppExit>| {
            save(cap);
            println!("screenshot {}", out.display());
            exit.write(AppExit::Success);
        },
    );
}
