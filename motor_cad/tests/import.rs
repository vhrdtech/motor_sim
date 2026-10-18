//! End to end: the parametric motors written to STEP, read back, classified; the scene through glTF; a real file.

use motor_cad::classify::{Classification, Topology, classify};
use motor_cad::overrides::Overrides;
use motor_cad::role::{Polarity, Role};
use motor_cad::scene::Scene;
use motor_cad::step_read::{CadModel, read_step_bytes};
use motor_cad::{generate, step_write};

fn round_trip(doc: &step_write::StepDoc) -> (CadModel, Classification) {
    let text = step_write::write_step(doc);
    let model = read_step_bytes(text.as_bytes()).unwrap();
    let c = classify(&model, &Overrides::default());
    (model, c)
}

fn count(m: &CadModel, c: &Classification, role: Role) -> usize {
    let _ = m;
    c.verdicts.iter().filter(|v| v.role == role).count()
}

#[test]
fn outrunner_follows_the_convention() {
    let (m, c) = round_trip(&generate::outrunner(&Default::default()));
    assert_eq!(m.parts.len(), 66);
    assert_eq!(c.topology, Topology::Outrunner);
    assert_eq!((c.slots, c.poles), (Some(12), Some(14)));
    assert!(c.axis.dir[2] > 0.999, "axis {:?}", c.axis.dir);
    for (p, v) in m.parts.iter().zip(&c.verdicts) {
        let tagged = motor_cad::role::parse_tag(&p.name).and_then(|t| t.role);
        // bearing rings and balls carry the tag of their sub-assembly in their product name
        assert_eq!(Some(v.role), tagged, "{}", p.name);
        assert!(v.confidence > 0.6, "{} {:.2}", p.name, v.confidence);
        assert_eq!(
            v.rotating,
            matches!(v.role, Role::Rotor | Role::Magnet | Role::Shaft),
            "{}",
            p.name
        );
    }
    let n = c
        .verdicts
        .iter()
        .filter(|v| v.polarity == Some(Polarity::N))
        .count();
    let s = c
        .verdicts
        .iter()
        .filter(|v| v.polarity == Some(Polarity::S))
        .count();
    assert_eq!((n, s), (7, 7));
}

#[test]
fn anonymised_outrunner_by_geometry_alone() {
    let o = generate::outrunner(&Default::default());
    let (m, c) = round_trip(&generate::anonymised(&o, "Motor"));
    assert_eq!(c.topology, Topology::Outrunner);
    assert_eq!((c.slots, c.poles), (Some(12), Some(14)));
    assert_eq!(count(&m, &c, Role::Winding), 12);
    assert_eq!(count(&m, &c, Role::Magnet), 14);
    assert_eq!(count(&m, &c, Role::Stator), 1);
    assert_eq!(count(&m, &c, Role::Shaft), 1);
    assert_eq!(count(&m, &c, Role::Rotor), 1);
    // 2 × (9 balls + 2 races)
    assert_eq!(count(&m, &c, Role::Bearing), 22);
    assert_eq!(count(&m, &c, Role::Wiring), 3);
    let rotating = c.verdicts.iter().filter(|v| v.rotating).count();
    assert_eq!(rotating, 14 + 1 + 1);
}

#[test]
fn stepper_by_keywords_and_geometry() {
    let (m, c) = round_trip(&generate::stepper(&Default::default()));
    assert_eq!(c.topology, Topology::Inrunner);
    assert_eq!(c.slots, Some(8));
    let role_of = |name: &str| {
        m.parts
            .iter()
            .zip(&c.verdicts)
            .find(|(p, _)| p.name == name)
            .map(|(_, v)| v.role)
            .unwrap()
    };
    assert_eq!(role_of("Stator lamination stack"), Role::Stator);
    assert_eq!(role_of("Phase coil"), Role::Winding);
    assert_eq!(role_of("Rotor cup front"), Role::Rotor);
    assert_eq!(role_of("Rotor magnet ring"), Role::Magnet);
    assert_eq!(role_of("625ZZ"), Role::Bearing);
    assert_eq!(role_of("Shaft 5 mm"), Role::Shaft);
    assert_eq!(role_of("Front flange"), Role::Housing);
    assert_eq!(role_of("Pan head screw M3x35"), Role::Fastener);
    assert_eq!(role_of("Connector JST PH 6"), Role::Wiring);
}

#[test]
fn overrides_win() {
    let (m, _) = round_trip(&generate::stepper(&Default::default()));
    let ov: Overrides =
        toml::from_str("[[part]]\nmatch = \"Rotor cup*\"\nrole = \"other\"\nrotating = false\n")
            .unwrap();
    let c = classify(&m, &ov);
    for (p, v) in m.parts.iter().zip(&c.verdicts) {
        if p.name.starts_with("Rotor cup") {
            assert_eq!(
                (v.role, v.rotating, v.confidence),
                (Role::Other, false, 1.0)
            );
        }
    }
}

#[test]
fn scene_survives_gltf() {
    let (m, c) = round_trip(&generate::outrunner(&Default::default()));
    let scene = Scene::build("outrunner", &m, &c);
    let back = Scene::from_glb(&scene.to_glb()).unwrap();
    assert_eq!(back.parts.len(), scene.parts.len());
    assert_eq!(
        (back.slots, back.poles, back.topology),
        (Some(12), Some(14), Topology::Outrunner)
    );
    for (a, b) in scene.parts.iter().zip(&back.parts) {
        assert_eq!(
            (a.role, a.rotating, a.polarity, &a.name),
            (b.role, b.rotating, b.polarity, &b.name)
        );
        assert_eq!(a.mesh.indices.len(), b.mesh.indices.len());
        for k in 0..3 {
            assert!((a.colour[k] - b.colour[k]).abs() < 1e-3);
        }
    }
}

#[test]
fn be4108_real_file() {
    let m = read_step_bytes(include_bytes!("data/BE4108.step")).unwrap();
    assert_eq!(m.parts.len(), 5);
    assert!((m.unit_m - 1e-3).abs() < 1e-12);
    let c = classify(&m, &Overrides::default());
    assert!(c.axis.dir[2].abs() > 0.99);
    let role_of = |name: &str| {
        m.parts
            .iter()
            .zip(&c.verdicts)
            .find(|(p, _)| p.name == name)
            .map(|(_, v)| v.role)
            .unwrap()
    };
    assert_eq!(role_of("MR115-2RS1"), Role::Bearing);
    assert_eq!(role_of("stator"), Role::Stator);
    assert_eq!(role_of("rotor"), Role::Rotor);
    assert_eq!(role_of("base"), Role::Housing);
    assert!(m.parts.iter().all(|p| p.colour.is_some()));
}
