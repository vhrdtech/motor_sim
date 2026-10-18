//! The scene the twin loads: classified parts with their meshes, colours and roles, the motor axis, slot and pole
//! counts. Saved as binary glTF (`.glb`) so any viewer shows it: one node per part under a root that turns the CAD
//! frame (Z up) into glTF's Y up, a PBR material per part, and the classification in the nodes' `extras`
//! (`role`, `confidence`, `rotating`, `polarity`, `path`, `instance`) and the root's (`motor_cad`: axis, topology,
//! slots, poles).

use crate::classify::{Classification, Topology};
use crate::geometry::Axis;
use crate::mesh::Mesh;
use crate::role::{Polarity, Role};
use crate::step_read::CadModel;
use serde::{Deserialize, Serialize};
use serde_json::json;

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ScenePart {
    pub name: String,
    pub instance: String,
    pub path: Vec<String>,
    pub role: Role,
    pub confidence: f64,
    pub rotating: bool,
    pub polarity: Option<Polarity>,
    /// sRGB 0..1: the CAD colour, else the role's
    pub colour: [f32; 3],
    /// the colour came from the CAD file
    pub cad_colour: bool,
    /// m, CAD frame
    pub mesh: Mesh,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Scene {
    pub name: String,
    pub axis: Axis,
    pub topology: Topology,
    pub slots: Option<usize>,
    pub poles: Option<usize>,
    pub parts: Vec<ScenePart>,
}

/// The convention's colour for a role (sRGB), for parts without a CAD colour.
pub fn role_colour(role: Role, polarity: Option<Polarity>) -> [f32; 3] {
    use crate::generate::colour as c;
    let rgb = match (role, polarity) {
        (Role::Magnet, Some(Polarity::N)) => c::MAGNET_N,
        (Role::Magnet, Some(Polarity::S)) => c::MAGNET_S,
        (Role::Magnet, None) => c::MAGNET,
        (Role::Winding, _) => c::COPPER,
        (Role::Stator, _) | (Role::Rotor, _) => c::LAMINATION,
        (Role::Shaft, _) => c::SHAFT,
        (Role::Bearing, _) => c::BEARING,
        (Role::Housing, _) => c::ALUMINIUM,
        (Role::Fastener, _) => c::FASTENER,
        (Role::Pcb, _) => c::PCB,
        (Role::Wiring, _) => [0.15, 0.15, 0.15],
        (Role::Sensor, _) => [0.2, 0.2, 0.25],
        (Role::Other, _) => [0.6, 0.6, 0.6],
    };
    rgb.map(|x| x as f32)
}

/// PBR (metallic, perceptual roughness) by role.
pub fn role_pbr(role: Role) -> (f32, f32) {
    match role {
        Role::Winding => (1.0, 0.35),
        Role::Shaft | Role::Bearing => (1.0, 0.2),
        Role::Magnet => (0.7, 0.35),
        Role::Stator | Role::Rotor => (0.9, 0.45),
        Role::Housing => (0.85, 0.4),
        Role::Fastener => (0.8, 0.5),
        Role::Pcb | Role::Wiring | Role::Sensor | Role::Other => (0.0, 0.6),
    }
}

fn srgb_to_linear(c: f32) -> f32 {
    if c <= 0.04045 {
        c / 12.92
    } else {
        ((c + 0.055) / 1.055).powf(2.4)
    }
}

fn linear_to_srgb(c: f32) -> f32 {
    if c <= 0.003_130_8 {
        c * 12.92
    } else {
        1.055 * c.powf(1.0 / 2.4) - 0.055
    }
}

#[derive(Debug, thiserror::Error)]
pub enum SceneError {
    #[error("glTF: {0}")]
    Gltf(String),
}

impl Scene {
    pub fn build(name: &str, model: &CadModel, c: &Classification) -> Scene {
        let parts = model
            .parts
            .iter()
            .zip(&c.verdicts)
            .map(|(p, v)| ScenePart {
                name: p.name.clone(),
                instance: p.instance.clone(),
                path: p.path.clone(),
                role: v.role,
                confidence: v.confidence,
                rotating: v.rotating,
                polarity: v.polarity,
                colour: p.colour.unwrap_or_else(|| role_colour(v.role, v.polarity)),
                cad_colour: p.colour.is_some(),
                mesh: p.mesh.clone(),
            })
            .collect();
        Scene {
            name: name.into(),
            axis: c.axis.clone(),
            topology: c.topology,
            slots: c.slots,
            poles: c.poles,
            parts,
        }
    }

    /// Binary glTF 2.0.
    pub fn to_glb(&self) -> Vec<u8> {
        let mut bin: Vec<u8> = Vec::new();
        let (mut views, mut accessors, mut meshes, mut materials, mut nodes) =
            (vec![], vec![], vec![], vec![], vec![]);
        let mut push_view = |bin: &mut Vec<u8>, bytes: &[u8], target: u32| {
            while bin.len() % 4 != 0 {
                bin.push(0);
            }
            let off = bin.len();
            bin.extend_from_slice(bytes);
            views.push(json!({"buffer": 0, "byteOffset": off, "byteLength": bytes.len(), "target": target}));
            views.len() - 1
        };
        for (i, p) in self.parts.iter().enumerate() {
            let m = &p.mesh;
            let (mut lo, mut hi) = ([f32::INFINITY; 3], [f32::NEG_INFINITY; 3]);
            for v in &m.positions {
                for k in 0..3 {
                    lo[k] = lo[k].min(v[k]);
                    hi[k] = hi[k].max(v[k]);
                }
            }
            let pos: Vec<u8> = m
                .positions
                .iter()
                .flat_map(|v| v.iter().flat_map(|x| x.to_le_bytes()))
                .collect();
            let nor: Vec<u8> = m
                .normals
                .iter()
                .flat_map(|v| v.iter().flat_map(|x| x.to_le_bytes()))
                .collect();
            let idx: Vec<u8> = m.indices.iter().flat_map(|x| x.to_le_bytes()).collect();
            let (vp, vn, vi) = (
                push_view(&mut bin, &pos, 34962),
                push_view(&mut bin, &nor, 34962),
                push_view(&mut bin, &idx, 34963),
            );
            let a = accessors.len();
            accessors.push(json!({"bufferView": vp, "componentType": 5126, "count": m.positions.len(), "type": "VEC3", "min": lo, "max": hi}));
            accessors.push(json!({"bufferView": vn, "componentType": 5126, "count": m.normals.len(), "type": "VEC3"}));
            accessors.push(json!({"bufferView": vi, "componentType": 5125, "count": m.indices.len(), "type": "SCALAR"}));
            let (metal, rough) = role_pbr(p.role);
            let lin = p.colour.map(srgb_to_linear);
            materials.push(json!({
                "name": format!("{} {}", p.role, i),
                "pbrMetallicRoughness": {"baseColorFactor": [lin[0], lin[1], lin[2], 1.0], "metallicFactor": metal, "roughnessFactor": rough},
                "doubleSided": true
            }));
            meshes.push(json!({"name": p.name, "primitives": [{"attributes": {"POSITION": a, "NORMAL": a + 1}, "indices": a + 2, "material": i}]}));
            nodes.push(json!({
                "name": if p.instance.is_empty() { p.name.clone() } else { format!("{} ({})", p.name, p.instance) },
                "mesh": i,
                "extras": {
                    "role": p.role, "confidence": p.confidence, "rotating": p.rotating, "polarity": p.polarity,
                    "part": p.name, "instance": p.instance, "path": p.path, "cad_colour": p.cad_colour
                }
            }));
        }
        let children: Vec<usize> = (0..nodes.len()).collect();
        // Z up (CAD) -> Y up (glTF): -90° about X
        let s = std::f64::consts::FRAC_1_SQRT_2;
        nodes.push(json!({
            "name": self.name,
            "rotation": [-s, 0.0, 0.0, s],
            "children": children,
            "extras": {"motor_cad": {
                "version": 1,
                "axis": {"origin": self.axis.origin, "dir": self.axis.dir, "how": self.axis.how},
                "topology": self.topology, "slots": self.slots, "poles": self.poles
            }}
        }));
        while bin.len() % 4 != 0 {
            bin.push(0);
        }
        let doc = json!({
            "asset": {"version": "2.0", "generator": concat!("motor_cad ", env!("CARGO_PKG_VERSION"))},
            "scene": 0,
            "scenes": [{"name": self.name, "nodes": [nodes.len() - 1]}],
            "nodes": nodes, "meshes": meshes, "materials": materials,
            "accessors": accessors, "bufferViews": views,
            "buffers": [{"byteLength": bin.len()}]
        });
        let mut js = serde_json::to_vec(&doc).expect("glTF JSON");
        while js.len() % 4 != 0 {
            js.push(b' ');
        }
        let total = 12 + 8 + js.len() + 8 + bin.len();
        let mut out = Vec::with_capacity(total);
        out.extend_from_slice(b"glTF");
        out.extend_from_slice(&2u32.to_le_bytes());
        out.extend_from_slice(&(total as u32).to_le_bytes());
        out.extend_from_slice(&(js.len() as u32).to_le_bytes());
        out.extend_from_slice(b"JSON");
        out.extend_from_slice(&js);
        out.extend_from_slice(&(bin.len() as u32).to_le_bytes());
        out.extend_from_slice(b"BIN\0");
        out.extend_from_slice(&bin);
        out
    }

    /// Reads a `.glb` written by [`Scene::to_glb`]; other glTF files load too, with roles from their extras when
    /// present (else `other`) and the axis along the root's Z.
    pub fn from_glb(bytes: &[u8]) -> Result<Scene, SceneError> {
        let g = gltf::Gltf::from_slice(bytes).map_err(|e| SceneError::Gltf(e.to_string()))?;
        let blob = g
            .blob
            .as_deref()
            .ok_or_else(|| SceneError::Gltf("no binary chunk".into()))?;
        let meta: serde_json::Value = g
            .nodes()
            .filter_map(|n| {
                n.extras()
                    .as_ref()
                    .and_then(|e| serde_json::from_str::<serde_json::Value>(e.get()).ok())
            })
            .find_map(|e| e.get("motor_cad").cloned())
            .unwrap_or_default();
        let axis = meta
            .get("axis")
            .and_then(|a| {
                Some(Axis {
                    origin: serde_json::from_value(a.get("origin")?.clone()).ok()?,
                    dir: serde_json::from_value(a.get("dir")?.clone()).ok()?,
                    how: a.get("how").and_then(|h| h.as_str()).unwrap_or("").into(),
                })
            })
            .unwrap_or(Axis {
                origin: [0.0; 3],
                dir: [0.0, 0.0, 1.0],
                how: "assumed Z".into(),
            });
        let mut parts = Vec::new();
        for node in g.nodes() {
            let Some(mesh) = node.mesh() else { continue };
            let ex: serde_json::Value = node
                .extras()
                .as_ref()
                .map(|e| serde_json::from_str(e.get()).unwrap_or_default())
                .unwrap_or_default();
            let role: Role = ex
                .get("role")
                .and_then(|r| serde_json::from_value(r.clone()).ok())
                .unwrap_or(Role::Other);
            let polarity: Option<Polarity> = ex
                .get("polarity")
                .and_then(|r| serde_json::from_value(r.clone()).ok());
            let mut m = Mesh::default();
            let mut colour = role_colour(role, polarity);
            for prim in mesh.primitives() {
                let reader = prim.reader(|_| Some(blob));
                let base = m.positions.len() as u32;
                let pos: Vec<[f32; 3]> = reader
                    .read_positions()
                    .map(|i| i.collect())
                    .unwrap_or_default();
                let n = pos.len();
                m.positions.extend(pos);
                m.normals.extend(
                    reader
                        .read_normals()
                        .map(|i| i.collect::<Vec<_>>())
                        .unwrap_or_else(|| vec![[0.0, 0.0, 1.0]; n]),
                );
                match reader.read_indices() {
                    Some(ix) => m.indices.extend(ix.into_u32().map(|i| i + base)),
                    None => m.indices.extend((0..n as u32).map(|i| i + base)),
                }
                let c = prim.material().pbr_metallic_roughness().base_color_factor();
                colour = [c[0], c[1], c[2]].map(linear_to_srgb);
            }
            // node transforms below the root (ours have none)
            let t = node.transform().matrix();
            let mat = [0, 1, 2, 3].map(|c| [0, 1, 2, 3].map(|r| t[c][r] as f64));
            m.transform(&mat);
            let s = |k: &str| ex.get(k).and_then(|v| v.as_str()).unwrap_or("").to_string();
            parts.push(ScenePart {
                name: if s("part").is_empty() {
                    node.name().unwrap_or("").to_string()
                } else {
                    s("part")
                },
                instance: s("instance"),
                path: ex
                    .get("path")
                    .and_then(|p| serde_json::from_value(p.clone()).ok())
                    .unwrap_or_default(),
                role,
                confidence: ex.get("confidence").and_then(|v| v.as_f64()).unwrap_or(0.0),
                rotating: ex
                    .get("rotating")
                    .and_then(|v| v.as_bool())
                    .unwrap_or(role.rotates()),
                polarity,
                colour,
                cad_colour: ex
                    .get("cad_colour")
                    .and_then(|v| v.as_bool())
                    .unwrap_or(true),
                mesh: m,
            });
        }
        let get = |k: &str| meta.get(k).and_then(|v| v.as_u64()).map(|x| x as usize);
        Ok(Scene {
            name: g
                .scenes()
                .next()
                .and_then(|s| s.name().map(String::from))
                .unwrap_or_default(),
            axis,
            topology: meta
                .get("topology")
                .and_then(|t| serde_json::from_value(t.clone()).ok())
                .unwrap_or(Topology::Unknown),
            slots: get("slots"),
            poles: get("poles"),
            parts,
        })
    }
}
