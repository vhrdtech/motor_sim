//! STEP import: the assembly tree flattened into parts (one per solid per instance), each with its product and
//! instance names, the names of the assemblies above it, its colour and layer, and a triangle mesh in the root's
//! frame in metres.
//!
//! monstertruck reads the B-rep and the product structure (`Table`, `step_assy`) and meshes the solids; colours,
//! layers, length units and string escapes come from our own walk over the parsed P21 records, which monstertruck
//! does not interpret.

use crate::mesh::Mesh;
use monstertruck::meshing::prelude::*;
use monstertruck::step::load::Table;
use rayon::prelude::*;
use std::collections::{HashMap, HashSet};
use step_p21::ast::{EntityInstance, Name, Parameter, Record};

/// One imported part: a solid of one product, as placed by one path through the assembly.
#[derive(Clone, Debug)]
pub struct Part {
    /// product name (`PRODUCT.name`), decoded
    pub name: String,
    /// instance name of the last link (`NEXT_ASSEMBLY_USAGE_OCCURRENCE.name`), empty at the root
    pub instance: String,
    /// product names from the root down to the parent
    pub path: Vec<String>,
    /// `PRODUCT.description`
    pub description: String,
    /// the product's identity in the file: instances of one product share it
    pub product_key: u64,
    /// index of this solid among the product's solids, and how many it has
    pub body: usize,
    pub bodies: usize,
    /// sRGB 0..1
    pub colour: Option<[f32; 3]>,
    pub layer: Option<String>,
    /// m, in the root frame
    pub mesh: Mesh,
}

/// A whole imported file.
#[derive(Clone, Debug, Default)]
pub struct CadModel {
    pub parts: Vec<Part>,
    /// FILE_SCHEMA, e.g. AUTOMOTIVE_DESIGN (AP214) or AP242_MANAGED_MODEL_BASED_3D_ENGINEERING_MIM_LF
    pub schema: String,
    /// FILE_NAME originating system
    pub system: String,
    /// metres per model length unit
    pub unit_m: f64,
    /// what was lost or guessed on the way
    pub notes: Vec<String>,
}

#[derive(Debug, thiserror::Error)]
pub enum ImportError {
    #[error("reading {0}: {1}")]
    Io(String, std::io::Error),
    #[error("STEP parse: {0}")]
    Parse(String),
}

/// Decodes a P21 string literal's content: `''`, `\\`, `\X2\...\X0\` (UTF-16), `\X4\...\X0\` (UTF-32), `\X\hh`
/// (ISO 8859-1) and `\S\c` (high half of ISO 8859-1).
pub fn decode_p21(s: &str) -> String {
    let b = s.as_bytes();
    let mut out = String::new();
    let mut i = 0;
    let hex =
        |h: &[u8]| u32::from_str_radix(std::str::from_utf8(h).unwrap_or("0"), 16).unwrap_or(0xFFFD);
    while i < b.len() {
        if b[i] == b'\\' && b[i..].starts_with(b"\\X2\\") || b[i..].starts_with(b"\\X4\\") {
            let width = if b[i + 2] == b'2' { 4 } else { 8 };
            i += 4;
            let mut units = Vec::new();
            while i + width <= b.len() && !b[i..].starts_with(b"\\X0\\") {
                units.push(hex(&b[i..i + width]));
                i += width;
            }
            i += 4;
            if width == 4 {
                let u16s: Vec<u16> = units.iter().map(|&u| u as u16).collect();
                out.push_str(&String::from_utf16_lossy(&u16s));
            } else {
                out.extend(units.iter().filter_map(|&u| char::from_u32(u)));
            }
        } else if b[i..].starts_with(b"\\X\\") && i + 5 <= b.len() {
            out.extend(char::from_u32(hex(&b[i + 3..i + 5])));
            i += 5;
        } else if b[i..].starts_with(b"\\S\\") && i + 4 <= b.len() {
            out.extend(char::from_u32(b[i + 3] as u32 + 128));
            i += 4;
        } else if b[i..].starts_with(b"\\\\") {
            out.push('\\');
            i += 2;
        } else if b[i..].starts_with(b"''") {
            out.push('\'');
            i += 2;
        } else {
            let c = s[i..].chars().next().unwrap();
            out.push(c);
            i += c.len_utf8();
        }
    }
    out
}

/// The parsed records, by entity id; a complex instance keeps all its partial records.
struct Raw {
    recs: HashMap<u64, Vec<Record>>,
}

fn refs(p: &Parameter, out: &mut Vec<u64>) {
    match p {
        Parameter::Ref(Name::Entity(i)) => out.push(*i),
        Parameter::List(v) => v.iter().for_each(|x| refs(x, out)),
        Parameter::Typed { parameter, .. } => refs(parameter, out),
        _ => {}
    }
}

fn params(r: &Record) -> &[Parameter] {
    match &r.parameter {
        Parameter::List(v) => v,
        _ => &[],
    }
}

fn real(p: &Parameter) -> Option<f64> {
    match p {
        Parameter::Real(x) => Some(*x),
        Parameter::Integer(i) => Some(*i as f64),
        Parameter::Typed { parameter, .. } => real(parameter),
        _ => None,
    }
}

fn string(p: &Parameter) -> Option<&str> {
    match p {
        Parameter::String(s) => Some(s),
        _ => None,
    }
}

impl Raw {
    fn new(entities: &[EntityInstance]) -> Raw {
        let mut recs = HashMap::with_capacity(entities.len());
        for e in entities {
            match e {
                EntityInstance::Simple { id, record } => {
                    recs.insert(*id, vec![record.clone()]);
                }
                EntityInstance::Complex { id, subsuper } => {
                    recs.insert(*id, subsuper.0.clone());
                }
            }
        }
        Raw { recs }
    }

    fn rec(&self, id: u64, name: &str) -> Option<&Record> {
        self.recs.get(&id)?.iter().find(|r| r.name == name)
    }

    fn is(&self, id: u64, name: &str) -> bool {
        self.rec(id, name).is_some()
    }

    fn of_type<'a>(&'a self, name: &'a str) -> impl Iterator<Item = (u64, &'a Record)> + 'a {
        self.recs
            .iter()
            .filter_map(move |(id, v)| v.iter().find(|r| r.name == name).map(|r| (*id, r)))
    }

    fn all_refs(&self, id: u64) -> Vec<u64> {
        let mut out = Vec::new();
        if let Some(v) = self.recs.get(&id) {
            for r in v {
                refs(&r.parameter, &mut out);
            }
        }
        out
    }

    /// The colour a style reference leads to: a bounded walk down to `COLOUR_RGB` or a pre-defined colour.
    fn colour_below(&self, start: u64) -> Option<[f32; 3]> {
        let mut stack = vec![(start, 0)];
        let mut seen = HashSet::new();
        while let Some((id, depth)) = stack.pop() {
            if depth > 10 || !seen.insert(id) {
                continue;
            }
            if let Some(r) = self.rec(id, "COLOUR_RGB") {
                let p = params(r);
                if let (Some(r), Some(g), Some(b)) = (
                    p.get(1).and_then(real),
                    p.get(2).and_then(real),
                    p.get(3).and_then(real),
                ) {
                    return Some([r as f32, g as f32, b as f32]);
                }
            }
            if let Some(r) = self.rec(id, "DRAUGHTING_PRE_DEFINED_COLOUR") {
                return params(r)
                    .first()
                    .and_then(string)
                    .and_then(predefined_colour);
            }
            // curve styles colour the edges, not the surface
            if self.is(id, "CURVE_STYLE") {
                continue;
            }
            for c in self.all_refs(id) {
                stack.push((c, depth + 1));
            }
        }
        None
    }

    /// Length unit in metres from the first length `SI_UNIT` or a conversion-based inch/foot.
    fn length_unit(&self) -> Option<f64> {
        for v in self.recs.values() {
            if !v.iter().any(|r| r.name == "LENGTH_UNIT") {
                continue;
            }
            if let Some(si) = v.iter().find(|r| r.name == "SI_UNIT") {
                let prefix = match params(si).first() {
                    Some(Parameter::Enumeration(e)) => e.as_str(),
                    _ => "",
                };
                return Some(match prefix {
                    "MILLI" => 1e-3,
                    "CENTI" => 1e-2,
                    "DECI" => 1e-1,
                    "KILO" => 1e3,
                    "MICRO" => 1e-6,
                    _ => 1.0,
                });
            }
            if let Some(cb) = v.iter().find(|r| r.name == "CONVERSION_BASED_UNIT") {
                let n = params(cb)
                    .first()
                    .and_then(string)
                    .unwrap_or("")
                    .to_ascii_lowercase();
                if n.contains("inch") {
                    return Some(0.0254);
                }
                if n.contains("foot") {
                    return Some(0.3048);
                }
            }
        }
        None
    }
}

fn predefined_colour(name: &str) -> Option<[f32; 3]> {
    Some(match name {
        "red" => [1.0, 0.0, 0.0],
        "green" => [0.0, 1.0, 0.0],
        "blue" => [0.0, 0.0, 1.0],
        "yellow" => [1.0, 1.0, 0.0],
        "magenta" => [1.0, 0.0, 1.0],
        "cyan" => [0.0, 1.0, 1.0],
        "black" => [0.0, 0.0, 0.0],
        "white" => [1.0, 1.0, 1.0],
        _ => return None,
    })
}

/// Colour and layer per solid: a style on the solid wins, else the most common style of its faces, else its shell's.
fn styles(raw: &Raw) -> (HashMap<u64, [f32; 3]>, HashMap<u64, String>) {
    // owner solid of each shell and face
    let mut owner: HashMap<u64, u64> = HashMap::new();
    for (sid, r) in raw
        .of_type("MANIFOLD_SOLID_BREP")
        .chain(raw.of_type("BREP_WITH_VOIDS"))
    {
        owner.insert(sid, sid);
        let mut shells = Vec::new();
        refs(&r.parameter, &mut shells);
        for sh in shells {
            owner.insert(sh, sid);
            let inner = if raw.is(sh, "ORIENTED_CLOSED_SHELL") {
                raw.all_refs(sh)
            } else {
                vec![sh]
            };
            for sh in inner {
                owner.insert(sh, sid);
                for f in raw.all_refs(sh) {
                    if raw.is(f, "ADVANCED_FACE") || raw.is(f, "FACE_SURFACE") {
                        owner.insert(f, sid);
                    }
                }
            }
        }
    }
    let mut solid_colour = HashMap::new();
    let mut face_votes: HashMap<u64, HashMap<[u32; 3], usize>> = HashMap::new();
    for kind in ["STYLED_ITEM", "OVER_RIDING_STYLED_ITEM"] {
        for (_, r) in raw.of_type(kind) {
            let p = params(r);
            let Some(Parameter::Ref(Name::Entity(item))) = p.get(2) else {
                continue;
            };
            let Some(&solid) = owner.get(item) else {
                continue;
            };
            let mut style_refs = Vec::new();
            if let Some(s) = p.get(1) {
                refs(s, &mut style_refs);
            }
            let Some(c) = style_refs.iter().find_map(|&s| raw.colour_below(s)) else {
                continue;
            };
            if *item == solid {
                solid_colour.insert(solid, c);
            } else {
                *face_votes
                    .entry(solid)
                    .or_default()
                    .entry(c.map(|x| x.to_bits()))
                    .or_default() += 1;
            }
        }
    }
    for (solid, votes) in face_votes {
        if let Some((c, _)) = votes.into_iter().max_by_key(|(_, n)| *n) {
            solid_colour.entry(solid).or_insert(c.map(f32::from_bits));
        }
    }
    let mut layer = HashMap::new();
    for (_, r) in raw.of_type("PRESENTATION_LAYER_ASSIGNMENT") {
        let p = params(r);
        let Some(name) = p.first().and_then(string) else {
            continue;
        };
        let mut items = Vec::new();
        if let Some(it) = p.get(2) {
            refs(it, &mut items);
        }
        for it in items {
            if let Some(&s) = owner.get(&it) {
                layer.entry(s).or_insert_with(|| decode_p21(name));
            }
        }
    }
    (solid_colour, layer)
}

/// Solids of a shape representation: its own solid items plus those of representations tied to it by a plain
/// `SHAPE_REPRESENTATION_RELATIONSHIP` (no transformation; FreeCAD/OCCT put the B-rep in a separate
/// `ADVANCED_BREP_SHAPE_REPRESENTATION` this way).
fn solids_of_rep(raw: &Raw, rel: &HashMap<u64, Vec<u64>>, sr: u64) -> Vec<u64> {
    let mut out = Vec::new();
    let mut stack = vec![sr];
    let mut seen = HashSet::new();
    while let Some(r) = stack.pop() {
        if !seen.insert(r) {
            continue;
        }
        let items = raw
            .recs
            .get(&r)
            .and_then(|v| v.iter().find(|x| x.name.ends_with("SHAPE_REPRESENTATION")))
            .map(|x| {
                let mut v = Vec::new();
                if let Some(p) = params(x).get(1) {
                    refs(p, &mut v);
                }
                v
            })
            .unwrap_or_default();
        for it in items {
            if raw.is(it, "MANIFOLD_SOLID_BREP") || raw.is(it, "BREP_WITH_VOIDS") {
                out.push(it);
            }
        }
        if let Some(n) = rel.get(&r) {
            stack.extend(n);
        }
    }
    out.sort_unstable();
    out.dedup();
    out
}

/// Meshes one solid (model units) with a chordal tolerance relative to its size.
fn mesh_solid(table: &Table, raw_id: u64, unit_m: f64) -> Result<Mesh, String> {
    let s = table
        .manifold_solid_brep
        .get(&raw_id)
        .ok_or("not a manifold solid")?;
    let solid = table.to_compressed_solid(s).map_err(|e| e.to_string())?;
    let mut out = Mesh::default();
    for shell in &solid.boundaries {
        let bb: BoundingBox<Point3> = shell.vertices.iter().collect();
        // 0.2 % of the shell's size, never finer than 20 µm
        let tol = f64::max(bb.diameter() * 0.002, 2e-5 / unit_m);
        let poly = shell.robust_triangulation(tol).to_polygon();
        let mut map: HashMap<(usize, Option<usize>), u32> = HashMap::new();
        let mut m = Mesh::default();
        let (pos, nor) = (poly.positions(), poly.normals());
        for face in poly.face_iter() {
            let ids: Vec<u32> = face
                .iter()
                .map(|v| {
                    *map.entry((v.pos, v.nor)).or_insert_with(|| {
                        let p = pos[v.pos];
                        m.positions.push([p.x as f32, p.y as f32, p.z as f32]);
                        let n = v.nor.map(|i| nor[i]).unwrap_or(Vector3::new(0.0, 0.0, 0.0));
                        m.normals.push([n.x as f32, n.y as f32, n.z as f32]);
                        m.positions.len() as u32 - 1
                    })
                })
                .collect();
            for k in 1..ids.len().saturating_sub(1) {
                m.indices.extend([ids[0], ids[k], ids[k + 1]]);
            }
        }
        fill_missing_normals(&mut m);
        out.append(&m);
    }
    Ok(out)
}

/// Flat normals where the mesher gave none.
fn fill_missing_normals(m: &mut Mesh) {
    let mut acc = vec![[0.0f64; 3]; m.positions.len()];
    for t in m.indices.as_chunks::<3>().0 {
        let p = |i: u32| m.positions[i as usize].map(|x| x as f64);
        let n = crate::mesh::cross(
            crate::mesh::sub(p(t[1]), p(t[0])),
            crate::mesh::sub(p(t[2]), p(t[0])),
        );
        for &i in t {
            acc[i as usize] = crate::mesh::add(acc[i as usize], n);
        }
    }
    for (n, a) in m.normals.iter_mut().zip(acc) {
        if n.iter().all(|x| *x == 0.0) {
            *n = crate::mesh::normalize(a).map(|x| x as f32);
        }
    }
}

/// Reads a STEP file.
pub fn read_step_file(path: &std::path::Path) -> Result<CadModel, ImportError> {
    let bytes = std::fs::read(path).map_err(|e| ImportError::Io(path.display().to_string(), e))?;
    read_step_bytes(&bytes)
}

/// Reads STEP text (UTF-8, or ISO 8859-1 where it is not).
pub fn read_step_bytes(bytes: &[u8]) -> Result<CadModel, ImportError> {
    let text: std::borrow::Cow<str> = match std::str::from_utf8(bytes) {
        Ok(s) => s.into(),
        Err(_) => bytes.iter().map(|&b| b as char).collect::<String>().into(),
    };
    let exchange = step_p21::parser::parse(&text).map_err(|e| ImportError::Parse(e.to_string()))?;
    let data = exchange
        .data
        .first()
        .ok_or_else(|| ImportError::Parse("no DATA section".into()))?;
    let raw = Raw::new(&data.entities);
    let table = Table::from_data_section(data);

    let mut model = CadModel {
        unit_m: raw.length_unit().unwrap_or(1e-3),
        ..Default::default()
    };
    for h in &exchange.header {
        let p = params(h);
        match h.name.as_str() {
            "FILE_SCHEMA" => {
                if let Some(Parameter::List(v)) = p.first() {
                    model.schema = v.iter().filter_map(string).collect::<Vec<_>>().join(", ");
                }
            }
            "FILE_NAME" => {
                model.system = p
                    .get(5)
                    .and_then(string)
                    .map(decode_p21)
                    .unwrap_or_default()
            }
            _ => {}
        }
    }
    if raw.length_unit().is_none() {
        model.notes.push("no length unit found, assumed mm".into());
    }
    let (colours, layers) = styles(&raw);

    // plain representation relationships, both ways
    let mut rel: HashMap<u64, Vec<u64>> = HashMap::new();
    for (id, v) in &raw.recs {
        if v.iter()
            .any(|r| r.name == "REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION")
        {
            continue;
        }
        if let Some(r) = v
            .iter()
            .find(|r| r.name == "SHAPE_REPRESENTATION_RELATIONSHIP")
        {
            let p = params(r);
            if let (Some(Parameter::Ref(Name::Entity(a))), Some(Parameter::Ref(Name::Entity(b)))) =
                (p.get(2), p.get(3))
            {
                rel.entry(*a).or_default().push(*b);
                rel.entry(*b).or_default().push(*a);
            }
        }
        let _ = id;
    }

    // placed solids: (solid id, product key, names..., matrix)
    struct Placed {
        solid: u64,
        key: u64,
        name: String,
        description: String,
        instance: String,
        path: Vec<String>,
        body: usize,
        bodies: usize,
        matrix: [[f64; 4]; 4],
    }
    let mut placed = Vec::new();
    let mut seen_solids = HashSet::new();
    match table.step_assy() {
        Ok(assy) => {
            for top in assy.top_nodes() {
                for path in assy.paths_iter(top.index()) {
                    let node = path.terminal_node();
                    let attrs = node.attributes();
                    let Some(sr) = table.shape_representation_of_node(node.entity()) else {
                        continue;
                    };
                    let solids = solids_of_rep(&raw, &rel, sr);
                    if solids.is_empty() {
                        continue;
                    }
                    let m: Matrix4 = path.matrix_try().unwrap_or(Matrix4::identity());
                    let names: Vec<String> = path
                        .nodes()
                        .iter()
                        .map(|n| decode_p21(&n.attributes().name))
                        .collect();
                    let instance = path
                        .edges()
                        .last()
                        .map(|e| decode_p21(&e.attributes().name))
                        .unwrap_or_default();
                    for (k, &s) in solids.iter().enumerate() {
                        seen_solids.insert(s);
                        placed.push(Placed {
                            solid: s,
                            key: sr,
                            name: decode_p21(&attrs.name),
                            description: decode_p21(&attrs.description),
                            instance: instance.clone(),
                            path: names[..names.len() - 1].to_vec(),
                            body: k,
                            bodies: solids.len(),
                            matrix: m.into(),
                        });
                    }
                }
            }
        }
        Err(e) => model
            .notes
            .push(format!("no product structure ({e}): solids listed flat")),
    }
    // solids no product reaches
    let mut loose: Vec<u64> = table
        .manifold_solid_brep
        .keys()
        .copied()
        .filter(|s| !seen_solids.contains(s))
        .collect();
    loose.sort_unstable();
    if !loose.is_empty() && !placed.is_empty() {
        model.notes.push(format!(
            "{} solids outside the product structure, listed flat",
            loose.len()
        ));
    }
    let n_loose = loose.len();
    for (k, s) in loose.into_iter().enumerate() {
        placed.push(Placed {
            solid: s,
            key: s,
            name: format!("Solid {}", k + 1),
            description: String::new(),
            instance: String::new(),
            path: Vec::new(),
            body: k,
            bodies: n_loose,
            matrix: Matrix4::identity().into(),
        });
    }

    // mesh each solid once
    let mut unique: Vec<u64> = placed.iter().map(|p| p.solid).collect();
    unique.sort_unstable();
    unique.dedup();
    let meshes: HashMap<u64, Result<Mesh, String>> = unique
        .par_iter()
        .map(|&s| (s, mesh_solid(&table, s, model.unit_m)))
        .collect();
    let u = model.unit_m;
    let scale = [
        [u, 0.0, 0.0, 0.0],
        [0.0, u, 0.0, 0.0],
        [0.0, 0.0, u, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ];
    for p in placed {
        match &meshes[&p.solid] {
            Ok(m) => {
                let mut mesh = m.clone();
                mesh.transform(&p.matrix);
                mesh.transform(&scale);
                model.parts.push(Part {
                    name: p.name,
                    instance: p.instance,
                    path: p.path,
                    description: p.description,
                    product_key: p.key,
                    body: p.body,
                    bodies: p.bodies,
                    colour: colours.get(&p.solid).copied(),
                    layer: layers.get(&p.solid).cloned(),
                    mesh,
                });
            }
            Err(e) => model.notes.push(format!(
                "solid #{} of '{}' not converted: {e}",
                p.solid, p.name
            )),
        }
    }
    Ok(model)
}

trait MatrixTry {
    fn matrix_try(&self) -> Option<Matrix4>;
}

impl<'a, S> MatrixTry
    for monstertruck::assembly::dag::Path<
        'a,
        monstertruck::assembly::assy::NodeEntity<
            S,
            monstertruck::step::load::convert::PartAttributes,
        >,
        monstertruck::assembly::assy::EdgeEntity<
            monstertruck::step::load::convert::NodeMatrix,
            monstertruck::step::load::convert::PartAttributes,
        >,
    >
{
    /// The path's placement: the product of its links' transforms, root first.
    fn matrix_try(&self) -> Option<Matrix4> {
        let mut m = Matrix4::identity();
        for e in self.edges() {
            m = m * Matrix4::try_from(e.matrix()).ok()?;
        }
        Some(m)
    }
}
