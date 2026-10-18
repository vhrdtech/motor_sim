//! STEP (AP214) writer for assemblies of named, coloured parts.
//!
//! monstertruck writes the B-rep of solids (`StepModels`), but its assembly writer (`StepDesign`) cannot carry real
//! solids yet (a solid's entity numbers are fixed when it is built) and nothing writes colours. So this module takes
//! the solids' records from `StepModels`, drops its one-part product preamble (`#1`..`#15`) and writes the product
//! structure itself after them: one `PRODUCT` per part or sub-assembly, `NEXT_ASSEMBLY_USAGE_OCCURRENCE` instances
//! with placements, `STYLED_ITEM` colours and `PRESENTATION_LAYER_ASSIGNMENT` layers, the way mechanical CAD exports
//! them. The real fix belongs in monstertruck (solids at any entity index in `StepDesign`, styled items); this
//! module shrinks to the colour part once that lands.

use monstertruck::modeling::{Matrix4, Point3, Solid, Vector3};
use monstertruck::step::save::{CompleteStepDisplay, StepHeaderDescriptor, StepModels};
use monstertruck::topology::compress::CompressedSolid;
use std::fmt::Write;

/// A product: a part made of solids, or an assembly of instances of other products.
pub struct Product {
    pub name: String,
    pub kind: ProductKind,
}

pub enum ProductKind {
    Part {
        solids: Vec<Solid>,
        /// sRGB 0..1, written as a surface colour on each solid
        colour: Option<[f64; 3]>,
        /// CAD layer name
        layer: Option<String>,
    },
    Assembly(Vec<Instance>),
}

/// One use of a product inside an assembly.
pub struct Instance {
    /// index into [`StepDoc::products`]
    pub product: usize,
    /// instance name (`NEXT_ASSEMBLY_USAGE_OCCURRENCE.name`), e.g. `Magnet:3`
    pub name: String,
    /// placement of the child in the parent, mm
    pub placement: Placement,
}

/// A rigid placement: origin, Z axis and X reference direction.
#[derive(Clone, Copy, Debug)]
pub struct Placement {
    pub origin: Point3,
    pub z: Vector3,
    pub x: Vector3,
}

impl Placement {
    pub const IDENTITY: Placement = Placement {
        origin: Point3::new(0.0, 0.0, 0.0),
        z: Vector3::new(0.0, 0.0, 1.0),
        x: Vector3::new(1.0, 0.0, 0.0),
    };

    /// Rotated about the Z axis by `angle` (rad), then moved by `origin`.
    pub fn rot_z(angle: f64, origin: Point3) -> Placement {
        Placement {
            origin,
            z: Vector3::new(0.0, 0.0, 1.0),
            x: Vector3::new(angle.cos(), angle.sin(), 0.0),
        }
    }

    pub fn translate(origin: Point3) -> Placement {
        Placement {
            origin,
            ..Placement::IDENTITY
        }
    }

    pub fn matrix(&self) -> Matrix4 {
        let y = self.z.cross(self.x);
        Matrix4::from_cols(
            self.x.extend(0.0),
            y.extend(0.0),
            self.z.extend(0.0),
            self.origin.to_homogeneous(),
        )
    }
}

/// A whole STEP document: products and the root product.
pub struct StepDoc {
    pub products: Vec<Product>,
    pub root: usize,
    /// `FILE_NAME` originating system
    pub system: String,
}

/// P21 string literal: apostrophes doubled, backslashes doubled, non-ASCII as `\X2\` UTF-16 hex.
pub fn p21_string(s: &str) -> String {
    let mut out = String::from("'");
    for c in s.chars() {
        match c {
            '\'' => out.push_str("''"),
            '\\' => out.push_str("\\\\"),
            c if c.is_ascii() && !c.is_ascii_control() => out.push(c),
            c => {
                let mut buf = [0u16; 2];
                out.push_str("\\X2\\");
                for u in c.encode_utf16(&mut buf) {
                    let _ = write!(out, "{u:04X}");
                }
                out.push_str("\\X0\\");
            }
        }
    }
    out.push('\'');
    out
}

fn real(x: f64) -> String {
    let s = format!("{x:?}");
    if s.contains('.') || s.contains('e') || s.contains('E') {
        s.replace('e', "E")
    } else {
        format!("{s}.")
    }
}

/// The records of a P21 data section text, as (id, record text without the trailing `;`).
fn records(data: &str) -> Vec<(u64, &str)> {
    let mut out = Vec::new();
    let bytes = data.as_bytes();
    let (mut i, mut start, mut in_str, mut depth) = (0, None::<usize>, false, 0i32);
    while i < bytes.len() {
        let c = bytes[i];
        if in_str {
            if c == b'\'' {
                if bytes.get(i + 1) == Some(&b'\'') {
                    i += 1;
                } else {
                    in_str = false;
                }
            }
        } else {
            match c {
                b'\'' => in_str = true,
                b'#' if start.is_none() => start = Some(i),
                b'(' => depth += 1,
                b')' => depth -= 1,
                b';' if depth == 0 => {
                    if let Some(s) = start.take() {
                        let rec = data[s..i].trim();
                        let id = rec[1..]
                            .split(|c: char| !c.is_ascii_digit())
                            .next()
                            .and_then(|n| n.parse().ok());
                        if let Some(id) = id {
                            out.push((id, rec));
                        }
                    }
                }
                _ => {}
            }
        }
        i += 1;
    }
    out
}

/// Writes the document as STEP text.
pub fn write_step(doc: &StepDoc) -> String {
    // the B-rep of every part's solids, through monstertruck
    let compressed: Vec<Vec<CompressedSolid<_, _, _>>> = doc
        .products
        .iter()
        .map(|p| match &p.kind {
            ProductKind::Part { solids, .. } => solids.iter().map(|s| s.compress()).collect(),
            ProductKind::Assembly(_) => Vec::new(),
        })
        .collect();
    let mut models = StepModels::default();
    let mut solid_ids: Vec<Vec<usize>> = Vec::new();
    for solids in &compressed {
        let mut ids = Vec::new();
        for s in solids {
            ids.push(models.next_idx());
            models.push_solid(s);
        }
        solid_ids.push(ids);
    }
    let brep = format!("{models}");
    let mut out = String::new();
    for (id, rec) in records(&brep) {
        if id >= 16 {
            out.push_str(rec);
            out.push_str(";\n");
        }
    }
    let mut e = Emitter {
        next: models.next_idx(),
        w: out,
    };
    e.product_structure(doc, &solid_ids);
    let header = StepHeaderDescriptor {
        file_name: doc.products[doc.root].name.clone(),
        organization_system: doc.system.clone(),
        ..Default::default()
    };
    // CompleteStepDisplay writes the header and wraps the data section
    format!("{}", CompleteStepDisplay::new(Raw(e.w), header))
}

struct Raw(String);

impl std::fmt::Display for Raw {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(&self.0)
    }
}

/// Entity numbering and text of the records after the B-rep.
struct Emitter {
    next: usize,
    w: String,
}

struct ProductIds {
    pd: usize,
    sr: usize,
    origin: usize,
}

impl Emitter {
    fn id(&mut self) -> usize {
        self.next += 1;
        self.next - 1
    }

    fn line(&mut self, id: usize, rec: &str) {
        let _ = writeln!(self.w, "#{id} = {rec};");
    }

    fn axis(&mut self, p: &Placement) -> usize {
        let (o, z, x, a) = (self.id(), self.id(), self.id(), self.id());
        self.line(
            o,
            &format!(
                "CARTESIAN_POINT('', ({}, {}, {}))",
                real(p.origin.x),
                real(p.origin.y),
                real(p.origin.z)
            ),
        );
        self.line(
            z,
            &format!(
                "DIRECTION('', ({}, {}, {}))",
                real(p.z.x),
                real(p.z.y),
                real(p.z.z)
            ),
        );
        self.line(
            x,
            &format!(
                "DIRECTION('', ({}, {}, {}))",
                real(p.x.x),
                real(p.x.y),
                real(p.x.z)
            ),
        );
        self.line(a, &format!("AXIS2_PLACEMENT_3D('', #{o}, #{z}, #{x})"));
        a
    }

    fn product_structure(&mut self, doc: &StepDoc, solid_ids: &[Vec<usize>]) {
        // contexts and units (mm, rad, sr)
        let (app, apd) = (self.id(), self.id());
        self.line(
            app,
            "APPLICATION_CONTEXT('core data for automotive mechanical design processes')",
        );
        self.line(apd, &format!("APPLICATION_PROTOCOL_DEFINITION('international standard', 'automotive_design', 2000, #{app})"));
        let (ctx, mm, rad, sr, unc) = (self.id(), self.id(), self.id(), self.id(), self.id());
        self.line(
            ctx,
            &format!(
                "( GEOMETRIC_REPRESENTATION_CONTEXT(3) GLOBAL_UNCERTAINTY_ASSIGNED_CONTEXT((#{unc})) \
                 GLOBAL_UNIT_ASSIGNED_CONTEXT((#{mm}, #{rad}, #{sr})) \
                 REPRESENTATION_CONTEXT('Context #1', '3D Context with UNIT and UNCERTAINTY') )"
            ),
        );
        self.line(
            mm,
            "( LENGTH_UNIT() NAMED_UNIT(*) SI_UNIT(.MILLI., .METRE.) )",
        );
        self.line(
            rad,
            "( NAMED_UNIT(*) PLANE_ANGLE_UNIT() SI_UNIT($, .RADIAN.) )",
        );
        self.line(
            sr,
            "( NAMED_UNIT(*) SI_UNIT($, .STERADIAN.) SOLID_ANGLE_UNIT() )",
        );
        self.line(
            unc,
            &format!("UNCERTAINTY_MEASURE_WITH_UNIT(LENGTH_MEASURE(1.E-06), #{mm}, 'distance_accuracy_value', 'confusion accuracy')"),
        );
        let (pc, pdc) = (self.id(), self.id());
        self.line(pc, &format!("PRODUCT_CONTEXT('', #{app}, 'mechanical')"));
        self.line(
            pdc,
            &format!("PRODUCT_DEFINITION_CONTEXT('part definition', #{app}, 'design')"),
        );

        // products
        let mut ids = Vec::new();
        let mut styled = Vec::new();
        let mut layers: Vec<(String, Vec<usize>)> = Vec::new();
        for (pi, p) in doc.products.iter().enumerate() {
            let origin = self.axis(&Placement::IDENTITY);
            let (prod, pdf, pd, pds, sr, sdr) = (
                self.id(),
                self.id(),
                self.id(),
                self.id(),
                self.id(),
                self.id(),
            );
            let name = p21_string(&p.name);
            self.line(prod, &format!("PRODUCT({name}, {name}, '', (#{pc}))"));
            self.line(
                pdf,
                &format!("PRODUCT_DEFINITION_FORMATION('', '', #{prod})"),
            );
            self.line(
                pd,
                &format!("PRODUCT_DEFINITION('design', '', #{pdf}, #{pdc})"),
            );
            self.line(pds, &format!("PRODUCT_DEFINITION_SHAPE('', '', #{pd})"));
            let mut items = vec![origin];
            let rep = match &p.kind {
                ProductKind::Part { colour, layer, .. } => {
                    items.extend(&solid_ids[pi]);
                    if let Some(c) = colour {
                        styled.extend(solid_ids[pi].iter().map(|&s| (s, *c)));
                    }
                    if let Some(l) = layer {
                        match layers.iter_mut().find(|(n, _)| n == l) {
                            Some((_, v)) => v.extend(&solid_ids[pi]),
                            None => layers.push((l.clone(), solid_ids[pi].clone())),
                        }
                    }
                    "ADVANCED_BREP_SHAPE_REPRESENTATION"
                }
                ProductKind::Assembly(_) => "SHAPE_REPRESENTATION",
            };
            self.line(sr, &format!("{rep}({name}, {}, #{ctx})", ref_list(&items)));
            self.line(
                sdr,
                &format!("SHAPE_DEFINITION_REPRESENTATION(#{pds}, #{sr})"),
            );
            ids.push(ProductIds { pd, sr, origin });
        }
        // instances: the child's representation placed in the parent's (rep_1 = child, rep_2 = parent)
        for (pi, p) in doc.products.iter().enumerate() {
            let ProductKind::Assembly(children) = &p.kind else {
                continue;
            };
            for (k, inst) in children.iter().enumerate() {
                let (parent, child) = (&ids[pi], &ids[inst.product]);
                let (child_origin, child_sr, child_pd) = (child.origin, child.sr, child.pd);
                let (parent_sr, parent_pd) = (parent.sr, parent.pd);
                let place = self.axis(&inst.placement);
                let (idt, rr, nauo, pds, cdsr) =
                    (self.id(), self.id(), self.id(), self.id(), self.id());
                let name = p21_string(&inst.name);
                self.line(
                    idt,
                    &format!("ITEM_DEFINED_TRANSFORMATION('', '', #{child_origin}, #{place})"),
                );
                self.line(
                    rr,
                    &format!(
                        "( REPRESENTATION_RELATIONSHIP('', '', #{child_sr}, #{parent_sr}) \
                         REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION(#{idt}) SHAPE_REPRESENTATION_RELATIONSHIP() )"
                    ),
                );
                self.line(
                    nauo,
                    &format!("NEXT_ASSEMBLY_USAGE_OCCURRENCE('{}', {name}, '', #{parent_pd}, #{child_pd}, $)", k + 1),
                );
                self.line(pds, &format!("PRODUCT_DEFINITION_SHAPE('', '', #{nauo})"));
                self.line(
                    cdsr,
                    &format!("CONTEXT_DEPENDENT_SHAPE_REPRESENTATION(#{rr}, #{pds})"),
                );
            }
        }
        // colours: the AP214 surface style chain per solid, collected in one presentation representation
        let mut styled_ids = Vec::new();
        for (solid, c) in styled {
            let ids: Vec<usize> = (0..8).map(|_| self.id()).collect();
            let [rgb, fasc, fas, ssfa, sss, ssu, psa, si] = ids[..] else {
                unreachable!()
            };
            self.line(
                rgb,
                &format!(
                    "COLOUR_RGB('', {}, {}, {})",
                    real(c[0]),
                    real(c[1]),
                    real(c[2])
                ),
            );
            self.line(fasc, &format!("FILL_AREA_STYLE_COLOUR('', #{rgb})"));
            self.line(fas, &format!("FILL_AREA_STYLE('', (#{fasc}))"));
            self.line(ssfa, &format!("SURFACE_STYLE_FILL_AREA(#{fas})"));
            self.line(sss, &format!("SURFACE_SIDE_STYLE('', (#{ssfa}))"));
            self.line(ssu, &format!("SURFACE_STYLE_USAGE(.BOTH., #{sss})"));
            self.line(psa, &format!("PRESENTATION_STYLE_ASSIGNMENT((#{ssu}))"));
            self.line(si, &format!("STYLED_ITEM('color', (#{psa}), #{solid})"));
            styled_ids.push(si);
        }
        if !styled_ids.is_empty() {
            let m = self.id();
            self.line(
                m,
                &format!(
                    "MECHANICAL_DESIGN_GEOMETRIC_PRESENTATION_REPRESENTATION('', {}, #{ctx})",
                    ref_list(&styled_ids)
                ),
            );
        }
        for (name, items) in layers {
            let l = self.id();
            self.line(
                l,
                &format!(
                    "PRESENTATION_LAYER_ASSIGNMENT({}, '', {})",
                    p21_string(&name),
                    ref_list(&items)
                ),
            );
        }
    }
}

fn ref_list(ids: &[usize]) -> String {
    format!(
        "({})",
        ids.iter()
            .map(|i| format!("#{i}"))
            .collect::<Vec<_>>()
            .join(", ")
    )
}
