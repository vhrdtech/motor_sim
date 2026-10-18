//! Parametric test motors written as STEP assemblies, so the import pipeline is exercised end to end from Rust:
//! an outrunner of the VESC preset's class (12 slots, 14 poles, 5065 size) that follows the naming convention
//! (role tags, magnet polarity colours, a rotor sub-assembly), a NEMA17-style hybrid stepper with plain descriptive
//! names (keywords only), and either of them flattened to one nameless, colourless product (geometry only).
//!
//! Lengths in mm, the motor axis is +Z through the origin. The parameter structs are a first, small geometry
//! description; FEM-10's will be richer (slot shapes, windings), and these generators should then build from it.

use crate::shapes::{self, Seg};
use crate::step_write::{Instance, Placement, Product, ProductKind, StepDoc};
use monstertruck::modeling::{Point3, Solid, builder};
use std::f64::consts::PI;

/// Convention colours (sRGB 0..1), see the convention page.
pub mod colour {
    pub const COPPER: [f64; 3] = [0.72, 0.45, 0.20];
    pub const MAGNET_N: [f64; 3] = [0.80, 0.16, 0.16];
    pub const MAGNET_S: [f64; 3] = [0.16, 0.28, 0.80];
    pub const MAGNET: [f64; 3] = [0.30, 0.30, 0.32];
    pub const LAMINATION: [f64; 3] = [0.42, 0.44, 0.48];
    pub const SHAFT: [f64; 3] = [0.78, 0.78, 0.80];
    pub const ALUMINIUM: [f64; 3] = [0.82, 0.83, 0.85];
    pub const ANODISED: [f64; 3] = [0.12, 0.12, 0.14];
    pub const BEARING: [f64; 3] = [0.62, 0.63, 0.66];
    pub const FASTENER: [f64; 3] = [0.10, 0.10, 0.10];
    pub const PCB: [f64; 3] = [0.08, 0.42, 0.20];
    pub const WHITE: [f64; 3] = [0.92, 0.92, 0.90];
}

fn part(name: &str, solids: Vec<Solid>, colour: Option<[f64; 3]>) -> Product {
    Product {
        name: name.into(),
        kind: ProductKind::Part {
            solids,
            colour,
            layer: None,
        },
    }
}

fn inst(product: usize, name: &str, placement: Placement) -> Instance {
    Instance {
        product,
        name: name.into(),
        placement,
    }
}

/// A ring of teeth (outward when `r_tip > r_yoke`: an outrunner stator; inward when smaller: an inrunner bore).
/// `w` is the tooth body half width, `phi` the tooth tip half angle, `r_shoe` the radius where the body meets the tip.
#[allow(clippy::too_many_arguments)] // one tooth outline: every argument is a dimension
pub fn teeth_loop(
    n: usize,
    r_yoke: f64,
    r_shoe: f64,
    r_tip: f64,
    w: f64,
    phi: f64,
    offset: f64,
    z: f64,
) -> monstertruck::modeling::Wire {
    let mut pts = Vec::new();
    let mut segs = Vec::new();
    let (ay, ab) = ((w / r_yoke).asin(), (w / r_shoe).asin());
    for i in 0..n {
        let t = offset + 2.0 * PI * i as f64 / n as f64;
        let ring = [
            (r_yoke, t - ay),
            (r_shoe, t - ab),
            (r_shoe, t - phi),
            (r_tip, t - phi),
            (r_tip, t + phi),
            (r_shoe, t + phi),
            (r_shoe, t + ab),
            (r_yoke, t + ay),
        ];
        pts.extend(ring.iter().map(|&(r, a)| [r * a.cos(), r * a.sin()]));
        segs.extend([
            Seg::Line,
            Seg::Line,
            Seg::Line,
            Seg::Arc,
            Seg::Line,
            Seg::Line,
            Seg::Line,
            Seg::Arc,
        ]);
    }
    shapes::loop_wire(&pts, &segs, z)
}

/// A ball bearing as a sub-assembly: inner ring, outer ring and `balls` balls (instances of one ball part).
fn bearing(
    doc: &mut Vec<Product>,
    name: &str,
    r_in: f64,
    r_out: f64,
    width: f64,
    balls: usize,
) -> usize {
    let t = (r_out - r_in) * 0.22;
    let rc = (r_in + r_out) / 2.0;
    let rb = (r_out - r_in) * 0.27;
    let inner = doc.len();
    doc.push(part(
        &format!("{name} inner ring"),
        vec![shapes::tube(r_in, r_in + t, 0.0, width)],
        Some(colour::BEARING),
    ));
    let outer = doc.len();
    doc.push(part(
        &format!("{name} outer ring"),
        vec![shapes::tube(r_out - t, r_out, 0.0, width)],
        Some(colour::BEARING),
    ));
    let ball = doc.len();
    doc.push(part(
        &format!("{name} ball"),
        vec![shapes::ball(Point3::new(0.0, 0.0, 0.0), rb)],
        Some(colour::BEARING),
    ));
    let mut children = vec![
        inst(inner, "inner ring", Placement::IDENTITY),
        inst(outer, "outer ring", Placement::IDENTITY),
    ];
    for k in 0..balls {
        let a = 2.0 * PI * k as f64 / balls as f64;
        children.push(inst(
            ball,
            &format!("ball:{}", k + 1),
            Placement::translate(Point3::new(rc * a.cos(), rc * a.sin(), width / 2.0)),
        ));
    }
    doc.push(Product {
        name: name.into(),
        kind: ProductKind::Assembly(children),
    });
    doc.len() - 1
}

/// The outrunner, mm.
#[derive(Clone, Debug)]
pub struct Outrunner {
    pub slots: usize,
    pub poles: usize,
    /// stator stack length
    pub stack: f64,
    pub stator_bore: f64,
    pub yoke_r: f64,
    pub tooth_half_width: f64,
    pub shoe_r: f64,
    pub stator_r: f64,
    pub air_gap: f64,
    pub magnet_thickness: f64,
    pub bell_wall: f64,
    pub shaft_r: f64,
}

impl Default for Outrunner {
    /// A 5065-class hobby outrunner, the motor class of the VESC preset (7 pole pairs).
    fn default() -> Self {
        Outrunner {
            slots: 12,
            poles: 14,
            stack: 28.0,
            stator_bore: 10.5,
            yoke_r: 13.5,
            tooth_half_width: 2.0,
            shoe_r: 18.6,
            stator_r: 20.0,
            air_gap: 0.5,
            magnet_thickness: 2.0,
            bell_wall: 2.5,
            shaft_r: 4.0,
        }
    }
}

pub fn outrunner(p: &Outrunner) -> StepDoc {
    let mut d: Vec<Product> = Vec::new();
    let z0 = 1.0; // stack bottom
    let z1 = z0 + p.stack;
    let (r_m0, r_m1) = (
        p.stator_r + p.air_gap,
        p.stator_r + p.air_gap + p.magnet_thickness,
    );
    let r_bell = r_m1 + p.bell_wall;

    // stator: laminated core (one solid here), coils as instances around the teeth
    let pitch = 2.0 * PI / p.slots as f64;
    let core = shapes::prism(
        vec![
            teeth_loop(
                p.slots,
                p.yoke_r,
                p.shoe_r,
                p.stator_r,
                p.tooth_half_width,
                pitch / 2.0 * 0.86,
                0.0,
                z0,
            ),
            shapes::circle_wire(0.0, 0.0, p.stator_bore, z0),
        ],
        z0,
        z1,
    );
    let stator_core = d.len();
    d.push(part(
        "Stator core 12N [stator]",
        vec![core],
        Some(colour::LAMINATION),
    ));
    let slot_half = p.yoke_r * (pitch / 2.0).sin() - p.tooth_half_width;
    let coil = shapes::coil(
        p.tooth_half_width + 0.2,
        p.stack / 2.0 + 0.2,
        p.tooth_half_width + 0.2 + slot_half * 0.8,
        p.stack / 2.0 + 0.2 + 1.8,
        (z0 + z1) / 2.0,
        p.yoke_r + 0.3,
        p.shoe_r - 0.3,
    );
    let coil_part = d.len();
    d.push(part("Coil [winding]", vec![coil], Some(colour::COPPER)));

    // mount: flange with holes and the stator hub tube
    let mut flange_wires = vec![
        shapes::circle_wire(0.0, 0.0, 22.0, -6.0),
        shapes::circle_wire(0.0, 0.0, p.shaft_r + 0.6, -6.0),
    ];
    for k in 0..4 {
        let a = PI / 4.0 + k as f64 * PI / 2.0;
        flange_wires.push(shapes::circle_wire(
            12.5 * a.cos(),
            12.5 * a.sin(),
            1.6,
            -6.0,
        ));
    }
    let flange = d.len();
    d.push(part(
        "Mount flange [housing]",
        vec![shapes::prism(flange_wires, -6.0, -2.0)],
        Some(colour::ANODISED),
    ));
    let hub = d.len();
    d.push(part(
        "Stator hub [housing]",
        vec![shapes::tube(8.0, p.stator_bore, -2.0, z1 + 1.0)],
        Some(colour::ALUMINIUM),
    ));

    // rotor sub-assembly: bell, magnets alternating N/S, shaft
    let bell_top = z1 + 4.0;
    let bell = shapes::revolved(&[
        [p.shaft_r, bell_top - 3.0],
        [r_m1, bell_top - 3.0],
        [r_m1, -0.3],
        [r_bell, -0.3],
        [r_bell, bell_top],
        [p.shaft_r, bell_top],
    ]);
    let bell_part = d.len();
    d.push(part("Bell [rotor]", vec![bell], Some(colour::ANODISED)));
    let pole_pitch = 2.0 * PI / p.poles as f64;
    let half = pole_pitch / 2.0 * 0.86;
    let magnet = |c| {
        part(
            "",
            vec![shapes::sector(r_m0, r_m1, -half, half, z0 - 0.5, z1 + 0.5)],
            Some(c),
        )
    };
    let mag_n = d.len();
    d.push(Product {
        name: "Magnet N [magnet]".into(),
        ..magnet(colour::MAGNET_N)
    });
    let mag_s = d.len();
    d.push(Product {
        name: "Magnet S [magnet]".into(),
        ..magnet(colour::MAGNET_S)
    });
    let shaft = d.len();
    d.push(part(
        "Shaft 8 mm [shaft]",
        vec![shapes::tube(0.0, p.shaft_r, -14.0, bell_top + 18.0)],
        Some(colour::SHAFT),
    ));
    let mut rotor_children = vec![
        inst(bell_part, "Bell", Placement::IDENTITY),
        inst(shaft, "Shaft", Placement::IDENTITY),
    ];
    for k in 0..p.poles {
        let (prod, pol) = if k % 2 == 0 {
            (mag_n, "N")
        } else {
            (mag_s, "S")
        };
        rotor_children.push(inst(
            prod,
            &format!("Magnet {pol}:{}", k + 1),
            Placement::rot_z(k as f64 * pole_pitch, Point3::new(0.0, 0.0, 0.0)),
        ));
    }
    let rotor = d.len();
    d.push(Product {
        name: "Rotor [rotor]".into(),
        kind: ProductKind::Assembly(rotor_children),
    });

    // bearings in the hub, an e-clip on the shaft below the flange
    let brg = bearing(&mut d, "Bearing 688ZZ [bearing]", p.shaft_r, 8.0, 5.0, 9);
    let clip = d.len();
    d.push(part(
        "E-clip 8 [fastener]",
        vec![shapes::tube(p.shaft_r, p.shaft_r + 2.0, -9.0, -8.4)],
        Some(colour::FASTENER),
    ));

    // screws: head + shank as two bodies of one part
    let screw = d.len();
    d.push(part(
        "Screw M3x8 ISO 4762 [fastener]",
        vec![
            shapes::tube(0.0, 2.75, -9.0, -6.0),
            shapes::tube(0.0, 1.5, -6.0, -1.0),
        ],
        Some(colour::FASTENER),
    ));

    // sensor board and phase leads
    let pcb = d.len();
    d.push(part(
        "Hall sensor PCB [pcb]",
        vec![shapes::tube(11.0, 19.0, -1.95, -1.35)],
        Some(colour::PCB),
    ));
    let lead_colours = [[0.85, 0.75, 0.10], [0.15, 0.35, 0.85], [0.80, 0.15, 0.15]];
    let mut leads = Vec::new();
    for (k, c) in lead_colours.iter().enumerate() {
        let a = -PI / 2.0 + (k as f64 - 1.0) * 0.12;
        let rod = shapes::rod(
            Point3::new(19.6 * a.cos(), 19.6 * a.sin(), -0.9),
            Point3::new(55.0 * a.cos(), 55.0 * a.sin(), -0.9),
            0.55,
        );
        leads.push(d.len());
        d.push(part(
            &format!("Phase lead {} [wiring]", ["A", "B", "C"][k]),
            vec![rod],
            Some(*c),
        ));
    }

    // the motor
    let mut top = vec![
        inst(stator_core, "Stator core", Placement::IDENTITY),
        inst(flange, "Mount flange", Placement::IDENTITY),
        inst(hub, "Stator hub", Placement::IDENTITY),
        inst(rotor, "Rotor", Placement::IDENTITY),
        inst(
            brg,
            "Bearing front",
            Placement::translate(Point3::new(0.0, 0.0, z1 - 5.0)),
        ),
        inst(
            brg,
            "Bearing rear",
            Placement::translate(Point3::new(0.0, 0.0, -1.5)),
        ),
        inst(clip, "E-clip", Placement::IDENTITY),
        inst(pcb, "Hall PCB", Placement::IDENTITY),
    ];
    for k in 0..p.slots {
        top.push(inst(
            coil_part,
            &format!("Coil:{}", k + 1),
            Placement::rot_z(k as f64 * pitch, Point3::new(0.0, 0.0, 0.0)),
        ));
    }
    for k in 0..4 {
        let a = PI / 4.0 + k as f64 * PI / 2.0;
        top.push(inst(
            screw,
            &format!("Screw:{}", k + 1),
            Placement::translate(Point3::new(12.5 * a.cos(), 12.5 * a.sin(), 0.0)),
        ));
    }
    for (k, &l) in leads.iter().enumerate() {
        top.push(inst(
            l,
            &format!("Phase lead {}", ["A", "B", "C"][k]),
            Placement::IDENTITY,
        ));
    }
    d.push(Product {
        name: "Outrunner 5065 12N14P".into(),
        kind: ProductKind::Assembly(top),
    });
    StepDoc {
        root: d.len() - 1,
        products: d,
        system: "motor_cad parametric outrunner".into(),
    }
}

/// The NEMA17-style hybrid stepper, mm.
#[derive(Clone, Debug)]
pub struct Stepper {
    /// stator poles
    pub poles: usize,
    /// rotor teeth per cup
    pub rotor_teeth: usize,
    /// body length (front face to rear face)
    pub length: f64,
    pub square: f64,
    pub rotor_r: f64,
    pub air_gap: f64,
    pub shaft_r: f64,
}

impl Default for Stepper {
    fn default() -> Self {
        Stepper {
            poles: 8,
            rotor_teeth: 50,
            length: 40.0,
            square: 42.3,
            rotor_r: 11.0,
            air_gap: 0.1,
            shaft_r: 2.5,
        }
    }
}

fn chamfered_square(half: f64, chamfer: f64, z: f64) -> monstertruck::modeling::Wire {
    let (h, c) = (half, half - chamfer);
    let pts = [
        [c, -h],
        [h, -c],
        [h, c],
        [c, h],
        [-c, h],
        [-h, c],
        [-h, -c],
        [-c, -h],
    ];
    shapes::loop_wire(&pts, &[], z)
}

pub fn stepper(p: &Stepper) -> StepDoc {
    let mut d: Vec<Product> = Vec::new();
    let half = p.square / 2.0;
    let hole = 15.5;
    let holes = |z: f64| {
        (0..4).map(move |k| {
            let a = PI / 4.0 + k as f64 * PI / 2.0;
            shapes::circle_wire(
                hole * 2f64.sqrt() * a.cos(),
                hole * 2f64.sqrt() * a.sin(),
                1.6,
                z,
            )
        })
    };
    let (rear0, rear1) = (0.0, 7.0);
    let (front0, front1) = (p.length - 7.0, p.length);

    let mut rear = vec![
        chamfered_square(half, 4.0, rear0),
        shapes::circle_wire(0.0, 0.0, 8.0, rear0),
    ];
    rear.extend(holes(rear0));
    let rear_cap = d.len();
    d.push(part(
        "Rear end cap",
        vec![shapes::prism(rear, rear0, rear1)],
        Some(colour::ALUMINIUM),
    ));
    let mut front = vec![
        chamfered_square(half, 4.0, front0),
        shapes::circle_wire(0.0, 0.0, 8.0, front0),
    ];
    front.extend(holes(front0));
    let front_cap = d.len();
    d.push(part(
        "Front flange",
        vec![
            shapes::prism(front, front0, front1),
            shapes::tube(8.0, 11.0, front1, front1 + 2.0),
        ],
        Some(colour::ALUMINIUM),
    ));

    // stator stack with the 8-pole bore
    let r_face = p.rotor_r + p.air_gap;
    let pitch = 2.0 * PI / p.poles as f64;
    let mut st = vec![
        chamfered_square(half - 0.15, 4.0, rear1),
        teeth_loop(
            p.poles,
            17.0,
            r_face + 1.6,
            r_face,
            2.6,
            pitch / 2.0 * 0.62,
            0.0,
            rear1,
        ),
    ];
    st.extend(holes(rear1));
    let stack = d.len();
    d.push(part(
        "Stator lamination stack",
        vec![shapes::prism(st, rear1, front0)],
        Some(colour::LAMINATION),
    ));
    let slot = 17.0 * (pitch / 2.0).sin() - 2.6;
    let mid = (rear1 + front0) / 2.0;
    let coil = shapes::coil(
        2.8,
        (front0 - rear1) / 2.0 + 0.2,
        2.8 + slot * 0.75,
        (front0 - rear1) / 2.0 + 1.6,
        mid,
        0.0,
        17.0 - r_face - 2.0,
    );
    // the coil is built along +X from 0; the pole points inward, so place it from the shoe out to the yoke
    let coil = builder::translated(
        &coil,
        monstertruck::modeling::Vector3::new(r_face + 1.8, 0.0, 0.0),
    );
    let coil_part = d.len();
    d.push(part("Phase coil", vec![coil], Some(colour::COPPER)));

    // rotor: two toothed cups half a tooth apart around an axial ring magnet, on the shaft
    let tooth = 2.0 * PI / p.rotor_teeth as f64;
    let cup = |offset: f64, z0: f64, z1: f64| {
        let mut pts = Vec::new();
        for i in 0..p.rotor_teeth {
            let a = offset + i as f64 * tooth;
            pts.extend([
                (p.rotor_r, a - tooth * 0.22),
                (p.rotor_r, a + tooth * 0.22),
                (p.rotor_r - 0.4, a + tooth * 0.28),
                (p.rotor_r - 0.4, a + tooth * 0.72),
            ]);
        }
        shapes::prism(
            vec![
                shapes::radial_loop(&pts, z0),
                shapes::circle_wire(0.0, 0.0, p.shaft_r, z0),
            ],
            z0,
            z1,
        )
    };
    let (rz0, rz1) = (rear1 + 1.0, front0 - 1.0);
    let rm = (rz0 + rz1) / 2.0;
    let cup_f = d.len();
    d.push(part(
        "Rotor cup front",
        vec![cup(0.0, rm + 0.6, rz1)],
        Some(colour::LAMINATION),
    ));
    let cup_r = d.len();
    d.push(part(
        "Rotor cup rear",
        vec![cup(tooth / 2.0, rz0, rm - 0.6)],
        Some(colour::LAMINATION),
    ));
    let ring = d.len();
    d.push(part(
        "Rotor magnet ring",
        vec![shapes::tube(p.shaft_r, p.rotor_r - 2.0, rm - 0.6, rm + 0.6)],
        Some(colour::MAGNET),
    ));
    let shaft = d.len();
    d.push(part(
        "Shaft 5 mm",
        vec![shapes::tube(0.0, p.shaft_r, -2.0, front1 + 24.0)],
        Some(colour::SHAFT),
    ));
    // bearings simplified to one ring each, named by designation only
    let brg = d.len();
    d.push(part(
        "625ZZ",
        vec![shapes::tube(p.shaft_r, 8.0, 0.0, 5.0)],
        Some(colour::BEARING),
    ));
    let screw = d.len();
    d.push(part(
        "Pan head screw M3x35",
        vec![
            shapes::tube(0.0, 2.75, -2.4, 0.0),
            shapes::tube(0.0, 1.5, 0.0, front0 + 3.0),
        ],
        Some(colour::FASTENER),
    ));
    let conn = d.len();
    d.push(part(
        "Connector JST PH 6",
        vec![shapes::cuboid(
            [half - 1.0, -6.0, 0.8],
            [half + 4.5, 6.0, 6.2],
        )],
        Some(colour::WHITE),
    ));
    let mut wires = Vec::new();
    for (k, (nm, c)) in [
        ("A+", [0.8, 0.15, 0.15]),
        ("A-", [0.15, 0.35, 0.85]),
        ("B+", [0.15, 0.6, 0.2]),
        ("B-", [0.1, 0.1, 0.1]),
    ]
    .iter()
    .enumerate()
    {
        let y = -3.0 + 2.0 * k as f64;
        let rod = shapes::rod(
            Point3::new(half + 4.5, y, 3.5),
            Point3::new(half + 40.0, y, 3.5),
            0.6,
        );
        wires.push(d.len());
        d.push(part(&format!("Wire {nm}"), vec![rod], Some(*c)));
    }

    let mut top = vec![
        inst(rear_cap, "Rear end cap", Placement::IDENTITY),
        inst(front_cap, "Front flange", Placement::IDENTITY),
        inst(stack, "Stator", Placement::IDENTITY),
        inst(cup_f, "Rotor cup front", Placement::IDENTITY),
        inst(cup_r, "Rotor cup rear", Placement::IDENTITY),
        inst(ring, "Magnet", Placement::IDENTITY),
        inst(shaft, "Shaft", Placement::IDENTITY),
        inst(
            brg,
            "Bearing rear",
            Placement::translate(Point3::new(0.0, 0.0, 1.0)),
        ),
        inst(
            brg,
            "Bearing front",
            Placement::translate(Point3::new(0.0, 0.0, front1 - 5.5)),
        ),
        inst(conn, "Connector", Placement::IDENTITY),
    ];
    for k in 0..p.poles {
        top.push(inst(
            coil_part,
            &format!("Coil {}", k + 1),
            Placement::rot_z(k as f64 * pitch, Point3::new(0.0, 0.0, 0.0)),
        ));
    }
    for k in 0..4 {
        let a = PI / 4.0 + k as f64 * PI / 2.0;
        let r = hole * 2f64.sqrt();
        top.push(inst(
            screw,
            &format!("Screw {}", k + 1),
            Placement::translate(Point3::new(r * a.cos(), r * a.sin(), 0.0)),
        ));
    }
    for (k, &w) in wires.iter().enumerate() {
        top.push(inst(w, &format!("Wire {}", k + 1), Placement::IDENTITY));
    }
    d.push(Product {
        name: "NEMA17 hybrid stepper 1.8 deg".into(),
        kind: ProductKind::Assembly(top),
    });
    StepDoc {
        root: d.len() - 1,
        products: d,
        system: "motor_cad parametric stepper".into(),
    }
}

/// The same document with every name, tag and colour removed (`Part 1`, `Body 1`, ...) and sub-assemblies kept:
/// the geometry-only path, as for a vendor model without useful names. (A true single-product flattening would bake
/// the placements into the B-rep; monstertruck then writes arcs as rational B-splines that its own parser hangs on,
/// so the instances stay.)
pub fn anonymised(doc: &StepDoc, name: &str) -> StepDoc {
    let products = doc
        .products
        .iter()
        .enumerate()
        .map(|(i, p)| Product {
            name: if i == doc.root {
                name.into()
            } else {
                format!("Part {}", i + 1)
            },
            kind: match &p.kind {
                ProductKind::Part { solids, .. } => ProductKind::Part {
                    solids: solids.clone(),
                    colour: None,
                    layer: None,
                },
                ProductKind::Assembly(ch) => ProductKind::Assembly(
                    ch.iter()
                        .enumerate()
                        .map(|(k, c)| Instance {
                            product: c.product,
                            name: format!("Body {}", k + 1),
                            placement: c.placement,
                        })
                        .collect(),
                ),
            },
        })
        .collect();
    StepDoc {
        products,
        root: doc.root,
        system: doc.system.clone(),
    }
}
