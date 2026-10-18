//! B-rep building blocks for the parametric motors, on monstertruck's modeling API. Lengths in mm (the STEP files we
//! write use millimetres, like every mechanical CAD), the motor axis is +Z.

use monstertruck::modeling::{builder, primitive, profile, *};
use std::f64::consts::PI;

/// A closed planar loop in the XY plane at `z`, from 2D points; `arcs` marks segments drawn as circle arcs about the
/// origin (segment i goes from point i to point i+1).
pub enum Seg {
    Line,
    /// A circle arc about the Z axis.
    Arc,
    /// A circle arc about the given centre (x, y).
    ArcAbout(f64, f64),
}

fn p3(x: f64, y: f64, z: f64) -> Point3 {
    Point3::new(x, y, z)
}

/// A loop of lines and arcs about the origin in the plane `z`.
pub fn loop_wire(points: &[[f64; 2]], segs: &[Seg], z: f64) -> Wire {
    let vs: Vec<Vertex> = points
        .iter()
        .map(|p| builder::vertex(p3(p[0], p[1], z)))
        .collect();
    let n = vs.len();
    let edges: Vec<Edge> = (0..n)
        .map(|i| {
            let (a, b) = (&vs[i], &vs[(i + 1) % n]);
            match segs.get(i).unwrap_or(&Seg::Line) {
                Seg::Line => builder::line(a, b),
                Seg::Arc | Seg::ArcAbout(..) => {
                    let (cx, cy) = match segs[i] {
                        Seg::ArcAbout(x, y) => (x, y),
                        _ => (0.0, 0.0),
                    };
                    let (pa, pb) = (points[i], points[(i + 1) % n]);
                    let (pa, pb) = ([pa[0] - cx, pa[1] - cy], [pb[0] - cx, pb[1] - cy]);
                    let r = (pa[0].hypot(pa[1]) + pb[0].hypot(pb[1])) / 2.0;
                    let (aa, mut ab) = (pa[1].atan2(pa[0]), pb[1].atan2(pb[0]));
                    // go the short way round, in the loop's direction
                    while ab - aa > PI {
                        ab -= 2.0 * PI;
                    }
                    while ab - aa < -PI {
                        ab += 2.0 * PI;
                    }
                    let am = (aa + ab) / 2.0;
                    builder::circle_arc(a, b, p3(cx + r * am.cos(), cy + r * am.sin(), z))
                }
            }
        })
        .collect();
    edges.into()
}

/// A full circle about the Z axis, radius `r`, in the plane `z`.
pub fn circle_wire(cx: f64, cy: f64, r: f64, z: f64) -> Wire {
    primitive::circle(p3(cx + r, cy, z), p3(cx, cy, z), Vector3::unit_z(), 4)
}

/// A circle about an arbitrary axis.
pub fn circle_wire_axis(center: Point3, axis: Vector3, start: Point3) -> Wire {
    primitive::circle(start, center, axis, 4)
}

/// Extrudes a planar profile (outer loop first, holes after) lying in the plane z0 up to z1.
pub fn prism(wires: Vec<Wire>, z0: f64, z1: f64) -> Solid {
    let _ = z0;
    profile::solid_from_planar_profile(wires, Vector3::new(0.0, 0.0, 1.0) * 1.0 * (z1 - z0))
        .expect("planar profile")
}

/// A tube (or disc with `r_in` = 0) along Z.
pub fn tube(r_in: f64, r_out: f64, z0: f64, z1: f64) -> Solid {
    let mut wires = vec![circle_wire(0.0, 0.0, r_out, z0)];
    if r_in > 0.0 {
        wires.push(circle_wire(0.0, 0.0, r_in, z0));
    }
    prism(wires, z0, z1)
}

/// A cylinder along an arbitrary axis from `a` to `b`.
pub fn rod(a: Point3, b: Point3, r: f64) -> Solid {
    let axis = (b - a).normalize();
    let helper = if axis.z.abs() < 0.9 {
        Vector3::unit_z()
    } else {
        Vector3::unit_x()
    };
    let radial = axis.cross(helper).normalize() * r;
    let wire = circle_wire_axis(a, axis, a + radial);
    let face = profile::attach_plane_normalized::<Curve, Surface>(vec![wire]).expect("disc");
    let mut solid: Solid = builder::extrude(&face, b - a);
    // the normalised face may point against the extrusion: a solid with its shell inside out still renders,
    // but keep it consistent
    if !solid.is_geometric_consistent() {
        solid.not();
    }
    solid
}

/// An annular sector (magnet, coil side, rotor segment) between radii and angles, extruded from z0 to z1.
pub fn sector(r_in: f64, r_out: f64, a0: f64, a1: f64, z0: f64, z1: f64) -> Solid {
    let pts = [
        [r_in * a0.cos(), r_in * a0.sin()],
        [r_out * a0.cos(), r_out * a0.sin()],
        [r_out * a1.cos(), r_out * a1.sin()],
        [r_in * a1.cos(), r_in * a1.sin()],
    ];
    let w = loop_wire(&pts, &[Seg::Line, Seg::Arc, Seg::Line, Seg::Arc], z0);
    prism(vec![w], z0, z1)
}

/// A closed (r, z) profile revolved about the Z axis; the profile must not touch the axis.
pub fn revolved(profile_rz: &[[f64; 2]]) -> Solid {
    let vs: Vec<Vertex> = profile_rz
        .iter()
        .map(|p| builder::vertex(p3(p[0], 0.0, p[1])))
        .collect();
    let n = vs.len();
    let edges: Vec<Edge> = (0..n)
        .map(|i| builder::line(&vs[i], &vs[(i + 1) % n]))
        .collect();
    let wire: Wire = edges.into();
    let face: Face = builder::try_attach_plane(vec![wire]).expect("profile plane");
    builder::revolve(
        &face,
        Point3::origin(),
        Vector3::unit_z(),
        builder::SweepAngle::Closed,
        4,
    )
}

/// A ball of radius `r` at `c`.
pub fn ball(c: Point3, r: f64) -> Solid {
    let v0 = builder::vertex(c + Vector3::new(0.0, 0.0, r));
    let wire: Wire = builder::revolve(
        &v0,
        c,
        Vector3::unit_x(),
        builder::SweepAngle::Partial(Rad(PI)),
        2,
    );
    let shell = builder::revolve_wire(&wire, c, Vector3::unit_z(), builder::SweepAngle::Closed, 4);
    Solid::new(vec![shell])
}

/// A box from two corners.
pub fn cuboid(a: [f64; 3], b: [f64; 3]) -> Solid {
    primitive::cuboid(BoundingBox::from_iter([
        p3(a[0], a[1], a[2]),
        p3(b[0], b[1], b[2]),
    ]))
}

/// Rotated copy about the Z axis.
pub fn rotated_z(s: &Solid, angle: f64) -> Solid {
    builder::rotated(s, Point3::origin(), Vector3::unit_z(), Rad(angle))
}

/// A rounded rectangle loop centred at the origin, half sizes `hx`, `hy`, corner radius `r`, counter-clockwise.
pub fn rounded_rect(hx: f64, hy: f64, r: f64, z: f64) -> Wire {
    let (ix, iy) = (hx - r, hy - r);
    let pts = [
        [ix, -hy],
        [hx, -iy],
        [hx, iy],
        [ix, hy],
        [-ix, hy],
        [-hx, iy],
        [-hx, -iy],
        [-ix, -hy],
    ];
    let segs = [
        Seg::ArcAbout(ix, -iy),
        Seg::Line,
        Seg::ArcAbout(ix, iy),
        Seg::Line,
        Seg::ArcAbout(-ix, iy),
        Seg::Line,
        Seg::ArcAbout(-ix, -iy),
        Seg::Line,
    ];
    loop_wire(&pts, &segs, z)
}

/// A coil around a radial tooth: a rounded rectangular ring, tangential half sizes `hole_t`/`out_t`, axial half
/// sizes `hole_a`/`out_a` about `z_mid`, from radius `r0` to `r1` along +X (the tooth at angle 0).
pub fn coil(
    hole_t: f64,
    hole_a: f64,
    out_t: f64,
    out_a: f64,
    z_mid: f64,
    r0: f64,
    r1: f64,
) -> Solid {
    let rr = 0.35 * (out_t - hole_t).min(out_a - hole_a);
    let outer = rounded_rect(out_t, out_a, rr + (out_t - hole_t).min(out_a - hole_a), 0.0);
    let hole = rounded_rect(hole_t, hole_a, rr, 0.0);
    let local = prism(vec![outer, hole], 0.0, r1 - r0);
    // local x -> world Y (tangential), local y -> world Z (axial), local z -> world X (radial)
    let m = Matrix4::from_cols(
        Vector4::new(0.0, 1.0, 0.0, 0.0),
        Vector4::new(0.0, 0.0, 1.0, 0.0),
        Vector4::new(1.0, 0.0, 0.0, 0.0),
        Vector4::new(r0, 0.0, z_mid, 1.0),
    );
    builder::transformed(&local, m)
}

/// A regular polygon-ish loop given as radius per point (closed, counter-clockwise), all lines.
pub fn radial_loop(points: &[(f64, f64)], z: f64) -> Wire {
    let pts: Vec<[f64; 2]> = points
        .iter()
        .map(|&(r, a)| [r * a.cos(), r * a.sin()])
        .collect();
    loop_wire(&pts, &[], z)
}
