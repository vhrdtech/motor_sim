//! Part classification with layered evidence: the override file, `[role]` tags (the naming convention), name
//! keywords, colours, then geometry about the motor axis. Each piece of evidence has a weight; a role's score is
//! their noisy-or, the best role wins, and its confidence drops when a second role scores close. Geometry also
//! checks names: a disagreement is reported as a conflict.

use crate::geometry::{self, Axis, Features, Ring};
use crate::mesh::{V3, dot};
use crate::overrides::Overrides;
use crate::role::{self, Polarity, Role};
use crate::step_read::{CadModel, Part};
use serde::{Deserialize, Serialize};

/// Where a piece of evidence came from, strongest first.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Source {
    Override,
    Tag,
    Name,
    Path,
    Colour,
    Geometry,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Evidence {
    pub role: Role,
    pub weight: f64,
    pub source: Source,
    pub why: String,
}

/// The verdict on one part.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Verdict {
    pub role: Role,
    /// 0..1
    pub confidence: f64,
    /// turns with the rotor
    pub rotating: bool,
    pub polarity: Option<Polarity>,
    pub evidence: Vec<Evidence>,
    /// names and geometry disagree
    pub conflict: Option<String>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Topology {
    /// rotor outside the stator
    Outrunner,
    /// rotor inside the stator
    Inrunner,
    Unknown,
}

/// The classification of a whole model.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Classification {
    pub axis: Axis,
    pub topology: Topology,
    /// envelope radius about the axis, m
    pub envelope_r: f64,
    /// winding count and magnet count found as rings (slots, poles)
    pub slots: Option<usize>,
    pub poles: Option<usize>,
    pub features: Vec<Features>,
    pub rings: Vec<Ring>,
    pub verdicts: Vec<Verdict>,
    pub notes: Vec<String>,
}

fn names_of(p: &Part) -> Vec<&str> {
    let mut v = vec![p.name.as_str()];
    if !p.instance.is_empty() && p.instance != p.name {
        v.push(&p.instance);
    }
    if !p.description.is_empty() && p.description != p.name {
        v.push(&p.description);
    }
    v
}

/// Name evidence: tags, keywords of the part, then tags and keywords of the assemblies above it.
fn name_evidence(
    p: &Part,
    ev: &mut Vec<Evidence>,
    rotating: &mut Option<bool>,
    polarity: &mut Option<Polarity>,
) {
    for n in names_of(p) {
        if let Some(t) = role::parse_tag(n) {
            if let Some(r) = t.role {
                ev.push(Evidence {
                    role: r,
                    weight: 0.97,
                    source: Source::Tag,
                    why: format!("tag in '{n}'"),
                });
            }
            if let Some(r) = t.rotating {
                *rotating = Some(r);
            }
            *polarity = polarity.or(t.polarity);
        }
    }
    let mut seen = Vec::new();
    for n in names_of(p) {
        for (k, (r, kw)) in role::keyword_roles(n).into_iter().enumerate() {
            if seen.contains(&r) {
                continue;
            }
            seen.push(r);
            let mut w = if k == 0 { 0.75 } else { 0.35 };
            // a name over many bodies names the whole thing ("Nema 34 stepper motor shaft D14" is a motor with a
            // shaft), not each body: geometry decides between them
            if p.bodies > 2 {
                w *= 0.4;
            }
            ev.push(Evidence {
                role: r,
                weight: w,
                source: Source::Name,
                why: format!("'{kw}' in '{n}'"),
            });
        }
    }
    // the assemblies above: the nearest says most
    for (depth, a) in p.path.iter().rev().enumerate() {
        let w = 0.55 / (1.0 + depth as f64);
        if let Some(t) = role::parse_tag(a) {
            if let Some(r) = t.role {
                if r == Role::Rotor {
                    rotating.get_or_insert(true);
                }
                // a rotor assembly holds magnets and shafts too: it says "rotating", not "rotor"
                if r != Role::Rotor || p.path.len() == depth + 1 {
                    ev.push(Evidence {
                        role: r,
                        weight: w,
                        source: Source::Path,
                        why: format!("inside '{a}'"),
                    });
                }
            }
            if let Some(rot) = t.rotating {
                rotating.get_or_insert(rot);
            }
        } else if let Some((r, kw)) = role::keyword_roles(a).into_iter().next()
            && matches!(
                r,
                Role::Bearing
                    | Role::Fastener
                    | Role::Wiring
                    | Role::Pcb
                    | Role::Sensor
                    | Role::Winding
            )
        {
            ev.push(Evidence {
                role: r,
                weight: w * 0.8,
                source: Source::Path,
                why: format!("'{kw}' in assembly '{a}'"),
            });
        }
    }
}

fn geo(ev: &mut [Vec<Evidence>], i: usize, role: Role, weight: f64, why: String) {
    ev[i].push(Evidence {
        role,
        weight,
        source: Source::Geometry,
        why,
    });
}

fn noisy_or(ws: impl Iterator<Item = f64>) -> f64 {
    1.0 - ws.fold(1.0, |acc, w| acc * (1.0 - w.clamp(0.0, 0.999)))
}

/// Classifies a model. `overrides` come from the TOML file beside it.
pub fn classify(model: &CadModel, overrides: &Overrides) -> Classification {
    let n = model.parts.len();
    // samples per part in proportion to area, within bounds
    let total_area: f64 = model
        .parts
        .iter()
        .map(|p| p.mesh.area())
        .sum::<f64>()
        .max(1e-30);
    let samples: Vec<Vec<V3>> = model
        .parts
        .iter()
        .map(|p| {
            p.mesh.surface_samples(
                ((p.mesh.area() / total_area) * 150_000.0).clamp(1500.0, 12_000.0) as usize,
            )
        })
        .collect();
    let meshes: Vec<&crate::mesh::Mesh> = model.parts.iter().map(|p| &p.mesh).collect();
    let mut notes = Vec::new();
    let axis = match &overrides.axis {
        Some(a) => Axis {
            origin: a.origin.map(|x| x * model.unit_m),
            dir: crate::mesh::normalize(a.direction),
            how: "from the override file".into(),
        },
        None => geometry::find_axis(&samples, &meshes),
    };
    let feats: Vec<Features> = model
        .parts
        .iter()
        .zip(&samples)
        .map(|(p, s)| geometry::features(&p.mesh, s, &axis))
        .collect();
    let (e1, e2) = axis.frame();
    let envelope = {
        let mut rs: Vec<f64> = samples
            .iter()
            .flatten()
            .map(|p| axis.cyl(*p, e1, e2).0)
            .collect();
        rs.sort_by(|a, b| a.partial_cmp(b).unwrap());
        rs.get(rs.len() * 95 / 100).copied().unwrap_or(0.0)
    };
    let length = {
        let zs = feats.iter().map(|f| (f.z_min, f.z_max));
        let (lo, hi) = zs.fold((f64::INFINITY, f64::NEG_INFINITY), |(a, b), (c, d)| {
            (a.min(c), b.max(d))
        });
        (hi - lo).max(1e-9)
    };
    let size = envelope.max(length / 2.0);
    let rings = geometry::rings(&feats, envelope);

    let mut ev: Vec<Vec<Evidence>> = vec![Vec::new(); n];
    let mut rot: Vec<Option<bool>> = vec![None; n];
    let mut pol: Vec<Option<Polarity>> = vec![None; n];
    let mut over: Vec<Option<&crate::overrides::PartOverride>> = vec![None; n];
    for (i, p) in model.parts.iter().enumerate() {
        if let Some(o) = overrides.find(p) {
            over[i] = Some(o);
            if let Some(r) = o.role {
                ev[i].push(Evidence {
                    role: r,
                    weight: 1.0,
                    source: Source::Override,
                    why: format!("override '{}'", o.pattern),
                });
            }
        }
        name_evidence(p, &mut ev[i], &mut rot[i], &mut pol[i]);
        if let Some(c) = p.colour
            && let Some((r, polarity, why)) = role::colour_hint(c)
        {
            ev[i].push(Evidence {
                role: r,
                weight: 0.3,
                source: Source::Colour,
                why: why.into(),
            });
            if r == Role::Magnet {
                pol[i] = pol[i].or(polarity);
            }
        }
    }

    // --- geometry ---
    let f = &feats;
    // radial range of a part's samples within an axial window
    let r_in_z = |i: usize, z0: f64, z1: f64| {
        let (mut lo, mut hi) = (f64::INFINITY, 0.0f64);
        for p in &samples[i] {
            let (r, _, z) = axis.cyl(*p, e1, e2);
            if z >= z0 && z <= z1 {
                lo = lo.min(r);
                hi = hi.max(r);
            }
        }
        (lo, hi)
    };
    let encloses = |i: usize| f[i].coverage > 0.97;
    let small = |i: usize| f[i].aabb.map(|b| b.diagonal()).unwrap_or(0.0) < 0.22 * size * 2.0;

    // balls and the rings around them
    let mut ball_rings = Vec::new();
    for ring in &rings {
        let m = &ring.members;
        let ball = m.iter().all(|&i| {
            let e = f[i].extents;
            let d = e[0];
            e[2] > 0.9 * e[0]
                && (f[i].volume / (std::f64::consts::PI / 6.0 * d * d * d) - 1.0).abs() < 0.15
        });
        if ball {
            for &i in m {
                geo(
                    &mut ev,
                    i,
                    Role::Bearing,
                    0.85,
                    format!("ball in a ring of {} balls", m.len()),
                );
            }
            ball_rings.push(ring);
        }
    }
    for i in 0..n {
        if !encloses(i) {
            continue;
        }
        for br in &ball_rings {
            let b = br.members[0];
            let z_overlap = f[i].z_min <= f[b].z_max && f[i].z_max >= f[b].z_min;
            let ring_thin = f[i].r_max - f[i].r_min < 0.8 * (f[b].r_c - 0.0).max(1e-9);
            let next_to = (f[i].r_max - f[b].r_c).abs() < 1.5 * f[b].extents[0]
                || (f[i].r_min - f[b].r_c).abs() < 1.5 * f[b].extents[0];
            if z_overlap && ring_thin && next_to && f[i].z_max - f[i].z_min < 2.5 * f[b].extents[0]
            {
                geo(
                    &mut ev,
                    i,
                    Role::Bearing,
                    0.8,
                    format!("race around a ring of {} balls", br.members.len()),
                );
            }
        }
    }

    // shaft
    for i in 0..n {
        let fi = &f[i];
        let aspect = fi.extents[0] / fi.extents[1].max(1e-12);
        if fi.along_axis > 0.97
            && aspect > 3.0
            && fi.r_c < 0.1 * envelope
            && fi.r_max < 0.3 * envelope
            && fi.extents[0] > 0.4 * length
        {
            geo(
                &mut ev,
                i,
                Role::Shaft,
                0.8,
                format!("long thin part on the axis ({aspect:.0}:1)"),
            );
        }
    }

    // stator: a toothed part around the axis
    let toothed: Vec<usize> = (0..n).filter(|&i| encloses(i) && f[i].order >= 6).collect();
    // coil rings: genus-1, off-axis, not balls, overlapping a toothed part radially
    let mut coil_ring: Option<&Ring> = None;
    let mut magnet_ring: Option<&Ring> = None;
    for ring in &rings {
        let m0 = ring.members[0];
        if ball_rings.iter().any(|b| b.members == ring.members) {
            continue;
        }
        let big = f[m0].aabb.map(|b| b.diagonal()).unwrap_or(0.0) > 0.25 * size;
        if f[m0].genus == Some(1) && big {
            let over_teeth = toothed
                .iter()
                .any(|&t| f[m0].r_max > f[t].r_min && f[m0].r_min < f[t].r_max);
            let w = if over_teeth { 0.75 } else { 0.45 };
            let why = format!(
                "ring of {} loops{}",
                ring.members.len(),
                if over_teeth {
                    " around the teeth of a toothed core"
                } else {
                    ""
                }
            );
            for &i in &ring.members {
                geo(&mut ev, i, Role::Winding, w, why.clone());
            }
            if coil_ring.is_none_or(|c| c.members.len() < ring.members.len()) {
                coil_ring = Some(ring);
            }
        } else if f[m0].genus == Some(0)
            && f[m0].fill > 0.55
            && big
            && ring.members.len() >= 4
            && ring.members.len() % 2 == 0
        {
            // blocks: magnets when they face a toothed core across a gap, along most of its length
            let facing = toothed.iter().any(|&t| {
                let gap_out = f[m0].r_min - f[t].r_max;
                let gap_in = f[t].r_min - f[m0].r_max;
                let radial = (gap_out > -0.01 * envelope && gap_out < 0.06 * envelope)
                    || (gap_in > -0.01 * envelope && gap_in < 0.06 * envelope);
                let overlap = f[m0].z_max.min(f[t].z_max) - f[m0].z_min.max(f[t].z_min);
                radial && overlap > 0.5 * (f[t].z_max - f[t].z_min)
            });
            if facing || toothed.is_empty() {
                let w = if facing { 0.75 } else { 0.3 };
                let why = format!(
                    "ring of {} blocks{}",
                    ring.members.len(),
                    if facing {
                        " facing a toothed core across a gap"
                    } else {
                        ""
                    }
                );
                for &i in &ring.members {
                    geo(&mut ev, i, Role::Magnet, w, why.clone());
                }
            }
            if facing && magnet_ring.is_none_or(|c| c.members.len() < ring.members.len()) {
                magnet_ring = Some(ring);
            }
        } else if !big {
            for &i in &ring.members {
                geo(
                    &mut ev,
                    i,
                    Role::Fastener,
                    0.5,
                    format!(
                        "one of {} small parts evenly around the axis",
                        ring.members.len()
                    ),
                );
            }
        }
    }
    for &t in &toothed {
        let wound = coil_ring.is_some_and(|c| {
            let m = c.members[0];
            f[m].r_max > f[t].r_min && f[m].r_min < f[t].r_max
        });
        if wound {
            geo(
                &mut ev,
                t,
                Role::Stator,
                0.85,
                format!(
                    "{} teeth carrying the {} coils",
                    f[t].order,
                    coil_ring.unwrap().members.len()
                ),
            );
        } else {
            let inside_other = toothed
                .iter()
                .any(|&o| o != t && f[t].r_max <= f[o].r_min * 1.02);
            if inside_other {
                geo(
                    &mut ev,
                    t,
                    Role::Rotor,
                    0.55,
                    format!("{} teeth inside another toothed core", f[t].order),
                );
            } else {
                geo(
                    &mut ev,
                    t,
                    Role::Stator,
                    0.5,
                    format!("toothed core, {} slots", f[t].order),
                );
            }
        }
    }
    let stator = (0..n).find(|&i| {
        ev[i]
            .iter()
            .any(|e| e.role == Role::Stator && e.source == Source::Geometry && e.weight > 0.8)
    });
    // rotor: around the axis, holding the magnet ring on its other side from the stator
    if let Some(mr) = magnet_ring {
        let m = mr.members[0];
        for i in 0..n {
            if !encloses(i) || mr.members.contains(&i) || toothed.contains(&i) {
                continue;
            }
            let (lo, hi) = r_in_z(i, f[m].z_min, f[m].z_max);
            if !lo.is_finite() {
                continue;
            }
            let holds_out = (lo - f[m].r_max).abs() < 0.04 * envelope && hi > f[m].r_max;
            let holds_in = (hi - f[m].r_min).abs() < 0.04 * envelope && lo < f[m].r_min;
            if holds_out || holds_in {
                geo(
                    &mut ev,
                    i,
                    Role::Rotor,
                    0.75,
                    format!("carries the ring of {} magnets", mr.members.len()),
                );
            }
        }
    }
    // hybrid stepper: an axially magnetised ring between two toothed rotor cups
    let cups: Vec<usize> = toothed
        .iter()
        .copied()
        .filter(|&t| {
            ev[t]
                .iter()
                .any(|e| e.role == Role::Rotor && e.source == Source::Geometry)
        })
        .collect();
    if cups.len() == 2 {
        let (a, b) = (&f[cups[0]], &f[cups[1]]);
        let (gap0, gap1) = (a.z_max.min(b.z_max), a.z_min.max(b.z_min));
        for i in 0..n {
            if cups.contains(&i) || !encloses(i) {
                continue;
            }
            let mid = (f[i].z_min + f[i].z_max) / 2.0;
            if mid > gap0
                && mid < gap1
                && f[i].r_max <= a.r_max.max(b.r_max)
                && f[i].r_max > 0.3 * a.r_max
            {
                geo(
                    &mut ev,
                    i,
                    Role::Magnet,
                    0.6,
                    "ring between the two rotor cups (axial magnet)".into(),
                );
            }
        }
    }
    // a stack of like plates across the axis: laminations (vendor models draw a stator stack plate by plate)
    let across = |i: usize| {
        let e = f[i].extents;
        e[2] < 0.08 * e[1] && (f[i].z_max - f[i].z_min) < 2.5 * e[2] && f[i].r_max > 0.3 * envelope
    };
    let mut in_stack = vec![false; n];
    for i in 0..n {
        if !across(i) || in_stack[i] {
            continue;
        }
        let like: Vec<usize> = (0..n)
            .filter(|&j| {
                across(j)
                    && (0..3).all(|k| {
                        (f[j].extents[k] - f[i].extents[k]).abs() < 0.05 * f[i].extents[k].max(1e-4)
                    })
            })
            .collect();
        if like.len() >= 6 {
            for &j in &like {
                in_stack[j] = true;
                geo(
                    &mut ev,
                    j,
                    Role::Stator,
                    0.7,
                    format!(
                        "one of {} like plates stacked along the axis (laminations)",
                        like.len()
                    ),
                );
            }
        }
    }
    // flat plates, long thin off-axis or leaving the envelope
    for i in 0..n {
        let e = f[i].extents;
        let thick = e[2];
        if !in_stack[i]
            && thick < 0.08 * e[1]
            && thick > 0.3e-3
            && thick < 3.5e-3
            && e[0] > 0.6 * envelope
        {
            geo(
                &mut ev,
                i,
                Role::Pcb,
                0.45,
                format!("flat plate {:.1} mm thick", thick * 1e3),
            );
        }
        let aspect = e[0] / e[1].max(1e-12);
        if aspect > 8.0
            && (f[i].along_axis < 0.9 || f[i].r_max > 1.15 * envelope)
            && f[i].r_c > 0.1 * envelope
        {
            geo(
                &mut ev,
                i,
                Role::Wiring,
                0.6,
                format!("long thin ({aspect:.0}:1) leaving the motor"),
            );
        }
        // small elongated parts parallel to the axis, off it
        if small(i)
            && f[i].along_axis > 0.95
            && aspect > 2.0
            && f[i].r_c > 0.15 * envelope
            && !encloses(i)
            && f[i].extents[0] < 0.6 * length
        {
            geo(
                &mut ev,
                i,
                Role::Fastener,
                0.35,
                "small rod parallel to the axis".into(),
            );
        }
    }
    // large parts around the axis without teeth that are nothing else yet: housing
    for i in 0..n {
        let has_geo = ev[i].iter().any(|e| e.source == Source::Geometry);
        if !has_geo && encloses(i) {
            if f[i].r_max > 0.6 * envelope {
                geo(
                    &mut ev,
                    i,
                    Role::Housing,
                    0.4,
                    "large part around the axis".into(),
                );
            } else if f[i].r_min > 0.1 * envelope {
                geo(
                    &mut ev,
                    i,
                    Role::Housing,
                    0.25,
                    "tube around the axis".into(),
                );
            }
        }
    }

    // --- verdicts ---
    let mut verdicts = Vec::with_capacity(n);
    for i in 0..n {
        let score = |r: Role| noisy_or(ev[i].iter().filter(|e| e.role == r).map(|e| e.weight));
        let mut ranked: Vec<(Role, f64)> = Role::ALL.iter().map(|&r| (r, score(r))).collect();
        ranked.sort_by(|a, b| b.1.partial_cmp(&a.1).unwrap());
        let (role, s1) = if ranked[0].1 > 0.0 {
            ranked[0]
        } else {
            (Role::Other, 0.0)
        };
        // a tag or override is the convention: only geometry can contest it
        let tagged = ev[i]
            .iter()
            .any(|e| matches!(e.source, Source::Tag | Source::Override) && e.role == role);
        let s2 = if tagged {
            ranked[1..]
                .iter()
                .map(|(r, _)| {
                    noisy_or(
                        ev[i]
                            .iter()
                            .filter(|e| e.role == *r && e.source == Source::Geometry)
                            .map(|e| e.weight),
                    )
                })
                .fold(0.0, f64::max)
        } else {
            ranked[1].1
        };
        let mut confidence = if ev[i]
            .iter()
            .any(|e| e.source == Source::Override && e.role == role)
        {
            1.0
        } else {
            s1 * (1.0 - 0.5 * s2 / s1.max(1e-9))
        };
        if role == Role::Other && s1 == 0.0 {
            confidence = 0.0;
        }
        // conflict: the names' best role against geometry's best role
        let best_of = |pred: &dyn Fn(&Evidence) -> bool| {
            ev[i]
                .iter()
                .filter(|e| pred(e))
                .max_by(|a, b| a.weight.partial_cmp(&b.weight).unwrap())
                .map(|e| e.role)
        };
        let named = best_of(&|e| matches!(e.source, Source::Tag | Source::Name | Source::Override));
        let geom = best_of(&|e| e.source == Source::Geometry && e.weight >= 0.6);
        let conflict = match (named, geom) {
            (Some(a), Some(b)) if a != b => Some(format!("name says {a}, geometry says {b}")),
            _ => None,
        };
        let o = over[i];
        let rotating = o
            .and_then(|o| o.rotating)
            .or(rot[i])
            .unwrap_or(role.rotates());
        let polarity = o
            .and_then(|o| o.polarity)
            .or(pol[i])
            .filter(|_| role == Role::Magnet);
        let mut evidence = ev[i].clone();
        evidence.sort_by(|a, b| b.weight.partial_cmp(&a.weight).unwrap());
        verdicts.push(Verdict {
            role,
            confidence,
            rotating,
            polarity,
            evidence,
            conflict,
        });
    }

    // topology and counts
    let topology = match (stator, magnet_ring) {
        (Some(s), Some(m)) => {
            if f[m.members[0]].r_c > f[s].r_c {
                Topology::Outrunner
            } else {
                Topology::Inrunner
            }
        }
        _ => {
            let st: Vec<usize> = (0..n)
                .filter(|&i| verdicts[i].role == Role::Stator)
                .collect();
            let ro: Vec<usize> = (0..n)
                .filter(|&i| matches!(verdicts[i].role, Role::Rotor | Role::Magnet))
                .collect();
            let r = |v: &[usize]| v.iter().map(|&i| f[i].r_max).fold(0.0, f64::max);
            if st.is_empty() || ro.is_empty() {
                Topology::Unknown
            } else if r(&ro) > r(&st) {
                Topology::Outrunner
            } else {
                Topology::Inrunner
            }
        }
    };
    let count = |role: Role| {
        rings
            .iter()
            .filter(|r| r.members.iter().all(|&i| verdicts[i].role == role))
            .map(|r| r.members.len())
            .max()
    };
    let slots = count(Role::Winding).or_else(|| stator.map(|s| f[s].order));
    let poles = count(Role::Magnet);
    if dot(axis.dir, axis.dir) < 0.5 {
        notes.push("no axis found".into());
    }
    Classification {
        axis,
        topology,
        envelope_r: envelope,
        slots,
        poles,
        features: feats,
        rings,
        verdicts,
        notes,
    }
}
