//! Geometry the classifier measures: the motor axis, each part in the motor's cylindrical frame (radii, axial
//! range, angular coverage, rotational symmetry order), shape descriptors (principal axes, aspect, sphericity,
//! genus) and rings of similar parts around the axis.

use crate::mesh::{Aabb, Mesh, V3, add, cross, dot, norm, normalize, scale, sub};
use serde::{Deserialize, Serialize};
use std::f64::consts::PI;

/// The motor axis, m.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Axis {
    pub origin: V3,
    pub dir: V3,
    /// how it was found
    pub how: String,
}

impl Axis {
    /// Perpendicular unit vectors completing a right-handed frame with `dir`.
    pub fn frame(&self) -> (V3, V3) {
        let helper = if self.dir[0].abs() < 0.9 {
            [1.0, 0.0, 0.0]
        } else {
            [0.0, 1.0, 0.0]
        };
        let e1 = normalize(cross(helper, self.dir));
        (e1, cross(self.dir, e1))
    }

    /// (r, θ, z) of a point.
    pub fn cyl(&self, p: V3, e1: V3, e2: V3) -> (f64, f64, f64) {
        let d = sub(p, self.origin);
        let z = dot(d, self.dir);
        let (x, y) = (dot(d, e1), dot(d, e2));
        (x.hypot(y), y.atan2(x), z)
    }
}

/// Symmetric 3x3 eigen decomposition (Jacobi): eigenvalues descending with unit eigenvectors.
pub fn eigen_sym(m: [[f64; 3]; 3]) -> ([f64; 3], [V3; 3]) {
    let mut a = m;
    let mut v = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
    for _ in 0..50 {
        let (mut p, mut q, mut max) = (0, 1, 0.0);
        for i in 0..3 {
            for j in i + 1..3 {
                if a[i][j].abs() > max {
                    (p, q, max) = (i, j, a[i][j].abs());
                }
            }
        }
        if max < 1e-30 {
            break;
        }
        let theta = (a[q][q] - a[p][p]) / (2.0 * a[p][q]);
        let t = theta.signum() / (theta.abs() + (theta * theta + 1.0).sqrt());
        let t = if theta == 0.0 { 1.0 } else { t };
        let c = 1.0 / (t * t + 1.0).sqrt();
        let s = t * c;
        let mut b = a;
        for k in 0..3 {
            b[k][p] = c * a[k][p] - s * a[k][q];
            b[k][q] = s * a[k][p] + c * a[k][q];
        }
        a = b;
        for k in 0..3 {
            b[p][k] = c * a[p][k] - s * a[q][k];
            b[q][k] = s * a[p][k] + c * a[q][k];
        }
        a = b;
        for row in &mut v {
            let (vp, vq) = (row[p], row[q]);
            row[p] = c * vp - s * vq;
            row[q] = s * vp + c * vq;
        }
    }
    let mut idx = [0, 1, 2];
    idx.sort_by(|&i, &j| {
        a[j][j]
            .partial_cmp(&a[i][i])
            .unwrap_or(std::cmp::Ordering::Equal)
    });
    let vals = idx.map(|i| a[i][i]);
    let vecs = idx.map(|i| [v[0][i], v[1][i], v[2][i]]);
    (vals, vecs)
}

/// Mean and covariance of points.
pub fn covariance(points: &[V3]) -> (V3, [[f64; 3]; 3]) {
    let n = points.len().max(1) as f64;
    let mean = scale(points.iter().fold([0.0; 3], |a, p| add(a, *p)), 1.0 / n);
    let mut c = [[0.0; 3]; 3];
    for p in points {
        let d = sub(*p, mean);
        for i in 0..3 {
            for j in 0..3 {
                c[i][j] += d[i] * d[j] / n;
            }
        }
    }
    (mean, c)
}

/// Shape and placement of one part, m.
#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct Features {
    pub area: f64,
    pub volume: f64,
    pub centroid: V3,
    pub aabb: Option<Aabb>,
    /// principal extents (full lengths along the principal axes, largest first)
    pub extents: [f64; 3],
    /// principal axes, largest extent first
    pub axes: [V3; 3],
    /// |cos| between the largest principal axis and the motor axis
    pub along_axis: f64,
    pub r_min: f64,
    pub r_max: f64,
    /// distance of the centroid from the axis
    pub r_c: f64,
    pub theta_c: f64,
    pub z_min: f64,
    pub z_max: f64,
    /// fraction of 360 one-degree bins the part covers around the axis
    pub coverage: f64,
    /// dominant rotational order of its outline and that harmonic's amplitude relative to the mean radius
    pub order: usize,
    pub order_amp: f64,
    /// genus of the welded mesh (0 for a block, 1 for a ring or coil), when closed
    pub genus: Option<i64>,
    /// volume / volume of the principal box
    pub fill: f64,
}

/// The genus of a closed triangle mesh after welding equal positions; None when it is not closed.
pub fn genus(mesh: &Mesh) -> Option<i64> {
    let mut id = std::collections::HashMap::new();
    let mut vid = Vec::with_capacity(mesh.positions.len());
    for p in &mesh.positions {
        let key = p.map(|x| (x as f64 * 1e7).round() as i64);
        let n = id.len();
        vid.push(*id.entry(key).or_insert(n));
    }
    let mut edges = std::collections::HashMap::new();
    let mut faces = 0i64;
    for t in mesh.indices.chunks_exact(3) {
        let v = [vid[t[0] as usize], vid[t[1] as usize], vid[t[2] as usize]];
        if v[0] == v[1] || v[1] == v[2] || v[0] == v[2] {
            continue;
        }
        faces += 1;
        for k in 0..3 {
            let (a, b) = (v[k], v[(k + 1) % 3]);
            *edges.entry((a.min(b), a.max(b))).or_insert(0) += 1;
        }
    }
    if faces == 0 || edges.values().any(|&n| n != 2) {
        return None;
    }
    let used: std::collections::HashSet<usize> = edges.keys().flat_map(|&(a, b)| [a, b]).collect();
    let chi = used.len() as i64 - edges.len() as i64 + faces;
    // shells: assume one (a part); χ = 2 − 2g
    Some((2 - chi) / 2)
}

/// Rotational order from slots: surface samples counted per angle bin in radial bands across the part; a band
/// with N evenly spaced angular gaps (no material at all) shows N teeth, slots or poles. Returns the largest such N
/// over the bands (the stepper's 8 poles beat its square's 4 corners) and the empty fraction of that band.
fn rotational_order(cyl: &[(f64, f64)], r_min: f64, r_max: f64) -> (usize, f64) {
    const ABINS: usize = 360;
    const EDGES: [f64; 10] = [0.0, 0.03, 0.1, 0.2, 0.35, 0.5, 0.65, 0.8, 0.97, 1.0];
    let span = (r_max - r_min).max(1e-12);
    let mut best = (0, 0.0);
    for w in EDGES.windows(2) {
        let (lo, hi) = (r_min + w[0] * span, r_min + w[1] * span);
        let mut counts = vec![0usize; ABINS];
        let mut total = 0usize;
        for &(r, t) in cyl {
            if r >= lo && r <= hi {
                counts[(((t + PI) / (2.0 * PI)) * ABINS as f64) as usize % ABINS] += 1;
                total += 1;
            }
        }
        if total < 10 * ABINS {
            continue;
        }
        let mean = total as f64 / ABINS as f64;
        let occ: Vec<bool> = counts.iter().map(|&c| c as f64 > 0.08 * mean).collect();
        let empty = occ.iter().filter(|o| !**o).count();
        if empty == 0 || empty == ABINS {
            continue;
        }
        // gap centres: runs of empty bins, walking from an occupied bin
        let start = occ.iter().position(|o| *o).unwrap();
        let mut centres = Vec::new();
        let mut run: Option<usize> = None;
        for k in 1..=ABINS {
            let b = (start + k) % ABINS;
            match (occ[b], run) {
                (false, None) => run = Some(k),
                (true, Some(r0)) => {
                    centres.push((r0 + k - 1) as f64 / 2.0);
                    run = None;
                }
                _ => {}
            }
        }
        let g = centres.len();
        if g < 3 {
            continue;
        }
        let gaps: Vec<f64> = (0..g)
            .map(|k| {
                if k + 1 < g {
                    centres[k + 1] - centres[k]
                } else {
                    centres[0] + ABINS as f64 - centres[k]
                }
            })
            .collect();
        let m = gaps.iter().sum::<f64>() / g as f64;
        let sd = (gaps.iter().map(|x| (x - m).powi(2)).sum::<f64>() / g as f64).sqrt();
        if sd / m < 0.2 && g > best.0 {
            best = (g, empty as f64 / ABINS as f64);
        }
    }
    best
}

pub const BINS: usize = 360;

/// Measures one part.
pub fn features(mesh: &Mesh, samples: &[V3], axis: &Axis) -> Features {
    let (e1, e2) = axis.frame();
    let mut f = Features {
        area: mesh.area(),
        volume: mesh.volume(),
        aabb: Some(mesh.aabb()),
        ..Default::default()
    };
    let pts: Vec<V3> = if samples.is_empty() {
        mesh.positions.iter().map(|p| p.map(|x| x as f64)).collect()
    } else {
        samples.to_vec()
    };
    let (mean, cov) = covariance(&pts);
    f.centroid = mean;
    let (_, vecs) = eigen_sym(cov);
    let mut ext = [0.0; 3];
    for (k, v) in vecs.iter().enumerate() {
        let (mut lo, mut hi) = (f64::INFINITY, f64::NEG_INFINITY);
        for p in mesh.positions.iter().map(|p| p.map(|x| x as f64)) {
            let s = dot(sub(p, mean), *v);
            lo = lo.min(s);
            hi = hi.max(s);
        }
        ext[k] = (hi - lo).max(0.0);
    }
    let mut order: Vec<usize> = vec![0, 1, 2];
    order.sort_by(|&a, &b| ext[b].partial_cmp(&ext[a]).unwrap());
    f.extents = order
        .iter()
        .map(|&i| ext[i])
        .collect::<Vec<_>>()
        .try_into()
        .unwrap();
    f.axes = order
        .iter()
        .map(|&i| vecs[i])
        .collect::<Vec<_>>()
        .try_into()
        .unwrap();
    f.along_axis = dot(f.axes[0], axis.dir).abs();
    let boxv = f.extents[0] * f.extents[1] * f.extents[2];
    f.fill = if boxv > 0.0 { f.volume / boxv } else { 0.0 };
    f.genus = genus(mesh);

    let (rc, tc, _) = axis.cyl(mean, e1, e2);
    f.r_c = rc;
    f.theta_c = tc;
    let (mut rmin, mut rmax, mut zmin, mut zmax) =
        (f64::INFINITY, 0.0f64, f64::INFINITY, f64::NEG_INFINITY);
    let mut hit = vec![false; BINS];
    for p in pts
        .iter()
        .copied()
        .chain(mesh.positions.iter().map(|p| p.map(|x| x as f64)))
    {
        let (r, t, z) = axis.cyl(p, e1, e2);
        rmin = rmin.min(r);
        rmax = rmax.max(r);
        zmin = zmin.min(z);
        zmax = zmax.max(z);
        let b = (((t + PI) / (2.0 * PI)) * BINS as f64) as usize % BINS;
        hit[b] = true;
    }
    f.r_min = rmin;
    f.r_max = rmax;
    f.z_min = zmin;
    f.z_max = zmax;
    f.coverage = hit.iter().filter(|h| **h).count() as f64 / BINS as f64;
    if f.coverage > 0.97 {
        let dense = mesh.surface_samples(60_000);
        let cyl: Vec<(f64, f64)> = dense
            .iter()
            .map(|p| {
                let (r, t, _) = axis.cyl(*p, e1, e2);
                (r, t)
            })
            .collect();
        (f.order, f.order_amp) = rotational_order(&cyl, rmin, rmax);
    }
    f
}

/// The motor axis: most motor parts are bodies of revolution about it (shaft, tubes, discs, rings, the stator
/// with its teeth). Each part whose second moments have two equal eigenvalues votes for the third eigenvector, by
/// area; the heaviest direction wins and passes through the area-weighted centroid of its voters; a long, thin
/// part lying on it (the shaft) refines the line. Falls back to the assembly's principal axis.
pub fn find_axis(samples: &[Vec<V3>], meshes: &[&Mesh]) -> Axis {
    struct Vote {
        dir: V3,
        at: V3,
        w: f64,
        rod: bool,
        len: f64,
    }
    let mut votes = Vec::new();
    for (k, pts) in samples.iter().enumerate() {
        if pts.len() < 50 {
            continue;
        }
        let (m, c) = covariance(pts);
        let (l, v) = eigen_sym(c);
        let top = l[0].max(1e-30);
        let close = |a: f64, b: f64| (a - b).abs() < 0.03 * top;
        let apart = |a: f64, b: f64| (a - b).abs() > 0.15 * top;
        let area = meshes[k].area();
        // disc-like: λ1 ≈ λ2 > λ3, axis v3; rod-like: λ1 > λ2 ≈ λ3, axis v1
        if close(l[0], l[1]) && apart(l[1], l[2]) {
            votes.push(Vote {
                dir: v[2],
                at: m,
                w: area,
                rod: false,
                len: 0.0,
            });
        } else if close(l[1], l[2]) && apart(l[0], l[1]) {
            votes.push(Vote {
                dir: v[0],
                at: m,
                w: area,
                rod: true,
                len: meshes[k].aabb().diagonal(),
            });
        }
    }
    let sign = |d: V3| {
        let s = if d[2].abs() > 0.3 {
            d[2]
        } else if d[1].abs() > 0.3 {
            d[1]
        } else {
            d[0]
        };
        if s < 0.0 { scale(d, -1.0) } else { d }
    };
    let mut best: Option<(f64, usize)> = None;
    for (i, a) in votes.iter().enumerate() {
        let w: f64 = votes
            .iter()
            .filter(|b| dot(a.dir, b.dir).abs() > 0.995)
            .map(|b| b.w)
            .sum();
        if best.is_none_or(|(bw, _)| w > bw) {
            best = Some((w, i));
        }
    }
    let total_area: f64 = meshes.iter().map(|m| m.area()).sum::<f64>().max(1e-30);
    if let Some((w, i)) = best {
        let mut dir = sign(votes[i].dir);
        let voters: Vec<&Vote> = votes
            .iter()
            .filter(|b| dot(dir, b.dir).abs() > 0.995)
            .collect();
        // the line through the voters' centroids: their mean, with the spread along the axis kept
        let wsum: f64 = voters.iter().map(|v| v.w).sum();
        let mut origin = scale(
            voters
                .iter()
                .fold([0.0; 3], |a, v| add(a, scale(v.at, v.w))),
            1.0 / wsum,
        );
        let mut how = format!(
            "symmetry axis of {} round parts ({:.0} % of the area)",
            voters.len(),
            100.0 * w / total_area
        );
        // refine to the longest rod among them that lies on that line
        let on_line = |p: V3| norm(sub(sub(p, origin), scale(dir, dot(sub(p, origin), dir))));
        if let Some(rod) = voters
            .iter()
            .filter(|v| v.rod)
            .filter(|v| {
                on_line(v.at)
                    < 0.1
                        * meshes
                            .iter()
                            .map(|m| m.aabb().diagonal())
                            .fold(0.0, f64::max)
            })
            .max_by(|a, b| a.len.partial_cmp(&b.len).unwrap())
        {
            origin = rod.at;
            dir = if dot(rod.dir, dir) < 0.0 {
                scale(rod.dir, -1.0)
            } else {
                rod.dir
            };
            how.push_str(", along the longest rod on it");
        }
        return Axis { origin, dir, how };
    }
    let all: Vec<V3> = samples.iter().flatten().copied().collect();
    let (mean, cov) = covariance(&all);
    let (vals, vecs) = eigen_sym(cov);
    let top = vals[0].max(1e-30);
    let d = [0, 1, 2].map(|i| {
        let o: Vec<f64> = (0..3)
            .filter(|&j| j != i)
            .map(|j| (vals[i] - vals[j]).abs())
            .collect();
        o[0].min(o[1]) / top
    });
    let i = (0..3)
        .max_by(|&a, &b| d[a].partial_cmp(&d[b]).unwrap())
        .unwrap();
    Axis {
        origin: mean,
        dir: sign(vecs[i]),
        how: format!(
            "no round parts: principal axis of the assembly ({:.2})",
            d[i]
        ),
    }
}

/// Sets of similar parts evenly spaced around the axis.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Ring {
    pub members: Vec<usize>,
    /// mean centroid radius, m
    pub r_c: f64,
    /// spacing regularity: std of the angular gaps / mean gap (0 = perfectly even)
    pub irregularity: f64,
}

/// Groups parts by shape (area and volume within a few per cent, same radial and axial place) and keeps the groups
/// of 3 or more off-axis members that sit evenly around the axis.
pub fn rings(f: &[Features], envelope: f64) -> Vec<Ring> {
    let n = f.len();
    let mut parent: Vec<usize> = (0..n).collect();
    fn find(p: &mut [usize], mut i: usize) -> usize {
        while p[i] != i {
            p[i] = p[p[i]];
            i = p[i];
        }
        i
    }
    let near = |a: f64, b: f64, tol: f64| (a - b).abs() <= tol * a.abs().max(b.abs()).max(1e-30);
    for i in 0..n {
        for j in i + 1..n {
            let (a, b) = (&f[i], &f[j]);
            if a.r_c < 0.05 * envelope || b.r_c < 0.05 * envelope {
                continue;
            }
            if near(a.area, b.area, 0.04)
                && near(a.volume, b.volume, 0.06)
                && (a.r_c - b.r_c).abs() < 0.03 * envelope
                && (a.z_min - b.z_min).abs() < 0.03 * envelope
                && (a.z_max - b.z_max).abs() < 0.03 * envelope
            {
                let (x, y) = (find(&mut parent, i), find(&mut parent, j));
                parent[x] = y;
            }
        }
    }
    let mut groups: std::collections::BTreeMap<usize, Vec<usize>> = Default::default();
    for i in 0..n {
        let r = find(&mut parent, i);
        groups.entry(r).or_default().push(i);
    }
    let mut out = Vec::new();
    for (_, m) in groups {
        if m.len() < 3 {
            continue;
        }
        let mut th: Vec<f64> = m.iter().map(|&i| f[i].theta_c).collect();
        th.sort_by(|a, b| a.partial_cmp(b).unwrap());
        let gaps: Vec<f64> = (0..th.len())
            .map(|k| {
                if k + 1 < th.len() {
                    th[k + 1] - th[k]
                } else {
                    th[0] + 2.0 * PI - th[k]
                }
            })
            .collect();
        let mean = gaps.iter().sum::<f64>() / gaps.len() as f64;
        let sd = (gaps.iter().map(|g| (g - mean).powi(2)).sum::<f64>() / gaps.len() as f64).sqrt();
        let irregularity = sd / mean;
        if irregularity < 0.25 {
            let r_c = m.iter().map(|&i| f[i].r_c).sum::<f64>() / m.len() as f64;
            out.push(Ring {
                members: m,
                r_c,
                irregularity,
            });
        }
    }
    out
}
