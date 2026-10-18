//! Triangle meshes of parts (f32, metres) and the geometry the classifier measures on them.

use serde::{Deserialize, Serialize};

pub type V3 = [f64; 3];

/// An indexed triangle mesh with per-vertex normals.
#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct Mesh {
    /// m
    pub positions: Vec<[f32; 3]>,
    pub normals: Vec<[f32; 3]>,
    pub indices: Vec<u32>,
}

pub fn sub(a: V3, b: V3) -> V3 {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}
pub fn add(a: V3, b: V3) -> V3 {
    [a[0] + b[0], a[1] + b[1], a[2] + b[2]]
}
pub fn scale(a: V3, s: f64) -> V3 {
    [a[0] * s, a[1] * s, a[2] * s]
}
pub fn dot(a: V3, b: V3) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}
pub fn cross(a: V3, b: V3) -> V3 {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}
pub fn norm(a: V3) -> f64 {
    dot(a, a).sqrt()
}
pub fn normalize(a: V3) -> V3 {
    let n = norm(a);
    if n > 0.0 { scale(a, 1.0 / n) } else { a }
}

fn v64(p: [f32; 3]) -> V3 {
    [p[0] as f64, p[1] as f64, p[2] as f64]
}

/// Axis-aligned bounding box, m.
#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
pub struct Aabb {
    pub min: V3,
    pub max: V3,
}

impl Aabb {
    pub const EMPTY: Aabb = Aabb {
        min: [f64::INFINITY; 3],
        max: [f64::NEG_INFINITY; 3],
    };

    pub fn push(&mut self, p: V3) {
        for i in 0..3 {
            self.min[i] = self.min[i].min(p[i]);
            self.max[i] = self.max[i].max(p[i]);
        }
    }

    pub fn union(mut self, o: &Aabb) -> Aabb {
        self.push(o.min);
        self.push(o.max);
        self
    }

    pub fn is_empty(&self) -> bool {
        self.min[0] > self.max[0]
    }

    pub fn size(&self) -> V3 {
        sub(self.max, self.min)
    }

    pub fn center(&self) -> V3 {
        scale(add(self.min, self.max), 0.5)
    }

    pub fn diagonal(&self) -> f64 {
        if self.is_empty() {
            0.0
        } else {
            norm(self.size())
        }
    }
}

impl Mesh {
    pub fn triangles(&self) -> impl Iterator<Item = [V3; 3]> + '_ {
        self.indices.chunks_exact(3).map(|t| {
            [
                v64(self.positions[t[0] as usize]),
                v64(self.positions[t[1] as usize]),
                v64(self.positions[t[2] as usize]),
            ]
        })
    }

    pub fn triangle_count(&self) -> usize {
        self.indices.len() / 3
    }

    pub fn aabb(&self) -> Aabb {
        let mut b = Aabb::EMPTY;
        for p in &self.positions {
            b.push(v64(*p));
        }
        b
    }

    /// Surface area, m².
    pub fn area(&self) -> f64 {
        self.triangles()
            .map(|[a, b, c]| 0.5 * norm(cross(sub(b, a), sub(c, a))))
            .sum()
    }

    /// Enclosed volume by the divergence theorem, m³ (absolute: shells may come inside out); approximate when the
    /// mesh is not closed.
    pub fn volume(&self) -> f64 {
        self.triangles()
            .map(|[a, b, c]| dot(a, cross(b, c)) / 6.0)
            .sum::<f64>()
            .abs()
    }

    /// Applies a 4x4 column-major transform (m) to positions and normals.
    pub fn transform(&mut self, m: &[[f64; 4]; 4]) {
        for p in &mut self.positions {
            let v = v64(*p);
            let r = [0, 1, 2].map(|i| m[0][i] * v[0] + m[1][i] * v[1] + m[2][i] * v[2] + m[3][i]);
            *p = r.map(|x| x as f32);
        }
        for n in &mut self.normals {
            let v = v64(*n);
            let r = normalize([0, 1, 2].map(|i| m[0][i] * v[0] + m[1][i] * v[1] + m[2][i] * v[2]));
            *n = r.map(|x| x as f32);
        }
    }

    pub fn append(&mut self, o: &Mesh) {
        let base = self.positions.len() as u32;
        self.positions.extend(&o.positions);
        self.normals.extend(&o.normals);
        self.indices.extend(o.indices.iter().map(|i| i + base));
    }

    /// Deterministic area-weighted surface samples (a low-discrepancy sequence per triangle budget), m.
    pub fn surface_samples(&self, n: usize) -> Vec<V3> {
        let tris: Vec<[V3; 3]> = self.triangles().collect();
        let areas: Vec<f64> = tris
            .iter()
            .map(|[a, b, c]| 0.5 * norm(cross(sub(*b, *a), sub(*c, *a))))
            .collect();
        let total: f64 = areas.iter().sum();
        if total <= 0.0 {
            return self.positions.iter().map(|p| v64(*p)).collect();
        }
        let mut out = Vec::with_capacity(n + tris.len());
        let mut carry = 0.0;
        let mut seq = 0u32;
        for (t, a) in tris.iter().zip(&areas) {
            carry += a / total * n as f64;
            while carry >= 1.0 {
                carry -= 1.0;
                seq = seq.wrapping_add(1);
                // R2 sequence, folded into the triangle
                let (mut u, mut v) = (
                    (seq as f64 * 0.754_877_666).fract(),
                    (seq as f64 * 0.569_840_291).fract(),
                );
                if u + v > 1.0 {
                    u = 1.0 - u;
                    v = 1.0 - v;
                }
                out.push(add(
                    t[0],
                    add(scale(sub(t[1], t[0]), u), scale(sub(t[2], t[0]), v)),
                ));
            }
        }
        out
    }

    /// Splits the mesh into connected components (by shared vertex positions).
    pub fn components(&self) -> Vec<Mesh> {
        let n = self.positions.len();
        let mut parent: Vec<usize> = (0..n).collect();
        fn find(p: &mut [usize], mut i: usize) -> usize {
            while p[i] != i {
                p[i] = p[p[i]];
                i = p[i];
            }
            i
        }
        // weld equal positions first
        let mut by_pos = std::collections::HashMap::new();
        for (i, p) in self.positions.iter().enumerate() {
            let key = p.map(|x| (x * 1e6).round() as i64);
            if let Some(&j) = by_pos.get(&key) {
                let (a, b) = (find(&mut parent, i), find(&mut parent, j));
                parent[a] = b;
            } else {
                by_pos.insert(key, i);
            }
        }
        for t in self.indices.chunks_exact(3) {
            let a = find(&mut parent, t[0] as usize);
            for &k in &t[1..] {
                let b = find(&mut parent, k as usize);
                parent[b] = a;
            }
        }
        let mut comp_of_root = std::collections::HashMap::new();
        let mut comps: Vec<Mesh> = Vec::new();
        let mut remap: Vec<Option<(usize, u32)>> = vec![None; n];
        for t in self.indices.chunks_exact(3) {
            let root = find(&mut parent, t[0] as usize);
            let ci = *comp_of_root.entry(root).or_insert_with(|| {
                comps.push(Mesh::default());
                comps.len() - 1
            });
            for &k in t {
                let k = k as usize;
                let local = match remap[k] {
                    Some((c, l)) if c == ci => l,
                    _ => {
                        let m = &mut comps[ci];
                        m.positions.push(self.positions[k]);
                        m.normals
                            .push(self.normals.get(k).copied().unwrap_or([0.0, 0.0, 1.0]));
                        let l = m.positions.len() as u32 - 1;
                        remap[k] = Some((ci, l));
                        l
                    }
                };
                comps[ci].indices.push(local);
            }
        }
        comps
    }
}
