//! Tables on uniform grids: [`Grid2`] and [`Grid3`] over borrowed `f32` slices, no allocation.
//!
//! A uniform axis finds its cell by one multiply (no search), which keeps lookups cheap on a CPU and trivial on an
//! FPGA. Values between nodes are bi- or trilinear; outside a bounded axis the edge cell extends linearly. A
//! periodic axis (rotor angle) wraps.

/// A uniform axis: `len` nodes at `min + k·step`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Axis {
    min: f32,
    step: f32,
    inv_step: f32,
    len: u32,
    periodic: bool,
}

impl Axis {
    /// `len` ≥ 2 nodes from `min` to `max` (both included). `None` for fewer nodes or `max <= min`.
    pub fn new(min: f32, max: f32, len: u32) -> Option<Axis> {
        if len < 2 || max.partial_cmp(&min) != Some(core::cmp::Ordering::Greater) {
            return None;
        }
        let step = (max - min) / (len - 1) as f32;
        Some(Axis {
            min,
            step,
            inv_step: 1.0 / step,
            len,
            periodic: false,
        })
    }

    /// `len` ≥ 2 nodes over one `period` starting at `min`; the node at `min + period` is the first one again (not
    /// stored). For the rotor angle: a table over 60° electrical with 60 nodes is `periodic(0.0, π/3, 60)`.
    pub fn periodic(min: f32, period: f32, len: u32) -> Option<Axis> {
        if len < 2 || period.partial_cmp(&0.0) != Some(core::cmp::Ordering::Greater) {
            return None;
        }
        let step = period / len as f32;
        Some(Axis {
            min,
            step,
            inv_step: 1.0 / step,
            len,
            periodic: true,
        })
    }

    /// Number of stored nodes.
    pub fn len(&self) -> usize {
        self.len as usize
    }

    /// Always false: an axis has at least two nodes.
    pub fn is_empty(&self) -> bool {
        false
    }

    /// Value at node `k`.
    pub fn node(&self, k: usize) -> f32 {
        self.min + k as f32 * self.step
    }

    /// Node spacing.
    pub fn step(&self) -> f32 {
        self.step
    }

    /// Cell index `k`, the next node `k1` (wrapped on a periodic axis) and the fraction within the cell (outside
    /// 0..=1 when extrapolating a bounded axis).
    fn locate(&self, v: f32) -> (usize, usize, f32) {
        let u = (v - self.min) * self.inv_step;
        if self.periodic {
            let n = self.len as f32;
            let mut u = u - n * libm::floorf(u / n);
            if u >= n {
                u = 0.0; // rounding of a value just below a full period
            }
            let k = (u as usize).min(self.len as usize - 1);
            let k1 = if k + 1 == self.len as usize { 0 } else { k + 1 };
            (k, k1, u - k as f32)
        } else {
            let k = if u.is_nan() {
                0
            } else {
                (libm::floorf(u).max(0.0) as usize).min(self.len as usize - 2)
            };
            (k, k + 1, u - k as f32)
        }
    }
}

/// A table over two uniform axes, `data[ix·ny + iy]` (y fastest).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Grid2<'a> {
    x: Axis,
    y: Axis,
    data: &'a [f32],
}

impl<'a> Grid2<'a> {
    /// `None` unless `data.len() == x.len()·y.len()`.
    pub fn new(x: Axis, y: Axis, data: &'a [f32]) -> Option<Self> {
        (data.len() == x.len() * y.len()).then_some(Grid2 { x, y, data })
    }

    /// First axis.
    pub fn x(&self) -> &Axis {
        &self.x
    }

    /// Second axis.
    pub fn y(&self) -> &Axis {
        &self.y
    }

    /// The values, y fastest.
    pub fn data(&self) -> &'a [f32] {
        self.data
    }

    /// Bilinear value at (x, y).
    pub fn eval(&self, x: f32, y: f32) -> f32 {
        self.eval_grad(x, y).0
    }

    /// Bilinear value and its gradient (∂/∂x, ∂/∂y) within the cell.
    pub fn eval_grad(&self, x: f32, y: f32) -> (f32, [f32; 2]) {
        let (i0, i1, fx) = self.x.locate(x);
        let (j0, j1, fy) = self.y.locate(y);
        let ny = self.y.len();
        let d = self.data;
        let (v00, v01) = (d[i0 * ny + j0], d[i0 * ny + j1]);
        let (v10, v11) = (d[i1 * ny + j0], d[i1 * ny + j1]);
        let a = v00 + (v01 - v00) * fy;
        let b = v10 + (v11 - v10) * fy;
        let dx = (b - a) * self.x.inv_step;
        let dy = ((v01 - v00) + ((v11 - v10) - (v01 - v00)) * fx) * self.y.inv_step;
        (a + (b - a) * fx, [dx, dy])
    }
}

/// A table over three uniform axes, `data[(ix·ny + iy)·nz + iz]` (z fastest).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Grid3<'a> {
    x: Axis,
    y: Axis,
    z: Axis,
    data: &'a [f32],
}

impl<'a> Grid3<'a> {
    /// `None` unless `data.len() == x.len()·y.len()·z.len()`.
    pub fn new(x: Axis, y: Axis, z: Axis, data: &'a [f32]) -> Option<Self> {
        (data.len() == x.len() * y.len() * z.len()).then_some(Grid3 { x, y, z, data })
    }

    /// First axis.
    pub fn x(&self) -> &Axis {
        &self.x
    }

    /// Second axis.
    pub fn y(&self) -> &Axis {
        &self.y
    }

    /// Third axis.
    pub fn z(&self) -> &Axis {
        &self.z
    }

    /// The values, z fastest.
    pub fn data(&self) -> &'a [f32] {
        self.data
    }

    /// Trilinear value at (x, y, z).
    pub fn eval(&self, x: f32, y: f32, z: f32) -> f32 {
        self.eval_grad(x, y, z).0
    }

    /// Trilinear value and its gradient (∂/∂x, ∂/∂y, ∂/∂z) within the cell.
    pub fn eval_grad(&self, x: f32, y: f32, z: f32) -> (f32, [f32; 3]) {
        let (i0, i1, fx) = self.x.locate(x);
        let (j0, j1, fy) = self.y.locate(y);
        let (k0, k1, fz) = self.z.locate(z);
        let (ny, nz) = (self.y.len(), self.z.len());
        let d = self.data;
        let at = |i: usize, j: usize| {
            let (v0, v1) = (d[(i * ny + j) * nz + k0], d[(i * ny + j) * nz + k1]);
            (v0 + (v1 - v0) * fz, v1 - v0)
        };
        // Along z first, then y, then x; each step keeps the value and the z slope.
        let (c00, s00) = at(i0, j0);
        let (c01, s01) = at(i0, j1);
        let (c10, s10) = at(i1, j0);
        let (c11, s11) = at(i1, j1);
        let (c0, c1) = (c00 + (c01 - c00) * fy, c10 + (c11 - c10) * fy);
        let (s0, s1) = (s00 + (s01 - s00) * fy, s10 + (s11 - s10) * fy);
        let dy0 = c01 - c00;
        let dy1 = c11 - c10;
        (
            c0 + (c1 - c0) * fx,
            [
                (c1 - c0) * self.x.inv_step,
                (dy0 + (dy1 - dy0) * fx) * self.y.inv_step,
                (s0 + (s1 - s0) * fx) * self.z.inv_step,
            ],
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn bilinear_reproduces_a_bilinear_function_with_extrapolation() {
        let f = |x: f32, y: f32| 1.0 + 2.0 * x - 3.0 * y + 0.5 * x * y;
        let (ax, ay) = (
            Axis::new(-1.0, 1.0, 5).unwrap(),
            Axis::new(0.0, 2.0, 3).unwrap(),
        );
        let mut data = [0.0; 15];
        for i in 0..5 {
            for j in 0..3 {
                data[i * 3 + j] = f(ax.node(i), ay.node(j));
            }
        }
        let g = Grid2::new(ax, ay, &data).unwrap();
        for (x, y) in [
            (0.3, 0.7),
            (-1.0, 0.0),
            (1.0, 2.0),
            (1.5, -0.5),
            (-2.0, 3.0),
        ] {
            let (v, [dx, dy]) = g.eval_grad(x, y);
            assert!((v - f(x, y)).abs() < 1e-5, "{x} {y}: {v}");
            assert!((dx - (2.0 + 0.5 * y)).abs() < 1e-4 && (dy - (-3.0 + 0.5 * x)).abs() < 1e-4);
        }
        assert!(Grid2::new(ax, ay, &data[..14]).is_none());
    }

    #[test]
    fn periodic_axis_wraps() {
        let tau = core::f32::consts::TAU;
        let (ax, ay) = (
            Axis::new(0.0, 1.0, 2).unwrap(),
            Axis::new(0.0, 1.0, 2).unwrap(),
        );
        let az = Axis::periodic(0.0, tau, 4).unwrap();
        // Value = z node index pattern 0, 1, 2, 3 then back to 0.
        let mut data = [0.0; 16];
        for (n, v) in data.iter_mut().enumerate() {
            *v = (n % 4) as f32;
        }
        let g = Grid3::new(ax, ay, az, &data).unwrap();
        let q = tau / 4.0;
        assert!((g.eval(0.5, 0.5, 1.5 * q) - 1.5).abs() < 1e-5);
        // Between the last node and the wrapped first one.
        assert!((g.eval(0.5, 0.5, 3.5 * q) - 1.5).abs() < 1e-5);
        assert!((g.eval(0.5, 0.5, 3.5 * q - 2.0 * tau) - 1.5).abs() < 1e-4);
        let (_, grad) = g.eval_grad(0.2, 0.9, 0.5 * q);
        assert!((grad[2] - 1.0 / q).abs() < 1e-4 && grad[0] == 0.0 && grad[1] == 0.0);
    }
}
