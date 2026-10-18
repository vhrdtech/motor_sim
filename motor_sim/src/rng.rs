//! Seeded pseudo-random numbers: xoshiro128** seeded through splitmix64, identical on every target.

/// Small, fast, seeded PRNG (xoshiro128**). Not cryptographic.
#[derive(Clone, Copy, Debug)]
pub(crate) struct Rng {
    s: [u32; 4],
    spare_normal: Option<f32>,
}

impl Rng {
    /// A generator for `seed`, with `stream` picking an independent sequence for the same seed.
    pub(crate) fn new(seed: u64, stream: u64) -> Self {
        let mut x = seed ^ stream.wrapping_mul(0xD1B5_4A32_D192_ED03);
        let mut next = || {
            x = x.wrapping_add(0x9E37_79B9_7F4A_7C15);
            let mut z = x;
            z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
            z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
            z ^ (z >> 31)
        };
        let a = next();
        let b = next();
        let mut s = [a as u32, (a >> 32) as u32, b as u32, (b >> 32) as u32];
        if s == [0; 4] {
            s[0] = 1;
        }
        Rng {
            s,
            spare_normal: None,
        }
    }

    pub(crate) fn next_u32(&mut self) -> u32 {
        let result = self.s[1].wrapping_mul(5).rotate_left(7).wrapping_mul(9);
        let t = self.s[1] << 9;
        self.s[2] ^= self.s[0];
        self.s[3] ^= self.s[1];
        self.s[1] ^= self.s[2];
        self.s[0] ^= self.s[3];
        self.s[2] ^= t;
        self.s[3] = self.s[3].rotate_left(11);
        result
    }

    /// Uniform in [0, 1).
    pub(crate) fn uniform(&mut self) -> f32 {
        (self.next_u32() >> 8) as f32 * (1.0 / 16_777_216.0)
    }

    /// True with probability `p` (one draw either way, so the sequence does not depend on `p`).
    pub(crate) fn chance(&mut self, p: f32) -> bool {
        self.uniform() < p
    }

    /// Standard normal sample (Box-Muller).
    pub(crate) fn normal(&mut self) -> f32 {
        if let Some(z) = self.spare_normal.take() {
            return z;
        }
        let u1 = 1.0 - self.uniform(); // (0, 1]
        let u2 = self.uniform();
        let r = libm::sqrtf(-2.0 * libm::logf(u1));
        let (s, c) = libm::sincosf(core::f32::consts::TAU * u2);
        self.spare_normal = Some(r * s);
        r * c
    }
}
