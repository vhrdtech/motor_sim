//! Multirate schedule: slow layers run every N base substeps (integer ratios), in a fixed order after the fast
//! layers, so a run is bit-identical for the same seed on every target.
//!
//! Order within a base substep: power stage → electromagnetics → mechanics → slow layers whose clock is due
//! (thermal, then NVH). Sensors sample once at the end of [`Motor::step`](crate::Motor::step).

/// Clock of a slow layer: due once every `every` base substeps, with the time since it was last due.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct SlowClock {
    every: u32,
    count: u32,
    elapsed_s: f64,
}

impl SlowClock {
    pub(crate) const fn new(every: u32) -> Self {
        SlowClock {
            every: if every == 0 { 1 } else { every },
            count: 0,
            elapsed_s: 0.0,
        }
    }

    /// Change the ratio; the clock restarts only if it differs.
    pub(crate) fn set_every(&mut self, every: u32) {
        if SlowClock::new(every).every != self.every {
            *self = SlowClock::new(every);
        }
    }

    /// Count one base substep of `h_s` seconds; returns the slow step (s) when the layer is due.
    #[inline]
    pub(crate) fn tick(&mut self, h_s: f32) -> Option<f64> {
        self.elapsed_s += h_s as f64;
        self.count += 1;
        if self.count < self.every {
            return None;
        }
        let dt = self.elapsed_s;
        self.count = 0;
        self.elapsed_s = 0.0;
        Some(dt)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn due_every_n_with_the_accumulated_step() {
        let mut c = SlowClock::new(4);
        let due: [Option<f64>; 8] = core::array::from_fn(|k| c.tick(if k < 4 { 1.0 } else { 0.5 }));
        assert_eq!(
            due,
            [None, None, None, Some(4.0), None, None, None, Some(2.0)]
        );
        let mut every_step = SlowClock::new(0);
        assert_eq!(every_step.tick(0.25), Some(0.25));
    }
}
