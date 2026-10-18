//! PWM carrier: which fraction of any interval a leg is high, from the carrier phase and its compare value.

/// Carrier shape.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum Align {
    /// Up-down counter (triangle from 0 at the period start to 1 at its middle and back): the leg is high while
    /// the carrier is below the duty, i.e. around the period boundaries, and all legs are low at the middle (where
    /// low-side shunts are sampled).
    #[default]
    Center,
    /// Up counter (sawtooth from 0 to 1): the leg is high from the period start for `duty·T`.
    Edge,
}

/// A PWM carrier starting its first period at simulated time 0. With a carrier set
/// ([`Motor::set_carrier`](crate::Motor::set_carrier)), duties are compare values held over the step, and each
/// substep gets the exact volt-seconds the legs deliver within it: steps of any length, independent of the PWM
/// period, average to the same per-period volt-seconds.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Carrier {
    /// PWM period, s.
    pub period_s: f32,
    /// Carrier shape.
    pub align: Align,
}

impl Carrier {
    /// A centre-aligned carrier at `freq_hz`.
    pub fn center(freq_hz: f32) -> Self {
        Carrier {
            period_s: 1.0 / freq_hz,
            align: Align::Center,
        }
    }

    /// Fraction (0..=1) of the interval [`t0_s`, `t0_s + h_s`] the legs with compare values `duty` are high.
    pub fn average(&self, duty: [f32; 3], t0_s: f64, h_s: f32) -> [f32; 3] {
        if h_s <= 0.0 {
            return duty;
        }
        let period = self.period_s as f64;
        // Phase of the interval start within its period, s; the interval may span several periods.
        let p0 = (t0_s - period * libm::floor(t0_s / period)) as f32;
        let p1 = p0 + h_s;
        duty.map(|d| (self.high_until(d, p1) - self.high_until(d, p0)) / h_s)
    }

    /// High time from the start of a period to `t` (s, ≥ 0, any number of periods).
    fn high_until(&self, d: f32, t: f32) -> f32 {
        let tp = self.period_s;
        let k = libm::floorf(t / tp);
        let u = t - k * tp;
        let within = match self.align {
            Align::Edge => u.min(d * tp),
            Align::Center => {
                let half = 0.5 * tp;
                if u <= half {
                    u.min(d * half)
                } else {
                    d * half + (u - (tp - d * half)).max(0.0)
                }
            }
        };
        k * d * tp + within
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn full_period_is_the_duty_and_quarters_add_up() {
        for align in [Align::Center, Align::Edge] {
            let c = Carrier {
                period_s: 40e-6,
                align,
            };
            let d = [0.0, 0.3, 0.9];
            for t0 in [0.0, 13e-6, 1.0 + 7e-6] {
                let full = c.average(d, t0, 40e-6);
                let mut sum = [0.0f32; 3];
                for q in 0..4 {
                    let a = c.average(d, t0 + q as f64 * 10e-6, 10e-6);
                    for k in 0..3 {
                        sum[k] += a[k] / 4.0;
                    }
                }
                for k in 0..3 {
                    assert!((full[k] - d[k]).abs() < 2e-4, "{align:?} {t0} {full:?}");
                    assert!((sum[k] - d[k]).abs() < 2e-4, "{align:?} {t0} {sum:?}");
                }
            }
        }
    }

    #[test]
    fn center_aligned_is_high_at_the_edges_low_in_the_middle() {
        let c = Carrier::center(25e3);
        let d = [0.5; 3];
        // First quarter: high while the carrier rises from 0 to 0.5, i.e. the whole quarter.
        assert!((c.average(d, 0.0, 10e-6)[0] - 1.0).abs() < 1e-5);
        assert!(c.average(d, 10e-6, 20e-6)[0].abs() < 1e-5);
        assert!((c.average(d, 30e-6, 10e-6)[0] - 1.0).abs() < 1e-5);
        // Over many periods.
        assert!((c.average([0.25; 3], 3.0, 1e-3)[0] - 0.25).abs() < 1e-4);
    }
}
