//! What the controller measures: the true state seen through ADCs, an encoder and hall sensors, plus faults.

use crate::Truth;
use crate::faults::SensorFaults;
use crate::params::SensorParams;
use crate::rng::Rng;

/// One sample of every sensor, taken at the end of a [`Motor::step`](crate::Motor::step). Values are already
/// quantised to the ADC step and converted back to SI units.
#[derive(Clone, Copy, Debug, PartialEq, Default)]
pub struct Sensors {
    /// Phase currents a, b, c, A (each with its own offset, noise and quantisation).
    pub i_abc_a: [f32; 3],
    /// DC link voltage, V.
    pub vdc_v: f32,
    /// Encoder position within one mechanical revolution, counts in `0..encoder_cpr` (0 without an encoder).
    pub encoder_count: u32,
    /// Hall lines, bit 0 = A, 1 = B, 2 = C. Forward rotation reads 5, 1, 3, 2, 6, 4.
    pub hall: u8,
}

pub(crate) struct SensorModel {
    noise: Rng,
    faults: Rng,
    encoder_lost: u32,
    pub last: Sensors,
}

/// Hall pattern for each 60° electrical sector starting at the d axis.
const HALL_SECTORS: [u8; 6] = [5, 1, 3, 2, 6, 4];

impl SensorModel {
    pub(crate) fn new(seed: u64) -> Self {
        SensorModel {
            noise: Rng::new(seed, 1),
            faults: Rng::new(seed, 2),
            encoder_lost: 0,
            last: Sensors::default(),
        }
    }

    pub(crate) fn sample(&mut self, p: &SensorParams, f: &SensorFaults, t: &Truth) -> Sensors {
        let mut out = self.last;

        let lsb_i = 2.0 * p.current_full_scale_a / pow2(p.current_adc_bits);
        let half = pow2(p.current_adc_bits) / 2.0;
        let i_true = [t.i_a, t.i_b, t.i_c];
        for k in 0..3 {
            let noise = self.noise.normal() * p.current_noise_a;
            let glitch = self.faults.chance(f.current_glitch_probability);
            let glitch_code = self.faults.uniform() * 2.0 * half - half;
            let raw = f.current_gain[k]
                * (i_true[k] + p.current_offset_a[k] + f.current_offset_a[k])
                + noise;
            let code = if glitch {
                libm::floorf(glitch_code)
            } else {
                libm::roundf(raw / lsb_i).max(-half).min(half - 1.0)
            };
            if !f.current_stuck[k] {
                out.i_abc_a[k] = code * lsb_i;
            }
        }

        let lsb_v = p.vdc_full_scale_v / pow2(p.vdc_adc_bits);
        let raw_v = t.vdc + f.vdc_offset_v + self.noise.normal() * p.vdc_noise_v;
        out.vdc_v = libm::roundf(raw_v / lsb_v)
            .max(0.0)
            .min(pow2(p.vdc_adc_bits) - 1.0)
            * lsb_v;

        let missed = self.faults.chance(f.encoder_missed_count_probability);
        out.encoder_count = if p.encoder_cpr == 0 {
            0
        } else {
            self.encoder_lost = self.encoder_lost.wrapping_add(missed as u32);
            let cpr = p.encoder_cpr as i64;
            let counts = libm::floor(
                (t.theta_m + p.encoder_offset_rad as f64) * cpr as f64 / core::f64::consts::TAU,
            ) as i64;
            (counts - (self.encoder_lost as i64 % cpr)).rem_euclid(cpr) as u32
        };

        let flip = self.faults.chance(f.hall_glitch_probability);
        let bit = self.faults.next_u32() % 3;
        out.hall = match f.hall_stuck {
            Some(h) => h & 0b111,
            None => {
                let a = crate::wrap_tau(t.theta_e as f64 + p.hall_offset_rad as f64);
                let sector = ((a / (core::f64::consts::PI / 3.0)) as usize).min(5);
                HALL_SECTORS[sector] ^ if flip { 1 << bit } else { 0 }
            }
        };

        self.last = out;
        out
    }
}

fn pow2(bits: u8) -> f32 {
    libm::ldexpf(1.0, bits as i32)
}
