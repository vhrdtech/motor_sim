//! Layer 2, power stage: the inverter turning commands into winding voltages. Today: average legs (MS-1) and the
//! PWM carrier, which gives each leg's exact volt-seconds over any interval so the step need not be the PWM period.
//! DC link, dead time, diodes and switching segments come with MS-4, MS-5 and MS-6.

mod carrier;
pub(crate) mod inverter;

pub use carrier::{Align, Carrier};

use crate::drive::Drive;
use crate::faults::InverterFaults;
use crate::rng::Rng;

/// What the power stage hands the electromagnetics for one base substep (plain data: the interface a transport
/// would carry when the layers run apart).
#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct WindingVolts {
    /// αβ voltage held over the substep, V.
    pub v_ab: [f32; 2],
    /// False with all switches off: the phases are open.
    pub on: bool,
}

/// State of the power stage.
#[derive(Clone, Copy)]
pub(crate) struct PowerStage {
    rng: Rng,
    pub carrier: Option<Carrier>,
    /// Leg duties (compare values with a carrier) after faults, held over the step; `None`: switches off.
    duty: Option<[f32; 3]>,
    /// Phase voltages and winding volts of `duty` held (without a carrier every substep gets these).
    held_v_ph: [f32; 3],
    held: WindingVolts,
}

impl PowerStage {
    pub fn new(seed: u64) -> Self {
        PowerStage {
            rng: Rng::new(seed, 3),
            carrier: None,
            duty: None,
            held_v_ph: [0.0; 3],
            held: WindingVolts {
                v_ab: [0.0; 2],
                on: false,
            },
        }
    }

    /// Latch the controller's command for the next step: duties after faults (one random draw per leg).
    pub fn command(&mut self, drive: &Drive, faults: &InverterFaults, vdc_v: f32) {
        self.duty = inverter::duties(drive, vdc_v)
            .map(|d| inverter::apply_faults(d, faults, &mut self.rng));
        self.held_v_ph = Self::phase_volts(self.duty, vdc_v);
        let (a, b) = inverter::clarke(self.held_v_ph);
        self.held = WindingVolts {
            v_ab: [a, b],
            on: self.duty.is_some(),
        };
    }

    /// Average leg duties over [t, t + h]: the latched duties, or their exact high time under the carrier.
    pub fn average_duty(&self, t_s: f64, h_s: f32) -> Option<[f32; 3]> {
        match (self.duty, self.carrier) {
            (Some(d), Some(c)) => Some(c.average(d, t_s, h_s)),
            (d, _) => d,
        }
    }

    /// Phase-to-neutral voltages from average duties, V.
    pub fn phase_volts(duty: Option<[f32; 3]>, vdc_v: f32) -> [f32; 3] {
        duty.map_or([0.0; 3], |d| inverter::phase_volts(d, vdc_v))
    }

    /// Average duties and phase voltages over a whole step [t, t + h].
    pub fn step_average(&self, vdc_v: f32, t_s: f64, h_s: f32) -> (Option<[f32; 3]>, [f32; 3]) {
        match self.carrier {
            None => (self.duty, self.held_v_ph),
            Some(_) => {
                let d = self.average_duty(t_s, h_s);
                (d, Self::phase_volts(d, vdc_v))
            }
        }
    }

    /// The winding voltage for the substep [t, t + h].
    #[inline]
    pub fn step(&self, vdc_v: f32, t_s: f64, h_s: f32) -> WindingVolts {
        if self.carrier.is_none() {
            return self.held;
        }
        let d = self.average_duty(t_s, h_s);
        let (a, b) = inverter::clarke(Self::phase_volts(d, vdc_v));
        WindingVolts {
            v_ab: [a, b],
            on: d.is_some(),
        }
    }
}
