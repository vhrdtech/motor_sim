//! Deterministic, `no_std` electric motor simulator for testing motor controllers in software.
//!
//! A [`Motor`] is the plant (a PMSM in dq, solved exactly per step), an average inverter, rotor mechanics and a
//! [`Load`]. The true state is in [`Truth`]; the controller should only read [`Sensors`] (quantised, noisy,
//! offset ADC readings, encoder, halls). [`Faults`] break the sensor and inverter paths, never the physics. The
//! same parameters and seed give the same run on every target.
//!
//! ```
//! use motor_sim::{Drive, Load, Motor, presets};
//!
//! let mut motor = Motor::new(presets::vesc_hobby_outrunner(), 42);
//! motor.set_load(Load::Viscous { b_nms_per_rad: 1e-4 });
//! let dt = 1.0 / 25_000.0; // one PWM period
//! for _ in 0..25_000 {
//!     // A controller would compute this from motor.sensors(); here: 3 V along q using the true angle.
//!     let (s, c) = (motor.truth().theta_e.sin(), motor.truth().theta_e.cos());
//!     motor.step(&Drive::AlphaBeta { v_alpha_v: -3.0 * s, v_beta_v: 3.0 * c }, dt);
//! }
//! assert!(motor.truth().omega_m > 100.0);
//! let _currents = motor.sensors().i_abc_a;
//! motor.visit(&mut |var: &motor_sim::Var, value| { let _ = (var.name, var.unit, value); });
//! ```
#![no_std]
#![deny(missing_docs)]

mod drive;
mod em;
mod faults;
#[cfg(feature = "maps")]
mod grid;
mod mechanics;
mod multirate;
mod nvh;
mod params;
mod plant;
mod power;
pub mod presets;
mod rng;
mod sensors;
mod stack;
mod thermal;
mod visit;

pub use drive::Drive;
pub use em::{AlphaBeta, Dq, Frame, LinearAlphaBeta, LinearDq, Linearised, Machine, Magnet};
#[cfg(feature = "maps")]
pub use em::{Field, MapMachine};
pub use faults::{Faults, InverterFaults, LegFault, SensorFaults};
#[cfg(feature = "maps")]
pub use grid::{Axis, Grid2, Grid3};
pub use mechanics::Load;
pub use nvh::{NvhLevel, NvhParams};
pub use params::{InverterParams, MechParams, MotorParams, PmsmParams, SensorParams};
pub use plant::{Plant, PlantParams};
pub use power::{Align, Carrier};
pub use sensors::Sensors;
pub use thermal::{ThermalLevel, ThermalParams};
pub use visit::{Source, Tier, Value, Var, Visitor};

use crate::stack::{Core, Layers};

/// The true physical state after the last step. Angles are of the rotor d axis from phase a.
#[derive(Clone, Copy, Debug, PartialEq, Default)]
pub struct Truth {
    /// Simulated time, s.
    pub time_s: f64,
    /// d-axis current, A.
    pub i_d: f32,
    /// q-axis current, A.
    pub i_q: f32,
    /// α current, A.
    pub i_alpha: f32,
    /// β current, A.
    pub i_beta: f32,
    /// Phase a current, A.
    pub i_a: f32,
    /// Phase b current, A.
    pub i_b: f32,
    /// Phase c current, A.
    pub i_c: f32,
    /// d voltage applied over the last step (at its mid-step angle), V.
    pub v_d: f32,
    /// q voltage applied over the last step (at its mid-step angle), V.
    pub v_q: f32,
    /// α voltage applied over the last step, V.
    pub v_alpha: f32,
    /// β voltage applied over the last step, V.
    pub v_beta: f32,
    /// Phase a to star point voltage, V (0 with the inverter off).
    pub v_a: f32,
    /// Phase b to star point voltage, V.
    pub v_b: f32,
    /// Phase c to star point voltage, V.
    pub v_c: f32,
    /// Leg duties after inverter faults, 0..=1 (0 with the inverter off).
    pub duty: [f32; 3],
    /// Electrical angle, rad in [0, 2π).
    pub theta_e: f32,
    /// Electrical speed, rad/s.
    pub omega_e: f32,
    /// Mechanical angle, multi-turn, rad.
    pub theta_m: f64,
    /// Mechanical speed, rad/s.
    pub omega_m: f32,
    /// Electromagnetic torque, N·m.
    pub torque_e: f32,
    /// Torque the load puts on the shaft, N·m (positive opposes positive rotation).
    pub torque_load: f32,
    /// Angle of the second inertia ([`Load::Inertia`]), multi-turn, rad.
    pub theta_load: f64,
    /// Speed of the second inertia, rad/s.
    pub omega_load: f32,
    /// DC link voltage, V.
    pub vdc: f32,
    /// DC link current averaged over the last step, A (negative: regenerating).
    pub i_dc: f32,
    /// DC link power averaged over the last step, W.
    pub p_dc: f32,
    /// Charge drawn from the DC link, Ah.
    pub charge_drawn_ah: f64,
    /// Charge returned to the DC link, Ah.
    pub charge_regen_ah: f64,
    /// Energy drawn from the DC link, Wh.
    pub energy_drawn_wh: f64,
    /// Energy returned to the DC link, Wh.
    pub energy_regen_wh: f64,
    /// Energy lost in the winding resistance, J.
    pub copper_loss_j: f64,
    /// Energy lost in the motor's own friction, J.
    pub friction_loss_j: f64,
    /// Energy delivered to the load through the shaft, J.
    pub load_work_j: f64,
}

/// A simulated motor: plant, inverter, mechanics, load, sensors and faults. The linear dq PMSM of
/// [`MotorParams`]; [`Plant`] runs the same layers with any [`Machine`].
pub struct Motor {
    params: MotorParams,
    core: Core,
}

impl Motor {
    /// A motor at rest with zero current, no faults and a free shaft. `seed` fixes noise and random faults.
    pub fn new(params: MotorParams, seed: u64) -> Self {
        let core = Core::new(&layers(&params), seed);
        Motor { params, core }
    }

    /// Advance by `dt_s` seconds (normally one PWM period; any length with a [carrier](Motor::set_carrier)) with
    /// `drive` held, then sample the sensors.
    pub fn step(&mut self, drive: &Drive, dt_s: f32) {
        self.core.step(&layers(&self.params), drive, dt_s);
    }

    /// The true state.
    pub fn truth(&self) -> &Truth {
        &self.core.truth
    }

    /// The sensor readings taken at the end of the last step.
    pub fn sensors(&self) -> &Sensors {
        &self.core.sensors.last
    }

    /// Parameters.
    pub fn params(&self) -> &MotorParams {
        &self.params
    }

    /// Change parameters between steps (a different DC link voltage, a heavier rotor, ...). The currents stay
    /// continuous across a change of inductances.
    pub fn params_mut(&mut self) -> &mut MotorParams {
        self.core.params_changed();
        &mut self.params
    }

    /// The load on the shaft.
    pub fn load(&self) -> &Load {
        &self.core.load
    }

    /// Change the load; it takes effect at the next step. Switching to [`Load::Inertia`] starts the second inertia
    /// where the rotor is, at rest relative to it.
    pub fn set_load(&mut self, load: Load) {
        self.core.set_load(load);
    }

    /// The PWM carrier, if set.
    pub fn carrier(&self) -> Option<&Carrier> {
        self.core.carrier.as_ref()
    }

    /// Set or clear the PWM carrier. Without one (the default) the duties of a step are its average, so a step
    /// should be one PWM period. With one, the duties are compare values held over the step and every substep
    /// gets the volt-seconds the legs really deliver within it, from the carrier phase at the simulated time:
    /// steps shorter (or longer) than the PWM period see the ripple of the average voltage.
    pub fn set_carrier(&mut self, carrier: Option<Carrier>) {
        self.core.carrier = carrier;
    }

    /// Active faults.
    pub fn faults(&self) -> &Faults {
        &self.core.faults
    }

    /// Inject or clear faults; they act from the next step.
    pub fn faults_mut(&mut self) -> &mut Faults {
        &mut self.core.faults
    }

    /// Energy stored in the winding inductances, J.
    pub fn magnetic_energy_j(&self) -> f64 {
        let (p, t) = (&self.params.pmsm, &self.core.truth);
        let (id, iq) = (t.i_d as f64, t.i_q as f64);
        0.75 * (p.ld_h as f64 * id * id + p.lq_h as f64 * iq * iq)
    }

    /// Kinetic energy of the rotor (without a [`Load::Inertia`]), J.
    pub fn kinetic_energy_j(&self) -> f64 {
        self.core.kinetic_energy_j(&self.params.mech)
    }

    /// List every variable with name, unit, rate tier and current value: [`Truth`] first, then [`Sensors`].
    pub fn visit(&self, visitor: &mut impl Visitor) {
        visit::visit(&self.core.truth, &self.core.sensors.last, visitor)
    }
}

/// The layers of a [`Motor`]: its PMSM as the machine, thermal and NVH off.
fn layers(p: &MotorParams) -> Layers<'_, PmsmParams> {
    Layers {
        machine: &p.pmsm,
        mech: &p.mech,
        inverter: &p.inverter,
        sensors: &p.sensors,
        thermal: &ThermalParams::OFF,
        nvh: &NvhParams::OFF,
        max_substep_s: p.max_substep_s,
    }
}

/// `x` wrapped into [0, 2π).
pub(crate) fn wrap_tau(x: f64) -> f64 {
    let r = libm::fmod(x, core::f64::consts::TAU);
    if r < 0.0 {
        r + core::f64::consts::TAU
    } else {
        r
    }
}
