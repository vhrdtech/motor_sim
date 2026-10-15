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

mod electrical;
mod faults;
mod inverter;
mod mechanics;
mod params;
pub mod presets;
mod rng;
mod sensors;
mod visit;

pub use faults::{Faults, InverterFaults, LegFault, SensorFaults};
pub use inverter::Drive;
pub use mechanics::Load;
pub use params::{InverterParams, MechParams, MotorParams, PmsmParams, SensorParams};
pub use sensors::Sensors;
pub use visit::{Source, Tier, Value, Var, Visitor};

use mechanics::MechState;
use rng::Rng;
use sensors::SensorModel;

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

/// A simulated motor: plant, inverter, mechanics, load, sensors and faults.
pub struct Motor {
    params: MotorParams,
    load: Load,
    faults: Faults,
    truth: Truth,
    sensors: SensorModel,
    inverter_rng: Rng,
}

impl Motor {
    /// A motor at rest with zero current, no faults and a free shaft. `seed` fixes noise and random faults.
    pub fn new(params: MotorParams, seed: u64) -> Self {
        let mut m = Motor {
            params,
            load: Load::None,
            faults: Faults::NONE,
            truth: Truth {
                vdc: params.inverter.vdc_v,
                ..Truth::default()
            },
            sensors: SensorModel::new(seed),
            inverter_rng: Rng::new(seed, 3),
        };
        m.sample_sensors();
        m
    }

    /// Advance by `dt_s` seconds (normally one PWM period) with `drive` held, then sample the sensors.
    pub fn step(&mut self, drive: &Drive, dt_s: f32) {
        let p = self.params;
        let vdc = p.inverter.vdc_v;
        let duty = inverter::duties(drive, vdc)
            .map(|d| inverter::apply_faults(d, &self.faults.inverter, &mut self.inverter_rng));
        let v_ph = duty.map_or([0.0; 3], |d| inverter::phase_volts(d, vdc));
        let v_ab = inverter::clarke(v_ph);

        let n = libm::ceilf(dt_s / p.max_substep_s).max(1.0) as u32;
        let h = dt_s / n as f32;
        let t = &mut self.truth;
        let pp = p.pmsm.pole_pairs as f32;
        let theta_e_start = t.theta_e;
        let omega_e_start = pp * t.omega_m;
        let mut mech = MechState {
            theta_m: t.theta_m,
            omega_m: t.omega_m,
            theta_load: t.theta_load,
            omega_load: t.omega_load,
        };
        let mut e_in = 0.0f64;
        for _ in 0..n {
            let theta_e = wrap_tau(pp as f64 * mech.theta_m) as f32;
            let omega_e = pp * mech.omega_m;
            let i0 = (t.i_d, t.i_q);
            let i1 = if duty.is_some() {
                electrical::step_exact(&p.pmsm, i0, v_ab, theta_e, omega_e, h)
            } else {
                (0.0, 0.0)
            };
            let ab0 = electrical::inv_park(i0.0, i0.1, theta_e);
            let ab1 = electrical::inv_park(i1.0, i1.1, theta_e + omega_e * h);
            let e = 1.5 * h * (v_ab.0 * (ab0.0 + ab1.0) + v_ab.1 * (ab0.1 + ab1.1)) / 2.0;
            let cu =
                0.75 * p.pmsm.rs_ohm * h * (i0.0 * i0.0 + i0.1 * i0.1 + i1.0 * i1.0 + i1.1 * i1.1);
            let te = 0.5
                * (electrical::torque(&p.pmsm, i0.0, i0.1)
                    + electrical::torque(&p.pmsm, i1.0, i1.1));
            let ms = mechanics::step(&p.mech, &self.load, &mut mech, te, t.time_s, h);

            t.i_d = i1.0;
            t.i_q = i1.1;
            t.time_s += h as f64;
            e_in += e as f64;
            t.copper_loss_j += cu as f64;
            t.friction_loss_j += ms.friction_loss_j;
            t.load_work_j += ms.load_work_j;
            t.torque_load = ms.torque_load_nm;
            let wh = e as f64 / 3600.0;
            let ah = wh / vdc as f64;
            if e >= 0.0 {
                t.energy_drawn_wh += wh;
                t.charge_drawn_ah += ah;
            } else {
                t.energy_regen_wh -= wh;
                t.charge_regen_ah -= ah;
            }
        }

        t.theta_m = mech.theta_m;
        t.omega_m = mech.omega_m;
        t.theta_load = mech.theta_load;
        t.omega_load = mech.omega_load;
        t.theta_e = wrap_tau(pp as f64 * mech.theta_m) as f32;
        t.omega_e = pp * mech.omega_m;
        let (ia, ib) = electrical::inv_park(t.i_d, t.i_q, t.theta_e);
        t.i_alpha = ia;
        t.i_beta = ib;
        [t.i_a, t.i_b, t.i_c] = inverter::inv_clarke(ia, ib);
        (t.v_alpha, t.v_beta) = v_ab;
        (t.v_d, t.v_q) =
            electrical::park(v_ab.0, v_ab.1, theta_e_start + 0.5 * omega_e_start * dt_s);
        [t.v_a, t.v_b, t.v_c] = v_ph;
        t.duty = duty.unwrap_or([0.0; 3]);
        t.torque_e = electrical::torque(&p.pmsm, t.i_d, t.i_q);
        t.vdc = vdc;
        t.p_dc = if dt_s > 0.0 {
            (e_in / dt_s as f64) as f32
        } else {
            0.0
        };
        t.i_dc = t.p_dc / vdc;
        self.sample_sensors();
    }

    fn sample_sensors(&mut self) {
        self.sensors
            .sample(&self.params.sensors, &self.faults.sensor, &self.truth);
    }

    /// The true state.
    pub fn truth(&self) -> &Truth {
        &self.truth
    }

    /// The sensor readings taken at the end of the last step.
    pub fn sensors(&self) -> &Sensors {
        &self.sensors.last
    }

    /// Parameters.
    pub fn params(&self) -> &MotorParams {
        &self.params
    }

    /// Change parameters between steps (a different DC link voltage, a heavier rotor, ...).
    pub fn params_mut(&mut self) -> &mut MotorParams {
        &mut self.params
    }

    /// The load on the shaft.
    pub fn load(&self) -> &Load {
        &self.load
    }

    /// Change the load; it takes effect at the next step. Switching to [`Load::Inertia`] starts the second inertia
    /// where the rotor is, at rest relative to it.
    pub fn set_load(&mut self, load: Load) {
        if matches!(load, Load::Inertia { .. }) && !matches!(self.load, Load::Inertia { .. }) {
            self.truth.theta_load = self.truth.theta_m;
            self.truth.omega_load = self.truth.omega_m;
        }
        self.load = load;
    }

    /// Active faults.
    pub fn faults(&self) -> &Faults {
        &self.faults
    }

    /// Inject or clear faults; they act from the next step.
    pub fn faults_mut(&mut self) -> &mut Faults {
        &mut self.faults
    }

    /// Energy stored in the winding inductances, J.
    pub fn magnetic_energy_j(&self) -> f64 {
        let (p, t) = (&self.params.pmsm, &self.truth);
        let (id, iq) = (t.i_d as f64, t.i_q as f64);
        0.75 * (p.ld_h as f64 * id * id + p.lq_h as f64 * iq * iq)
    }

    /// Kinetic energy of the rotor (without a [`Load::Inertia`]), J.
    pub fn kinetic_energy_j(&self) -> f64 {
        let w = self.truth.omega_m as f64;
        0.5 * self.params.mech.inertia_kgm2 as f64 * w * w
    }

    /// List every variable with name, unit, rate tier and current value: [`Truth`] first, then [`Sensors`].
    pub fn visit(&self, visitor: &mut impl Visitor) {
        visit::visit(&self.truth, &self.sensors.last, visitor)
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
