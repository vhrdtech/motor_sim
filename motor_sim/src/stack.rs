//! The layer stack: the state of every layer and the step that runs them in fixed order. [`Motor`](crate::Motor) and
//! [`Plant`](crate::Plant) are views on it that differ only in where the machine comes from.

use crate::drive::Drive;
use crate::em::{self, Frame, Machine};
use crate::faults::Faults;
use crate::mechanics::{self, Load, MechState};
use crate::multirate::SlowClock;
use crate::nvh::NvhParams;
use crate::params::{InverterParams, MechParams, SensorParams};
use crate::power::{Carrier, inverter};
use crate::rng::Rng;
use crate::sensors::SensorModel;
use crate::thermal::ThermalParams;
use crate::{Truth, wrap_tau};

/// Parameters of every layer, borrowed for one step.
pub(crate) struct Layers<'a, M> {
    pub machine: &'a M,
    pub mech: &'a MechParams,
    pub inverter: &'a InverterParams,
    pub sensors: &'a SensorParams,
    pub thermal: &'a ThermalParams,
    pub nvh: &'a NvhParams,
    pub max_substep_s: f32,
}

pub(crate) struct Core {
    pub load: Load,
    pub faults: Faults,
    pub truth: Truth,
    pub sensors: SensorModel,
    pub carrier: Option<Carrier>,
    inverter_rng: Rng,
    /// Rotor speed at the start of the previous substep, rad/s, to predict the mid-step speed.
    omega_m_prev: f32,
    /// Flux linkage minus the magnet flux, in the machine's frame, V·s.
    flux: [f32; 2],
    /// The parameters may have changed: re-derive `flux` from the currents before the next step (so currents stay
    /// continuous) and pick up the slow layers' rates.
    params_stale: bool,
    thermal_clock: SlowClock,
    nvh_clock: SlowClock,
}

impl Core {
    pub fn new<M: Machine>(p: &Layers<M>, seed: u64) -> Self {
        let mut c = Core {
            load: Load::None,
            faults: Faults::NONE,
            truth: Truth {
                vdc: p.inverter.vdc_v,
                ..Truth::default()
            },
            sensors: SensorModel::new(seed),
            carrier: None,
            inverter_rng: Rng::new(seed, 3),
            omega_m_prev: 0.0,
            flux: p.machine.flux([0.0; 2], 0.0),
            params_stale: false,
            thermal_clock: SlowClock::new(p.thermal.every),
            nvh_clock: SlowClock::new(p.nvh.every),
        };
        c.sample_sensors(p.sensors);
        c
    }

    /// Flux relative to the magnets in the machine's frame, V·s.
    pub fn flux(&self) -> [f32; 2] {
        self.flux
    }

    /// The parameters may change before the next step.
    pub fn params_changed(&mut self) {
        self.params_stale = true;
    }

    pub fn set_load(&mut self, load: Load) {
        if matches!(load, Load::Inertia { .. }) && !matches!(self.load, Load::Inertia { .. }) {
            self.truth.theta_load = self.truth.theta_m;
            self.truth.omega_load = self.truth.omega_m;
        }
        self.load = load;
    }

    fn sample_sensors(&mut self, p: &SensorParams) {
        self.sensors.sample(p, &self.faults.sensor, &self.truth);
    }

    /// Advance by `dt_s` with `drive` held, then sample the sensors.
    pub fn step<M: Machine>(&mut self, p: &Layers<M>, drive: &Drive, dt_s: f32) {
        let m = p.machine;
        let vdc = p.inverter.vdc_v;
        let pp = m.pole_pairs() as f32;
        if self.params_stale {
            let t = &self.truth;
            let i = <M::Frame as Frame>::from_dq([t.i_d, t.i_q], t.theta_e);
            self.flux = m.flux(i, t.theta_e);
            self.thermal_clock.set_every(p.thermal.every);
            self.nvh_clock.set_every(p.nvh.every);
            self.params_stale = false;
        }

        // Layer 2: duties after faults, then the volts each substep sees.
        let duty = inverter::duties(drive, vdc)
            .map(|d| inverter::apply_faults(d, &self.faults.inverter, &mut self.inverter_rng));
        let carrier = self.carrier.filter(|_| duty.is_some());
        let step_duty = match (duty, carrier) {
            (Some(d), Some(c)) => Some(c.average(d, self.truth.time_s, dt_s)),
            _ => duty,
        };
        let v_ph = step_duty.map_or([0.0; 3], |d| inverter::phase_volts(d, vdc));
        let v_ab_step = inverter::clarke(v_ph);

        let n = libm::ceilf(dt_s / p.max_substep_s).max(1.0) as u32;
        let h = dt_s / n as f32;
        let t = &mut self.truth;
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
            let v_ab = match (duty, carrier) {
                (Some(d), Some(c)) => {
                    inverter::clarke(inverter::phase_volts(c.average(d, t.time_s, h), vdc))
                }
                _ => v_ab_step,
            };
            let theta_e = wrap_tau(pp as f64 * mech.theta_m) as f32;
            // Freeze ω for the electrical step at the predicted mid-step speed, so the electrical and mechanical
            // sides exchange the same energy to second order.
            let omega_m = match self.load {
                Load::Speed { .. } | Load::Locked => mech.omega_m,
                _ => 1.5 * mech.omega_m - 0.5 * self.omega_m_prev,
            };
            self.omega_m_prev = mech.omega_m;
            let omega_e = pp * omega_m;
            let theta_e1 = theta_e + omega_e * h;

            // Layer 3: flux step, current from the map, torque from the co-energy.
            let x0 = self.flux;
            let (x1, i0, i1) = if duty.is_some() {
                let lin = m.linearise(x0, theta_e);
                let x1 = em::advance(m, x0, &lin, [v_ab.0, v_ab.1], theta_e, omega_e, h);
                (x1, lin.i, m.current(x1, theta_e1))
            } else {
                // Phases open: the currents drop to zero at once.
                (m.flux([0.0; 2], theta_e1), m.current(x0, theta_e), [0.0; 2])
            };
            let ab0 = <M::Frame as Frame>::to_ab(i0, theta_e);
            let ab1 = <M::Frame as Frame>::to_ab(i1, theta_e1);
            let e = 1.5 * h * (v_ab.0 * (ab0[0] + ab1[0]) + v_ab.1 * (ab0[1] + ab1[1])) / 2.0;
            let cu = 0.75
                * m.rs_ohm()
                * h
                * (i0[0] * i0[0] + i0[1] * i0[1] + i1[0] * i1[0] + i1[1] * i1[1]);
            let te = 0.5 * (em::torque(m, x0, i0, theta_e) + em::torque(m, x1, i1, theta_e1));
            self.flux = x1;

            // Layer 4: mechanics.
            let ms = mechanics::step(p.mech, &self.load, &mut mech, te, t.time_s, h);

            // Layers 5 and 6 at their own rates.
            if let Some(dt) = self.thermal_clock.tick(h) {
                p.thermal.step(dt);
            }
            if let Some(dt) = self.nvh_clock.tick(h) {
                p.nvh.step(dt);
            }

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
        let i = m.current(self.flux, t.theta_e);
        [t.i_d, t.i_q] = <M::Frame as Frame>::to_dq(i, t.theta_e);
        [t.i_alpha, t.i_beta] = <M::Frame as Frame>::to_ab(i, t.theta_e);
        [t.i_a, t.i_b, t.i_c] = inverter::inv_clarke(t.i_alpha, t.i_beta);
        (t.v_alpha, t.v_beta) = v_ab_step;
        (t.v_d, t.v_q) = em::park(
            v_ab_step.0,
            v_ab_step.1,
            theta_e_start + 0.5 * omega_e_start * dt_s,
        );
        [t.v_a, t.v_b, t.v_c] = v_ph;
        t.duty = step_duty.unwrap_or([0.0; 3]);
        t.torque_e = em::torque(m, self.flux, i, t.theta_e);
        t.vdc = vdc;
        t.p_dc = if dt_s > 0.0 {
            (e_in / dt_s as f64) as f32
        } else {
            0.0
        };
        t.i_dc = t.p_dc / vdc;
        self.sample_sensors(p.sensors);
    }

    /// Energy in the winding field, J.
    pub fn magnetic_energy_j<M: Machine>(&self, m: &M) -> f64 {
        1.5 * m.field_energy(self.flux, self.truth.theta_e) as f64
    }

    /// Kinetic energy of the rotor (without a [`Load::Inertia`]), J.
    pub fn kinetic_energy_j(&self, mech: &MechParams) -> f64 {
        let w = self.truth.omega_m as f64;
        0.5 * mech.inertia_kgm2 as f64 * w * w
    }
}
