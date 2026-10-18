//! The layer stack: the state of every layer and the tick that runs them in a fixed order.
//!
//! Layers talk only through plain-data values (`RotorMotion`, `WindingVolts`, `EmStep`, `MechStep`), one exchange
//! per base substep, so any layer could later run elsewhere (another process or PC, a HIL rig, an FPGA) with a
//! transport carrying those values. The whole state is `Copy`: a snapshot is a copy.

use crate::Truth;
use crate::drive::Drive;
use crate::em::{Electromagnetics, Frame, Machine, park};
use crate::faults::Faults;
use crate::mechanics::{Load, Mechanics};
use crate::multirate::{SimClock, SlowClock};
use crate::params::PlantParams;
use crate::power::{PowerStage, inverter};
use crate::sensors::SensorModel;

#[derive(Clone, Copy)]
pub(crate) struct Core {
    pub clock: SimClock,
    pub load: Load,
    pub faults: Faults,
    pub truth: Truth,
    pub sensors: SensorModel,
    pub power: PowerStage,
    pub em: Electromagnetics,
    mech: Mechanics,
    thermal_clock: SlowClock,
    nvh_clock: SlowClock,
    /// The parameters may have changed: re-sync the layers before the next step.
    params_stale: bool,
}

impl Core {
    pub fn new<M: Machine>(p: &PlantParams<M>, seed: u64) -> Self {
        let mut c = Core {
            clock: SimClock::default(),
            load: Load::None,
            faults: Faults::NONE,
            truth: Truth {
                vdc: p.inverter.vdc_v,
                ..Truth::default()
            },
            sensors: SensorModel::new(seed),
            power: PowerStage::new(seed),
            em: Electromagnetics::new(&p.machine),
            mech: Mechanics::new(),
            thermal_clock: SlowClock::new(p.thermal.every),
            nvh_clock: SlowClock::new(p.nvh.every),
            params_stale: false,
        };
        c.sensors.sample(&p.sensors, &c.faults.sensor, &c.truth);
        c
    }

    /// The parameters may change before the next step.
    pub fn params_changed(&mut self) {
        self.params_stale = true;
        self.em.mark_stale();
    }

    pub fn set_load(&mut self, load: Load) {
        if matches!(load, Load::Inertia { .. }) && !matches!(self.load, Load::Inertia { .. }) {
            self.mech.state.theta_load = self.mech.state.theta_m;
            self.mech.state.omega_load = self.mech.state.omega_m;
            self.truth.theta_load = self.truth.theta_m;
            self.truth.omega_load = self.truth.omega_m;
        }
        self.load = load;
    }

    /// Advance by `dt_s` with `drive` held, then sample the sensors.
    pub fn step<M: Machine>(&mut self, p: &PlantParams<M>, drive: &Drive, dt_s: f32) {
        let m = &p.machine;
        let vdc = p.inverter.vdc_v;
        if self.params_stale {
            let t = &self.truth;
            self.em.sync(m, [t.i_d, t.i_q], t.theta_e);
            self.thermal_clock.set_every(p.thermal.every);
            self.nvh_clock.set_every(p.nvh.every);
            self.params_stale = false;
        }

        // Layer 1 → 2: the command, held over the step.
        self.power.command(drive, &self.faults.inverter, vdc);
        let t_start = self.clock.time_s;
        let theta_e_start = self.truth.theta_e;
        let omega_e_start = m.pole_pairs() as f32 * self.truth.omega_m;

        let n = libm::ceilf(dt_s / p.max_substep_s).max(1.0) as u32;
        let h = dt_s / n as f32;
        let t = &mut self.truth;
        let mut e_in = 0.0f64;
        for _ in 0..n {
            // One base tick: mechanics → rotor motion; power stage → winding volts; electromagnetics → torque;
            // mechanics; slow layers when due.
            let rotor = self.mech.rotor(m.pole_pairs(), &self.load);
            let volts = self.power.step(vdc, self.clock.time_s, h);
            let em = self.em.step(m, &volts, &rotor, h);
            let ms = self
                .mech
                .step(&p.mech, &self.load, em.torque_nm, self.clock.time_s, h);
            if let Some(dt) = self.thermal_clock.tick(h) {
                p.thermal.step(dt);
            }
            if let Some(dt) = self.nvh_clock.tick(h) {
                p.nvh.step(dt);
            }
            self.clock.tick(h);

            let e = em.energy_in_j;
            e_in += e as f64;
            t.copper_loss_j += em.copper_j as f64;
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

        let ms = &self.mech.state;
        let pp = m.pole_pairs() as f32;
        t.time_s = self.clock.time_s;
        t.theta_m = ms.theta_m;
        t.omega_m = ms.omega_m;
        t.theta_load = ms.theta_load;
        t.omega_load = ms.omega_load;
        t.theta_e = crate::wrap_tau(pp as f64 * ms.theta_m) as f32;
        t.omega_e = pp * ms.omega_m;
        let (i, te) = self.em.outputs(m, t.theta_e);
        [t.i_d, t.i_q] = <M::Frame as Frame>::to_dq(i, t.theta_e);
        [t.i_alpha, t.i_beta] = <M::Frame as Frame>::to_ab(i, t.theta_e);
        [t.i_a, t.i_b, t.i_c] = inverter::inv_clarke(t.i_alpha, t.i_beta);
        // Step averages of the inverter outputs.
        let (duty, v_ph) = self.power.step_average(vdc, t_start, dt_s);
        (t.v_alpha, t.v_beta) = inverter::clarke(v_ph);
        (t.v_d, t.v_q) = park(
            t.v_alpha,
            t.v_beta,
            theta_e_start + 0.5 * omega_e_start * dt_s,
        );
        [t.v_a, t.v_b, t.v_c] = v_ph;
        t.duty = duty.unwrap_or([0.0; 3]);
        t.torque_e = te;
        t.vdc = vdc;
        t.p_dc = if dt_s > 0.0 {
            (e_in / dt_s as f64) as f32
        } else {
            0.0
        };
        t.i_dc = t.p_dc / vdc;
        self.sensors.sample(&p.sensors, &self.faults.sensor, t);
    }

    /// Kinetic energy of the rotor (without a [`Load::Inertia`]), J.
    pub fn kinetic_energy_j(&self, inertia_kgm2: f32) -> f64 {
        let w = self.truth.omega_m as f64;
        0.5 * inertia_kgm2 as f64 * w * w
    }
}
