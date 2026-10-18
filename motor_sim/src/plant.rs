//! [`Plant`]: the layered core with any [`Machine`], for what [`Motor`](crate::Motor) (a linear dq PMSM from
//! [`MotorParams`]) can't express: a full inductance matrix, flux maps, the stationary frame, layer levels.

use crate::Truth;
use crate::drive::Drive;
use crate::em::Machine;
use crate::faults::Faults;
use crate::mechanics::Load;
use crate::nvh::NvhParams;
use crate::params::{InverterParams, MechParams, MotorParams, SensorParams};
use crate::power::Carrier;
use crate::sensors::Sensors;
use crate::stack::{Core, Layers};
use crate::thermal::ThermalParams;
use crate::visit::{self, Visitor};

/// Everything a [`Plant`] needs: the machine and the parameters of the other layers.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PlantParams<M> {
    /// Electromagnetics.
    pub machine: M,
    /// Mechanical parameters.
    pub mech: MechParams,
    /// Inverter.
    pub inverter: InverterParams,
    /// Sensors.
    pub sensors: SensorParams,
    /// Thermal layer.
    pub thermal: ThermalParams,
    /// Vibration and sound layer.
    pub nvh: NvhParams,
    /// Longest base substep, s (see [`MotorParams::max_substep_s`]).
    pub max_substep_s: f32,
}

impl<M> PlantParams<M> {
    /// `machine` with mechanics, inverter, sensors and substep of `p`; thermal and NVH off.
    pub fn with_machine(p: &MotorParams, machine: M) -> Self {
        PlantParams {
            machine,
            mech: p.mech,
            inverter: p.inverter,
            sensors: p.sensors,
            thermal: ThermalParams::OFF,
            nvh: NvhParams::OFF,
            max_substep_s: p.max_substep_s,
        }
    }

    fn layers(&self) -> Layers<'_, M> {
        Layers {
            machine: &self.machine,
            mech: &self.mech,
            inverter: &self.inverter,
            sensors: &self.sensors,
            thermal: &self.thermal,
            nvh: &self.nvh,
            max_substep_s: self.max_substep_s,
        }
    }
}

/// A simulated motor with any [`Machine`]: the same layers, step and outputs as [`Motor`](crate::Motor).
///
/// ```
/// use motor_sim::{Drive, LinearDq, Load, Plant, PlantParams, presets};
///
/// let base = presets::ipm();
/// // The IPM with a mutual dq inductance (linearised cross-saturation).
/// let machine = LinearDq::from_pmsm(&base.pmsm, 0.03e-3);
/// let mut plant = Plant::new(PlantParams::with_machine(&base, machine), 7);
/// plant.set_load(Load::Locked);
/// for _ in 0..1000 {
///     plant.step(&Drive::AlphaBeta { v_alpha_v: 0.0, v_beta_v: 1.0 }, 10e-6);
/// }
/// // A q-axis voltage drives some d current through the mutual inductance on its way to steady state.
/// assert!(plant.truth().i_q > 1.0);
/// ```
pub struct Plant<M: Machine> {
    params: PlantParams<M>,
    core: Core,
}

impl<M: Machine> Plant<M> {
    /// A plant at rest with zero current, no faults, a free shaft and no carrier. `seed` fixes noise and random
    /// faults.
    pub fn new(params: PlantParams<M>, seed: u64) -> Self {
        let core = Core::new(&params.layers(), seed);
        Plant { params, core }
    }

    /// Advance by `dt_s` seconds with `drive` held, then sample the sensors. Without a carrier `dt_s` is normally
    /// one PWM period; with one, any length.
    pub fn step(&mut self, drive: &Drive, dt_s: f32) {
        self.core.step(&self.params.layers(), drive, dt_s);
    }

    /// The true state.
    pub fn truth(&self) -> &Truth {
        &self.core.truth
    }

    /// The sensor readings taken at the end of the last step.
    pub fn sensors(&self) -> &Sensors {
        &self.core.sensors.last
    }

    /// Flux linkage relative to the magnets in the machine's frame, V·s (the engine's state).
    pub fn flux(&self) -> [f32; 2] {
        self.core.flux()
    }

    /// Parameters.
    pub fn params(&self) -> &PlantParams<M> {
        &self.params
    }

    /// Change parameters between steps. The currents stay continuous across a change of the machine.
    pub fn params_mut(&mut self) -> &mut PlantParams<M> {
        self.core.params_changed();
        &mut self.params
    }

    /// The load on the shaft.
    pub fn load(&self) -> &Load {
        &self.core.load
    }

    /// Change the load; see [`Motor::set_load`](crate::Motor::set_load).
    pub fn set_load(&mut self, load: Load) {
        self.core.set_load(load);
    }

    /// The PWM carrier, if set.
    pub fn carrier(&self) -> Option<&Carrier> {
        self.core.carrier.as_ref()
    }

    /// Set or clear the PWM carrier; see [`Motor::set_carrier`](crate::Motor::set_carrier).
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

    /// Energy in the winding field from the machine's map (1.5·W̃), J.
    pub fn magnetic_energy_j(&self) -> f64 {
        self.core.magnetic_energy_j(&self.params.machine)
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
