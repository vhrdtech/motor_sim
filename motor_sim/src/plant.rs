//! [`Plant`]: a simulated motor with any [`Machine`]; [`Motor`](crate::Motor) is the one with MS-1's linear dq
//! PMSM.

use crate::Truth;
use crate::drive::Drive;
use crate::em::Machine;
use crate::faults::Faults;
use crate::mechanics::Load;
use crate::multirate::SimClock;
use crate::params::PlantParams;
use crate::power::Carrier;
use crate::sensors::Sensors;
use crate::stack::Core;
use crate::visit::{self, Visitor};

/// A simulated motor: inverter, electromagnetics (the [`Machine`] `M`), mechanics, load, sensors and faults, run
/// as a stack of layers in a fixed order per base substep. The true state is in [`Truth`]; a controller should only
/// read [`Sensors`].
///
/// ```
/// use motor_sim::{Drive, LinearDq, Load, Plant, presets};
///
/// let base = presets::ipm();
/// // The IPM with a mutual dq inductance (linearised cross-saturation).
/// let machine = LinearDq::from_pmsm(&base.machine, 0.03e-3);
/// let mut plant = Plant::new(base.with_machine(machine), 7);
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

/// The whole state of a [`Plant`] (every layer, load, faults, carrier, random generators, clock), without its
/// parameters: a copy taken by [`Plant::snapshot`] and put back by [`Plant::restore`], for replay, rewind and
/// branching runs. Restoring and stepping with the same inputs repeats the run bit for bit.
#[derive(Clone, Copy)]
pub struct PlantState(Core);

impl<M: Machine> Plant<M> {
    /// A plant at rest with zero current, no faults, a free shaft and no carrier. `seed` fixes noise and random
    /// faults.
    pub fn new(params: PlantParams<M>, seed: u64) -> Self {
        let core = Core::new(&params, seed);
        Plant { params, core }
    }

    /// Advance by `dt_s` seconds with `drive` held, then sample the sensors. Without a carrier `dt_s` is normally
    /// one PWM period; with one, any length. Longer steps are split into base substeps of at most
    /// [`PlantParams::max_substep_s`].
    pub fn step(&mut self, drive: &Drive, dt_s: f32) {
        self.core.step(&self.params, drive, dt_s);
    }

    /// The true state.
    pub fn truth(&self) -> &Truth {
        &self.core.truth
    }

    /// The sensor readings taken at the end of the last step.
    pub fn sensors(&self) -> &Sensors {
        &self.core.sensors.last
    }

    /// The simulation clock: base substeps taken and simulated time.
    pub fn clock(&self) -> SimClock {
        self.core.clock
    }

    /// Copy of the whole state.
    pub fn snapshot(&self) -> PlantState {
        PlantState(self.core)
    }

    /// Put back a state from [`Plant::snapshot`], exactly as it was. The parameters stay as they are now.
    pub fn restore(&mut self, state: &PlantState) {
        self.core = state.0;
    }

    /// Flux linkage relative to the magnets in the machine's frame, V·s (the electromagnetic state).
    pub fn flux(&self) -> [f32; 2] {
        self.core.em.flux()
    }

    /// Parameters.
    pub fn params(&self) -> &PlantParams<M> {
        &self.params
    }

    /// Change parameters between steps (a different DC link voltage, a heavier rotor, other inductances, ...). The
    /// currents stay continuous across a change of the machine.
    pub fn params_mut(&mut self) -> &mut PlantParams<M> {
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
        self.core.power.carrier.as_ref()
    }

    /// Set or clear the PWM carrier. Without one (the default) the duties of a step are its average, so a step
    /// should be one PWM period. With one, the duties are compare values held over the step and every substep
    /// gets the volt-seconds the legs really deliver within it, from the carrier phase at the simulated time:
    /// steps shorter (or longer) than the PWM period see the ripple of the average voltage.
    pub fn set_carrier(&mut self, carrier: Option<Carrier>) {
        self.core.power.carrier = carrier;
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
        self.core
            .em
            .field_energy_j(&self.params.machine, self.core.truth.theta_e)
    }

    /// Kinetic energy of the rotor (without a [`Load::Inertia`]), J.
    pub fn kinetic_energy_j(&self) -> f64 {
        self.core.kinetic_energy_j(self.params.mech.inertia_kgm2)
    }

    /// List every variable with name, unit, rate tier and current value: [`Truth`] first, then [`Sensors`].
    pub fn visit(&self, visitor: &mut impl Visitor) {
        visit::visit(&self.core.truth, &self.core.sensors.last, visitor)
    }
}
