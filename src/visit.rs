//! A listing of every variable the simulator exposes, with name, unit and the rate it is worth watching at.
//!
//! The tiers follow VESC: [`Tier::Pwm`] changes every PWM period (25 kHz ISR: currents, voltages, angles),
//! [`Tier::Control`] is slower state (1 kHz: speed, torque, DC current), [`Tier::Telemetry`] are totals worth a few
//! Hz (50 Hz VESC Tool poll: energy and charge counters).

use crate::{Sensors, Truth};

/// How fast a variable changes, as a hint for how often to sample or observe it.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum Tier {
    /// Every PWM period (~25 kHz).
    Pwm,
    /// Control loop rate (~1 kHz).
    Control,
    /// Telemetry (~50 Hz).
    Telemetry,
}

/// Where a variable comes from.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum Source {
    /// True physics ([`Truth`]): what a controller can't see on real hardware.
    Truth,
    /// What the controller measures ([`Sensors`]).
    Sensor,
}

/// Description of one variable.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct Var {
    /// Field name in [`Truth`] or [`Sensors`]; unique within its source.
    pub name: &'static str,
    /// SI unit (`"A"`, `"V"`, `"rad/s"`, `"N·m"`, ...), `""` for counts and ratios.
    pub unit: &'static str,
    /// Rate hint.
    pub tier: Tier,
    /// Truth or sensor.
    pub source: Source,
}

/// Current value of a variable.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Value {
    /// Most quantities.
    F32(f32),
    /// Accumulators that grow (time, multi-turn angle, energy, charge).
    F64(f64),
    /// Counts and bit patterns.
    U32(u32),
}

/// Receives every variable from [`Motor::visit`](crate::Motor::visit), in a fixed order.
pub trait Visitor {
    /// Called once per variable.
    fn var(&mut self, var: &Var, value: Value);
}

impl<F: FnMut(&Var, Value)> Visitor for F {
    fn var(&mut self, var: &Var, value: Value) {
        self(var, value)
    }
}

pub(crate) fn visit(t: &Truth, s: &Sensors, v: &mut impl Visitor) {
    use Source::*;
    use Tier::*;
    use Value::*;
    let mut f = |name, unit, tier, source, value| {
        v.var(
            &Var {
                name,
                unit,
                tier,
                source,
            },
            value,
        )
    };
    f("time_s", "s", Telemetry, Truth, F64(t.time_s));
    f("i_d", "A", Pwm, Truth, F32(t.i_d));
    f("i_q", "A", Pwm, Truth, F32(t.i_q));
    f("i_alpha", "A", Pwm, Truth, F32(t.i_alpha));
    f("i_beta", "A", Pwm, Truth, F32(t.i_beta));
    f("i_a", "A", Pwm, Truth, F32(t.i_a));
    f("i_b", "A", Pwm, Truth, F32(t.i_b));
    f("i_c", "A", Pwm, Truth, F32(t.i_c));
    f("v_d", "V", Pwm, Truth, F32(t.v_d));
    f("v_q", "V", Pwm, Truth, F32(t.v_q));
    f("v_alpha", "V", Pwm, Truth, F32(t.v_alpha));
    f("v_beta", "V", Pwm, Truth, F32(t.v_beta));
    f("v_a", "V", Pwm, Truth, F32(t.v_a));
    f("v_b", "V", Pwm, Truth, F32(t.v_b));
    f("v_c", "V", Pwm, Truth, F32(t.v_c));
    f("duty_a", "", Pwm, Truth, F32(t.duty[0]));
    f("duty_b", "", Pwm, Truth, F32(t.duty[1]));
    f("duty_c", "", Pwm, Truth, F32(t.duty[2]));
    f("theta_e", "rad", Pwm, Truth, F32(t.theta_e));
    f("omega_e", "rad/s", Control, Truth, F32(t.omega_e));
    f("theta_m", "rad", Control, Truth, F64(t.theta_m));
    f("omega_m", "rad/s", Control, Truth, F32(t.omega_m));
    f("torque_e", "N·m", Control, Truth, F32(t.torque_e));
    f("torque_load", "N·m", Control, Truth, F32(t.torque_load));
    f("theta_load", "rad", Control, Truth, F64(t.theta_load));
    f("omega_load", "rad/s", Control, Truth, F32(t.omega_load));
    f("vdc", "V", Control, Truth, F32(t.vdc));
    f("i_dc", "A", Control, Truth, F32(t.i_dc));
    f("p_dc", "W", Control, Truth, F32(t.p_dc));
    f(
        "charge_drawn_ah",
        "Ah",
        Telemetry,
        Truth,
        F64(t.charge_drawn_ah),
    );
    f(
        "charge_regen_ah",
        "Ah",
        Telemetry,
        Truth,
        F64(t.charge_regen_ah),
    );
    f(
        "energy_drawn_wh",
        "Wh",
        Telemetry,
        Truth,
        F64(t.energy_drawn_wh),
    );
    f(
        "energy_regen_wh",
        "Wh",
        Telemetry,
        Truth,
        F64(t.energy_regen_wh),
    );
    f("copper_loss_j", "J", Telemetry, Truth, F64(t.copper_loss_j));
    f(
        "friction_loss_j",
        "J",
        Telemetry,
        Truth,
        F64(t.friction_loss_j),
    );
    f("load_work_j", "J", Telemetry, Truth, F64(t.load_work_j));
    f("i_a", "A", Pwm, Sensor, F32(s.i_abc_a[0]));
    f("i_b", "A", Pwm, Sensor, F32(s.i_abc_a[1]));
    f("i_c", "A", Pwm, Sensor, F32(s.i_abc_a[2]));
    f("vdc", "V", Control, Sensor, F32(s.vdc_v));
    f("encoder_count", "", Pwm, Sensor, U32(s.encoder_count));
    f("hall", "", Pwm, Sensor, U32(s.hall as u32));
}
