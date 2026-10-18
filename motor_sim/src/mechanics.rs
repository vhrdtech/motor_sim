//! Rotor mechanics and the loads on the shaft.

use crate::params::MechParams;

/// What the shaft drives. Positive load torque opposes positive rotation.
#[derive(Clone, Copy, Debug)]
pub enum Load {
    /// Free shaft: only the motor's own friction.
    None,
    /// Constant torque, N·m, acting at any speed including standstill (a hanging weight, not friction).
    Constant {
        /// Load torque, N·m.
        torque_nm: f32,
    },
    /// Viscous: torque = b·ω.
    Viscous {
        /// N·m·s/rad.
        b_nms_per_rad: f32,
    },
    /// Fan or propeller: torque = k·ω·|ω|.
    Fan {
        /// N·m·s²/rad².
        k_nms2_per_rad2: f32,
    },
    /// A second inertia coupled through a torsion spring and damper, with viscous drag of its own. The spring must
    /// stay well below the substep's Nyquist: `max_substep_s · √(k/J) < 1`.
    Inertia {
        /// Load inertia, kg·m².
        inertia_kgm2: f32,
        /// Coupling stiffness, N·m/rad.
        stiffness_nm_per_rad: f32,
        /// Coupling damping, N·m·s/rad.
        damping_nms_per_rad: f32,
        /// Drag on the load inertia, N·m·s/rad.
        viscous_nms_per_rad: f32,
    },
    /// Torque as a function of time since start (s), N·m, acting like [`Load::Constant`]. For loads computed at
    /// run time, call [`Motor::set_load`](crate::Motor::set_load) between steps instead.
    Profile(fn(f64) -> f32),
    /// A dynamometer holding the shaft at this speed, rad/s (mechanical), whatever the motor does.
    Speed {
        /// Shaft speed, rad/s.
        omega_m_rad_s: f32,
    },
    /// Shaft locked at its current angle.
    Locked,
}

/// How the rotor moves over one base substep, as the electromagnetics sees it (plain data).
#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct RotorMotion {
    /// Electrical angle at the start, rad in [0, 2π).
    pub theta_e: f32,
    /// Electrical speed held over the substep, rad/s.
    pub omega_e: f32,
}

/// State of the mechanics layer.
#[derive(Clone, Copy)]
pub(crate) struct Mechanics {
    pub state: MechState,
    /// Rotor speed at the start of the previous substep, rad/s, to predict the mid-step speed.
    omega_m_prev: f32,
}

impl Mechanics {
    pub fn new() -> Self {
        Mechanics {
            state: MechState {
                theta_m: 0.0,
                omega_m: 0.0,
                theta_load: 0.0,
                omega_load: 0.0,
            },
            omega_m_prev: 0.0,
        }
    }

    /// The rotor motion for the next substep. Freezes ω at the predicted mid-step speed, so the electrical and
    /// mechanical sides exchange the same energy to second order.
    pub fn rotor(&mut self, pole_pairs: u32, load: &Load) -> RotorMotion {
        let s = &self.state;
        let pp = pole_pairs as f32;
        let omega_m = match load {
            Load::Speed { .. } | Load::Locked => s.omega_m,
            _ => 1.5 * s.omega_m - 0.5 * self.omega_m_prev,
        };
        self.omega_m_prev = s.omega_m;
        RotorMotion {
            theta_e: crate::wrap_tau(pp as f64 * s.theta_m) as f32,
            omega_e: pp * omega_m,
        }
    }

    /// One substep with the electromagnetic torque `te` (N·m, average over the substep) at time `t_s`.
    pub fn step(&mut self, m: &MechParams, load: &Load, te: f32, t_s: f64, h: f32) -> MechStep {
        step(m, load, &mut self.state, te, t_s, h)
    }
}

/// Mechanical state, kept in [`Truth`](crate::Truth).
#[derive(Clone, Copy)]
pub(crate) struct MechState {
    pub theta_m: f64,
    pub omega_m: f32,
    pub theta_load: f64,
    pub omega_load: f32,
}

/// Result of one mechanical step: the torque the load puts on the shaft and the energies over the step.
pub(crate) struct MechStep {
    pub torque_load_nm: f32,
    pub friction_loss_j: f64,
    pub load_work_j: f64,
}

/// Advance the mechanics by `h` with the electromagnetic torque `te` (N·m, average over the step) at time `t_s`.
///
/// Viscous terms are implicit, spring and fan torques explicit; the energy terms use the mid-step speed, so
/// `ΔE_kin = (te − friction − load)·ω_mid·h` holds exactly.
fn step(m: &MechParams, load: &Load, s: &mut MechState, te: f32, t_s: f64, h: f32) -> MechStep {
    let omega0 = s.omega_m;
    let friction_of = |w: f32| m.viscous_nms_per_rad * w + m.coulomb_nm * sign(w);
    match *load {
        Load::Locked => {
            s.omega_m = 0.0;
            return MechStep {
                torque_load_nm: te,
                friction_loss_j: 0.0,
                load_work_j: 0.0,
            };
        }
        Load::Speed { omega_m_rad_s: w } => {
            s.omega_m = w;
            s.theta_m += (w * h) as f64;
            let tl = te - friction_of(w);
            return MechStep {
                torque_load_nm: tl,
                friction_loss_j: (friction_of(w) * w * h) as f64,
                load_work_j: (tl * w * h) as f64,
            };
        }
        _ => {}
    }

    // Load torque independent of ω_new (explicit part) and the load's viscous coefficient (implicit part).
    let (t_ext, b_load) = match *load {
        Load::None => (0.0, 0.0),
        Load::Constant { torque_nm } => (torque_nm, 0.0),
        Load::Viscous { b_nms_per_rad } => (0.0, b_nms_per_rad),
        Load::Fan { k_nms2_per_rad2 } => (k_nms2_per_rad2 * omega0 * libm::fabsf(omega0), 0.0),
        Load::Inertia {
            stiffness_nm_per_rad,
            damping_nms_per_rad,
            ..
        } => (
            stiffness_nm_per_rad * (s.theta_m - s.theta_load) as f32
                + damping_nms_per_rad * (omega0 - s.omega_load),
            0.0,
        ),
        Load::Profile(f) => (f(t_s), 0.0),
        Load::Speed { .. } | Load::Locked => unreachable!(),
    };

    let j = m.inertia_kgm2;
    let b = m.viscous_nms_per_rad + b_load;
    let tc = m.coulomb_nm;
    let drive = te - t_ext;
    let solve = |dir: f32| (j * omega0 + h * (drive - tc * dir)) / (j + h * b);
    let mut stuck = false;
    let mut dir = sign(omega0);
    let omega1 = if omega0 == 0.0 {
        if libm::fabsf(drive) <= tc {
            stuck = true;
            0.0
        } else {
            dir = sign(drive);
            solve(dir)
        }
    } else {
        let w = solve(dir);
        if tc > 0.0 && w * omega0 < 0.0 {
            stuck = true;
            0.0
        } else {
            w
        }
    };
    let w_mid = 0.5 * (omega0 + omega1);
    s.omega_m = omega1;
    s.theta_m += (w_mid * h) as f64;

    let tl = t_ext + b_load * omega1;
    let load_work = (tl * w_mid * h) as f64;
    let friction = if stuck {
        // Coulomb friction stopped the rotor: it took whatever kinetic energy was left.
        (te * w_mid * h) as f64
            - load_work
            - 0.5 * j as f64 * ((omega1 * omega1 - omega0 * omega0) as f64)
    } else {
        ((m.viscous_nms_per_rad * omega1 + tc * dir) * w_mid * h) as f64
    };

    if let Load::Inertia {
        inertia_kgm2,
        viscous_nms_per_rad,
        ..
    } = *load
    {
        let w0 = s.omega_load;
        let w1 = (inertia_kgm2 * w0 + h * t_ext) / (inertia_kgm2 + h * viscous_nms_per_rad);
        s.omega_load = w1;
        s.theta_load += (0.5 * (w0 + w1) * h) as f64;
    }

    MechStep {
        torque_load_nm: tl,
        friction_loss_j: friction,
        load_work_j: load_work,
    }
}

fn sign(x: f32) -> f32 {
    if x > 0.0 {
        1.0
    } else if x < 0.0 {
        -1.0
    } else {
        0.0
    }
}
