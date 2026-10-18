//! Model parameters. All SI: Ω, H, Wb, kg·m², N·m, V, A, rad.

use crate::nvh::NvhParams;
use crate::thermal::ThermalParams;

/// Electrical parameters of a PMSM in the rotor (dq) frame, amplitude-invariant transform.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PmsmParams {
    /// Pole pairs (VESC "motor poles" / 2).
    pub pole_pairs: u32,
    /// Phase resistance, Ω. Must be > 0.
    pub rs_ohm: f32,
    /// d-axis inductance, H.
    pub ld_h: f32,
    /// q-axis inductance, H (Lq > Ld for an IPM, Lq = Ld for an SPM).
    pub lq_h: f32,
    /// Permanent magnet flux linkage, Wb (V·s/rad electrical, phase peak).
    pub flux_linkage_wb: f32,
}

impl PmsmParams {
    /// Torque constant for i_d = 0, N·m/A of i_q: 1.5 · p · λ.
    pub fn kt_nm_per_a(&self) -> f32 {
        1.5 * self.pole_pairs as f32 * self.flux_linkage_wb
    }
}

/// Rotor mechanics.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MechParams {
    /// Rotor inertia, kg·m². Must be > 0.
    pub inertia_kgm2: f32,
    /// Viscous friction of the motor itself, N·m·s/rad.
    pub viscous_nms_per_rad: f32,
    /// Coulomb (dry) friction of the motor itself, N·m; also the breakaway torque at standstill.
    pub coulomb_nm: f32,
}

/// Average inverter with an ideal DC link (sag is an L2 feature).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct InverterParams {
    /// DC link voltage, V.
    pub vdc_v: f32,
}

/// What the controller's sensors see: ADC ranges, noise and calibration offsets, encoder and hall placement.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SensorParams {
    /// Phase current ADC range: readings span ± this, A.
    pub current_full_scale_a: f32,
    /// Phase current ADC resolution, bits.
    pub current_adc_bits: u8,
    /// Phase current noise, A rms (gaussian, before quantisation).
    pub current_noise_a: f32,
    /// Phase current offsets a controller would calibrate out, A.
    pub current_offset_a: [f32; 3],
    /// DC link voltage ADC range: 0 to this, V.
    pub vdc_full_scale_v: f32,
    /// DC link voltage ADC resolution, bits.
    pub vdc_adc_bits: u8,
    /// DC link voltage noise, V rms.
    pub vdc_noise_v: f32,
    /// Incremental encoder counts per mechanical revolution (after x4 decoding). 0: no encoder.
    pub encoder_cpr: u32,
    /// Encoder zero relative to the rotor d axis, mechanical rad.
    pub encoder_offset_rad: f32,
    /// Hall sensor placement relative to the rotor d axis, electrical rad.
    pub hall_offset_rad: f32,
}

/// Everything a [`Plant`](crate::Plant) needs: the machine (electromagnetics) and the parameters of the other
/// layers. [`MotorParams`] is the one with MS-1's linear dq PMSM.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PlantParams<M> {
    /// Electromagnetics: the machine and its map.
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
    /// Longest base substep, s: [`Plant::step`](crate::Plant::step) splits longer `dt` into substeps so the
    /// mechanics (springs, friction) stay accurate. 50 µs by default; the electrical part is exact at any step for a
    /// linear machine.
    pub max_substep_s: f32,
}

/// Parameters of a [`Motor`](crate::Motor): a linear dq PMSM.
pub type MotorParams = PlantParams<PmsmParams>;

impl<M> PlantParams<M> {
    /// The same mechanics, inverter, sensors, slow layers and substep with another machine.
    pub fn with_machine<N>(&self, machine: N) -> PlantParams<N> {
        PlantParams {
            machine,
            mech: self.mech,
            inverter: self.inverter,
            sensors: self.sensors,
            thermal: self.thermal,
            nvh: self.nvh,
            max_substep_s: self.max_substep_s,
        }
    }
}
