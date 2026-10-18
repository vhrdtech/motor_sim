//! Ready-made motors. The VESC one uses the vedderb/bldc firmware defaults; the other two are plausible motors of
//! their class, not a specific datasheet.

use crate::nvh::NvhParams;
use crate::params::*;
use crate::thermal::ThermalParams;

/// VESC 6 style sensors: 0.5 mΩ shunt and gain 20 on a 3.3 V 12-bit ADC (±165 A), 39k/2.2k Vin divider.
fn vesc_sensors() -> SensorParams {
    SensorParams {
        current_full_scale_a: 165.0,
        current_adc_bits: 12,
        current_noise_a: 0.1,
        current_offset_a: [0.3, -0.2, 0.1],
        vdc_full_scale_v: 3.3 * (39.0 + 2.2) / 2.2,
        vdc_adc_bits: 12,
        vdc_noise_v: 0.02,
        encoder_cpr: 8192,
        encoder_offset_rad: 0.0,
        hall_offset_rad: 0.0,
    }
}

/// The hobby outrunner VESC firmware assumes before motor detection (`mcconf_default.h`): R = 15 mΩ, L = 7 µH,
/// λ = 2.45 mWb, 14 poles, 3 cell battery (11.1 V). Inertia and friction are guesses for a ~50 mm outrunner.
pub fn vesc_hobby_outrunner() -> MotorParams {
    MotorParams {
        machine: PmsmParams {
            pole_pairs: 7,
            rs_ohm: 0.015,
            ld_h: 7e-6,
            lq_h: 7e-6,
            flux_linkage_wb: 0.00245,
        },
        mech: MechParams {
            inertia_kgm2: 5e-5,
            viscous_nms_per_rad: 2e-6,
            coulomb_nm: 0.002,
        },
        inverter: InverterParams { vdc_v: 11.1 },
        sensors: vesc_sensors(),
        thermal: ThermalParams::OFF,
        nvh: NvhParams::OFF,
        max_substep_s: 50e-6,
    }
}

/// Small industrial surface-magnet servo, 48 V, ~400 W at 3000 rpm: 8 poles, kt ≈ 0.1 N·m/A.
pub fn industrial_spm() -> MotorParams {
    MotorParams {
        machine: PmsmParams {
            pole_pairs: 4,
            rs_ohm: 0.3,
            ld_h: 0.6e-3,
            lq_h: 0.6e-3,
            flux_linkage_wb: 0.0167,
        },
        mech: MechParams {
            inertia_kgm2: 4e-5,
            viscous_nms_per_rad: 1e-5,
            coulomb_nm: 0.01,
        },
        inverter: InverterParams { vdc_v: 48.0 },
        sensors: SensorParams {
            current_full_scale_a: 25.0,
            current_adc_bits: 12,
            current_noise_a: 0.01,
            current_offset_a: [0.05, -0.03, 0.02],
            vdc_full_scale_v: 60.0,
            vdc_adc_bits: 12,
            vdc_noise_v: 0.02,
            encoder_cpr: 4096,
            encoder_offset_rad: 0.0,
            hall_offset_rad: 0.0,
        },
        thermal: ThermalParams::OFF,
        nvh: NvhParams::OFF,
        max_substep_s: 50e-6,
    }
}

/// Interior permanent magnet motor, 48 V, ~1.5 kW class: 8 poles, Lq ≈ 2.2·Ld, so reluctance torque and MTPA
/// matter.
pub fn ipm() -> MotorParams {
    MotorParams {
        machine: PmsmParams {
            pole_pairs: 4,
            rs_ohm: 0.05,
            ld_h: 0.2e-3,
            lq_h: 0.45e-3,
            flux_linkage_wb: 0.012,
        },
        mech: MechParams {
            inertia_kgm2: 2e-4,
            viscous_nms_per_rad: 2e-5,
            coulomb_nm: 0.02,
        },
        inverter: InverterParams { vdc_v: 48.0 },
        sensors: SensorParams {
            current_full_scale_a: 100.0,
            current_adc_bits: 12,
            current_noise_a: 0.05,
            current_offset_a: [0.1, -0.1, 0.05],
            vdc_full_scale_v: 60.0,
            vdc_adc_bits: 12,
            vdc_noise_v: 0.02,
            encoder_cpr: 4096,
            encoder_offset_rad: 0.0,
            hall_offset_rad: 0.0,
        },
        thermal: ThermalParams::OFF,
        nvh: NvhParams::OFF,
        max_substep_s: 50e-6,
    }
}
