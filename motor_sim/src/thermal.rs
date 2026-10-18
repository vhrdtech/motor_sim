//! Layer 5, thermal: slow (~1 kHz) lumped network feeding R(T), λ(T), friction(T) back (MS-3). Today only the
//! slot in the multirate schedule.

/// How much of the thermal layer runs.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
#[non_exhaustive]
pub enum ThermalLevel {
    /// Temperatures stay at their reference values.
    #[default]
    Off,
}

/// Thermal layer settings.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ThermalParams {
    /// Level of detail.
    pub level: ThermalLevel,
    /// Runs once every this many base substeps (100 at 10 µs: 1 kHz).
    pub every: u32,
}

impl ThermalParams {
    /// Thermal layer off.
    pub const OFF: ThermalParams = ThermalParams {
        level: ThermalLevel::Off,
        every: 100,
    };

    /// One slow step of `dt_s` seconds.
    pub(crate) fn step(&self, _dt_s: f64) {
        match self.level {
            ThermalLevel::Off => {}
        }
    }
}

impl Default for ThermalParams {
    fn default() -> Self {
        Self::OFF
    }
}
