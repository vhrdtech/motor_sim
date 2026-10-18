//! Layer 6, vibration and sound: modal synthesis driven by forces from electromagnetics and mechanics, pressure at
//! microphones at audio rate (MS-28, MS-29); output only. Today only the slot in the multirate schedule.

/// How much of the NVH layer runs.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
#[non_exhaustive]
pub enum NvhLevel {
    /// No vibration or sound output.
    #[default]
    Off,
}

/// NVH layer settings.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct NvhParams {
    /// Level of detail.
    pub level: NvhLevel,
    /// Runs once every this many base substeps (2 at 10 µs: 50 kHz, decimated to the audio rate).
    pub every: u32,
}

impl NvhParams {
    /// NVH layer off.
    pub const OFF: NvhParams = NvhParams {
        level: NvhLevel::Off,
        every: 2,
    };

    /// One slow step of `dt_s` seconds.
    pub(crate) fn step(&self, _dt_s: f64) {
        match self.level {
            NvhLevel::Off => {}
        }
    }
}

impl Default for NvhParams {
    fn default() -> Self {
        Self::OFF
    }
}
