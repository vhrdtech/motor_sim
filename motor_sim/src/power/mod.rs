//! Layer 2, power stage: the inverter turning commands into winding voltages. Today: average legs (MS-1) and the
//! PWM carrier, which gives each leg's exact volt-seconds over any interval so the step need not be the PWM period.
//! DC link, dead time, diodes and switching segments come with MS-4, MS-5 and MS-6.

mod carrier;
pub(crate) mod inverter;

pub use carrier::{Align, Carrier};
