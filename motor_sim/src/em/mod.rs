//! Layer 3, electromagnetics: the machine as a magnetic two-port with flux as state.
//!
//! A [`Machine`] is a map between flux linkage and current in its [`Frame`] (the frame is a property of the map,
//! not of the engine), plus resistance and pole pairs. The engine integrates the flux with the exponential step in
//! `etd.rs`, gets the current back from the map and the torque from the co-energy. Linear machines:
//! [`PmsmParams`](crate::PmsmParams) (MS-1), [`LinearDq`] (full constant L), [`LinearAlphaBeta`] (stationary
//! frame); tables: [`MapMachine`] (feature `maps`).

mod etd;
mod frame;
mod layer;
mod linear;
mod machine;
#[cfg(feature = "maps")]
mod table;

pub(crate) use etd::advance;
pub(crate) use frame::park;
pub use frame::{AlphaBeta, Dq, Frame};
pub(crate) use layer::Electromagnetics;
pub use linear::{LinearAlphaBeta, LinearDq};
pub(crate) use machine::torque;
pub use machine::{Linearised, Machine, Magnet};
#[cfg(feature = "maps")]
pub use table::{Field, MapMachine};
