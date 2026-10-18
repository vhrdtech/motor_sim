//! Motor geometry from CAD for the twin and, later, the FEM: STEP import into an assembly tree of parts, part
//! classification by role with evidence and confidence, a scene description and glTF export, and parametric test
//! motors written to STEP. Pure Rust (monstertruck for the B-rep, STEP and meshing), std only.

pub mod generate;
pub mod mesh;
pub mod shapes;
pub mod step_read;
pub mod step_write;
