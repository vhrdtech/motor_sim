//! Motor geometry from CAD for the twin and, later, the FEM: STEP import into an assembly tree of parts, part
//! classification by role with evidence and confidence, a scene description and glTF export, and parametric test
//! motors written to STEP. Pure Rust (monstertruck for the B-rep, STEP and meshing), std only.

// geometry walks parallel tables (features of part i and j, matrix rows): indices read better than zipped iterators
#![allow(clippy::needless_range_loop)]

pub mod classify;
pub mod generate;
pub mod geometry;
pub mod mesh;
pub mod overrides;
pub mod report;
pub mod role;
pub mod scene;
pub mod shapes;
pub mod step_read;
pub mod step_write;
