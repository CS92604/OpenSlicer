//! Mesh and math primitives shared by every OpenSlicer crate.

pub use glam::{Mat4, Vec3};

pub mod control;
mod mesh;
pub use mesh::{Aabb, Mesh, MeshStats};
