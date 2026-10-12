//! Geometry engine: plane cutting with capping, planar regions, component
//! separation and (in later modules) Booleans and repair.
//!
//! Every operation that changes a mesh returns a report that states whether the
//! result is watertight and whether volume was conserved. Callers must look at
//! the report; nothing here claims success silently.

pub mod boolean;
pub mod build;
pub mod bvh;
pub mod components;
pub mod crop;
pub mod cut;
pub mod hollow;
pub mod plane;
pub mod poly2d;
pub mod region;
pub mod repair;
pub mod section;

#[cfg(test)]
pub(crate) mod tests_support;

pub use plane::Plane;
