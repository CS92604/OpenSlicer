//! Splitting oversized models into printable pieces.
//!
//! * [`fit`]: machine build-volume constraints and print-orientation search
//! * [`plan`] / [`exec`]: cut plans and their verified execution
//! * [`layout`]: planar, multi-planar, modular, grid and staggered layouts
//! * [`autotile`]: automatic tiling that minimises the number of pieces
//! * [`interfaces`]: which pieces touch, and over what area

pub mod autotile;
pub mod exec;
pub mod fit;
pub mod interfaces;
pub mod layout;
pub mod occupancy;
pub mod plan;
pub mod reduce;

#[cfg(test)]
pub(crate) mod tests_shapes;
