//! Orchestration for OpenSlicer: printer profiles, the split-and-connect pipeline, assembly planning,
//! export with a manifest, and native project files. This crate has no UI; the CLI and the desktop app both use it.

pub mod assembly;
pub mod export;
pub mod pipeline;
pub mod profile;
pub mod project;
pub mod settings;
pub mod stock;
