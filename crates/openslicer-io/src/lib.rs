//! Mesh file import and export.

pub mod stl;

use openslicer_core::Mesh;
use std::path::Path;

#[derive(Debug, thiserror::Error)]
pub enum IoError {
    #[error("could not read or write file: {0}")]
    Io(#[from] std::io::Error),
    #[error("invalid STL file: {0}")]
    InvalidStl(String),
    #[error("unsupported file type '{0}'. Supported formats: STL")]
    UnsupportedFormat(String),
    #[error("the file contains no triangles")]
    Empty,
}

/// Loads a mesh, choosing the reader from the file extension.
pub fn load_mesh(path: &Path) -> Result<Mesh, IoError> {
    let ext = path.extension().and_then(|e| e.to_str()).unwrap_or("").to_ascii_lowercase();
    load_mesh_as(path, &ext)
}

/// Loads a mesh using the reader for `ext` regardless of the file name
/// (used to verify files written under a temporary name).
pub fn load_mesh_as(path: &Path, ext: &str) -> Result<Mesh, IoError> {
    match ext {
        "stl" => stl::read(&std::fs::read(path)?),
        other => Err(IoError::UnsupportedFormat(other.to_string())),
    }
}
