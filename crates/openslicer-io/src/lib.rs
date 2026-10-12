//! Mesh file import and export.

pub mod obj;
pub mod stl;
pub mod threemf;

use openslicer_core::Mesh;
use std::path::Path;

#[derive(Debug, thiserror::Error)]
pub enum IoError {
    #[error("could not read or write file: {0}")]
    Io(#[from] std::io::Error),
    #[error("invalid STL file: {0}")]
    InvalidStl(String),
    #[error("invalid OBJ file: {0}")]
    InvalidObj(String),
    #[error("invalid 3MF file: {0}")]
    Invalid3mf(String),
    #[error("unsupported file type '{0}'. Supported formats: STL, OBJ, 3MF")]
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
        "obj" => obj::read(&std::fs::read(path)?),
        "3mf" => threemf::read(&std::fs::read(path)?),
        other => Err(IoError::UnsupportedFormat(other.to_string())),
    }
}

/// Welds a triangle soup with a tolerance of 1e-6 of its bounding-box diagonal, so floating-point noise in
/// exported files does not create false open edges. Original coordinates are kept.
pub(crate) fn weld(tris: Vec<[glam::Vec3; 3]>) -> Mesh {
    let (mut lo, mut hi) = (glam::Vec3::splat(f32::MAX), glam::Vec3::splat(f32::MIN));
    for v in tris.iter().flatten() {
        lo = lo.min(*v);
        hi = hi.max(*v);
    }
    Mesh::from_triangle_soup(tris, ((hi - lo).length() * 1e-6).max(1e-9))
}

/// Writes a mesh by extension: `.stl` (binary), `.obj` or `.3mf`.
pub fn save_mesh(path: &Path, mesh: &Mesh) -> Result<(), IoError> {
    let ext = path.extension().and_then(|e| e.to_str()).unwrap_or("").to_ascii_lowercase();
    let name = path.file_stem().map(|s| s.to_string_lossy().into_owned()).unwrap_or_else(|| "model".into());
    let bytes: Vec<u8> = match ext.as_str() {
        "stl" => stl::write_binary(mesh),
        "obj" => obj::write(mesh, &name).into_bytes(),
        "3mf" => threemf::write(&[(name.as_str(), mesh)])?,
        other => return Err(IoError::UnsupportedFormat(other.to_string())),
    };
    write_atomic(path, &bytes)
}

/// Writes `bytes` to a temporary file next to `path` and renames it into place, so a failure never leaves a
/// half-written file and never destroys an existing one.
pub fn write_atomic(path: &Path, bytes: &[u8]) -> Result<(), IoError> {
    use std::io::Write;
    let mut tmp = path.as_os_str().to_owned();
    tmp.push(".openslicer-tmp");
    let tmp = std::path::PathBuf::from(tmp);
    let result = (|| -> std::io::Result<()> {
        let mut f = std::fs::File::create(&tmp)?;
        f.write_all(bytes)?;
        f.sync_all()?;
        std::fs::rename(&tmp, path)
    })();
    if result.is_err() {
        let _ = std::fs::remove_file(&tmp);
    }
    result.map_err(IoError::Io)
}

#[cfg(test)]
mod tests {
    use super::*;
    use glam::Vec3;

    #[test]
    fn save_and_load_round_trip_for_every_format() {
        let dir = std::env::temp_dir().join(format!("openslicer-io-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let mut m = Mesh::cuboid(Vec3::new(10.0, 20.0, 30.0));
        m.translate(Vec3::new(5.0, 5.0, 5.0));
        for ext in ["stl", "obj", "3mf"] {
            let path = dir.join(format!("part.{ext}"));
            save_mesh(&path, &m).unwrap();
            let back = load_mesh(&path).unwrap();
            let s = back.stats();
            assert!(s.is_watertight(), "{ext}: {s:?}");
            assert!((s.signed_volume - 6000.0).abs() < 1e-2, "{ext}: {}", s.signed_volume);
            assert_eq!(back.bounds().unwrap().size(), m.bounds().unwrap().size(), "{ext}");
            assert!(!dir.join(format!("part.{ext}.openslicer-tmp")).exists());
        }
        assert!(matches!(save_mesh(&dir.join("x.step"), &m), Err(IoError::UnsupportedFormat(_))));
        std::fs::remove_dir_all(&dir).ok();
    }

    #[test]
    fn a_failed_write_does_not_destroy_the_existing_file() {
        let dir = std::env::temp_dir().join(format!("openslicer-io-atomic-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let target = dir.join("keep.stl");
        std::fs::write(&target, b"original").unwrap();
        // A directory in the way of the temporary name makes the write fail.
        std::fs::create_dir(dir.join("keep.stl.openslicer-tmp")).unwrap();
        assert!(write_atomic(&target, b"new").is_err());
        assert_eq!(std::fs::read(&target).unwrap(), b"original");
        std::fs::remove_dir_all(&dir).ok();
    }
}
