//! Wavefront OBJ reading and writing (geometry only).
//!
//! Faces with more than three vertices are triangulated as fans, `v/vt/vn` references and negative (relative)
//! indices are supported, and materials, normals and texture coordinates are ignored. All objects and groups
//! in a file are merged into one mesh.

use crate::IoError;
use glam::Vec3;
use openslicer_core::Mesh;

pub fn read(data: &[u8]) -> Result<Mesh, IoError> {
    let text = String::from_utf8_lossy(data);
    let mut positions: Vec<Vec3> = Vec::new();
    let mut tris: Vec<[Vec3; 3]> = Vec::new();
    for (lineno, raw) in text.lines().enumerate() {
        let line = raw.split('#').next().unwrap_or("").trim();
        let mut it = line.split_whitespace();
        match it.next() {
            Some("v") => {
                let c: Vec<f32> = it
                    .take(3)
                    .map(|s| s.parse::<f32>())
                    .collect::<Result<_, _>>()
                    .map_err(|_| IoError::InvalidObj(format!("bad vertex on line {}", lineno + 1)))?;
                if c.len() != 3 || c.iter().any(|x| !x.is_finite()) {
                    return Err(IoError::InvalidObj(format!("bad vertex on line {}", lineno + 1)));
                }
                positions.push(Vec3::new(c[0], c[1], c[2]));
            }
            Some("f") => {
                let mut idx: Vec<usize> = Vec::new();
                for tok in it {
                    let first = tok.split('/').next().unwrap_or("");
                    let n: i64 =
                        first.parse().map_err(|_| IoError::InvalidObj(format!("bad face index '{tok}' on line {}", lineno + 1)))?;
                    let resolved = if n > 0 { n - 1 } else { positions.len() as i64 + n };
                    if resolved < 0 || resolved as usize >= positions.len() {
                        return Err(IoError::InvalidObj(format!("face on line {} refers to a vertex that does not exist", lineno + 1)));
                    }
                    idx.push(resolved as usize);
                }
                if idx.len() < 3 {
                    return Err(IoError::InvalidObj(format!("face on line {} has fewer than three vertices", lineno + 1)));
                }
                for k in 1..idx.len() - 1 {
                    tris.push([positions[idx[0]], positions[idx[k]], positions[idx[k + 1]]]);
                }
            }
            _ => {}
        }
    }
    if tris.is_empty() {
        return Err(IoError::Empty);
    }
    Ok(crate::weld(tris))
}

pub fn write(mesh: &Mesh, name: &str) -> String {
    let mut out = String::with_capacity(mesh.vertices.len() * 32 + mesh.triangles.len() * 24);
    out.push_str("# OpenSlicer OBJ export\n");
    out.push_str(&format!("o {name}\n"));
    for v in &mesh.vertices {
        out.push_str(&format!("v {} {} {}\n", v.x, v.y, v.z));
    }
    for t in &mesh.triangles {
        out.push_str(&format!("f {} {} {}\n", t[0] + 1, t[1] + 1, t[2] + 1));
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn quads_slashes_comments_and_negative_indices() {
        let src = "# a unit square pyramid\nv 0 0 0\nv 1 0 0\nv 1 1 0\nv 0 1 0\nv 0.5 0.5 1\n\
                   f 1//1 4//1 3//1 2//1   # quad base\nf 1/1 2/2 5/3\nf -3 -2 5\nf -4 -3 5\nf -1 -4 -5 \n";
        let m = read(src.as_bytes()).unwrap();
        assert_eq!(m.vertices.len(), 5);
        assert_eq!(m.triangles.len(), 2 + 4);
    }

    #[test]
    fn round_trip_preserves_volume() {
        let mut m = Mesh::cuboid(Vec3::new(10.0, 20.0, 30.0));
        m.translate(Vec3::new(1.0, 2.0, 3.0));
        let back = read(write(&m, "cube").as_bytes()).unwrap();
        assert_eq!(back.triangles.len(), 12);
        assert!((back.stats().signed_volume - 6000.0).abs() < 1e-3);
        assert!(back.stats().is_watertight());
    }

    #[test]
    fn malformed_files_are_rejected_with_the_line_number() {
        assert!(matches!(read(b"v 1 2\nf 1 2 3\n"), Err(IoError::InvalidObj(m)) if m.contains("line 1")));
        assert!(matches!(read(b"v 0 0 0\nv 1 0 0\nf 1 2 9\n"), Err(IoError::InvalidObj(m)) if m.contains("does not exist")));
        assert!(matches!(read(b"v 0 0 0\nf 1 1\n"), Err(IoError::InvalidObj(_))));
        assert!(matches!(read(b"# nothing\n"), Err(IoError::Empty)));
        assert!(matches!(read(b"v nan 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 3\n"), Err(IoError::InvalidObj(_))));
    }
}
