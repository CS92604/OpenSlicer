//! STL reading (binary and ASCII, auto-detected) and binary writing.

use crate::IoError;
use glam::Vec3;
use openslicer_core::Mesh;

const HEADER_LEN: usize = 80;
const RECORD_LEN: usize = 50;

/// Parses STL data. Binary vs ASCII is decided by the exact size a binary
/// file of the declared triangle count must have, because many binary files
/// also begin with the word "solid".
pub fn read(data: &[u8]) -> Result<Mesh, IoError> {
    if data.len() >= HEADER_LEN + 4 {
        let n = u32::from_le_bytes(data[HEADER_LEN..HEADER_LEN + 4].try_into().unwrap()) as usize;
        if n.checked_mul(RECORD_LEN).and_then(|b| b.checked_add(HEADER_LEN + 4)) == Some(data.len()) {
            return read_binary(data, n);
        }
    }
    let trimmed = data.iter().skip_while(|b| b.is_ascii_whitespace());
    let head: Vec<u8> = trimmed.take(5).copied().collect();
    if head.eq_ignore_ascii_case(b"solid") {
        return read_ascii(data);
    }
    Err(IoError::InvalidStl("size does not match a binary STL and the file is not ASCII STL (truncated or corrupt?)".into()))
}

fn check(v: Vec3, what: &str) -> Result<Vec3, IoError> {
    if v.is_finite() {
        Ok(v)
    } else {
        Err(IoError::InvalidStl(format!("{what} contains a NaN or infinite coordinate")))
    }
}

fn read_binary(data: &[u8], n: usize) -> Result<Mesh, IoError> {
    if n == 0 {
        return Err(IoError::Empty);
    }
    let f = |b: &[u8]| f32::from_le_bytes(b.try_into().unwrap());
    let mut tris = Vec::with_capacity(n);
    for i in 0..n {
        let rec = &data[HEADER_LEN + 4 + i * RECORD_LEN..][..RECORD_LEN];
        let mut t = [Vec3::ZERO; 3];
        for (k, vt) in t.iter_mut().enumerate() {
            let o = 12 + k * 12;
            *vt = check(Vec3::new(f(&rec[o..o + 4]), f(&rec[o + 4..o + 8]), f(&rec[o + 8..o + 12])), &format!("triangle {i}"))?;
        }
        tris.push(t);
    }
    Ok(weld(tris))
}

/// Merges coincident vertices. Tolerance is 1e-6 of the bounding-box diagonal,
/// comfortably above f32 rounding noise but far below any printable feature.
fn weld(tris: Vec<[Vec3; 3]>) -> Mesh {
    let (mut lo, mut hi) = (Vec3::splat(f32::MAX), Vec3::splat(f32::MIN));
    for v in tris.iter().flatten() {
        lo = lo.min(*v);
        hi = hi.max(*v);
    }
    Mesh::from_triangle_soup(tris, ((hi - lo).length() * 1e-6).max(1e-9))
}

fn read_ascii(data: &[u8]) -> Result<Mesh, IoError> {
    let text = String::from_utf8_lossy(data);
    let mut tris = Vec::new();
    let mut cur: Vec<Vec3> = Vec::with_capacity(3);
    for (lineno, line) in text.lines().enumerate() {
        let mut it = line.split_whitespace();
        match it.next().map(|s| s.to_ascii_lowercase()).as_deref() {
            Some("vertex") => {
                let mut c = [0f32; 3];
                for slot in &mut c {
                    *slot = it
                        .next()
                        .and_then(|s| s.parse::<f32>().ok())
                        .ok_or_else(|| IoError::InvalidStl(format!("bad vertex on line {}", lineno + 1)))?;
                }
                cur.push(check(Vec3::from(c), &format!("line {}", lineno + 1))?);
            }
            Some("endfacet") => {
                if cur.len() != 3 {
                    return Err(IoError::InvalidStl(format!("facet ending on line {} has {} vertices, expected 3", lineno + 1, cur.len())));
                }
                tris.push([cur[0], cur[1], cur[2]]);
                cur.clear();
            }
            _ => {}
        }
    }
    if !cur.is_empty() {
        return Err(IoError::InvalidStl("file ends in the middle of a facet".into()));
    }
    if tris.is_empty() {
        return Err(IoError::Empty);
    }
    Ok(weld(tris))
}

/// Serialises a mesh as binary STL.
pub fn write_binary(mesh: &Mesh) -> Vec<u8> {
    let mut out = Vec::with_capacity(HEADER_LEN + 4 + mesh.triangles.len() * RECORD_LEN);
    let mut header = [0u8; HEADER_LEN];
    let tag = b"OpenSlicer binary STL";
    header[..tag.len()].copy_from_slice(tag);
    out.extend_from_slice(&header);
    out.extend_from_slice(&(mesh.triangles.len() as u32).to_le_bytes());
    for i in 0..mesh.triangles.len() {
        let [a, b, c] = mesh.triangle(i);
        let n = (b - a).cross(c - a).normalize_or_zero();
        for v in [n, a, b, c] {
            for x in v.to_array() {
                out.extend_from_slice(&x.to_le_bytes());
            }
        }
        out.extend_from_slice(&[0, 0]);
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    fn cube() -> Mesh {
        Mesh::cuboid(Vec3::new(10.0, 20.0, 30.0))
    }

    #[test]
    fn binary_round_trip_preserves_geometry() {
        let m = cube();
        let back = read(&write_binary(&m)).unwrap();
        assert_eq!(back.triangles.len(), 12);
        assert_eq!(back.vertices.len(), 8);
        let (a, b) = (m.stats(), back.stats());
        assert!((a.signed_volume - b.signed_volume).abs() < 1e-9);
        assert!(b.is_watertight());
    }

    #[test]
    fn ascii_is_parsed() {
        let mut s = String::from("solid t\n");
        let m = cube();
        for i in 0..m.triangles.len() {
            s.push_str("facet normal 0 0 0\n outer loop\n");
            for v in m.triangle(i) {
                s.push_str(&format!("  vertex {} {} {}\n", v.x, v.y, v.z));
            }
            s.push_str(" endloop\nendfacet\n");
        }
        s.push_str("endsolid t\n");
        let back = read(s.as_bytes()).unwrap();
        assert!(back.stats().is_watertight());
        assert!((back.stats().signed_volume - 6000.0).abs() < 1e-6);
    }

    #[test]
    fn binary_file_starting_with_solid_is_still_binary() {
        let mut data = write_binary(&cube());
        data[..5].copy_from_slice(b"solid");
        assert_eq!(read(&data).unwrap().triangles.len(), 12);
    }

    #[test]
    fn seam_vertices_differing_by_float_noise_are_welded() {
        // Two triangles sharing an edge whose endpoints differ by ~1e-5 mm in the file.
        let tri = |a: [f32; 3], b: [f32; 3], c: [f32; 3]| [Vec3::from(a), Vec3::from(b), Vec3::from(c)];
        let t0 = tri([0., 0., 0.], [100., 0., 0.], [0., 100., 0.]);
        let t1 = tri([100.00001, 0., 0.], [0., 100.00001, 0.], [100., 100., 0.]);
        let mesh = Mesh::new([t0, t1].iter().flatten().copied().collect(), vec![[0, 1, 2], [3, 4, 5]]);
        let back = read(&write_binary(&mesh)).unwrap();
        assert_eq!(back.vertices.len(), 4);
    }

    #[test]
    fn truncated_binary_is_rejected() {
        let data = write_binary(&cube());
        assert!(matches!(read(&data[..data.len() - 7]), Err(IoError::InvalidStl(_))));
    }

    #[test]
    fn nan_coordinates_are_rejected() {
        let mut data = write_binary(&cube());
        data[84 + 12..84 + 16].copy_from_slice(&f32::NAN.to_le_bytes());
        assert!(matches!(read(&data), Err(IoError::InvalidStl(_))));
    }

    #[test]
    fn unsupported_extension_is_reported() {
        let r = crate::load_mesh(std::path::Path::new("x.step"));
        assert!(matches!(r, Err(IoError::UnsupportedFormat(_))));
    }
}
