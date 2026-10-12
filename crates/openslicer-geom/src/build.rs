//! Primitive solids: extrusions, lofts and solids of revolution.
//!
//! All builders return closed, consistently oriented (outward normals) meshes with shared vertices, so
//! they are valid operands for Boolean operations. Shapes are built in a local frame (+Z up) and placed
//! into the world with [`LocalFrame`].

use crate::region::{ring_area, Polygon};
use glam::{DVec2, DVec3, Vec3};
use openslicer_core::Mesh;

/// Removes repeated and exactly-collinear vertices from a ring. Collinear points would otherwise yield
/// zero-area triangles during triangulation.
pub fn clean_ring(ring: &[DVec2]) -> Vec<DVec2> {
    let mut pts: Vec<DVec2> = Vec::with_capacity(ring.len());
    for &p in ring {
        if pts.last().is_none_or(|q| (*q - p).length() > 1e-9) {
            pts.push(p);
        }
    }
    while pts.len() > 1 && (pts[0] - pts[pts.len() - 1]).length() <= 1e-9 {
        pts.pop();
    }
    loop {
        let n = pts.len();
        if n < 3 {
            break;
        }
        let drop = (0..n).find(|&i| {
            let (a, b, c) = (pts[(i + n - 1) % n], pts[i], pts[(i + 1) % n]);
            let (u, v) = (b - a, c - b);
            u.perp_dot(v).abs() <= 1e-9 * u.length() * v.length()
        });
        match drop {
            Some(i) => {
                pts.remove(i);
            }
            None => break,
        }
    }
    pts
}

fn cap_triangles(rings: &[Vec<DVec2>]) -> Vec<[usize; 3]> {
    let mut flat = Vec::new();
    let mut holes = Vec::new();
    let mut count = 0;
    for (i, r) in rings.iter().enumerate() {
        if i > 0 {
            holes.push(count);
        }
        for p in r {
            flat.extend([p.x, p.y]);
        }
        count += r.len();
    }
    let idx = earcutr::earcut(&flat, &holes, 2).unwrap_or_default();
    let pts: Vec<DVec2> = rings.iter().flatten().copied().collect();
    idx.chunks_exact(3)
        .map(|t| {
            let area = (pts[t[1]] - pts[t[0]]).perp_dot(pts[t[2]] - pts[t[0]]);
            if area >= 0.0 {
                [t[0], t[1], t[2]]
            } else {
                [t[0], t[2], t[1]]
            }
        })
        .collect()
}

/// Prism over `polygons` from `z0` to `z1`.
pub fn extrude(polygons: &[Polygon], z0: f64, z1: f64) -> Mesh {
    let mut verts: Vec<Vec3> = Vec::new();
    let mut tris: Vec<[u32; 3]> = Vec::new();
    for poly in polygons {
        let mut outer = clean_ring(&poly.outer);
        if ring_area(&outer) < 0.0 {
            outer.reverse();
        }
        let mut rings = vec![outer];
        for h in &poly.holes {
            let mut h = clean_ring(h);
            if ring_area(&h) > 0.0 {
                h.reverse();
            }
            if h.len() >= 3 {
                rings.push(h);
            }
        }
        if rings[0].len() < 3 {
            continue;
        }
        let base = verts.len() as u32;
        let n: u32 = rings.iter().map(|r| r.len() as u32).sum();
        for z in [z0, z1] {
            for p in rings.iter().flatten() {
                verts.push(Vec3::new(p.x as f32, p.y as f32, z as f32));
            }
        }
        for t in cap_triangles(&rings) {
            let (a, b, c) = (t[0] as u32, t[1] as u32, t[2] as u32);
            tris.push([base + n + a, base + n + b, base + n + c]); // top, +z
            tris.push([base + a, base + c, base + b]); // bottom, -z
        }
        let mut start = 0u32;
        for r in &rings {
            let len = r.len() as u32;
            for k in 0..len {
                let (a, b) = (start + k, start + (k + 1) % len);
                tris.push([base + a, base + b, base + n + b]);
                tris.push([base + a, base + n + b, base + n + a]);
            }
            start += len;
        }
    }
    Mesh::new(verts, tris)
}

/// Solid between two simple polygons with the same vertex count, `bottom` at `z0` and `top` at `z1`
/// (a frustum when `top` is a scaled copy of `bottom`).
pub fn loft(bottom: &[DVec2], top: &[DVec2], z0: f64, z1: f64) -> Mesh {
    assert_eq!(bottom.len(), top.len(), "loft rings need matching vertex counts");
    let n = bottom.len();
    let mut b = bottom.to_vec();
    let mut t = top.to_vec();
    if ring_area(&b) < 0.0 {
        b.reverse();
        t.reverse();
    }
    let mut verts: Vec<Vec3> = b.iter().map(|p| Vec3::new(p.x as f32, p.y as f32, z0 as f32)).collect();
    verts.extend(t.iter().map(|p| Vec3::new(p.x as f32, p.y as f32, z1 as f32)));
    let mut tris = Vec::new();
    for k in 0..n {
        let k2 = (k + 1) % n;
        tris.push([k as u32, k2 as u32, (n + k2) as u32]);
        tris.push([k as u32, (n + k2) as u32, (n + k) as u32]);
    }
    for tri in cap_triangles(&[b.clone()]) {
        tris.push([tri[0] as u32, tri[2] as u32, tri[1] as u32]); // bottom
    }
    for tri in cap_triangles(&[t.clone()]) {
        tris.push([(n + tri[0]) as u32, (n + tri[1]) as u32, (n + tri[2]) as u32]);
        // top
    }
    Mesh::new(verts, tris)
}

/// Solid of revolution about the Z axis. `profile` lists `(radius, z)` points from the bottom
/// to the top; the first and last point must have radius 0 (they sit on the axis).
pub fn lathe(profile: &[(f64, f64)], segments: usize) -> Mesh {
    assert!(profile.len() >= 3 && profile[0].0 == 0.0 && profile[profile.len() - 1].0 == 0.0, "profile must start and end on the axis");
    let m = profile.len();
    let mut verts = vec![Vec3::new(0.0, 0.0, profile[0].1 as f32)];
    for &(r, z) in &profile[1..m - 1] {
        for k in 0..segments {
            let a = std::f64::consts::TAU * k as f64 / segments as f64;
            verts.push(Vec3::new((r * a.cos()) as f32, (r * a.sin()) as f32, z as f32));
        }
    }
    let top = verts.len() as u32;
    verts.push(Vec3::new(0.0, 0.0, profile[m - 1].1 as f32));
    let ring = |j: usize, k: usize| (1 + (j - 1) * segments + k % segments) as u32;
    let rings = m - 2;
    let mut tris = Vec::new();
    for k in 0..segments {
        tris.push([0, ring(1, k + 1), ring(1, k)]);
        tris.push([top, ring(rings, k), ring(rings, k + 1)]);
        for j in 1..rings {
            let (a, b, c, d) = (ring(j, k), ring(j, k + 1), ring(j + 1, k + 1), ring(j + 1, k));
            tris.push([a, b, c]);
            tris.push([a, c, d]);
        }
    }
    Mesh::new(verts, tris)
}

/// Cylinder along +Z from `z0` to `z1` with optional 45° chamfers at the bottom and top.
pub fn cylinder(radius: f64, z0: f64, z1: f64, chamfer_bottom: f64, chamfer_top: f64, segments: usize) -> Mesh {
    let mut p = vec![(0.0, z0)];
    if chamfer_bottom > 0.0 {
        p.push((radius - chamfer_bottom, z0));
        p.push((radius, z0 + chamfer_bottom));
    } else {
        p.push((radius, z0));
    }
    if chamfer_top > 0.0 {
        p.push((radius, z1 - chamfer_top));
        p.push((radius - chamfer_top, z1));
    } else {
        p.push((radius, z1));
    }
    p.push((0.0, z1));
    lathe(&p, segments)
}

/// Axis-aligned box.
pub fn cuboid(min: DVec3, max: DVec3) -> Mesh {
    extrude(&[crate::poly2d::rect(DVec2::new(min.x, min.y), DVec2::new(max.x, max.y))], min.z, max.z)
}

/// A local right-handed frame used to place shapes built around the origin into the world.
#[derive(Clone, Copy, Debug)]
pub struct LocalFrame {
    pub origin: DVec3,
    pub x: DVec3,
    pub y: DVec3,
    pub z: DVec3,
}

impl LocalFrame {
    /// Frame on a plane (`z` = plane normal) positioned at the 2D point `p`, rotated by `angle` about the normal.
    pub fn on_plane(frame: &crate::plane::Frame, p: DVec2, angle: f64) -> Self {
        let (s, c) = angle.sin_cos();
        let x = frame.u * c + frame.v * s;
        Self { origin: frame.to_3d(p), x, y: frame.n.cross(x), z: frame.n }
    }

    pub fn point(&self, p: DVec3) -> DVec3 {
        self.origin + self.x * p.x + self.y * p.y + self.z * p.z
    }

    /// Places a locally-built mesh into the world. Orientation is preserved (the frame is right-handed).
    pub fn place(&self, mesh: &Mesh) -> Mesh {
        Mesh::new(mesh.vertices.iter().map(|v| self.point(v.as_dvec3()).as_vec3()).collect(), mesh.triangles.clone())
    }
}

/// Concatenates meshes into one (disjoint or touching solids), keeping all vertices.
pub fn merge(meshes: &[Mesh]) -> Mesh {
    let mut out = Mesh::default();
    for m in meshes {
        let base = out.vertices.len() as u32;
        out.vertices.extend(m.vertices.iter().copied());
        out.triangles.extend(m.triangles.iter().map(|t| [t[0] + base, t[1] + base, t[2] + base]));
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::poly2d::{circle, difference, rect};
    use std::f64::consts::PI;

    fn check(m: &Mesh, expected_volume: f64, rel: f64) {
        let s = m.stats();
        assert!(s.is_watertight(), "{s:?}");
        assert!(s.signed_volume > 0.0, "inward normals: {s:?}");
        assert!((s.signed_volume - expected_volume).abs() <= rel * expected_volume, "{} vs {expected_volume}", s.signed_volume);
    }

    #[test]
    fn extrude_rectangle_and_ring() {
        check(&extrude(&[rect(DVec2::new(0.0, 0.0), DVec2::new(10.0, 20.0))], 0.0, 5.0), 1000.0, 1e-6);
        let ring = difference(
            &[rect(DVec2::new(-10.0, -10.0), DVec2::new(10.0, 10.0))],
            &[Polygon { outer: circle(DVec2::ZERO, 4.0, 64), holes: vec![] }],
        );
        let expected = 400.0 - 0.5 * 64.0 * 16.0 * (std::f64::consts::TAU / 64.0).sin();
        check(&extrude(&ring, 0.0, 3.0), expected * 3.0, 1e-4);
    }

    #[test]
    fn extrude_removes_collinear_points() {
        // A rectangle with extra collinear vertices on every side must still triangulate without zero-area triangles.
        let pts: Vec<DVec2> = vec![(0.0, 0.0), (5.0, 0.0), (10.0, 0.0), (10.0, 5.0), (10.0, 10.0), (5.0, 10.0), (0.0, 10.0), (0.0, 5.0)]
            .into_iter()
            .map(|(x, y)| DVec2::new(x, y))
            .collect();
        let m = extrude(&[Polygon { outer: pts, holes: vec![] }], 0.0, 2.0);
        check(&m, 200.0, 1e-6);
        assert_eq!(m.stats().degenerate_triangles, 0);
    }

    #[test]
    fn frustum_volume_matches_the_formula() {
        let sq = |s: f64| vec![DVec2::new(-s, -s), DVec2::new(s, -s), DVec2::new(s, s), DVec2::new(-s, s)];
        let m = loft(&sq(5.0), &sq(2.0), 0.0, 6.0);
        let (a1, a2): (f64, f64) = (100.0, 16.0);
        check(&m, 6.0 / 3.0 * (a1 + a2 + (a1 * a2).sqrt()), 1e-6);
    }

    #[test]
    fn cylinder_and_chamfered_cylinder_volumes() {
        check(&cylinder(4.0, 0.0, 10.0, 0.0, 0.0, 96), PI * 16.0 * 10.0, 2e-3);
        let m = cylinder(4.0, 0.0, 10.0, 0.0, 1.0, 96);
        // Frustum on top: radius 4 -> 3 over 1 mm: V = pi*h/3*(R^2 + R r + r^2)
        let expected = PI * 16.0 * 9.0 + PI * 1.0 / 3.0 * (16.0 + 12.0 + 9.0);
        check(&m, expected, 2e-3);
    }

    #[test]
    fn local_frame_places_shapes_without_flipping_them() {
        let plane = crate::Plane::from_point_normal(DVec3::new(0.0, 0.0, 10.0), DVec3::new(1.0, 1.0, 1.0)).unwrap();
        let f = LocalFrame::on_plane(&plane.frame(), DVec2::new(3.0, -2.0), 0.7);
        let pin = f.place(&cylinder(2.0, 0.0, 8.0, 0.0, 0.5, 48));
        let s = pin.stats();
        assert!(s.is_watertight() && s.signed_volume > 0.0);
        // The pin's base centre lies on the plane, its axis along the normal.
        assert!(plane.signed_distance(f.origin).abs() < 1e-9);
        assert!((f.z - plane.n).length() < 1e-12);
        assert!((f.x.cross(f.y) - f.z).length() < 1e-12);
    }

    #[test]
    fn merge_keeps_both_solids_valid() {
        let m = merge(&[cuboid(DVec3::ZERO, DVec3::splat(2.0)), cuboid(DVec3::new(5.0, 0.0, 0.0), DVec3::new(7.0, 2.0, 2.0))]);
        check(&m, 16.0, 1e-9);
    }
}
