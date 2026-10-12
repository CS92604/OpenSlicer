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

/// Solid under a height field. `heights` holds `nx * ny` samples (row-major, x fastest) at spacing `dx`, `dy`
/// starting at the origin; the solid extends from `floor` up to each height. Heights must exceed `floor`.
/// The top surface is triangulated with alternating diagonals, the walls follow the border, and the bottom is a
/// fan from its centre, so the mesh is watertight with outward normals.
pub fn heightfield(heights: &[f32], nx: usize, ny: usize, dx: f64, dy: f64, floor: f64) -> Result<Mesh, String> {
    if nx < 2 || ny < 2 || heights.len() != nx * ny {
        return Err("a height field needs at least 2 x 2 samples".into());
    }
    if heights.iter().any(|h| !h.is_finite() || (*h as f64) <= floor) {
        return Err("every height must be finite and above the floor".into());
    }
    let mut verts: Vec<Vec3> = Vec::with_capacity(nx * ny + 2 * (nx + ny));
    for j in 0..ny {
        for i in 0..nx {
            verts.push(Vec3::new((i as f64 * dx) as f32, (j as f64 * dy) as f32, heights[j * nx + i]));
        }
    }
    let top = |i: usize, j: usize| (j * nx + i) as u32;
    let mut tris: Vec<[u32; 3]> = Vec::with_capacity(nx * ny * 2);
    for j in 0..ny - 1 {
        for i in 0..nx - 1 {
            let (a, b, c, d) = (top(i, j), top(i + 1, j), top(i + 1, j + 1), top(i, j + 1));
            if (i + j) % 2 == 0 {
                tris.push([a, b, c]);
                tris.push([a, c, d]);
            } else {
                tris.push([a, b, d]);
                tris.push([b, c, d]);
            }
        }
    }
    // Border walks (counter-clockwise seen from above): bottom row, right column, top row reversed, left column reversed.
    let mut border: Vec<(usize, usize)> = Vec::new();
    border.extend((0..nx - 1).map(|i| (i, 0)));
    border.extend((0..ny - 1).map(|j| (nx - 1, j)));
    border.extend((1..nx).rev().map(|i| (i, ny - 1)));
    border.extend((1..ny).rev().map(|j| (0, j)));
    let base = verts.len() as u32;
    for &(i, j) in &border {
        verts.push(Vec3::new((i as f64 * dx) as f32, (j as f64 * dy) as f32, floor as f32));
    }
    let n = border.len() as u32;
    for k in 0..n {
        let k2 = (k + 1) % n;
        let (ta, tb) = (top(border[k as usize].0, border[k as usize].1), top(border[k2 as usize].0, border[k2 as usize].1));
        tris.push([base + k, base + k2, tb]);
        tris.push([base + k, tb, ta]);
    }
    let centre = verts.len() as u32;
    verts.push(Vec3::new((((nx - 1) as f64) * dx * 0.5) as f32, (((ny - 1) as f64) * dy * 0.5) as f32, floor as f32));
    for k in 0..n {
        tris.push([centre, base + (k + 1) % n, base + k]); // faces down
    }
    Ok(Mesh::new(verts, tris))
}

/// Replaces sharp corners of a polyline with circular fillets of (at most) `radius`, so a tube swept along it
/// cannot pinch. Gentle turns (under 10 degrees, as on finely sampled curves) are kept as they are.
fn fillet_path(pts: &[DVec3], radius: f64, closed: bool) -> Vec<DVec3> {
    let n = pts.len();
    let mut out: Vec<DVec3> = Vec::with_capacity(n * 4);
    let range: Vec<usize> = if closed { (0..n).collect() } else { (1..n - 1).collect() };
    if !closed {
        out.push(pts[0]);
    }
    for i in if closed { 0..n } else { 1..n - 1 } {
        let (p, c, q) = (pts[(i + n - 1) % n], pts[i], pts[(i + 1) % n]);
        let (d0, d1) = ((c - p).normalize(), (q - c).normalize());
        let turn = d0.dot(d1).clamp(-1.0, 1.0).acos();
        if turn < 10f64.to_radians() || turn > std::f64::consts::PI - 1e-3 {
            out.push(c);
            continue;
        }
        // Tangent length for a fillet of this radius, limited so neighbouring fillets do not overlap.
        let limit = 0.45 * (c - p).length().min((q - c).length());
        let t = (radius * (turn / 2.0).tan()).min(limit);
        let r = t / (turn / 2.0).tan();
        let (a, b) = (c - d0 * t, c + d1 * t);
        let centre = a + (d1 - d0 * d0.dot(d1)).normalize() * r;
        let steps = ((turn / 10f64.to_radians()).ceil() as usize).max(2);
        let (va, vb) = ((a - centre).normalize(), (b - centre).normalize());
        let axis = va.cross(vb);
        let angle = va.dot(vb).clamp(-1.0, 1.0).acos();
        for k in 0..=steps {
            let f = angle * k as f64 / steps as f64;
            let rot = va * f.cos() + axis.cross(va).normalize_or_zero() * f.sin();
            out.push(centre + rot * r);
        }
    }
    if !closed {
        out.push(pts[n - 1]);
    }
    let _ = range;
    out
}

/// Tube of the given radius swept along a polyline, with flat end caps (or closed into a loop).
/// Sharp corners are rounded with fillets of about three tube radii (less where segments are short).
pub fn tube(path: &[DVec3], radius: f64, segments: usize, closed: bool) -> Result<Mesh, String> {
    let mut pts: Vec<DVec3> = Vec::with_capacity(path.len());
    for &p in path {
        if pts.last().is_none_or(|q| (*q - p).length() > 1e-6) {
            pts.push(p);
        }
    }
    if closed && pts.len() > 1 && (pts[0] - pts[pts.len() - 1]).length() <= 1e-6 {
        pts.pop();
    }
    if pts.len() < 2 || (closed && pts.len() < 3) || segments < 3 || radius <= 0.0 {
        return Err("a tube needs at least two path points, three segments and a positive radius".into());
    }
    let pts = fillet_path(&pts, radius * 3.0, closed);
    let n = pts.len();
    let dir = |i: usize| -> DVec3 { (pts[(i + 1) % n] - pts[i]).normalize() };
    let tangents: Vec<DVec3> = (0..n)
        .map(|i| {
            if closed {
                (dir((i + n - 1) % n) + dir(i)).try_normalize().unwrap_or(dir(i))
            } else if i == 0 {
                dir(0)
            } else if i == n - 1 {
                dir(n - 2)
            } else {
                (dir(i - 1) + dir(i)).try_normalize().unwrap_or(dir(i))
            }
        })
        .collect();
    // Parallel transport of a reference normal along the path.
    let mut nrm = {
        let helper = if tangents[0].z.abs() < 0.9 { DVec3::Z } else { DVec3::X };
        tangents[0].cross(helper).normalize()
    };
    let mut normals = Vec::with_capacity(n);
    for t in &tangents {
        nrm = (nrm - *t * nrm.dot(*t)).try_normalize().unwrap_or(nrm);
        normals.push(nrm);
    }
    let ring = |i: usize| -> Vec<DVec3> {
        let (t, a) = (tangents[i], normals[i]);
        let b = t.cross(a);
        (0..segments)
            .map(|k| {
                let ang = std::f64::consts::TAU * k as f64 / segments as f64;
                pts[i] + (a * ang.cos() + b * ang.sin()) * radius
            })
            .collect()
    };
    let rings: Vec<Vec<DVec3>> = (0..n).map(ring).collect();
    // A closed tube must join its last ring to its first; a twisted transported frame needs the best rotation.
    let mut closing_shift = 0usize;
    if closed {
        let (last, first) = (&rings[n - 1], &rings[0]);
        closing_shift = (0..segments)
            .min_by(|&s1, &s2| {
                let cost = |s: usize| (0..segments).map(|k| (last[k] - first[(k + s) % segments]).length_squared()).sum::<f64>();
                cost(s1).total_cmp(&cost(s2))
            })
            .unwrap();
    }
    let mut verts: Vec<Vec3> = rings.iter().flatten().map(|p| p.as_vec3()).collect();
    let id = |ring: usize, k: usize| (ring * segments + k % segments) as u32;
    let mut tris = Vec::new();
    let spans = if closed { n } else { n - 1 };
    for i in 0..spans {
        let j = (i + 1) % n;
        let wrap = closed && j == 0;
        for k in 0..segments {
            let (a, b) = (id(i, k), id(i, k + 1));
            let (c, d) = if wrap { (id(0, k + closing_shift + 1), id(0, k + closing_shift)) } else { (id(j, k + 1), id(j, k)) };
            tris.push([a, b, c]);
            tris.push([a, c, d]);
        }
    }
    if !closed {
        let c0 = verts.len() as u32;
        verts.push(pts[0].as_vec3());
        let c1 = verts.len() as u32;
        verts.push(pts[n - 1].as_vec3());
        for k in 0..segments {
            tris.push([c0, id(0, k + 1), id(0, k)]);
            tris.push([c1, id(n - 1, k), id(n - 1, k + 1)]);
        }
    }
    let mut m = Mesh::new(verts, tris);
    // Orient outward: the winding depends on the path's handedness.
    if m.stats().signed_volume < 0.0 {
        for t in &mut m.triangles {
            t.swap(1, 2);
        }
    }
    Ok(m)
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
    use std::f64::consts::{PI, TAU};

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
    fn heightfield_solid_is_watertight_with_the_right_volume() {
        // 5 x 4 samples, spacing 2 mm: footprint 8 x 6 mm, flat 3 mm high over a floor at 0 -> 144 mm3.
        let flat = heightfield(&[3.0; 20], 5, 4, 2.0, 2.0, 0.0).unwrap();
        check(&flat, 8.0 * 6.0 * 3.0, 1e-9);
        // A linear ramp in x (heights 1..5 over 4 intervals): mean height 3.
        let ramp: Vec<f32> = (0..4).flat_map(|_| (0..5).map(|i| 1.0 + i as f32)).collect();
        let m = heightfield(&ramp, 5, 4, 2.0, 2.0, 0.0).unwrap();
        check(&m, 8.0 * 6.0 * 3.0, 1e-6);
        assert!(heightfield(&[0.0; 20], 5, 4, 2.0, 2.0, 0.0).is_err(), "heights at the floor are rejected");
        assert!(heightfield(&[1.0; 3], 5, 4, 2.0, 2.0, 0.0).is_err());
    }

    #[test]
    fn heightfield_with_noise_stays_a_valid_solid() {
        let mut rng = 12345u64;
        let h: Vec<f32> = (0..40 * 30)
            .map(|_| {
                rng = rng.wrapping_mul(6364136223846793005).wrapping_add(1);
                1.0 + ((rng >> 40) as f32 / 16777216.0) * 4.0
            })
            .collect();
        let m = heightfield(&h, 40, 30, 0.5, 0.5, 0.0).unwrap();
        let s = m.stats();
        assert!(s.is_watertight() && s.signed_volume > 0.0, "{s:?}");
    }

    #[test]
    fn straight_tube_has_the_cylinder_volume() {
        let t = tube(&[DVec3::ZERO, DVec3::new(0.0, 0.0, 20.0)], 2.0, 64, false).unwrap();
        check(&t, PI * 4.0 * 20.0, 2e-3);
    }

    #[test]
    fn bent_and_closed_tubes_are_valid_solids() {
        let bent =
            tube(&[DVec3::ZERO, DVec3::new(10.0, 0.0, 0.0), DVec3::new(10.0, 10.0, 0.0), DVec3::new(10.0, 10.0, 10.0)], 1.0, 24, false)
                .unwrap();
        let s = bent.stats();
        assert!(s.is_watertight() && s.signed_volume > 0.0, "{s:?}");
        // A closed square loop.
        let sq = [DVec3::new(0.0, 0.0, 0.0), DVec3::new(20.0, 0.0, 0.0), DVec3::new(20.0, 20.0, 0.0), DVec3::new(0.0, 20.0, 0.0)];
        let ring = tube(&sq, 1.0, 24, true).unwrap();
        let s = ring.stats();
        assert!(s.is_watertight() && s.signed_volume > 0.0, "{s:?}");
        // Volume ~ cross-section area times centreline length: rounded corners shorten the path slightly.
        assert!(s.signed_volume > PI * 70.0 && s.signed_volume < PI * 80.0, "{}", s.signed_volume);
        // A circular loop.
        let circle: Vec<DVec3> =
            (0..64).map(|k| DVec3::new(30.0 * (TAU * k as f64 / 64.0).cos(), 30.0 * (TAU * k as f64 / 64.0).sin(), 5.0)).collect();
        let ring = tube(&circle, 1.5, 16, true).unwrap();
        let s = ring.stats();
        assert!(s.is_watertight() && s.signed_volume > 0.0, "{s:?}");
        assert!(tube(&[DVec3::ZERO], 1.0, 8, false).is_err());
    }

    #[test]
    fn merge_keeps_both_solids_valid() {
        let m = merge(&[cuboid(DVec3::ZERO, DVec3::splat(2.0)), cuboid(DVec3::new(5.0, 0.0, 0.0), DVec3::new(7.0, 2.0, 2.0))]);
        check(&m, 16.0, 1e-9);
    }
}
