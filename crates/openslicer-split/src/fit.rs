//! Build-volume fit checking and print-orientation search.
//!
//! A piece fits when some rotation brings its bounding box inside the usable
//! build volume. The search considers each world axis and each supplied
//! "preferred down" direction (for example the outward normal of a cut face, so
//! the flat cut face lies on the bed) as the vertical axis, and for each of them
//! rotates about it through the edge directions of the footprint's convex hull
//! plus a fixed angular sweep.

use glam::{DMat3, DVec2, DVec3};
use openslicer_core::Mesh;

/// Tolerance when comparing sizes against the build volume (mm).
pub const FIT_EPSILON: f64 = 1e-3;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Orientation {
    /// Rotation taking model coordinates to print coordinates (+Z up, rows are the print axes).
    pub rotation: DMat3,
    /// Bounding-box extents in print coordinates.
    pub size: DVec3,
    /// Model-space direction that becomes +Z.
    pub up: DVec3,
    /// True when the orientation came from a preferred-down direction (flat face on the bed).
    pub preferred: bool,
}

pub fn fits(size: DVec3, usable: DVec3) -> bool {
    size.x <= usable.x + FIT_EPSILON && size.y <= usable.y + FIT_EPSILON && size.z <= usable.z + FIT_EPSILON
}

/// Convex hull (counter-clockwise, no repeated points) by Andrew's monotone chain.
pub fn convex_hull(mut pts: Vec<DVec2>) -> Vec<DVec2> {
    pts.sort_by(|a, b| a.x.total_cmp(&b.x).then(a.y.total_cmp(&b.y)));
    pts.dedup();
    if pts.len() < 3 {
        return pts;
    }
    let cross = |o: DVec2, a: DVec2, b: DVec2| (a - o).perp_dot(b - o);
    let mut hull: Vec<DVec2> = Vec::with_capacity(pts.len());
    for &p in &pts {
        while hull.len() >= 2 && cross(hull[hull.len() - 2], hull[hull.len() - 1], p) <= 0.0 {
            hull.pop();
        }
        hull.push(p);
    }
    let lower = hull.len() + 1;
    for &p in pts.iter().rev().skip(1) {
        while hull.len() >= lower && cross(hull[hull.len() - 2], hull[hull.len() - 1], p) <= 0.0 {
            hull.pop();
        }
        hull.push(p);
    }
    hull.pop();
    hull
}

fn basis_for(up: DVec3) -> (DVec3, DVec3) {
    let helper = if up.z.abs() < 0.9 { DVec3::Z } else { DVec3::X };
    let e1 = helper.cross(up).normalize();
    (e1, up.cross(e1))
}

/// Footprint extents of `hull` rotated by `theta`.
fn extents(hull: &[DVec2], theta: f64) -> DVec2 {
    let (s, c) = theta.sin_cos();
    let (mut lo, mut hi) = (DVec2::splat(f64::MAX), DVec2::splat(f64::MIN));
    for p in hull {
        let q = DVec2::new(c * p.x + s * p.y, -s * p.x + c * p.y);
        lo = lo.min(q);
        hi = hi.max(q);
    }
    hi - lo
}

/// Finds the best orientation that fits `usable`, or `None` when no considered orientation fits.
/// Among fitting orientations, ones standing on a `preferred_down` face win, then the smallest bounding box.
pub fn best_orientation(mesh: &Mesh, usable: DVec3, preferred_down: &[DVec3]) -> Option<Orientation> {
    search(mesh, usable, preferred_down, true)
}

/// The orientation with the smallest bounding-box volume regardless of fit (used to describe a part that does not fit).
pub fn most_compact_orientation(mesh: &Mesh, preferred_down: &[DVec3]) -> Option<Orientation> {
    search(mesh, DVec3::splat(f64::INFINITY), preferred_down, false)
}

fn search(mesh: &Mesh, usable: DVec3, preferred_down: &[DVec3], respect_preference: bool) -> Option<Orientation> {
    if mesh.vertices.is_empty() {
        return None;
    }
    let verts: Vec<DVec3> = mesh.vertices.iter().map(|v| v.as_dvec3()).collect();
    let mut ups: Vec<(DVec3, bool)> = [DVec3::X, DVec3::Y, DVec3::Z].iter().map(|&a| (a, false)).collect();
    for d in preferred_down {
        let up = -d.normalize_or_zero();
        if up == DVec3::ZERO {
            continue;
        }
        // Merge with an existing candidate pointing the same way.
        if let Some(slot) = ups.iter_mut().find(|(u, _)| u.dot(up) > 1.0 - 1e-9) {
            slot.1 = true;
        } else {
            ups.push((up, true));
        }
    }

    let mut best: Option<(Orientation, (u8, f64, f64))> = None;
    for (up, preferred) in ups {
        let (e1, e2) = basis_for(up);
        let (mut zmin, mut zmax) = (f64::MAX, f64::MIN);
        let pts: Vec<DVec2> = verts
            .iter()
            .map(|v| {
                let z = v.dot(up);
                zmin = zmin.min(z);
                zmax = zmax.max(z);
                DVec2::new(v.dot(e1), v.dot(e2))
            })
            .collect();
        let height = zmax - zmin;
        if height > usable.z + FIT_EPSILON {
            continue;
        }
        let hull = convex_hull(pts);
        let mut angles: Vec<f64> = (0..36).map(|k| (k as f64) * std::f64::consts::PI / 36.0).collect();
        for i in 0..hull.len() {
            let d = hull[(i + 1) % hull.len()] - hull[i];
            if d.length_squared() > 0.0 {
                angles.push(d.y.atan2(d.x));
            }
        }
        for theta in angles {
            let ext = extents(&hull, theta);
            for (swap, size2) in [(false, ext), (true, DVec2::new(ext.y, ext.x))] {
                let size = DVec3::new(size2.x, size2.y, height);
                if !fits(size, usable) {
                    continue;
                }
                let phi = if swap { theta + std::f64::consts::FRAC_PI_2 } else { theta };
                let (s, c) = phi.sin_cos();
                let a = e1 * c + e2 * s;
                let b = -e1 * s + e2 * c;
                let rotation = DMat3::from_cols(DVec3::new(a.x, b.x, up.x), DVec3::new(a.y, b.y, up.y), DVec3::new(a.z, b.z, up.z));
                // Rows (a, b, up): print x = a·p, y = b·p, z = up·p.
                let volume = size.x * size.y * size.z;
                let key = (if preferred && respect_preference { 0u8 } else { 1u8 }, volume, height);
                // Prefer preferred faces, then the smallest bounding box; among equal boxes the lowest print height (parts lie flat).
                let better = |k: &(u8, f64, f64)| {
                    key.0 < k.0 || (key.0 == k.0 && (key.1 < k.1 * (1.0 - 1e-9) || (key.1 <= k.1 * (1.0 + 1e-9) && key.2 < k.2 - 1e-9)))
                };
                if best.as_ref().is_none_or(|(_, k)| better(k)) {
                    best = Some((Orientation { rotation, size, up, preferred }, key));
                }
            }
        }
    }
    best.map(|(o, _)| o)
}

/// Applies an orientation: rotates the mesh into print coordinates and moves its minimum corner to the origin.
pub fn apply_orientation(mesh: &Mesh, o: &Orientation) -> Mesh {
    let rotated: Vec<DVec3> = mesh.vertices.iter().map(|v| o.rotation * v.as_dvec3()).collect();
    let min = rotated.iter().fold(DVec3::splat(f64::MAX), |m, p| m.min(*p));
    Mesh::new(rotated.iter().map(|p| (*p - min).as_vec3()).collect(), mesh.triangles.clone())
}

#[cfg(test)]
mod tests {
    use super::*;
    use glam::Vec3;

    fn slab(x: f32, y: f32, z: f32) -> Mesh {
        Mesh::cuboid(Vec3::new(x, y, z))
    }

    #[test]
    fn hull_of_a_square_with_interior_points() {
        let pts = vec![
            DVec2::new(0.0, 0.0),
            DVec2::new(4.0, 0.0),
            DVec2::new(4.0, 4.0),
            DVec2::new(0.0, 4.0),
            DVec2::new(2.0, 2.0),
            DVec2::new(2.0, 0.0),
        ];
        let h = convex_hull(pts);
        assert_eq!(h.len(), 4);
        assert!(polygon_area(&h) > 15.99);
    }

    fn polygon_area(h: &[DVec2]) -> f64 {
        (0..h.len()).map(|i| h[i].perp_dot(h[(i + 1) % h.len()])).sum::<f64>() * 0.5
    }

    #[test]
    fn long_part_fits_by_lying_down_or_standing() {
        // 300 x 100 x 100 does not fit 200x200x350 as-is on X, but fits standing on its end (X as height).
        let m = slab(300.0, 100.0, 100.0);
        let o = best_orientation(&m, DVec3::new(200.0, 200.0, 350.0), &[]).unwrap();
        assert!(fits(o.size, DVec3::new(200.0, 200.0, 350.0)));
    }

    #[test]
    fn part_that_cannot_fit_is_reported_as_none() {
        let m = slab(400.0, 400.0, 400.0);
        assert!(best_orientation(&m, DVec3::new(250.0, 250.0, 250.0), &[]).is_none());
    }

    #[test]
    fn diagonal_rotation_makes_a_long_thin_part_fit() {
        // 270 x 20 x 10: too long for a 220 x 220 bed along an axis, but the bed diagonal is 311 mm.
        let m = slab(270.0, 20.0, 10.0);
        let o = best_orientation(&m, DVec3::new(220.0, 220.0, 250.0), &[]);
        // Standing it upright (270 high) does not fit 250 either, so only a diagonal footprint works.
        let o = o.expect("should fit on the diagonal");
        assert!(fits(o.size, DVec3::new(220.0, 220.0, 250.0)), "{:?}", o.size);
    }

    #[test]
    fn preferred_down_face_is_chosen_when_it_fits() {
        let m = slab(100.0, 100.0, 30.0);
        // Put the +x face down: that stands the part up on its 30 x 100 face.
        let o = best_orientation(&m, DVec3::new(200.0, 200.0, 200.0), &[DVec3::X]).unwrap();
        assert!(o.preferred);
        assert!((o.up - (-DVec3::X)).length() < 1e-9);
        let placed = apply_orientation(&m, &o);
        let b = placed.bounds().unwrap();
        assert!(b.min.length() < 1e-4);
        assert!((placed.stats().signed_volume - 300000.0).abs() < 1.0, "rotation must preserve volume and handedness");
    }

    #[test]
    fn orientation_matrix_is_a_proper_rotation() {
        let m = slab(60.0, 40.0, 20.0);
        for down in [DVec3::new(1.0, 2.0, 3.0), DVec3::new(0.0, 0.0, 1.0), DVec3::new(-1.0, 1.0, 0.0)] {
            let o = best_orientation(&m, DVec3::splat(500.0), &[down]).unwrap();
            assert!((o.rotation.determinant() - 1.0).abs() < 1e-9);
            let placed = apply_orientation(&m, &o);
            assert!((placed.stats().signed_volume - 48000.0).abs() < 0.5);
            let sz = placed.bounds().unwrap().size().as_dvec3();
            assert!((sz - o.size).length() < 1e-3, "{sz:?} vs {:?}", o.size);
        }
    }
}
