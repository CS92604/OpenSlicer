//! Cropping a closed mesh to an axis-aligned box.
//!
//! Used to run expensive local checks (connector fit, wall thickness) on a small closed solid instead of
//! a whole piece: the Boolean checks that follow then cost almost nothing.

use crate::cut::{cut_plane, CutOptions};
use crate::plane::Plane;
use glam::Vec3;
use openslicer_core::{Aabb, Mesh};

/// The part of `mesh` inside `window`, as a closed solid. Returns `Ok(None)` when no material is inside the
/// window, and an error when the local solid cannot be closed (the mesh is open near the window).
pub fn crop_to_box(mesh: &Mesh, window: &Aabb) -> Result<Option<Mesh>, String> {
    if mesh.is_empty() {
        return Ok(None);
    }
    // Cutting the whole piece (rather than pre-selecting nearby triangles) keeps the mesh closed at every step,
    // so each cut is exact; every cut also shrinks what the next one has to process.
    let mut cur = mesh.clone();
    let opts = CutOptions { verify: false, ..Default::default() };
    for axis in 0..3 {
        for (pos, keep_above) in [(window.min[axis] as f64, true), (window.max[axis] as f64, false)] {
            let r = cut_plane(&cur, &Plane::axis(axis, pos), &opts);
            if r.report.open_chains > 0 {
                return Err("the mesh is not closed near this location".into());
            }
            cur = if keep_above { r.above } else { r.below };
            if cur.is_empty() {
                return Ok(None);
            }
        }
    }
    let s = cur.stats();
    if !s.is_watertight() {
        return Err(format!("the local solid is not watertight ({} open edges)", s.boundary_edges));
    }
    Ok(Some(cur))
}

/// Box of the given half size around a bounding box, grown by `pad` on every side.
pub fn padded(b: &Aabb, pad: f32) -> Aabb {
    Aabb { min: b.min - Vec3::splat(pad), max: b.max + Vec3::splat(pad) }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::tests_support::*;

    #[test]
    fn crop_of_a_cube_corner_has_the_right_volume() {
        let mut m = Mesh::cuboid(Vec3::splat(100.0));
        m.translate(Vec3::splat(50.0));
        let w = Aabb { min: Vec3::new(80.0, 80.0, 80.0), max: Vec3::new(120.0, 120.0, 120.0) };
        let c = crop_to_box(&m, &w).unwrap().unwrap();
        assert!((c.stats().signed_volume - 20.0 * 20.0 * 20.0).abs() < 1e-2);
    }

    #[test]
    fn crop_outside_the_mesh_is_none() {
        let m = Mesh::cuboid(Vec3::splat(10.0));
        let w = Aabb { min: Vec3::splat(50.0), max: Vec3::splat(60.0) };
        assert!(crop_to_box(&m, &w).unwrap().is_none());
    }

    #[test]
    fn crop_of_a_curved_shell_wall_is_a_closed_solid() {
        let shell = sphere_shell(50.0, 47.0, 96, 48);
        let w = Aabb { min: Vec3::new(40.0, -5.0, -5.0), max: Vec3::new(60.0, 5.0, 5.0) };
        let c = crop_to_box(&shell, &w).unwrap().unwrap();
        let s = c.stats();
        assert!(s.is_watertight() && s.signed_volume > 0.0);
        // ~3 mm wall over a ~10x10 mm patch (slightly larger area on the curved surface)
        assert!(s.signed_volume > 200.0 && s.signed_volume < 400.0, "{}", s.signed_volume);
    }

    #[test]
    fn crop_of_the_whole_mesh_returns_it_unchanged() {
        let m = torus(30.0, 10.0, 48, 24);
        let w = Aabb { min: Vec3::splat(-100.0), max: Vec3::splat(100.0) };
        let c = crop_to_box(&m, &w).unwrap().unwrap();
        assert!((c.stats().signed_volume - m.stats().signed_volume).abs() / m.stats().signed_volume < 1e-9);
    }

    #[test]
    fn crop_of_an_open_mesh_reports_it() {
        let mut m = Mesh::cuboid(Vec3::splat(10.0));
        m.triangles.truncate(10); // drops the -x face at x = -5
                                  // A window that never reaches the hole sees a perfectly good closed local solid ...
        let away = Aabb { min: Vec3::splat(-3.0), max: Vec3::splat(3.0) };
        assert!(crop_to_box(&m, &away).unwrap().is_some());
        // ... and one that spans the hole is refused instead of returning a wrong answer.
        let across = Aabb { min: Vec3::new(-8.0, -3.0, -3.0), max: Vec3::new(0.0, 3.0, 3.0) };
        assert!(crop_to_box(&m, &across).is_err());
    }
}
