//! Cross-sections and planar faces.

use crate::cut::{chain_loops, cut_plane, group_loops, polygons_from, CutOptions};
use crate::plane::Plane;
use crate::region::PlanarRegion;
use glam::DVec2;
use openslicer_core::Mesh;
use rustc_hash::FxHashSet;

/// The cross-section of a closed mesh with a plane, as polygons with holes in the plane's frame.
/// Returns an error when the section cannot be closed (open input geometry).
pub fn cross_section(mesh: &Mesh, plane: &Plane) -> Result<PlanarRegion, String> {
    let r = cut_plane(mesh, plane, &CutOptions { verify: false, ..Default::default() });
    if r.report.open_chains > 0 {
        return Err(format!("the cross-section has {} open chain(s): the mesh is not closed at this plane", r.report.open_chains));
    }
    if let Some(p) = r.report.problems.first() {
        return Err(p.clone());
    }
    Ok(r.section)
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Facing {
    /// Faces whose outward normal points along the plane normal.
    Along,
    /// Faces whose outward normal points against the plane normal.
    Against,
}

/// The planar faces of `mesh` that lie on `plane` (every vertex within `tol`) and face the given way,
/// merged into polygons with holes in the plane's frame.
pub fn planar_faces(mesh: &Mesh, plane: &Plane, facing: Facing, tol: f64) -> PlanarRegion {
    let frame = plane.frame();
    let mut directed: FxHashSet<(u32, u32)> = FxHashSet::default();
    for t in &mesh.triangles {
        let p = [t[0], t[1], t[2]].map(|v| mesh.vertices[v as usize].as_dvec3());
        if p.iter().any(|&q| plane.signed_distance(q).abs() > tol) {
            continue;
        }
        let normal = (p[1] - p[0]).cross(p[2] - p[0]);
        let along = normal.dot(plane.n);
        let wanted = match facing {
            Facing::Along => along > 0.0,
            Facing::Against => along < 0.0,
        };
        if !wanted {
            continue;
        }
        for k in 0..3 {
            let (a, b) = (t[k], t[(k + 1) % 3]);
            directed.insert(if facing == Facing::Along { (a, b) } else { (b, a) });
        }
    }
    let mut edges: Vec<(u32, u32)> = directed.iter().filter(|&&(a, b)| !directed.contains(&(b, a))).copied().collect();
    edges.sort_unstable();
    let pos = |v: u32| -> DVec2 { frame.to_2d(mesh.vertices[v as usize].as_dvec3()) };
    let (loops, _open) = chain_loops(&edges, &pos);
    let polygons = match group_loops(&loops, &pos) {
        Ok(g) => polygons_from(&loops, &g, &pos),
        Err(_) => Vec::new(),
    };
    PlanarRegion { plane: *plane, frame, polygons }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::tests_support::*;
    use glam::{DVec3, Vec3};

    #[test]
    fn cross_section_of_a_tube_is_an_annulus() {
        let t = tube(20.0, 8.0, 60.0, 128);
        let s = cross_section(&t, &Plane::axis(2, 30.0)).unwrap();
        let expected = std::f64::consts::PI * (400.0 - 64.0);
        assert_eq!(s.polygons.len(), 1);
        assert_eq!(s.polygons[0].holes.len(), 1);
        assert!((s.area() - expected).abs() / expected < 0.01);
    }

    #[test]
    fn cross_section_outside_the_mesh_is_empty() {
        let s = cross_section(&tube(20.0, 8.0, 60.0, 32), &Plane::axis(2, 100.0)).unwrap();
        assert!(s.is_empty());
    }

    #[test]
    fn open_mesh_section_is_an_error_not_a_wrong_answer() {
        let mut m = Mesh::cuboid(Vec3::splat(10.0));
        m.translate(Vec3::splat(5.0));
        m.triangles.truncate(10); // drops the -x face: a z-plane now crosses the open side
        let r = cross_section(&m, &Plane::axis(2, 5.0));
        assert!(r.is_err(), "{r:?}");
        // A plane parallel to the missing face still sections the remaining walls into a closed loop.
        assert!(cross_section(&m, &Plane::axis(0, 5.0)).is_ok());
    }

    #[test]
    fn planar_faces_find_the_cap_of_each_half() {
        let mut m = Mesh::cuboid(Vec3::splat(10.0));
        m.translate(Vec3::splat(5.0));
        let plane = Plane::axis(2, 5.0);
        let r = cut_plane(&m, &plane, &CutOptions::default());
        let top = planar_faces(&r.above, &plane, Facing::Against, 1e-4); // the upper half's cap faces down
        let bottom = planar_faces(&r.below, &plane, Facing::Along, 1e-4);
        assert!((top.area() - 100.0).abs() < 1e-6, "{}", top.area());
        assert!((bottom.area() - 100.0).abs() < 1e-6);
        assert!(planar_faces(&r.above, &plane, Facing::Along, 1e-4).is_empty());
    }

    #[test]
    fn planar_faces_keep_holes() {
        let t = tube(20.0, 8.0, 60.0, 64);
        let r = cut_plane(&t, &Plane::axis(2, 30.0), &CutOptions::default());
        let f = planar_faces(&r.below, &Plane::axis(2, 30.0), Facing::Along, 1e-4);
        assert_eq!(f.polygons.len(), 1);
        assert_eq!(f.polygons[0].holes.len(), 1);
        let _ = DVec3::ZERO;
    }
}
