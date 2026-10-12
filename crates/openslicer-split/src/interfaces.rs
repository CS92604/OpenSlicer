//! Interfaces: where two pieces meet along a cut, and over what area.
//!
//! Interfaces are found geometrically: for each cut plane, the flat faces that
//! lie on the plane are extracted from the pieces on either side and intersected.
//! This works for any set of pieces, including ones cut in several stages.

use crate::exec::Piece;
use openslicer_geom::poly2d;
use openslicer_geom::region::{PlanarRegion, Polygon};
use openslicer_geom::section::{planar_faces, Facing};
use openslicer_geom::Plane;

#[derive(Clone, Debug)]
pub struct Interface {
    /// Piece on the negative side of `plane` (its face points along `plane.n`).
    pub a: usize,
    /// Piece on the positive side of `plane` (its face points against `plane.n`).
    pub b: usize,
    /// Cut plane; the normal points from `a` towards `b`.
    pub plane: Plane,
    /// Overlap of the two faces, in the plane's frame.
    pub region: PlanarRegion,
    pub area: f64,
}

/// Minimum shared area (mm²) for two pieces to count as adjacent.
pub const MIN_INTERFACE_AREA: f64 = 1e-3;

fn same_plane(p: &Plane, q: &Plane, tol: f64) -> bool {
    (p.n.dot(q.n) > 1.0 - 1e-9 && (p.d - q.d).abs() <= tol) || (p.n.dot(q.n) < -1.0 + 1e-9 && (p.d + q.d).abs() <= tol)
}

/// Finds all interfaces across `planes`. `a` / `b` hold piece ids.
pub fn find_interfaces(pieces: &[Piece], planes: &[Plane], tol: f64) -> Vec<Interface> {
    // Distinct planes only.
    let mut unique: Vec<Plane> = Vec::new();
    for p in planes {
        if !unique.iter().any(|q| same_plane(p, q, tol)) {
            unique.push(*p);
        }
    }
    let mut out = Vec::new();
    for plane in unique {
        // Signed distances of the piece from the plane. The box corners give a quick rejection; the
        // vertices give the exact range, which matters for oblique planes (a rotated grid) because a
        // box corner can lie well beyond a plane that the piece itself only touches.
        let extent = |p: &Piece| {
            let (mut lo, mut hi) = (f64::MAX, f64::MIN);
            for i in 0..8 {
                let c = glam::DVec3::new(
                    if i & 1 == 0 { p.bounds.min.x } else { p.bounds.max.x } as f64,
                    if i & 2 == 0 { p.bounds.min.y } else { p.bounds.max.y } as f64,
                    if i & 4 == 0 { p.bounds.min.z } else { p.bounds.max.z } as f64,
                );
                let s = plane.signed_distance(c);
                lo = lo.min(s);
                hi = hi.max(s);
            }
            if lo > tol || hi < -tol {
                return (lo, hi);
            }
            let (mut lo, mut hi) = (f64::MAX, f64::MIN);
            for v in &p.mesh.vertices {
                let s = plane.signed_distance(v.as_dvec3());
                lo = lo.min(s);
                hi = hi.max(s);
            }
            (lo, hi)
        };
        let mut below: Vec<(usize, Vec<Polygon>)> = Vec::new();
        let mut above: Vec<(usize, Vec<Polygon>)> = Vec::new();
        for p in pieces {
            let (lo, hi) = extent(p);
            if hi <= tol && hi >= -tol {
                let f = planar_faces(&p.mesh, &plane, Facing::Along, tol);
                if !f.is_empty() {
                    below.push((p.id, f.polygons));
                }
            } else if lo >= -tol && lo <= tol {
                let f = planar_faces(&p.mesh, &plane, Facing::Against, tol);
                if !f.is_empty() {
                    above.push((p.id, f.polygons));
                }
            }
        }
        for (a, pa) in &below {
            for (b, pb) in &above {
                let overlap = poly2d::intersect(pa, pb);
                let area = poly2d::area(&overlap);
                if area > MIN_INTERFACE_AREA {
                    out.push(Interface {
                        a: *a,
                        b: *b,
                        plane,
                        region: PlanarRegion { plane, frame: plane.frame(), polygons: overlap },
                        area,
                    });
                }
            }
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::exec::{execute_plan, SplitOptions};
    use crate::layout::*;
    use glam::Vec3;
    use openslicer_core::control::Control;
    use openslicer_core::Mesh;

    #[test]
    fn grid_pieces_touch_their_neighbours_with_the_right_areas() {
        let mut m = Mesh::cuboid(Vec3::new(200.0, 100.0, 50.0));
        m.translate(Vec3::new(100.0, 50.0, 25.0));
        let plan = grid_plan(extents(&m, &GridFrame::WORLD), &GridFrame::WORLD, [2, 2, 1]);
        let r = execute_plan(&m, &plan, &SplitOptions { tolerance: None, separate_solids: true }, &Control::none()).unwrap();
        assert!(r.report.is_valid());
        let ifs = find_interfaces(&r.pieces, &r.planes, 1e-3);
        // 2x2 grid: 4 face-adjacent pairs, each 50x100 or 100x50 = 5000 / 2500... areas: x-cut faces are 50(y)x50(z), y-cut faces 100(x)x50(z)
        assert_eq!(ifs.len(), 4, "{:?}", ifs.iter().map(|i| (i.a, i.b, i.area)).collect::<Vec<_>>());
        let mut areas: Vec<f64> = ifs.iter().map(|i| i.area).collect();
        areas.sort_by(|a, b| a.total_cmp(b));
        // x-cut at x=100 is split by the y cut into two halves of 50 x 50 = 2500 each; y-cut at y=50 splits into 100x50 halves = 5000 each? (each y-cut half spans x 100 wide)
        assert!((areas[0] - 2500.0).abs() < 1.0 && (areas[1] - 2500.0).abs() < 1.0);
        assert!((areas[2] - 2500.0 * 2.0).abs() < 1.0 && (areas[3] - 5000.0).abs() < 1.0, "{areas:?}");
    }

    #[test]
    fn interface_normal_points_from_a_to_b() {
        let mut m = Mesh::cuboid(Vec3::new(20.0, 20.0, 20.0));
        m.translate(Vec3::splat(10.0));
        let r = execute_plan(&m, &crate::plan::CutPlan::single(Plane::axis(2, 10.0)), &SplitOptions::default(), &Control::none()).unwrap();
        let ifs = find_interfaces(&r.pieces, &r.planes, 1e-3);
        assert_eq!(ifs.len(), 1);
        let i = &ifs[0];
        let pa = r.pieces.iter().find(|p| p.id == i.a).unwrap();
        let pb = r.pieces.iter().find(|p| p.id == i.b).unwrap();
        assert!(pa.bounds.center().z < pb.bounds.center().z);
        assert!((i.area - 400.0).abs() < 1e-6);
    }

    #[test]
    fn oblique_cut_planes_still_produce_an_interface() {
        let mut m = Mesh::cuboid(Vec3::new(100.0, 100.0, 20.0));
        m.translate(Vec3::new(50.0, 50.0, 10.0));
        let normal = glam::DVec3::new(1.0, 1.0, 0.0).normalize();
        let plane = Plane::from_point_normal(glam::DVec3::new(50.0, 50.0, 0.0), normal).unwrap();
        let r = execute_plan(&m, &crate::plan::CutPlan::single(plane), &SplitOptions::default(), &Control::none()).unwrap();
        assert_eq!(r.pieces.len(), 2);
        let ifs = find_interfaces(&r.pieces, &r.planes, 1e-3);
        assert_eq!(ifs.len(), 1, "the two halves share a 141 × 20 mm face");
        assert!((ifs[0].area - 100.0 * 2f64.sqrt() * 20.0).abs() < 0.5, "{}", ifs[0].area);
    }

    #[test]
    fn pieces_that_only_share_an_edge_are_not_adjacent() {
        // Two cubes touching only along an edge.
        let a = {
            let mut m = Mesh::cuboid(Vec3::splat(10.0));
            m.translate(Vec3::splat(5.0));
            m
        };
        let b = {
            let mut m = Mesh::cuboid(Vec3::splat(10.0));
            m.translate(Vec3::new(15.0, 15.0, 5.0));
            m
        };
        let mut pa = Piece::new(a).unwrap();
        let mut pb = Piece::new(b).unwrap();
        pa.id = 1;
        pb.id = 2;
        let ifs = find_interfaces(&[pa, pb], &[Plane::axis(0, 10.0)], 1e-3);
        assert!(ifs.is_empty());
    }
}
