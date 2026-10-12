//! Finger (comb) joints for thin and curved walls.
//!
//! Pins, tabs and lips need wall material around them, so they cannot be used in a 3 mm skin. A finger joint
//! needs none: alternate strips along the seam are handed over from one piece to the other. In every even strip
//! the lower piece grows a "finger" that reaches `length` past the cut plane, taking over exactly the material
//! the upper piece had there; the upper piece loses that material plus a clearance gap. The fingers are made by
//! Boolean operations with the real pieces, so they follow whatever the wall does (curves, thin skins, holes),
//! lock the pieces against sliding along the seam, and stack with other joints.
//!
//! The result is verified: both pieces must stay closed solids and their volumes must change by exactly the
//! material that was moved.

use crate::shapes::{combine, extent_along, strip_polygon};
use glam::DVec2;
use openslicer_core::Mesh;
use openslicer_geom::boolean::{boolean, BoolOp};
use openslicer_geom::build::{self, LocalFrame};
use openslicer_geom::plane::Frame;
use openslicer_geom::region::Polygon;

#[derive(Clone, Copy, Debug)]
pub struct CombParams {
    /// Strip width along the seam, in mm.
    pub width: f64,
    /// How far fingers reach past the cut plane, in mm.
    pub length: f64,
    /// Gap between a finger and the recess that receives it, in mm.
    pub clearance: f64,
}

#[derive(Debug)]
pub struct CombResult {
    /// Lower piece with its fingers.
    pub lower: Mesh,
    /// Upper piece with its recesses.
    pub upper: Mesh,
    /// Volume moved from the upper to the lower piece (mm³).
    pub moved: f64,
    /// Volume removed from the upper piece, including the clearance (mm³).
    pub removed: f64,
    pub fingers: usize,
}

fn vol(m: &Mesh) -> f64 {
    if m.is_empty() {
        0.0
    } else {
        m.stats().signed_volume
    }
}

fn prisms(frame: &Frame, region: &[Polygon], angle: f64, ranges: &[(f64, f64)], z1: f64) -> Result<Mesh, String> {
    let place = LocalFrame::on_plane(frame, DVec2::ZERO, 0.0);
    let meshes: Vec<Mesh> =
        ranges.iter().map(|&(a, b)| place.place(&build::extrude(&strip_polygon(region, angle, a, b), -1.0, z1))).collect();
    combine(meshes)
}

/// Builds a finger joint across the interface `region` (in `frame`, whose normal points from `lower` to `upper`).
pub fn comb_joint(lower: &Mesh, upper: &Mesh, frame: &Frame, region: &[Polygon], angle: f64, p: &CombParams) -> Result<CombResult, String> {
    let (lo, hi) = extent_along(region, angle);
    let span = hi - lo;
    let strips = ((span / p.width).round() as usize).max(2);
    let w = span / strips as f64;
    if w <= 2.0 * p.clearance + 1.0 {
        return Err(format!("the seam is {span:.1} mm long, too short for finger strips with a {:.2} mm clearance", p.clearance));
    }
    // Strip boundaries: the outer ones reach past the seam (so they never graze its extreme vertices) and the
    // inner ones are nudged off the regular grid, so they do not land exactly on mesh vertices, which would
    // create near-coincident points that fail verification after export.
    let edge = |k: usize| -> f64 {
        if k == 0 {
            lo - 1.0
        } else if k == strips {
            hi + 1.0
        } else {
            lo + (k as f64 + 0.0173) * w
        }
    };
    let even: Vec<usize> = (0..strips).step_by(2).collect();
    let fingers: Vec<(f64, f64)> = even.iter().map(|&k| (edge(k), edge(k + 1))).collect();
    let cutters: Vec<(f64, f64)> = fingers.iter().map(|&(a, b)| (a - p.clearance, b + p.clearance)).collect();

    let finger_solid = prisms(frame, region, angle, &fingers, p.length)?;
    let cutter_solid = prisms(frame, region, angle, &cutters, p.length + p.clearance)?;

    let chunk = boolean(upper, &finger_solid, BoolOp::Intersection)
        .map_err(|e| format!("could not cut the fingers out of the upper piece: {e}"))?
        .0;
    if chunk.is_empty() {
        return Err("there is no material within reach of the fingers".into());
    }
    let moved = vol(&chunk);
    let removed_mesh = boolean(upper, &cutter_solid, BoolOp::Intersection).map_err(|e| format!("could not measure the recesses: {e}"))?.0;
    let removed = vol(&removed_mesh);

    let new_lower = boolean(lower, &chunk, BoolOp::Union).map_err(|e| format!("could not add the fingers to the lower piece: {e}"))?.0;
    let new_upper =
        boolean(upper, &cutter_solid, BoolOp::Difference).map_err(|e| format!("could not cut the recesses into the upper piece: {e}"))?.0;

    for (name, m, expected) in [("lower", &new_lower, vol(lower) + moved), ("upper", &new_upper, vol(upper) - removed)] {
        let s = m.stats();
        if !s.is_watertight() || s.signed_volume <= 0.0 {
            return Err(format!(
                "the {name} piece is not a valid solid after the finger joint ({} open edges, {} degenerate triangles)",
                s.boundary_edges, s.degenerate_triangles
            ));
        }
        if (s.signed_volume - expected).abs() > 1e-5 * expected.abs() + 1e-3 {
            return Err(format!("the {name} piece has volume {:.3} mm³ but {:.3} mm³ was expected", s.signed_volume, expected));
        }
    }
    Ok(CombResult { lower: new_lower, upper: new_upper, moved, removed, fingers: fingers.len() })
}

#[cfg(test)]
mod tests {
    use super::*;
    use glam::Vec3;
    use openslicer_geom::plane::Plane;
    use openslicer_geom::section::{planar_faces, Facing};

    fn halves(thickness: f32) -> (Mesh, Mesh) {
        // Two plates butted together at x = 0, each 60 mm long and 120 mm wide.
        let mut a = Mesh::cuboid(Vec3::new(60.0, 120.0, thickness));
        a.translate(Vec3::new(-30.0, 0.0, 0.0));
        let mut b = Mesh::cuboid(Vec3::new(60.0, 120.0, thickness));
        b.translate(Vec3::new(30.0, 0.0, 0.0));
        (a, b)
    }

    #[test]
    fn a_two_millimetre_wall_gets_a_verified_finger_joint() {
        let (a, b) = halves(2.0);
        // The cut face of the upper piece, with the frame whose normal points from the lower to the upper piece.
        let face = planar_faces(&b, &Plane::axis(0, 0.0), Facing::Against, 1e-3);
        let (frame, region) = (face.frame, face.polygons);
        let along = |ang: f64| {
            let (lo, hi) = extent_along(&region, ang);
            hi - lo
        };
        let angle = if along(0.0) >= along(std::f64::consts::FRAC_PI_2) { 0.0 } else { std::f64::consts::FRAC_PI_2 };
        let r = comb_joint(&a, &b, &frame, &region, angle, &CombParams { width: 20.0, length: 8.0, clearance: 0.2 }).unwrap();
        assert_eq!(r.fingers, 3);
        assert!((r.lower.stats().signed_volume - (vol(&a) + r.moved)).abs() < 1e-3);
        assert!(r.removed > r.moved, "the recess is larger than the finger by the clearance");
        // Each finger is about 20 x 2 x 8 mm (the strip edges are nudged off the regular grid).
        assert!((r.moved / (3.0 * 20.0 * 2.0 * 8.0) - 1.0).abs() < 0.02, "{}", r.moved);
        // Assembled, the pieces must not overlap.
        let overlap = boolean(&r.lower, &r.upper, BoolOp::Intersection).unwrap().0;
        assert!(vol(&overlap) < 1e-3, "pieces overlap by {} mm³", vol(&overlap));
    }
}
