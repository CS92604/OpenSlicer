//! Moving automatic cut planes a few millimetres so they avoid tiny awkward islands in the cross-section.
//!
//! A plane that grazes the top of a bump or a curved surface leaves a small floating island in its section,
//! which becomes a sliver piece that is hard to print and hard to join. This looks for a nearby plane position
//! that has fewer such islands while still cutting the model. The model is never changed, only where the cut goes.

use openslicer_core::Mesh;
use openslicer_geom::region::ring_area;
use openslicer_geom::section::cross_section;
use openslicer_geom::Plane;
use openslicer_split::plan::CutPlan;

/// One plane that was moved.
#[derive(Clone, Debug)]
pub struct Nudge {
    /// Index of the cut in the plan.
    pub cut: usize,
    /// Distance the plane moved along its normal (mm).
    pub shift: f64,
    pub islands_before: usize,
    pub islands_after: usize,
}

/// Fractions of the allowed range that are tried on each side. They avoid round numbers, so a candidate is
/// unlikely to land exactly on a flat face of the model.
const TRIES: [f64; 4] = [0.19, 0.43, 0.67, 1.0];

/// Number of section polygons smaller than `min_area` (mm²), or `None` when the plane does not cut the model
/// or its section cannot be computed.
fn tiny_islands(mesh: &Mesh, plane: &Plane, min_area: f64) -> Option<usize> {
    let region = cross_section(mesh, plane).ok()?;
    if region.polygons.is_empty() {
        return None;
    }
    let net = |p: &openslicer_geom::region::Polygon| ring_area(&p.outer) + p.holes.iter().map(|h| ring_area(h)).sum::<f64>();
    Some(region.polygons.iter().filter(|p| net(p) < min_area).count())
}

/// Moves unscoped planes of `plan` by at most `range` mm where that reduces the number of tiny islands.
pub fn avoid_slivers(mesh: &Mesh, plan: &mut CutPlan, range: f64, min_area: f64) -> Vec<Nudge> {
    let mut moved = Vec::new();
    if range.is_nan() || range <= 0.0 {
        return moved;
    }
    for (i, cut) in plan.cuts.iter_mut().enumerate() {
        if !cut.scope.is_empty() {
            continue;
        }
        let Some(before) = tiny_islands(mesh, &cut.plane, min_area) else { continue };
        if before == 0 {
            continue;
        }
        let mut best: Option<(usize, f64)> = None;
        'search: for f in TRIES {
            for sign in [1.0, -1.0] {
                let shift = sign * f * range;
                let candidate = Plane { n: cut.plane.n, d: cut.plane.d + shift };
                if let Some(n) = tiny_islands(mesh, &candidate, min_area) {
                    if n < best.map_or(before, |b| b.0) {
                        best = Some((n, shift));
                        if n == 0 {
                            break 'search;
                        }
                    }
                }
            }
        }
        if let Some((after, shift)) = best {
            cut.plane.d += shift;
            moved.push(Nudge { cut: i, shift, islands_before: before, islands_after: after });
        }
    }
    moved
}

#[cfg(test)]
mod tests {
    use super::*;
    use glam::Vec3;
    use openslicer_geom::boolean::{boolean, BoolOp};

    /// A 60 × 60 × 10 slab (z from 0 to 10) with a 4 × 4 × 4 bump on top (z from 10 to 14).
    fn slab_with_bump() -> Mesh {
        let mut slab = Mesh::cuboid(Vec3::new(60.0, 60.0, 10.0));
        slab.translate(Vec3::new(0.0, 0.0, 5.0));
        let mut bump = Mesh::cuboid(Vec3::new(4.0, 4.0, 4.0));
        bump.translate(Vec3::new(0.0, 0.0, 11.9));
        boolean(&slab, &bump, BoolOp::Union).unwrap().0
    }

    #[test]
    fn a_plane_through_a_small_bump_moves_to_a_clean_section() {
        let mesh = slab_with_bump();
        let mut plan = CutPlan::single(Plane::axis(2, 12.0));
        assert_eq!(tiny_islands(&mesh, &plan.cuts[0].plane, 25.0), Some(1), "the bump is a 16 mm² island");
        let moved = avoid_slivers(&mesh, &mut plan, 4.0, 25.0);
        assert_eq!(moved.len(), 1);
        assert_eq!((moved[0].islands_before, moved[0].islands_after), (1, 0));
        let z = plan.cuts[0].plane.d;
        assert!(z < 10.0 && z > 8.0, "moved below the slab top, still inside the model: {z}");
        assert!(moved[0].shift.abs() <= 4.0 + 1e-9);
    }

    #[test]
    fn a_clean_plane_is_left_alone_and_a_plane_that_cannot_improve_stays() {
        let mesh = slab_with_bump();
        let mut plan = CutPlan::single(Plane::axis(2, 5.0));
        assert!(avoid_slivers(&mesh, &mut plan, 4.0, 25.0).is_empty());
        assert_eq!(plan.cuts[0].plane.d, 5.0);
        // A range too small to leave the bump cannot help, so the plane must not move.
        let mut stuck = CutPlan::single(Plane::axis(2, 12.0));
        assert!(avoid_slivers(&mesh, &mut stuck, 1.0, 25.0).is_empty());
        assert_eq!(stuck.cuts[0].plane.d, 12.0);
        // Range 0 disables the search.
        assert!(avoid_slivers(&mesh, &mut stuck, 0.0, 25.0).is_empty());
    }
}
