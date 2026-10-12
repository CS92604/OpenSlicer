//! Cut layouts: planar, multi-planar, modular (periodic), grid and staggered (brick) patterns.
//!
//! A layout is defined in a [`GridFrame`]: a right-handed frame whose Z axis is the model's Z axis and
//! whose horizontal axes are rotated by `yaw`. Rotating the frame lets a grid follow the main direction
//! of an elongated part (which is how the automatic tiler reduces piece counts).

use crate::plan::{CutPlan, PlannedCut};
use glam::DVec3;
use openslicer_core::Mesh;
use openslicer_geom::Plane;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct GridFrame {
    /// Rotation of the horizontal axes about +Z, in radians.
    pub yaw: f64,
}

impl GridFrame {
    pub const WORLD: GridFrame = GridFrame { yaw: 0.0 };

    pub fn axes(&self) -> [DVec3; 3] {
        let (s, c) = self.yaw.sin_cos();
        [DVec3::new(c, s, 0.0), DVec3::new(-s, c, 0.0), DVec3::Z]
    }
}

/// (min, max) of the mesh along each frame axis.
pub fn extents(mesh: &Mesh, frame: &GridFrame) -> [(f64, f64); 3] {
    let axes = frame.axes();
    let mut e = [(f64::MAX, f64::MIN); 3];
    for v in &mesh.vertices {
        let p = v.as_dvec3();
        for (i, a) in axes.iter().enumerate() {
            let s = p.dot(*a);
            e[i].0 = e[i].0.min(s);
            e[i].1 = e[i].1.max(s);
        }
    }
    e
}

fn axis_plane(frame: &GridFrame, axis: usize, pos: f64) -> Plane {
    let n = frame.axes()[axis];
    Plane { n, d: pos }
}

const AXIS_NAMES: [&str; 3] = ["X", "Y", "Z"];

/// Equal division of each axis into `divisions[i]` cells (1 means no cuts on that axis).
pub fn grid_plan(ext: [(f64, f64); 3], frame: &GridFrame, divisions: [usize; 3]) -> CutPlan {
    let mut plan = CutPlan::default();
    for axis in 0..3 {
        let n = divisions[axis].max(1);
        let (lo, hi) = ext[axis];
        for k in 1..n {
            let pos = lo + (hi - lo) * k as f64 / n as f64;
            plan.cuts.push(PlannedCut::new(axis_plane(frame, axis, pos), format!("{} {}/{}", AXIS_NAMES[axis], k, n)));
        }
    }
    plan
}

/// Periodic cuts every `spacing[i]` mm starting `offset[i]` mm from the model's minimum on that axis.
/// `None` disables an axis. Cuts that would fall outside the model are omitted.
pub fn modular_plan(ext: [(f64, f64); 3], frame: &GridFrame, spacing: [Option<f64>; 3], offset: [f64; 3]) -> CutPlan {
    let mut plan = CutPlan::default();
    for axis in 0..3 {
        let Some(step) = spacing[axis].filter(|s| *s > 1e-6) else { continue };
        let (lo, hi) = ext[axis];
        let mut k = 0;
        loop {
            let pos = lo + offset[axis] + step * k as f64;
            if pos >= hi - 1e-9 {
                break;
            }
            if pos > lo + 1e-9 {
                plan.cuts.push(PlannedCut::new(axis_plane(frame, axis, pos), format!("{} @ {:.1}", AXIS_NAMES[axis], pos - lo)));
            }
            k += 1;
        }
    }
    plan
}

/// Staggered ("brick") pattern. The model is divided into rows along `row_axis`; in each row the cuts along
/// `along_axis` are spaced `brick_len` apart, and odd rows are shifted by `stagger * brick_len`, so joints never line up.
pub fn brick_plan(
    ext: [(f64, f64); 3],
    frame: &GridFrame,
    row_axis: usize,
    along_axis: usize,
    row_height: f64,
    brick_len: f64,
    stagger: f64,
) -> CutPlan {
    assert!(row_axis != along_axis && row_axis < 3 && along_axis < 3);
    let mut plan = CutPlan::default();
    let (rlo, rhi) = ext[row_axis];
    let (alo, ahi) = ext[along_axis];
    let rows = (((rhi - rlo) / row_height).ceil() as usize).max(1);
    let row_bounds: Vec<(f64, f64)> =
        (0..rows).map(|r| (rlo + r as f64 * row_height, (rlo + (r + 1) as f64 * row_height).min(rhi))).collect();
    for (r, &(lo, _)) in row_bounds.iter().enumerate().skip(1) {
        plan.cuts.push(PlannedCut::new(axis_plane(frame, row_axis, lo), format!("row {}", r)));
    }
    for (r, &(lo, hi)) in row_bounds.iter().enumerate() {
        let shift = if r % 2 == 1 { stagger * brick_len } else { 0.0 };
        let mut scope = Vec::new();
        if r > 0 {
            scope.push(axis_plane(frame, row_axis, lo));
        }
        if r + 1 < rows {
            scope.push(axis_plane(frame, row_axis, hi).flipped());
        }
        let mut k = 0;
        loop {
            let pos = alo + shift + brick_len * k as f64;
            if pos >= ahi - 1e-9 {
                break;
            }
            if pos > alo + 1e-9 {
                plan.cuts
                    .push(PlannedCut::new(axis_plane(frame, along_axis, pos), format!("row {} joint {}", r + 1, k)).scoped(scope.clone()));
            }
            k += 1;
        }
    }
    plan
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::exec::{execute_plan, SplitOptions};
    use glam::Vec3;
    use openslicer_core::control::Control;

    fn block(size: Vec3) -> Mesh {
        let mut m = Mesh::cuboid(size);
        m.translate(size * 0.5);
        m
    }

    fn run(m: &Mesh, plan: &CutPlan) -> crate::exec::SplitResult {
        execute_plan(m, plan, &SplitOptions { tolerance: None, separate_solids: true }, &Control::none()).unwrap()
    }

    #[test]
    fn grid_3x2x2_gives_twelve_valid_pieces() {
        let m = block(Vec3::new(300.0, 200.0, 100.0));
        let plan = grid_plan(extents(&m, &GridFrame::WORLD), &GridFrame::WORLD, [3, 2, 2]);
        assert_eq!(plan.len(), 2 + 1 + 1);
        let r = run(&m, &plan);
        assert!(r.report.is_valid(), "{:?}", r.report);
        assert_eq!(r.pieces.len(), 12);
        assert!(r.pieces.iter().all(|p| (p.volume - 300.0 * 200.0 * 100.0 / 12.0).abs() < 1e-1));
    }

    #[test]
    fn modular_spacing_omits_cuts_outside_and_at_the_ends() {
        let m = block(Vec3::new(1000.0, 10.0, 10.0));
        let plan = modular_plan(extents(&m, &GridFrame::WORLD), &GridFrame::WORLD, [Some(250.0), None, None], [0.0; 3]);
        assert_eq!(plan.len(), 3, "cuts at 250, 500, 750 only");
        let plan2 = modular_plan(extents(&m, &GridFrame::WORLD), &GridFrame::WORLD, [Some(250.0), None, None], [100.0, 0.0, 0.0]);
        assert_eq!(plan2.len(), 4, "offset 100 gives 100, 350, 600, 850");
    }

    #[test]
    fn brick_pattern_staggers_the_joints_between_rows() {
        let m = block(Vec3::new(400.0, 10.0, 200.0));
        // rows along Z (axis 2), joints along X (axis 0); 100 mm rows, 100 mm bricks, half-brick stagger
        let plan = brick_plan(extents(&m, &GridFrame::WORLD), &GridFrame::WORLD, 2, 0, 100.0, 100.0, 0.5);
        let r = run(&m, &plan);
        assert!(r.report.is_valid(), "{:?}", r.report);
        // Row 0: joints at 100,200,300 -> 4 bricks. Row 1: joints at 50,150,250,350 -> 5 pieces.
        assert_eq!(r.pieces.len(), 9);
        let mut xs0: Vec<f32> = r.pieces.iter().filter(|p| p.bounds.max.z <= 100.5).map(|p| p.bounds.min.x).collect();
        let mut xs1: Vec<f32> = r.pieces.iter().filter(|p| p.bounds.min.z >= 99.5).map(|p| p.bounds.min.x).collect();
        xs0.sort_by(|a, b| a.total_cmp(b));
        xs1.sort_by(|a, b| a.total_cmp(b));
        assert_eq!(xs0.len(), 4);
        assert_eq!(xs1.len(), 5);
        for x in &xs1[1..] {
            assert!(!xs0.iter().any(|y| (y - x).abs() < 1.0), "joint at {x} lines up with the row below");
        }
    }

    #[test]
    fn yawed_grid_cuts_along_rotated_axes() {
        let f = GridFrame { yaw: std::f64::consts::FRAC_PI_4 };
        let m = block(Vec3::new(200.0, 200.0, 20.0));
        let plan = grid_plan(extents(&m, &f), &f, [2, 1, 1]);
        assert_eq!(plan.len(), 1);
        let n = plan.cuts[0].plane.n;
        assert!((n - DVec3::new(0.5f64.sqrt(), 0.5f64.sqrt(), 0.0)).length() < 1e-12);
        let r = run(&m, &plan);
        assert!(r.report.is_valid(), "{:?}", r.report);
        assert_eq!(r.pieces.len(), 2);
    }
}
