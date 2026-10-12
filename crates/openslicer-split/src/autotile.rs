//! Automatic tiling: choose a grid that makes every piece fit the machine with as few pieces as possible.
//!
//! For each candidate grid orientation (rotation about Z) and each assignment of grid axes to printer axes,
//! the number of cells per axis is the smallest that keeps every cell within the usable build volume, and
//! the number of occupied cells (cells that actually contain material) estimates the piece count. The
//! candidate with the fewest estimated pieces wins; ties go to fewer cuts. Cells are of equal size, so no
//! thin remainder slice is produced along any axis.

use crate::layout::{extents, grid_plan, GridFrame};
use crate::occupancy::{occupancy, Grid};
use crate::plan::CutPlan;
use glam::DVec3;
use openslicer_core::control::{Cancelled, Control};
use openslicer_core::Mesh;

/// Which printer axis each grid axis may map to.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum OrientationPolicy {
    /// The model's up axis stays the printer's Z axis; only X and Y may swap. Pieces print flat, which is faster and sturdier.
    KeepUpright,
    /// Any assignment, including standing pieces on end. Can reduce the piece count, at the cost of tall prints.
    Any,
}

#[derive(Clone, Debug)]
pub struct TilingOptions {
    /// Usable build volume (X, Y, Z) in mm.
    pub usable: DVec3,
    /// Space reserved per printer axis for features that protrude from a piece (connector pins, lips).
    pub allowance: DVec3,
    /// Also try grids rotated about Z (steps of `yaw_step_deg`) to follow elongated parts.
    pub search_yaw: bool,
    pub yaw_step_deg: f64,
    pub orientation: OrientationPolicy,
    /// Candidates producing more cells than this are skipped.
    pub max_cells: usize,
}

impl TilingOptions {
    pub fn new(usable: DVec3) -> Self {
        Self {
            usable,
            allowance: DVec3::ZERO,
            search_yaw: true,
            yaw_step_deg: 15.0,
            orientation: OrientationPolicy::KeepUpright,
            max_cells: 20_000,
        }
    }
}

#[derive(Clone, Debug)]
pub struct TilingPlan {
    pub plan: CutPlan,
    pub frame: GridFrame,
    /// Cells along each grid axis.
    pub divisions: [usize; 3],
    /// Cell size along each grid axis (mm).
    pub cell: [f64; 3],
    /// Printer axis (0 = X, 1 = Y, 2 = Z) assigned to each grid axis.
    pub printer_axis: [usize; 3],
    /// Occupied cells; an estimate of the piece count (the actual count can be lower for hollow or sparse parts).
    pub estimated_pieces: usize,
    pub candidates_evaluated: usize,
}

#[derive(Debug, thiserror::Error)]
pub enum TilingError {
    #[error("the build volume leaves no usable space after reserving the connector allowance")]
    NoUsableVolume,
    #[error("the mesh is empty")]
    EmptyMesh,
    #[error("no tiling found: every candidate needs more than {0} cells; use a larger build volume or a coarser setting")]
    TooManyCells(usize),
    #[error(transparent)]
    Cancelled(#[from] Cancelled),
}

const PERMUTATIONS: [[usize; 3]; 6] = [[0, 1, 2], [0, 2, 1], [1, 0, 2], [1, 2, 0], [2, 0, 1], [2, 1, 0]];

pub fn auto_tile(mesh: &Mesh, opts: &TilingOptions, ctl: &Control) -> Result<TilingPlan, TilingError> {
    if mesh.is_empty() {
        return Err(TilingError::EmptyMesh);
    }
    let usable = opts.usable - opts.allowance;
    if usable.min_element() < 1.0 {
        return Err(TilingError::NoUsableVolume);
    }
    let mut yaws = vec![0.0f64];
    if opts.search_yaw {
        let step = opts.yaw_step_deg.clamp(1.0, 45.0);
        let mut a = step;
        while a < 90.0 - 1e-9 {
            yaws.push(a.to_radians());
            a += step;
        }
    }

    let mut best: Option<(TilingPlan, (usize, usize, u64))> = None;
    let mut evaluated = 0usize;
    let mut skipped_large = false;
    for (yi, &yaw) in yaws.iter().enumerate() {
        ctl.check()?;
        ctl.report(yi as f32 / yaws.len() as f32, &format!("Evaluating grid orientation {:.0}°", yaw.to_degrees()));
        let frame = GridFrame { yaw };
        let ext = extents(mesh, &frame);
        let size = [ext[0].1 - ext[0].0, ext[1].1 - ext[1].0, ext[2].1 - ext[2].0];
        for perm in PERMUTATIONS {
            if opts.orientation == OrientationPolicy::KeepUpright && perm[2] != 2 {
                continue;
            }
            let mut n = [1usize; 3];
            for i in 0..3 {
                let cap = usable[perm[i]];
                n[i] = ((size[i] - 1e-6) / cap).ceil().max(1.0) as usize;
            }
            let cells = n[0] * n[1] * n[2];
            if cells > opts.max_cells {
                skipped_large = true;
                continue;
            }
            // Equivalent permutations (same divisions and same per-axis caps) give the same grid; evaluate once.
            let cell = [size[0] / n[0] as f64, size[1] / n[1] as f64, size[2] / n[2] as f64];
            let grid = Grid { origin: [ext[0].0, ext[1].0, ext[2].0], cell, n };
            let occ = occupancy(mesh, &frame, &grid);
            evaluated += 1;
            let occupied = occ.iter().filter(|&&c| c).count();
            let cuts = (n[0] - 1) + (n[1] - 1) + (n[2] - 1);
            // Prefer fewer pieces, then fewer cuts, then the larger smallest cell (more even tiling).
            let key = (occupied, cuts, (-(cell.iter().cloned().fold(f64::MAX, f64::min)) * 1000.0) as i64 as u64);
            if best.as_ref().is_none_or(|(_, k)| key < *k) {
                let plan = grid_plan(ext, &frame, n);
                best = Some((
                    TilingPlan { plan, frame, divisions: n, cell, printer_axis: perm, estimated_pieces: occupied, candidates_evaluated: 0 },
                    key,
                ));
            }
        }
    }
    let Some((mut plan, _)) = best else {
        return Err(TilingError::TooManyCells(opts.max_cells));
    };
    let _ = skipped_large;
    plan.candidates_evaluated = evaluated;
    Ok(plan)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::exec::{execute_plan, SplitOptions};
    use crate::fit::best_orientation;
    use glam::Vec3;

    fn block(size: Vec3) -> Mesh {
        let mut m = Mesh::cuboid(size);
        m.translate(size * 0.5);
        m
    }

    fn all_fit(result: &crate::exec::SplitResult, usable: DVec3) -> bool {
        result.pieces.iter().all(|p| best_orientation(&p.mesh, usable, &[]).is_some())
    }

    #[test]
    fn box_is_divided_into_the_minimum_equal_cells() {
        let m = block(Vec3::new(500.0, 300.0, 100.0));
        let usable = DVec3::new(220.0, 220.0, 250.0);
        let t = auto_tile(&m, &TilingOptions { search_yaw: false, ..TilingOptions::new(usable) }, &Control::none()).unwrap();
        assert_eq!(t.divisions.iter().product::<usize>(), 6, "{t:?}"); // 3 x 2 x 1
        assert_eq!(t.estimated_pieces, 6);
        let r = execute_plan(&m, &t.plan, &SplitOptions { tolerance: None, separate_solids: true }, &Control::none()).unwrap();
        assert!(r.report.is_valid(), "{:?}", r.report);
        assert_eq!(r.pieces.len(), 6);
        assert!(all_fit(&r, usable));
    }

    #[test]
    fn standing_pieces_on_end_can_reduce_the_count_when_allowed() {
        let m = block(Vec3::new(500.0, 300.0, 100.0));
        let usable = DVec3::new(220.0, 220.0, 250.0);
        let t = auto_tile(
            &m,
            &TilingOptions { search_yaw: false, orientation: OrientationPolicy::Any, ..TilingOptions::new(usable) },
            &Control::none(),
        )
        .unwrap();
        assert_eq!(t.estimated_pieces, 4, "{t:?}");
        let r = execute_plan(&m, &t.plan, &SplitOptions { tolerance: None, separate_solids: true }, &Control::none()).unwrap();
        assert!(r.report.is_valid());
        assert!(all_fit(&r, usable));
    }

    #[test]
    fn model_that_already_fits_gets_no_cuts() {
        let m = block(Vec3::new(100.0, 100.0, 100.0));
        let t = auto_tile(&m, &TilingOptions::new(DVec3::new(220.0, 220.0, 250.0)), &Control::none()).unwrap();
        assert!(t.plan.is_empty());
        assert_eq!(t.estimated_pieces, 1);
    }

    #[test]
    fn diagonal_bar_needs_fewer_pieces_with_a_rotated_grid() {
        let mut bar = block(Vec3::new(600.0, 40.0, 30.0));
        let rot = glam::Mat3::from_rotation_z(30f32.to_radians());
        for v in &mut bar.vertices {
            *v = rot * *v;
        }
        let usable = DVec3::new(220.0, 220.0, 250.0);
        let aligned = auto_tile(&bar, &TilingOptions { search_yaw: false, ..TilingOptions::new(usable) }, &Control::none()).unwrap();
        let searched = auto_tile(&bar, &TilingOptions { yaw_step_deg: 5.0, ..TilingOptions::new(usable) }, &Control::none()).unwrap();
        assert!(searched.estimated_pieces < aligned.estimated_pieces, "{} vs {}", searched.estimated_pieces, aligned.estimated_pieces);
        let r = execute_plan(&bar, &searched.plan, &SplitOptions { tolerance: None, separate_solids: true }, &Control::none()).unwrap();
        assert!(r.report.is_valid(), "{:?}", r.report);
        assert!(all_fit(&r, usable));
        assert!(r.pieces.len() <= searched.estimated_pieces);
    }

    #[test]
    fn oversized_torus_is_tiled_into_valid_pieces_that_all_fit() {
        let torus = crate::tests_shapes::torus(150.0, 50.0, 96, 48); // 400 x 400 x 100
        let usable = DVec3::new(220.0, 220.0, 250.0);
        let t = auto_tile(&torus, &TilingOptions::new(usable), &Control::none()).unwrap();
        let r = execute_plan(&torus, &t.plan, &SplitOptions { tolerance: None, separate_solids: true }, &Control::none()).unwrap();
        assert!(r.report.is_valid(), "{:?}", r.report);
        assert!(all_fit(&r, usable));
        assert!(r.pieces.len() <= t.estimated_pieces);
        assert!(r.report.volume_error < 1e-8, "{}", r.report.volume_error);
        assert!(r.pieces.len() >= 4);
    }

    #[test]
    fn connector_allowance_shrinks_the_cells() {
        let m = block(Vec3::new(400.0, 100.0, 100.0));
        let usable = DVec3::new(220.0, 220.0, 250.0);
        let base = auto_tile(&m, &TilingOptions { search_yaw: false, ..TilingOptions::new(usable) }, &Control::none()).unwrap();
        assert_eq!(base.divisions.iter().product::<usize>(), 2);
        // Reserving 40 mm leaves 180 mm usable: 400 needs 3 cells.
        let t = auto_tile(
            &m,
            &TilingOptions { allowance: DVec3::new(40.0, 40.0, 0.0), search_yaw: false, ..TilingOptions::new(usable) },
            &Control::none(),
        )
        .unwrap();
        assert_eq!(t.divisions.iter().product::<usize>(), 3);
    }

    #[test]
    fn impossible_requests_are_errors_not_panics() {
        let m = block(Vec3::splat(100.0));
        let e =
            auto_tile(&m, &TilingOptions { allowance: DVec3::splat(500.0), ..TilingOptions::new(DVec3::splat(250.0)) }, &Control::none());
        assert!(matches!(e, Err(TilingError::NoUsableVolume)));
        assert!(matches!(
            auto_tile(&Mesh::default(), &TilingOptions::new(DVec3::splat(250.0)), &Control::none()),
            Err(TilingError::EmptyMesh)
        ));
        let tiny = auto_tile(
            &block(Vec3::splat(5000.0)),
            &TilingOptions { max_cells: 10, ..TilingOptions::new(DVec3::splat(250.0)) },
            &Control::none(),
        );
        assert!(matches!(tiny, Err(TilingError::TooManyCells(10))));
    }
}
