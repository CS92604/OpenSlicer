//! Verified execution of a cut plan.

use crate::plan::CutPlan;
use glam::DVec3;
use openslicer_core::control::{Cancelled, Control};
use openslicer_core::{Aabb, Mesh};
use openslicer_geom::components::separate_solids;
use openslicer_geom::cut::{cut_plane, CutOptions};
use openslicer_geom::Plane;
use rayon::prelude::*;

#[derive(Clone, Debug)]
pub struct Piece {
    /// Unique 1-based identifier, stable for a given result.
    pub id: usize,
    /// Human-readable label such as `P07`.
    pub name: String,
    pub mesh: Mesh,
    pub bounds: Aabb,
    pub volume: f64,
}

impl Piece {
    pub fn new(mesh: Mesh) -> Option<Self> {
        let bounds = mesh.bounds()?;
        let volume = mesh.stats().signed_volume;
        Some(Self { id: 0, name: String::new(), mesh, bounds, volume })
    }
}

#[derive(Clone, Debug, Default)]
pub struct SplitOptions {
    /// Snap tolerance override (mm); `None` uses the cutter's size-relative default.
    pub tolerance: Option<f64>,
    /// Separate disconnected solids into their own pieces.
    pub separate_solids: bool,
}

#[derive(Clone, Debug)]
pub struct CutRecord {
    pub index: usize,
    pub label: String,
    pub plane: Plane,
    pub pieces_cut: usize,
    pub section_area: f64,
}

#[derive(Clone, Debug, Default)]
pub struct SplitReport {
    pub input_watertight: bool,
    pub input_volume: f64,
    pub output_volume: f64,
    /// Relative difference between input and output volume (watertight input only).
    pub volume_error: f64,
    pub cuts: Vec<CutRecord>,
    pub notes: Vec<String>,
    /// Correctness problems; empty means every cut verified and the volume is conserved.
    pub problems: Vec<String>,
}

impl SplitReport {
    pub fn is_valid(&self) -> bool {
        self.problems.is_empty()
    }
}

#[derive(Clone, Debug)]
pub struct SplitResult {
    pub pieces: Vec<Piece>,
    /// Distinct planes that were actually used to cut, in plan order.
    pub planes: Vec<Plane>,
    pub report: SplitReport,
}

/// Volume conservation limit across a whole plan (each cut is also verified individually).
const TOTAL_VOLUME_TOLERANCE: f64 = 1e-5;

fn straddles(bounds: &Aabb, plane: &Plane, tol: f64) -> bool {
    let (mut lo, mut hi) = (f64::MAX, f64::MIN);
    for i in 0..8 {
        let c = DVec3::new(
            if i & 1 == 0 { bounds.min.x } else { bounds.max.x } as f64,
            if i & 2 == 0 { bounds.min.y } else { bounds.max.y } as f64,
            if i & 4 == 0 { bounds.min.z } else { bounds.max.z } as f64,
        );
        let s = plane.signed_distance(c);
        lo = lo.min(s);
        hi = hi.max(s);
    }
    lo < -tol && hi > tol
}

/// Executes `plan` on `mesh`. Every cut is verified; the report lists any problem and nothing is claimed silently.
pub fn execute_plan(mesh: &Mesh, plan: &CutPlan, opts: &SplitOptions, ctl: &Control) -> Result<SplitResult, Cancelled> {
    let mut report = SplitReport::default();
    let in_stats = mesh.stats();
    report.input_watertight = in_stats.is_watertight();
    report.input_volume = in_stats.signed_volume;
    if !report.input_watertight {
        report.notes.push(format!(
            "the input mesh is not watertight ({} open edges, {} non-manifold edges); cuts through the open regions cannot be capped",
            in_stats.boundary_edges, in_stats.non_manifold_edges
        ));
    }
    if in_stats.signed_volume < 0.0 {
        report.problems.push("the input mesh has inverted normals (negative volume); repair it before splitting".into());
    }

    let Some(first) = Piece::new(mesh.clone()) else {
        report.problems.push("the input mesh is empty".into());
        return Ok(SplitResult { pieces: Vec::new(), planes: Vec::new(), report });
    };
    let diag = first.bounds.size().length() as f64;
    let tol = opts.tolerance.unwrap_or_else(|| (diag * 1e-6).clamp(1e-5, 1e-3));
    let cut_opts = CutOptions { tolerance: Some(tol), cap: true, verify: true };

    let mut pieces = vec![first];
    let mut used_planes: Vec<Plane> = Vec::new();
    let total = plan.cuts.len().max(1);
    for (ci, cut) in plan.cuts.iter().enumerate() {
        ctl.check()?;
        ctl.report(ci as f32 / total as f32, &format!("Cutting {} of {} ({})", ci + 1, plan.cuts.len(), cut.label));
        let in_scope = |p: &Piece| {
            let c = p.bounds.center().as_dvec3();
            cut.scope.iter().all(|s| s.signed_distance(c) >= -tol)
        };
        // Pieces the plane crosses, processed in parallel.
        let results: Vec<(usize, openslicer_geom::cut::CutResult)> = pieces
            .par_iter()
            .enumerate()
            .filter(|(_, p)| in_scope(p) && straddles(&p.bounds, &cut.plane, tol))
            .map(|(i, p)| (i, cut_plane(&p.mesh, &cut.plane, &cut_opts)))
            .collect();
        ctl.check()?;
        let mut record = CutRecord { index: ci, label: cut.label.clone(), plane: cut.plane, pieces_cut: results.len(), section_area: 0.0 };
        let mut replaced: Vec<Option<Vec<Piece>>> = vec![None; pieces.len()];
        for (i, r) in results {
            for pr in &r.report.problems {
                report.problems.push(format!("{} on piece {}: {pr}", cut.label, i + 1));
            }
            for n in &r.report.notes {
                if !report.notes.contains(n) {
                    report.notes.push(n.clone());
                }
            }
            record.section_area += r.report.cap_area;
            let halves: Vec<Piece> = [r.above, r.below].into_iter().filter(|m| !m.is_empty()).filter_map(Piece::new).collect();
            replaced[i] = Some(halves);
        }
        if record.pieces_cut > 0 {
            used_planes.push(cut.plane);
        }
        report.cuts.push(record);
        let mut next = Vec::with_capacity(pieces.len() + record_len(&replaced));
        for (p, rep) in pieces.into_iter().zip(replaced) {
            match rep {
                Some(halves) => next.extend(halves),
                None => next.push(p),
            }
        }
        pieces = next;
    }

    if opts.separate_solids {
        ctl.report(0.95, "Separating disconnected solids");
        let mut separated = Vec::with_capacity(pieces.len());
        for p in pieces {
            let solids = separate_solids(&p.mesh);
            if solids.len() > 1 {
                report.notes.push(format!("a piece contained {} disconnected solids; they were separated", solids.len()));
                separated.extend(solids.into_iter().filter_map(Piece::new));
            } else {
                separated.push(p);
            }
        }
        pieces = separated;
    }

    for p in &pieces {
        if p.volume <= 0.0 {
            report.problems.push(format!("a piece has non-positive volume ({:.6} mm³), its normals may be inverted", p.volume));
        }
    }
    number_pieces(&mut pieces);
    report.output_volume = pieces.iter().map(|p| p.volume).sum();
    if report.input_watertight && report.input_volume > 0.0 {
        report.volume_error = (report.output_volume - report.input_volume).abs() / report.input_volume;
        if report.volume_error > TOTAL_VOLUME_TOLERANCE {
            report.problems.push(format!(
                "material was lost or gained: the pieces total {:.3} mm³ but the model is {:.3} mm³ (relative error {:.2e})",
                report.output_volume, report.input_volume, report.volume_error
            ));
        }
    }
    ctl.report(1.0, "Done");
    Ok(SplitResult { pieces, planes: used_planes, report })
}

fn record_len(replaced: &[Option<Vec<Piece>>]) -> usize {
    replaced.iter().flatten().map(Vec::len).sum()
}

/// Assigns stable ids ordered bottom-to-top, then front-to-back, then left-to-right.
pub fn number_pieces(pieces: &mut [Piece]) {
    let diag = pieces.iter().map(|p| p.bounds.size().length()).fold(0.0f32, f32::max).max(1.0) as f64;
    let bucket = (diag * 1e-3).max(1e-3);
    let key = |p: &Piece| {
        let c = p.bounds.center().as_dvec3();
        ((c.z / bucket).round() as i64, (c.y / bucket).round() as i64, (c.x / bucket).round() as i64)
    };
    pieces.sort_by_key(|p| key(p));
    let width = pieces.len().to_string().len().max(2);
    for (i, p) in pieces.iter_mut().enumerate() {
        p.id = i + 1;
        p.name = format!("P{:0width$}", i + 1);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use glam::Vec3;

    fn block(size: Vec3) -> Mesh {
        let mut m = Mesh::cuboid(size);
        m.translate(size * 0.5);
        m
    }

    fn opts() -> SplitOptions {
        SplitOptions { tolerance: None, separate_solids: true }
    }

    #[test]
    fn three_parallel_cuts_make_four_pieces_that_sum_to_the_whole() {
        let m = block(Vec3::new(400.0, 100.0, 50.0));
        let plan = CutPlan::from_planes([Plane::axis(0, 100.0), Plane::axis(0, 200.0), Plane::axis(0, 300.0)]);
        let r = execute_plan(&m, &plan, &opts(), &Control::none()).unwrap();
        assert!(r.report.is_valid(), "{:?}", r.report);
        assert_eq!(r.pieces.len(), 4);
        assert!(r.pieces.iter().all(|p| (p.volume - 500000.0).abs() < 1e-2));
        assert_eq!(r.planes.len(), 3);
        // ids are unique and ordered left to right here
        let ids: Vec<usize> = r.pieces.iter().map(|p| p.id).collect();
        assert_eq!(ids, vec![1, 2, 3, 4]);
        assert!(r.pieces.windows(2).all(|w| w[0].bounds.center().x < w[1].bounds.center().x));
    }

    #[test]
    fn plane_that_misses_the_model_changes_nothing() {
        let m = block(Vec3::splat(10.0));
        let r = execute_plan(&m, &CutPlan::single(Plane::axis(0, 100.0)), &opts(), &Control::none()).unwrap();
        assert_eq!(r.pieces.len(), 1);
        assert!(r.planes.is_empty());
        assert_eq!(r.report.cuts[0].pieces_cut, 0);
    }

    #[test]
    fn scoped_cut_only_affects_pieces_in_scope() {
        let m = block(Vec3::new(100.0, 100.0, 10.0));
        let mut plan = CutPlan::single(Plane::axis(1, 50.0)); // two rows
                                                              // Cut only the upper row (y > 50) at x = 25.
        plan.cuts.push(crate::plan::PlannedCut::new(Plane::axis(0, 25.0), "row cut").scoped(vec![Plane::axis(1, 50.0)]));
        let r = execute_plan(&m, &plan, &opts(), &Control::none()).unwrap();
        assert!(r.report.is_valid(), "{:?}", r.report);
        assert_eq!(r.pieces.len(), 3);
    }

    #[test]
    fn cancellation_stops_the_plan() {
        use std::sync::atomic::{AtomicBool, Ordering};
        let flag = AtomicBool::new(true);
        let m = block(Vec3::splat(10.0));
        let r = execute_plan(&m, &CutPlan::single(Plane::axis(0, 5.0)), &opts(), &Control::with_cancel(&flag));
        assert_eq!(r.unwrap_err(), Cancelled);
        flag.store(false, Ordering::Relaxed);
    }

    #[test]
    fn inverted_or_open_input_is_reported() {
        let mut m = block(Vec3::splat(10.0));
        m.triangles.pop();
        let r = execute_plan(&m, &CutPlan::single(Plane::axis(2, 5.0)), &opts(), &Control::none()).unwrap();
        assert!(!r.report.input_watertight);
        assert!(!r.report.notes.is_empty());
        let mut inv = block(Vec3::splat(10.0));
        for t in &mut inv.triangles {
            t.swap(1, 2);
        }
        let r = execute_plan(&inv, &CutPlan::default(), &opts(), &Control::none()).unwrap();
        assert!(!r.report.is_valid());
    }

    #[test]
    fn disconnected_islands_become_separate_pieces() {
        // Two blocks far apart cut by a plane between them: each side is one piece; no cut needed to separate them either.
        let mut m = block(Vec3::splat(10.0));
        let mut b = block(Vec3::splat(10.0));
        b.translate(Vec3::new(100.0, 0.0, 0.0));
        let base = m.vertices.len() as u32;
        m.vertices.extend(b.vertices.iter().copied());
        m.triangles.extend(b.triangles.iter().map(|t| [t[0] + base, t[1] + base, t[2] + base]));
        let r = execute_plan(&m, &CutPlan::default(), &opts(), &Control::none()).unwrap();
        assert_eq!(r.pieces.len(), 2);
        assert!(r.report.is_valid());
    }
}
