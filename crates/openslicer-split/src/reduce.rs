//! Piece reduction: merge tiny pieces back into a neighbour.
//!
//! A tiny piece (volume below a threshold) is a nuisance to print and assemble. It is merged into the
//! neighbour it shares the largest interface with, using a verified Boolean union, but only when the
//! merged piece still fits the build volume. Material is never discarded.

use crate::exec::{number_pieces, Piece, SplitResult};
use crate::fit::best_orientation;
use crate::interfaces::{find_interfaces, Interface};
use glam::DVec3;
use openslicer_core::control::{Cancelled, Control};
use openslicer_geom::boolean::{boolean, BoolOp};

#[derive(Clone, Debug)]
pub struct ReduceOptions {
    /// Pieces smaller than this volume (mm³) are merge candidates.
    pub min_volume: f64,
    pub usable: DVec3,
    /// Merge tolerance for interface detection (mm).
    pub tolerance: f64,
}

#[derive(Clone, Debug, Default)]
pub struct ReduceReport {
    /// (name of the absorbed piece, name of the piece it was merged into), using the names before renumbering.
    pub merged: Vec<(String, String)>,
    /// Tiny pieces that could not be merged, with the reason.
    pub kept: Vec<(String, String)>,
}

/// Merges tiny pieces. Updates `result.pieces` (renumbered) and returns what happened.
pub fn reduce_pieces(result: &mut SplitResult, opts: &ReduceOptions, ctl: &Control) -> Result<ReduceReport, Cancelled> {
    let mut report = ReduceReport::default();
    let mut skip: Vec<usize> = Vec::new(); // ids that could not be merged
    loop {
        ctl.check()?;
        let candidate = result
            .pieces
            .iter()
            .filter(|p| p.volume < opts.min_volume && !skip.contains(&p.id))
            .min_by(|a, b| a.volume.total_cmp(&b.volume))
            .map(|p| p.id);
        let Some(tiny_id) = candidate else { break };
        let interfaces = find_interfaces(&result.pieces, &result.planes, opts.tolerance);
        let mut neighbours: Vec<(&Interface, usize)> = interfaces
            .iter()
            .filter_map(|i| {
                if i.a == tiny_id {
                    Some((i, i.b))
                } else if i.b == tiny_id {
                    Some((i, i.a))
                } else {
                    None
                }
            })
            .collect();
        neighbours.sort_by(|x, y| y.0.area.total_cmp(&x.0.area));
        let tiny = result.pieces.iter().find(|p| p.id == tiny_id).unwrap().clone();
        ctl.report(0.5, &format!("Merging {}", tiny.name));
        let mut merged_into: Option<(usize, Piece)> = None;
        let mut reason = String::from("it touches no other piece");
        for (_, nid) in neighbours {
            let other = result.pieces.iter().find(|p| p.id == nid).unwrap();
            match boolean(&other.mesh, &tiny.mesh, BoolOp::Union) {
                Ok((mesh, rep)) => {
                    let expected = rep.volume_a + rep.volume_b;
                    if (rep.volume_result - expected).abs() > 1e-5 * expected {
                        reason = format!("the union with {} changed the volume unexpectedly", other.name);
                        continue;
                    }
                    if best_orientation(&mesh, opts.usable, &[]).is_none() {
                        reason = format!("merging with {} would exceed the build volume", other.name);
                        continue;
                    }
                    if let Some(p) = Piece::new(mesh) {
                        merged_into = Some((nid, p));
                        break;
                    }
                }
                Err(e) => reason = format!("merging with {} failed: {e}", other.name),
            }
        }
        match merged_into {
            Some((nid, mut merged)) => {
                let other = result.pieces.iter().find(|p| p.id == nid).unwrap().name.clone();
                report.merged.push((tiny.name.clone(), other));
                merged.id = nid;
                result.pieces.retain(|p| p.id != tiny_id);
                *result.pieces.iter_mut().find(|p| p.id == nid).unwrap() = merged;
            }
            None => {
                skip.push(tiny_id);
                report.kept.push((tiny.name.clone(), reason));
            }
        }
    }
    if !report.merged.is_empty() {
        number_pieces(&mut result.pieces);
        result.report.output_volume = result.pieces.iter().map(|p| p.volume).sum();
        if result.report.input_watertight && result.report.input_volume > 0.0 {
            result.report.volume_error = (result.report.output_volume - result.report.input_volume).abs() / result.report.input_volume;
            if result.report.volume_error > 1e-5 {
                result
                    .report
                    .problems
                    .push(format!("volume changed after merging pieces (relative error {:.2e})", result.report.volume_error));
            }
        }
        result.report.notes.push(format!("{} tiny piece(s) merged into neighbours", report.merged.len()));
    }
    Ok(report)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::exec::{execute_plan, SplitOptions};
    use crate::plan::CutPlan;
    use glam::Vec3;
    use openslicer_core::Mesh;
    use openslicer_geom::Plane;

    fn block(size: Vec3) -> Mesh {
        let mut m = Mesh::cuboid(size);
        m.translate(size * 0.5);
        m
    }

    fn split(m: &Mesh, xs: &[f64]) -> SplitResult {
        let plan = CutPlan::from_planes(xs.iter().map(|&x| Plane::axis(0, x)));
        execute_plan(m, &plan, &SplitOptions { tolerance: None, separate_solids: true }, &Control::none()).unwrap()
    }

    #[test]
    fn sliver_is_merged_into_its_neighbour_without_losing_material() {
        let m = block(Vec3::new(400.0, 50.0, 50.0));
        let mut r = split(&m, &[200.0, 399.0]); // 1 mm sliver at the end
        assert_eq!(r.pieces.len(), 3);
        let rep = reduce_pieces(
            &mut r,
            &ReduceOptions { min_volume: 10_000.0, usable: DVec3::new(250.0, 250.0, 250.0), tolerance: 1e-3 },
            &Control::none(),
        )
        .unwrap();
        assert_eq!(rep.merged.len(), 1, "{rep:?}");
        assert_eq!(r.pieces.len(), 2);
        assert!(r.report.is_valid(), "{:?}", r.report);
        let total: f64 = r.pieces.iter().map(|p| p.volume).sum();
        assert!((total - 400.0 * 2500.0).abs() < 1.0);
        let big = r.pieces.iter().map(|p| p.bounds.size().x).fold(0.0f32, f32::max);
        assert!((big - 200.0).abs() < 1e-2, "merged piece spans 199 + 1 mm = 200 mm, got {big}");
        assert!(r.pieces.iter().all(|p| (p.bounds.size().x - 200.0).abs() < 1e-2));
        assert!(r.pieces.iter().all(|p| p.mesh.stats().is_watertight()));
        assert_eq!(r.pieces.iter().map(|p| p.id).collect::<Vec<_>>(), vec![1, 2]);
    }

    #[test]
    fn merge_that_would_not_fit_the_machine_is_refused_and_explained() {
        let m = block(Vec3::new(400.0, 50.0, 50.0));
        let mut r = split(&m, &[200.0, 399.0]);
        // Usable X of 200 mm: the 201 mm merged piece cannot fit... but it could stand up; forbid that with a short Z.
        let rep = reduce_pieces(
            &mut r,
            &ReduceOptions { min_volume: 10_000.0, usable: DVec3::new(200.5, 200.5, 40.0), tolerance: 1e-3 },
            &Control::none(),
        )
        .unwrap();
        assert_eq!(rep.merged.len(), 0);
        assert_eq!(rep.kept.len(), 1);
        assert!(rep.kept[0].1.contains("exceed"), "{:?}", rep.kept);
        assert_eq!(r.pieces.len(), 3, "nothing is dropped");
    }
}
