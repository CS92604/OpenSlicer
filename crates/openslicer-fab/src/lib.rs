//! Fabrication methods that build a physical object differently from simply cutting it apart:
//! stacked layers, interlocking rib grids, blocks, flat plates, wireframes, depth reliefs, and
//! models derived from images (lithophanes, reliefs, halftones).
//!
//! Every method returns [`FabResult`]: the parts, plus a report that states how closely the parts
//! represent the source (volume ratio, geometric error where it is defined) and lists problems.
//! Parts that do not fit the machine are tiled with the splitting engine, never silently left oversized.

pub mod blocks;
pub mod image_tools;
pub mod interlock;
pub mod plates;
pub mod relief;
pub mod stack;
pub mod wireframe;

use glam::DVec3;
use openslicer_core::control::{Cancelled, Control};
use openslicer_geom::region::Polygon;
use openslicer_split::autotile::{auto_tile, OrientationPolicy, TilingOptions};
use openslicer_split::exec::{execute_plan, numbering_order, piece_name, Piece, SplitOptions};
use openslicer_split::fit::best_orientation;

/// One printable (or cuttable) part produced by a method.
#[derive(Clone, Debug)]
pub struct Part {
    pub piece: Piece,
    /// Descriptive label, for example `Layer 4` or `X rib 2` (the piece itself carries the numbered name).
    pub label: String,
    /// For flat stock (plates, layers): the 2D outline and the sheet thickness, for laser/CNC export.
    pub outline: Option<Outline>,
}

#[derive(Clone, Debug)]
pub struct Outline {
    pub polygons: Vec<Polygon>,
    pub thickness: f64,
}

#[derive(Clone, Debug, Default)]
pub struct FabReport {
    pub method: String,
    pub source_volume: f64,
    pub parts_volume: f64,
    /// Volume of the parts relative to the source (1.0 = same material; below 1 for skeletal methods, around 1 for stepped approximations).
    pub fill_ratio: f64,
    /// An upper bound on how far the parts' surface can deviate from the source, in mm, where the method defines one.
    pub max_deviation: Option<f64>,
    pub notes: Vec<String>,
    /// Correctness problems; empty means every part is a valid watertight solid and fits the machine.
    pub problems: Vec<String>,
}

impl FabReport {
    pub fn is_valid(&self) -> bool {
        self.problems.is_empty()
    }
}

#[derive(Clone, Debug)]
pub struct FabResult {
    pub parts: Vec<Part>,
    pub report: FabReport,
}

#[derive(Debug, thiserror::Error)]
pub enum FabError {
    #[error("{0}")]
    Invalid(String),
    #[error(transparent)]
    Cancelled(#[from] Cancelled),
}

impl FabResult {
    pub(crate) fn finish(mut parts: Vec<Part>, mut report: FabReport) -> Self {
        // Reorder the parts themselves (not just their pieces) so outlines stay with their solids.
        let pieces: Vec<Piece> = parts.iter().map(|p| p.piece.clone()).collect();
        let order = numbering_order(&pieces);
        let mut sorted: Vec<Part> = order.iter().map(|&i| parts[i].clone()).collect();
        let count = sorted.len();
        for (i, part) in sorted.iter_mut().enumerate() {
            part.piece.id = i + 1;
            part.piece.name = piece_name(i, count);
        }
        parts = sorted;
        report.parts_volume = parts.iter().map(|p| p.piece.volume).sum();
        if report.source_volume > 0.0 {
            report.fill_ratio = report.parts_volume / report.source_volume;
        }
        for p in &parts {
            let s = p.piece.mesh.stats();
            if !s.is_watertight() || s.signed_volume <= 0.0 {
                report.problems.push(format!(
                    "{} is not a valid solid ({} open edges, volume {:.3} mm³)",
                    p.piece.name, s.boundary_edges, s.signed_volume
                ));
            }
        }
        Self { parts, report }
    }
}

/// Makes sure every part fits `usable`, tiling parts that cannot be oriented to fit with the splitting engine.
/// Adds notes about what was tiled, and problems for anything that still does not fit.
pub fn ensure_fit(parts: Vec<Part>, usable: DVec3, report: &mut FabReport, ctl: &Control) -> Result<Vec<Part>, Cancelled> {
    let mut out = Vec::with_capacity(parts.len());
    let mut tiled = 0usize;
    for part in parts {
        ctl.check()?;
        if best_orientation(&part.piece.mesh, usable, &[]).is_some() {
            out.push(part);
            continue;
        }
        let opts = TilingOptions { orientation: OrientationPolicy::Any, ..TilingOptions::new(usable) };
        let Ok(tiling) = auto_tile(&part.piece.mesh, &opts, ctl) else {
            report.problems.push(format!("{} does not fit the build volume and could not be tiled", part.piece.name));
            out.push(part);
            continue;
        };
        let split = execute_plan(&part.piece.mesh, &tiling.plan, &SplitOptions { tolerance: None, separate_solids: true }, ctl)?;
        for pr in &split.report.problems {
            report.problems.push(format!("tiling {}: {pr}", part.piece.name));
        }
        tiled += 1;
        for (i, piece) in split.pieces.into_iter().enumerate() {
            if best_orientation(&piece.mesh, usable, &[]).is_none() {
                report.problems.push(format!("{} (tile {}) still does not fit the build volume", part.piece.name, i + 1));
            }
            out.push(Part { piece, label: format!("{} (tile {})", part.label, out.len() + 1), outline: None });
        }
    }
    if tiled > 0 {
        report.notes.push(format!("{tiled} part(s) were larger than the build volume and were tiled into smaller parts"));
    }
    Ok(out)
}
