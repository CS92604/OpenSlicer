//! The split-and-connect pipeline: repair, plan, cut, merge slivers, find interfaces, generate connectors,
//! verify machine fit, and (optionally) nest and verify the assembly.
//!
//! Every stage reports what it found. [`SplitOutcome::problems`] collects everything that means the result
//! must not be trusted (failed verification, parts that do not fit, open geometry); an empty list means every
//! check that was run passed.

use crate::profile::PrinterProfile;
use crate::settings::{Layout, SplitSettings};
use glam::DVec3;
use openslicer_core::control::{Cancelled, Control};
use openslicer_core::{Mesh, MeshStats};
use openslicer_geom::repair::{repair, RepairOptions, RepairReport};
use openslicer_geom::Plane;
use openslicer_joints::apply::{generate_joints, verify_assembly, ConnectorRecord, JointReport};
use openslicer_joints::settings::ConnectorKind;
use openslicer_split::autotile::{auto_tile, TilingOptions, TilingPlan};
use openslicer_split::exec::{execute_plan, Piece, SplitOptions, SplitReport};
use openslicer_split::interfaces::{find_interfaces, Interface};
use openslicer_split::layout::{brick_plan, extents, grid_plan, modular_plan, GridFrame};
use openslicer_split::nest::{nest, verify_fit, NestOptions, NestResult, PartFit};
use openslicer_split::plan::CutPlan;
use openslicer_split::reduce::{reduce_pieces, ReduceOptions, ReduceReport};

#[derive(Debug, thiserror::Error)]
pub enum PipelineError {
    #[error("{0}")]
    Invalid(String),
    #[error(transparent)]
    Cancelled(#[from] Cancelled),
}

#[derive(Clone, Debug)]
pub struct TilingSummary {
    pub divisions: [usize; 3],
    pub cell: [f64; 3],
    pub yaw_deg: f64,
    pub estimated_pieces: usize,
    pub candidates: usize,
}

#[derive(Debug)]
pub struct SplitOutcome {
    pub source: MeshStats,
    pub repair: Option<RepairReport>,
    pub tiling: Option<TilingSummary>,
    pub cut_count: usize,
    pub split: SplitReport,
    pub reduce: Option<ReduceReport>,
    /// Distinct cut planes that were used.
    pub planes: Vec<Plane>,
    /// Interfaces between pieces, found before connectors were added.
    pub interfaces: Vec<Interface>,
    pub joint_report: Option<JointReport>,
    pub connectors: Vec<ConnectorRecord>,
    /// Final parts (connectors applied).
    pub pieces: Vec<Piece>,
    /// Separate parts to print (dowels).
    pub loose: Vec<Piece>,
    pub fits: Vec<PartFit>,
    pub nest: Option<NestResult>,
    pub assembly_problems: Vec<String>,
    /// One record per labelled piece when labels were requested.
    pub labels: Vec<crate::labels::LabelRecord>,
    pub notes: Vec<String>,
    /// Everything that means the result cannot be trusted. Empty means every check that was run passed.
    pub problems: Vec<String>,
}

impl SplitOutcome {
    pub fn is_valid(&self) -> bool {
        self.problems.is_empty()
    }
}

/// Extra length a connector adds to a part along any axis (male parts stand proud of the cut face).
pub fn connector_allowance(settings: &SplitSettings) -> f64 {
    match &settings.connectors {
        None => 0.0,
        Some(c) => match c.kind {
            ConnectorKind::Dowel => 0.0,
            ConnectorKind::Terrace => 2.0 * c.height * (c.steps.saturating_sub(1)) as f64,
            _ => 2.0 * c.height,
        },
    }
}

fn plan_for(
    mesh: &Mesh,
    printer: &PrinterProfile,
    s: &SplitSettings,
    ctl: &Control,
) -> Result<(CutPlan, Option<TilingSummary>, Option<TilingPlan>), PipelineError> {
    let usable = printer.usable();
    let frame_of = |yaw_deg: f64| GridFrame { yaw: yaw_deg.to_radians() };
    Ok(match &s.layout {
        Layout::Auto { search_yaw, yaw_step_deg } => {
            let allowance = DVec3::splat(connector_allowance(s));
            let opts = TilingOptions {
                allowance,
                search_yaw: *search_yaw,
                yaw_step_deg: *yaw_step_deg,
                orientation: s.orientation,
                ..TilingOptions::new(usable)
            };
            let t = auto_tile(mesh, &opts, ctl).map_err(|e| match e {
                openslicer_split::autotile::TilingError::Cancelled(c) => PipelineError::Cancelled(c),
                other => PipelineError::Invalid(other.to_string()),
            })?;
            let summary = TilingSummary {
                divisions: t.divisions,
                cell: t.cell,
                yaw_deg: t.frame.yaw.to_degrees(),
                estimated_pieces: t.estimated_pieces,
                candidates: t.candidates_evaluated,
            };
            (t.plan.clone(), Some(summary), Some(t))
        }
        Layout::Grid { divisions, yaw_deg } => {
            if divisions.contains(&0) {
                return Err(PipelineError::Invalid("grid divisions must be at least 1 on every axis".into()));
            }
            let f = frame_of(*yaw_deg);
            (grid_plan(extents(mesh, &f), &f, *divisions), None, None)
        }
        Layout::Modular { spacing, offset, yaw_deg } => {
            if spacing.iter().flatten().any(|s| s.is_nan() || *s <= 0.0) {
                return Err(PipelineError::Invalid("cut spacing must be positive".into()));
            }
            let f = frame_of(*yaw_deg);
            (modular_plan(extents(mesh, &f), &f, *spacing, *offset), None, None)
        }
        Layout::Brick { row_axis, along_axis, row_height, brick_length, stagger } => {
            if *row_axis > 2
                || *along_axis > 2
                || row_axis == along_axis
                || row_height.is_nan()
                || *row_height <= 0.0
                || brick_length.is_nan()
                || *brick_length <= 0.0
            {
                return Err(PipelineError::Invalid("the brick layout needs two different axes and positive sizes".into()));
            }
            let f = GridFrame::WORLD;
            (brick_plan(extents(mesh, &f), &f, *row_axis, *along_axis, *row_height, *brick_length, *stagger), None, None)
        }
        Layout::Planes(specs) => {
            let mut planes = Vec::new();
            for (i, p) in specs.iter().enumerate() {
                planes.push(p.to_plane().ok_or_else(|| PipelineError::Invalid(format!("cut plane {} has a zero normal", i + 1)))?);
            }
            (CutPlan::from_planes(planes), None, None)
        }
    })
}

/// Runs the whole pipeline on `mesh` (in world coordinates, mm).
pub fn run_split(mesh: &Mesh, printer: &PrinterProfile, settings: &SplitSettings, ctl: &Control) -> Result<SplitOutcome, PipelineError> {
    printer.validate().map_err(PipelineError::Invalid)?;
    if mesh.is_empty() {
        return Err(PipelineError::Invalid("the model is empty".into()));
    }
    let usable = printer.usable();
    let mut problems: Vec<String> = Vec::new();
    let mut notes: Vec<String> = Vec::new();

    // 1. Repair.
    ctl.report(0.0, "Checking the model");
    let source = mesh.stats();
    let mut work = mesh.clone();
    let mut repair_report = None;
    if settings.repair_first {
        let opts = RepairOptions { detect_self_intersections: mesh.triangles.len() <= 500_000, ..Default::default() };
        let (fixed, rep) = repair(mesh, &opts);
        if rep.changed() {
            notes.push(format!(
                "model repaired: {} vertices welded, {} degenerate and {} duplicate triangles removed, {} triangles flipped, {} holes filled",
                rep.vertices_welded, rep.degenerate_removed, rep.duplicates_removed, rep.triangles_flipped, rep.holes_filled
            ));
        }
        for r in &rep.remaining {
            problems.push(format!("model: {r}"));
        }
        work = fixed;
        repair_report = Some(rep);
    } else if !source.is_watertight() || source.is_inverted() {
        problems.push("model: the mesh is not a clean closed solid; enable repair or fix it first".into());
    }
    ctl.check()?;

    // 2. Plan.
    ctl.report(0.05, "Planning the cuts");
    let (mut plan, tiling, _) = plan_for(&work, printer, settings, &ctl.sub(0.05, 0.15))?;
    if settings.avoid_slivers && !matches!(settings.layout, Layout::Planes(_)) {
        let diag = work.bounds().map_or(0.0, |b| f64::from((b.max - b.min).length()));
        let range = (diag * 0.02).clamp(0.5, 4.0);
        for n in crate::nudge::avoid_slivers(&work, &mut plan, range, 25.0) {
            notes.push(format!(
                "cut {} moved {:+.1} mm to avoid {} tiny island(s) in its cross-section ({} remain)",
                n.cut + 1,
                n.shift,
                n.islands_before,
                n.islands_after
            ));
        }
    }
    let cut_count = plan.len();
    if let Some(t) = &tiling {
        notes.push(format!(
            "auto-tiling: {}×{}×{} cells of {:.0}×{:.0}×{:.0} mm, grid rotated {:.0}° (about {} pieces; {} layouts compared)",
            t.divisions[0], t.divisions[1], t.divisions[2], t.cell[0], t.cell[1], t.cell[2], t.yaw_deg, t.estimated_pieces, t.candidates
        ));
    }

    // 3. Cut.
    let mut split = execute_plan(&work, &plan, &SplitOptions { tolerance: None, separate_solids: true }, &ctl.sub(0.15, 0.55))?;
    for p in &split.report.problems {
        problems.push(format!("split: {p}"));
    }
    for n in &split.report.notes {
        notes.push(format!("split: {n}"));
    }

    // 4. Merge slivers.
    let mut reduce = None;
    if let Some(min_volume) = settings.merge_pieces_below {
        let rep = reduce_pieces(&mut split, &ReduceOptions { min_volume, usable, tolerance: 1e-3 }, &ctl.sub(0.55, 0.6))?;
        for (name, why) in &rep.kept {
            notes.push(format!("{name} is below the size limit but was kept: {why}"));
        }
        for p in &split.report.problems {
            if !problems.iter().any(|q| q.ends_with(p.as_str())) {
                problems.push(format!("split: {p}"));
            }
        }
        reduce = Some(rep);
    }

    // 5. Interfaces.
    ctl.report(0.6, "Finding where pieces meet");
    let interfaces = find_interfaces(&split.pieces, &split.planes, 1e-3);

    // 6. Connectors.
    let mut pieces = split.pieces.clone();
    let mut loose = Vec::new();
    let mut connectors = Vec::new();
    let mut joint_report = None;
    if let Some(cs) = &settings.connectors {
        let mut cs = cs.clone();
        cs.min_feature = cs.min_feature.max(printer.min_feature);
        if interfaces.is_empty() {
            notes.push("connectors were requested but the model was not cut, so there is nothing to connect".into());
        } else {
            let res = generate_joints(&split.pieces, &interfaces, &cs, &ctl.sub(0.65, 0.9))?;
            for p in &res.report.problems {
                problems.push(format!("connectors: {p}"));
            }
            for (i, reason) in &res.report.skipped {
                let (a, b) = (interfaces[*i].a, interfaces[*i].b);
                let name = |id: usize| split.pieces.iter().find(|p| p.id == id).map_or_else(|| id.to_string(), |p| p.name.clone());
                notes.push(format!("no connector between {} and {}: {reason}", name(a), name(b)));
            }
            for n in &res.report.notes {
                notes.push(format!("connectors: {n}"));
            }
            if res.report.left_open > 0 {
                notes.push(format!("connectors: {} join(s) left plain by your choice", res.report.left_open));
            }
            let accounted = res.report.interfaces_connected + res.report.left_open;
            if accounted < res.report.interfaces_total {
                problems.push(format!(
                    "connectors: {} of {} interfaces have no connector",
                    res.report.interfaces_total - accounted,
                    res.report.interfaces_total
                ));
            }
            pieces = res.pieces.clone();
            loose = res.loose.clone();
            connectors = res.connectors.clone();
            joint_report = Some(res.report.clone());
            if settings.verify_assembly {
                let ap = verify_assembly(&res, &interfaces, &ctl.sub(0.9, 0.95))?;
                for p in ap {
                    problems.push(format!("assembly: {p}"));
                }
            }
        }
    }

    // 6b. Part labels.
    let mut labels = Vec::new();
    if settings.labels.mode != crate::labels::LabelMode::Off {
        settings.labels.validate().map_err(PipelineError::Invalid)?;
        ctl.report(0.93, "Labelling the parts");
        let steps = crate::assembly::plan_assembly(&pieces, &interfaces, &connectors, &loose);
        labels = crate::labels::apply_labels(&mut pieces, &interfaces, &steps, &settings.labels, &ctl.sub(0.93, 0.95))?;
        let placed = labels.iter().filter(|l| l.placed).count();
        notes.push(format!("labels: {placed} of {} parts labelled ({})", labels.len(), settings.labels.mode.label().to_lowercase()));
        for l in labels.iter().filter(|l| !l.placed) {
            notes.push(format!("labels: no label on {}: {}", l.text.split('-').next().unwrap_or(&l.text), l.note));
        }
    }

    // 7. Machine fit (with connectors in place) and nesting.
    ctl.report(0.95, "Verifying the parts fit the printer");
    let all: Vec<Piece> = pieces.iter().chain(loose.iter()).cloned().collect();
    let down = preferred_down(&interfaces, &connectors);
    let fits = verify_fit(&all, usable, &down);
    for f in fits.iter().filter(|f| !f.fits) {
        problems.push(format!(
            "fit: {} ({:.0} × {:.0} × {:.0} mm) does not fit the build volume {:.0} × {:.0} × {:.0} mm",
            f.name, f.size.x, f.size.y, f.size.z, usable.x, usable.y, usable.z
        ));
    }
    let nested = if settings.nest { Some(nest(&all, &NestOptions { preferred_down: down, ..NestOptions::new(usable) })) } else { None };
    ctl.report(1.0, "Done");

    Ok(SplitOutcome {
        source,
        repair: repair_report,
        tiling,
        cut_count,
        planes: split.planes.clone(),
        split: split.report.clone(),
        reduce,
        interfaces,
        joint_report,
        connectors,
        pieces,
        loose,
        fits,
        nest: nested,
        assembly_problems: Vec::new(),
        labels,
        notes,
        problems,
    })
}

/// For each piece, the outward normals of cut faces that are good to print face-down (faces without protruding connectors).
fn preferred_down(interfaces: &[Interface], connectors: &[ConnectorRecord]) -> Vec<(usize, Vec<DVec3>)> {
    let mut out: Vec<(usize, Vec<DVec3>)> = Vec::new();
    let mut add = |id: usize, n: DVec3| match out.iter_mut().find(|(i, _)| *i == id) {
        Some((_, v)) => v.push(n),
        None => out.push((id, vec![n])),
    };
    for (ii, iface) in interfaces.iter().enumerate() {
        let male_here = |piece: usize| connectors.iter().any(|c| c.interface == ii && c.male_piece == Some(piece));
        if !male_here(iface.a) {
            add(iface.a, iface.plane.n);
        }
        if !male_here(iface.b) {
            add(iface.b, -iface.plane.n);
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use glam::Vec3;
    use openslicer_joints::settings::ConnectorSettings;

    fn block(size: Vec3) -> Mesh {
        let mut m = Mesh::cuboid(size);
        m.translate(size * 0.5);
        m
    }

    fn printer() -> PrinterProfile {
        PrinterProfile::default() // 220 x 220 x 250
    }

    #[test]
    fn oversized_block_is_split_connected_and_every_part_fits() {
        let m = block(Vec3::new(500.0, 300.0, 100.0));
        let s = SplitSettings { connectors: Some(ConnectorSettings::new(ConnectorKind::Pin)), ..Default::default() };
        let out = run_split(&m, &printer(), &s, &Control::none()).unwrap();
        assert!(out.is_valid(), "{:#?}", out.problems);
        assert!(out.pieces.len() >= 6);
        assert!(out.fits.iter().all(|f| f.fits));
        assert!(!out.connectors.is_empty());
        assert_eq!(out.joint_report.as_ref().unwrap().interfaces_connected, out.interfaces.len());
        // Volume check: pieces total the model minus clearance gaps.
        let total: f64 = out.pieces.iter().chain(&out.loose).map(|p| p.volume).sum();
        assert!(total < 500.0 * 300.0 * 100.0 + 1e-3 && total > 500.0 * 300.0 * 100.0 * 0.995, "{total}");
        assert!(out.notes.iter().any(|n| n.contains("auto-tiling")));
        assert!(out.repair.as_ref().unwrap().is_clean());
    }

    #[test]
    fn connector_allowance_keeps_pins_inside_the_build_volume() {
        // 440 mm is exactly two 220 mm cells without an allowance; with 8 mm pins on both ends it needs three.
        let m = block(Vec3::new(440.0, 100.0, 50.0));
        let plain = run_split(
            &m,
            &printer(),
            &SplitSettings { layout: Layout::Auto { search_yaw: false, yaw_step_deg: 15.0 }, ..Default::default() },
            &Control::none(),
        )
        .unwrap();
        assert_eq!(plain.pieces.len(), 2);
        let pinned = run_split(
            &m,
            &printer(),
            &SplitSettings {
                layout: Layout::Auto { search_yaw: false, yaw_step_deg: 15.0 },
                connectors: Some(ConnectorSettings::new(ConnectorKind::Pin)),
                ..Default::default()
            },
            &Control::none(),
        )
        .unwrap();
        assert!(pinned.is_valid(), "{:#?}", pinned.problems);
        assert_eq!(pinned.pieces.len(), 3);
        assert!(pinned.fits.iter().all(|f| f.fits));
    }

    #[test]
    fn explicit_planes_grid_modular_and_brick_layouts_all_work() {
        let m = block(Vec3::new(200.0, 100.0, 100.0));
        let layouts = vec![
            Layout::Planes(vec![crate::settings::PlaneSpec { point: [100.0, 0.0, 0.0], normal: [1.0, 0.0, 0.0] }]),
            Layout::Grid { divisions: [2, 2, 1], yaw_deg: 0.0 },
            Layout::Modular { spacing: [Some(50.0), None, None], offset: [0.0; 3], yaw_deg: 0.0 },
            Layout::Brick { row_axis: 2, along_axis: 0, row_height: 50.0, brick_length: 100.0, stagger: 0.5 },
        ];
        let expected = [2, 4, 4, 5];
        for (l, e) in layouts.into_iter().zip(expected) {
            let out = run_split(&m, &printer(), &SplitSettings { layout: l.clone(), ..Default::default() }, &Control::none()).unwrap();
            assert!(out.is_valid(), "{l:?}: {:#?}", out.problems);
            assert_eq!(out.pieces.len(), e, "{l:?}");
        }
    }

    #[test]
    fn parts_that_do_not_fit_are_reported_as_problems() {
        let m = block(Vec3::new(500.0, 300.0, 100.0));
        // One plane only: the halves are still too big for the printer.
        let s = SplitSettings {
            layout: Layout::Planes(vec![crate::settings::PlaneSpec { point: [250.0, 0.0, 0.0], normal: [1.0, 0.0, 0.0] }]),
            ..Default::default()
        };
        let out = run_split(&m, &printer(), &s, &Control::none()).unwrap();
        assert!(!out.is_valid());
        assert!(out.problems.iter().any(|p| p.starts_with("fit:")), "{:?}", out.problems);
    }

    #[test]
    fn damaged_models_are_repaired_first_and_open_ones_are_flagged() {
        let mut m = block(Vec3::splat(100.0));
        m.triangles[3].swap(1, 2); // inconsistent winding
        let out =
            run_split(&m, &printer(), &SplitSettings { layout: Layout::Planes(vec![]), ..Default::default() }, &Control::none()).unwrap();
        assert!(out.is_valid(), "{:?}", out.problems);
        assert!(out.notes.iter().any(|n| n.contains("repaired")));
        // A model with a hole that is too big to fill is reported.
        let mut open = block(Vec3::splat(100.0));
        open.triangles.truncate(10);
        let s = SplitSettings { layout: Layout::Planes(vec![]), ..Default::default() };
        let out = run_split(&open, &printer(), &s, &Control::none()).unwrap();
        // The 2-triangle hole is small enough to fill (square face), so it comes out clean:
        assert!(out.is_valid(), "{:?}", out.problems);
        assert!(out.repair.as_ref().unwrap().holes_filled >= 1);
        // Without repair the same model is a problem.
        let out = run_split(
            &open,
            &printer(),
            &SplitSettings { repair_first: false, layout: Layout::Planes(vec![]), ..Default::default() },
            &Control::none(),
        )
        .unwrap();
        assert!(!out.is_valid());
    }

    #[test]
    fn nesting_verification_and_progress_cancellation() {
        let m = block(Vec3::new(300.0, 100.0, 40.0));
        let s = SplitSettings {
            nest: true,
            verify_assembly: true,
            connectors: Some(ConnectorSettings::new(ConnectorKind::Pin)),
            ..Default::default()
        };
        let out = run_split(&m, &printer(), &s, &Control::none()).unwrap();
        assert!(out.is_valid(), "{:?}", out.problems);
        let n = out.nest.as_ref().unwrap();
        assert!(n.all_fit() && !n.plates.is_empty());
        // Cancellation.
        use std::sync::atomic::AtomicBool;
        let flag = AtomicBool::new(true);
        assert!(matches!(run_split(&m, &printer(), &s, &Control::with_cancel(&flag)), Err(PipelineError::Cancelled(_))));
    }

    #[test]
    fn invalid_inputs_are_errors() {
        let m = block(Vec3::splat(10.0));
        assert!(run_split(&Mesh::default(), &printer(), &SplitSettings::default(), &Control::none()).is_err());
        assert!(run_split(
            &m,
            &PrinterProfile { build_volume: [1.0; 3], ..Default::default() },
            &SplitSettings::default(),
            &Control::none()
        )
        .is_err());
        let bad = SplitSettings {
            layout: Layout::Planes(vec![crate::settings::PlaneSpec { point: [0.0; 3], normal: [0.0; 3] }]),
            ..Default::default()
        };
        assert!(run_split(&m, &printer(), &bad, &Control::none()).is_err());
        let bad = SplitSettings { layout: Layout::Grid { divisions: [0, 1, 1], yaw_deg: 0.0 }, ..Default::default() };
        assert!(run_split(&m, &printer(), &bad, &Control::none()).is_err());
    }
}
