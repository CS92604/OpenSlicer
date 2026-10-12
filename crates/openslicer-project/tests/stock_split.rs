//! Splits each built-in sample model through the real pipeline for common printer sizes.

use openslicer_core::control::Control;
use openslicer_joints::settings::{ConnectorKind, ConnectorSettings};
use openslicer_project::pipeline::{run_split, SplitOutcome};
use openslicer_project::profile::PrinterProfile;
use openslicer_project::settings::SplitSettings;
use openslicer_project::stock::build;

fn printer(prefix: &str) -> PrinterProfile {
    PrinterProfile::presets().into_iter().find(|p| p.name.starts_with(prefix)).unwrap()
}

fn run(id: &str, printer: &PrinterProfile, kind: ConnectorKind) -> SplitOutcome {
    let settings = SplitSettings { connectors: Some(ConnectorSettings::new(kind)), nest: true, ..Default::default() };
    let out = run_split(&build(id).unwrap(), printer, &settings, &Control::none()).unwrap();
    assert!(out.pieces.len() > 1, "{id} is bigger than {} and must be cut", printer.name);
    assert!(out.fits.iter().all(|f| f.fits), "{id}: every part must fit {}", printer.name);
    assert!(out.pieces.iter().all(|p| p.mesh.stats().is_watertight()), "{id}: every part must be a closed solid");
    out
}

#[test]
fn the_solid_wing_splits_and_connects_on_common_printers() {
    for name in ["Bambu Lab A1 mini", "Prusa MK4", "Creality K1 Max"] {
        let out = run("wing", &printer(name), ConnectorKind::Pin);
        eprintln!("wing on {name}: {} parts, {} connectors", out.pieces.len(), out.connectors.len());
        assert!(out.problems.is_empty(), "{name}: {:?}", out.problems);
        assert!(out.connectors.len() >= out.interfaces.len(), "{name}: every interface needs a connector");
    }
}

/// Regression: a grid rotated off the axes used to yield no interfaces, so the parts came out unconnected
/// with only a note. Auto-tiling picks a 75° grid for the wing on an Ender-3.
#[test]
fn a_rotated_tiling_still_finds_every_interface() {
    let out = run("wing", &printer("Creality Ender-3 /"), ConnectorKind::Pin);
    assert!(out.notes.iter().any(|n| n.contains("rotated 75")), "{:?}", out.notes);
    assert!(!out.interfaces.is_empty());
    assert!(out.connectors.len() >= out.interfaces.len() || !out.problems.is_empty());
}

/// A 3-4 mm curved shell cannot take 6 mm pins, so every seam falls back to a finger joint. The whole assembly
/// is checked for interpenetration.
#[test]
fn thin_curved_shells_are_joined_by_finger_joints() {
    for (id, prefix) in [("dome", "Prusa MK4"), ("panel", "Prusa MK4"), ("panel", "Bambu Lab A1 mini")] {
        let printer = printer(prefix);
        let settings =
            SplitSettings { connectors: Some(ConnectorSettings::new(ConnectorKind::Pin)), verify_assembly: true, ..Default::default() };
        let out = run_split(&build(id).unwrap(), &printer, &settings, &Control::none()).unwrap();
        let report = out.joint_report.as_ref().expect("connectors were requested");
        eprintln!(
            "{id} on {prefix}: {} parts, {}/{} interfaces connected, problems {:?}",
            out.pieces.len(),
            report.interfaces_connected,
            report.interfaces_total,
            out.problems
        );
        assert!(out.fits.iter().all(|f| f.fits));
        assert!(out.pieces.iter().all(|p| p.mesh.stats().is_watertight()), "{id}: every part must be closed");
        assert_eq!(report.interfaces_connected, report.interfaces_total, "{id}: {:?}", report.skipped);
        assert!(out.connectors.iter().all(|c| c.kind == ConnectorKind::Comb));
        assert!(out.problems.is_empty(), "{id} on {prefix}: {:?}", out.problems);
        // Material is only moved or removed as clearance: pieces add up to the source minus the clearance gaps.
        let total: f64 = out.pieces.iter().map(|p| p.volume).sum();
        let gaps = report.volume_removed - report.volume_added;
        let expected = out.source.signed_volume - gaps;
        assert!(
            (total - expected).abs() < 2e-3 * out.source.signed_volume,
            "{id}: parts total {total:.0} mm³, expected {expected:.0} (gaps {gaps:.0})"
        );
        assert!(gaps > 0.0 && gaps < 0.05 * out.source.signed_volume, "{id}: clearance gaps {gaps:.1} mm³");
    }
}

#[test]
fn labels_are_engraved_or_raised_and_verified_on_real_parts() {
    use openslicer_project::labels::{LabelMode, LabelOptions};
    for (id, prefix, mode) in [
        ("wing", "Prusa MK4", LabelMode::Inset),
        ("wing", "Prusa MK4", LabelMode::Outset),
        ("dome", "Prusa MK4", LabelMode::Inset),
        ("dome", "Prusa MK4", LabelMode::Outset),
        ("panel", "Prusa MK4", LabelMode::Inset),
    ] {
        let settings = SplitSettings {
            connectors: Some(ConnectorSettings::new(ConnectorKind::Pin)),
            labels: LabelOptions { mode, ..Default::default() },
            ..Default::default()
        };
        let out = run_split(&build(id).unwrap(), &printer(prefix), &settings, &Control::none()).unwrap();
        let placed = out.labels.iter().filter(|l| l.placed).count();
        eprintln!("{id} {mode:?}: {placed}/{} labelled; problems {:?}", out.labels.len(), out.problems);
        for l in &out.labels {
            eprintln!("   {} placed={} {}", l.text, l.placed, l.note);
        }
        assert_eq!(out.labels.len(), out.pieces.len());
        assert!(out.pieces.iter().all(|p| p.mesh.stats().is_watertight()), "{id}: parts must stay closed");
        assert!(out.problems.is_empty(), "{id}: {:?}", out.problems);
    }
}

/// Regression: finger-joint parts used to contain near-duplicate vertices that the importer welded into
/// non-manifold edges, so the export verification (re-import) failed. Every stock model must export and verify,
/// with labels and the assembly CSV.
#[test]
fn every_stock_model_exports_and_verifies_after_a_round_trip() {
    use openslicer_project::export::{export, verify_export, ExportOptions};
    use openslicer_project::labels::{LabelMode, LabelOptions};
    for (id, mode) in [("dome", LabelMode::Inset), ("panel", LabelMode::Outset), ("wing", LabelMode::Inset)] {
        let printer = printer("Prusa MK4");
        let settings = SplitSettings {
            connectors: Some(ConnectorSettings::new(ConnectorKind::Pin)),
            labels: LabelOptions { mode, ..Default::default() },
            nest: true,
            ..Default::default()
        };
        let out = run_split(&build(id).unwrap(), &printer, &settings, &Control::none()).unwrap();
        assert!(out.problems.is_empty(), "{id}: {:?}", out.problems);
        let dir = std::env::temp_dir().join(format!("openslicer-stock-export-{id}-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        let rep = export(&out, &printer, &ExportOptions { model_name: id.into(), ..Default::default() }, &dir, &Control::none())
            .unwrap_or_else(|e| panic!("{id}: {e}"));
        assert!(rep.verification.is_ok(), "{id}: {:?}", rep.verification);
        assert!(verify_export(&dir).is_ok());
        let csv = std::fs::read_to_string(dir.join(format!("{id}_assembly.csv"))).unwrap();
        assert_eq!(csv.lines().count(), 1 + out.pieces.len());
        assert!(rep.manifest.parts.iter().all(|p| p.label.is_some()), "{id}: every part carries its label");
        let _ = std::fs::remove_dir_all(&dir);
    }
}

/// A hand-made choice for one join replaces the automatic connector for that join only, a join left plain is
/// reported as a choice (not as a failure), and finger joints are marked as needing glue in the instructions.
#[test]
fn per_join_choices_replace_the_automatic_connector_and_finger_joints_need_glue() {
    use openslicer_joints::settings::InterfaceOverride;
    use openslicer_project::assembly::plan_assembly;
    let printer = printer("Prusa MK4");
    let model = build("wing").unwrap();
    let base = SplitSettings { connectors: Some(ConnectorSettings::new(ConnectorKind::Pin)), ..Default::default() };
    let auto = run_split(&model, &printer, &base, &Control::none()).unwrap();
    assert!(auto.problems.is_empty(), "{:?}", auto.problems);
    assert!(auto.interfaces.len() >= 3);
    let (first, second) = (&auto.interfaces[0], &auto.interfaces[1]);

    let mut cs = ConnectorSettings::new(ConnectorKind::Pin);
    cs.overrides = vec![
        InterfaceOverride { a: first.b, b: first.a, choice: None },
        InterfaceOverride { a: second.a, b: second.b, choice: Some(ConnectorKind::Comb) },
    ];
    let out = run_split(&model, &printer, &SplitSettings { connectors: Some(cs), ..Default::default() }, &Control::none()).unwrap();
    assert!(out.problems.is_empty(), "{:?}", out.problems);
    assert_eq!(out.joint_report.as_ref().unwrap().left_open, 1);
    let on = |ii: usize| -> Vec<ConnectorKind> { out.connectors.iter().filter(|c| c.interface == ii).map(|c| c.kind).collect() };
    assert!(on(0).is_empty(), "the join left plain has no connector");
    assert!(on(1).iter().all(|k| *k == ConnectorKind::Comb) && !on(1).is_empty(), "the chosen join uses a finger joint: {:?}", on(1));
    assert!((2..out.interfaces.len()).all(|i| on(i).iter().all(|k| *k == ConnectorKind::Pin)), "the other joins keep the default");
    assert!(out.notes.iter().any(|n| n.contains("left plain by your choice")));

    let steps = plan_assembly(&out.pieces, &out.interfaces, &out.connectors, &out.loose);
    assert!(steps.iter().any(|s| s.needs_glue), "a finger joint is marked as needing glue");
    let plain = plan_assembly(&auto.pieces, &auto.interfaces, &auto.connectors, &auto.loose);
    assert!(plain.iter().all(|s| !s.needs_glue), "pins never need glue");
}

/// STEP export of real finger-joint parts re-imports with the same volume.
#[test]
fn step_export_of_a_split_model_verifies() {
    use openslicer_project::export::{export, ExportFormat, ExportOptions};
    let printer = printer("Prusa MK4");
    let settings = SplitSettings { connectors: Some(ConnectorSettings::new(ConnectorKind::Comb)), ..Default::default() };
    let out = run_split(&build("dome").unwrap(), &printer, &settings, &Control::none()).unwrap();
    assert!(out.problems.is_empty(), "{:?}", out.problems);
    let dir = std::env::temp_dir().join(format!("openslicer-step-export-{}", std::process::id()));
    let _ = std::fs::remove_dir_all(&dir);
    let opts = ExportOptions { model_name: "dome".into(), format: ExportFormat::Step, ..Default::default() };
    let rep = export(&out, &printer, &opts, &dir, &Control::none()).unwrap();
    assert!(rep.verification.is_ok(), "{:?}", rep.verification);
    assert!(rep.manifest.parts.iter().all(|p| p.file.ends_with(".step")));
    let _ = std::fs::remove_dir_all(&dir);
}
