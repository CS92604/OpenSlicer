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

/// Thin curved shells cannot take 6 mm pins; the pipeline must say so instead of reporting success.
#[test]
fn thin_shells_are_never_reported_clean_when_they_were_left_unconnected() {
    for id in ["dome", "panel"] {
        let out = run(id, &printer("Prusa MK4"), ConnectorKind::Pin);
        let report = out.joint_report.as_ref().expect("connectors were requested");
        assert!(report.interfaces_connected < report.interfaces_total, "{id}: a 3-4 mm wall cannot hold a pin");
        assert!(!out.problems.is_empty(), "{id}: unconnected interfaces must be reported as problems");
        assert!(report.skipped.iter().all(|(_, why)| why.contains("thick")), "{id}: {:?}", report.skipped);
    }
}
