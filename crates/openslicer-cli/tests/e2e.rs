//! End-to-end tests: real files in, real files out, through the same entry point as the binary.

use glam::Vec3;
use openslicer_core::Mesh;
use openslicer_io::{load_mesh, save_mesh};
use std::path::PathBuf;

fn dir(tag: &str) -> PathBuf {
    let d = std::env::temp_dir().join(format!("openslicer-cli-{tag}-{}", std::process::id()));
    let _ = std::fs::remove_dir_all(&d);
    std::fs::create_dir_all(&d).unwrap();
    d
}

fn run(args: &[&str]) -> (i32, String, String) {
    let (mut out, mut err) = (Vec::new(), Vec::new());
    let mut argv = vec!["openslicer-cli"];
    argv.extend_from_slice(args);
    let code = openslicer_cli::run(argv, &mut out, &mut err);
    (code, String::from_utf8(out).unwrap(), String::from_utf8(err).unwrap())
}

fn block(path: &std::path::Path, size: Vec3) {
    let mut m = Mesh::cuboid(size);
    m.translate(size * 0.5);
    save_mesh(path, &m).unwrap();
}

#[test]
fn split_connect_export_and_verify_a_block_larger_than_the_printer() {
    let d = dir("split");
    let model = d.join("big.stl");
    block(&model, Vec3::new(500.0, 300.0, 100.0));
    let out = d.join("out");
    let (code, text, err) =
        run(&["split", model.to_str().unwrap(), "-o", out.to_str().unwrap(), "--connector", "pin", "--nest", "--verify-assembly"]);
    assert_eq!(code, 0, "{text}\n{err}");
    assert!(text.contains("parts from") && text.contains("fit the printer") && text.contains("verified by re-import"), "{text}");
    assert!(out.join("manifest.json").is_file() && out.join("big_instructions.html").is_file());
    let (code, text, _) = run(&["verify", out.to_str().unwrap()]);
    assert_eq!(code, 0, "{text}");
    // Total part volume equals the source (pins only move material between parts, minus clearance gaps).
    let mut total = 0.0;
    for e in std::fs::read_dir(&out).unwrap().flatten() {
        let p = e.path();
        if p.extension().is_some_and(|x| x == "stl") && !p.to_string_lossy().contains("plate") {
            let m = load_mesh(&p).unwrap();
            assert!(m.stats().is_watertight(), "{}", p.display());
            let b = m.bounds().unwrap().size();
            assert!(b.x <= 220.01 && b.y <= 220.01 || b.x <= 220.01 && b.z <= 220.01 || b.y <= 220.01 && b.z <= 220.01);
            total += m.stats().signed_volume;
        }
    }
    let source = 500.0 * 300.0 * 100.0;
    assert!((total - source).abs() / source < 0.01, "{total} vs {source}");
    // Corrupt a part: verify must now fail.
    let victim = std::fs::read_dir(&out).unwrap().flatten().map(|e| e.path()).find(|p| p.extension().is_some_and(|x| x == "stl")).unwrap();
    let mut bytes = std::fs::read(&victim).unwrap();
    let n = bytes.len();
    bytes[n - 5] ^= 0xff;
    std::fs::write(&victim, bytes).unwrap();
    assert_eq!(run(&["verify", out.to_str().unwrap()]).0, 1);
    let _ = std::fs::remove_dir_all(&d);
}

#[test]
fn batch_split_writes_one_folder_per_model_and_reports_failures() {
    let d = dir("batch");
    let (a, b) = (d.join("a.stl"), d.join("b.stl"));
    block(&a, Vec3::new(300.0, 100.0, 50.0));
    block(&b, Vec3::new(100.0, 400.0, 50.0));
    let out = d.join("out");
    let (code, text, _) =
        run(&["split", a.to_str().unwrap(), b.to_str().unwrap(), "-o", out.to_str().unwrap(), "--printer", "ender-3 v3 ke"]);
    assert_eq!(code, 0, "{text}");
    assert!(out.join("a/manifest.json").is_file() && out.join("b/manifest.json").is_file());
    // A missing input fails that model but the others still run, and the exit code says so.
    let out2 = d.join("out2");
    let (code, text, _) = run(&["split", a.to_str().unwrap(), d.join("missing.stl").to_str().unwrap(), "-o", out2.to_str().unwrap()]);
    assert_eq!(code, 1, "{text}");
    assert!(out2.join("a/manifest.json").is_file() && text.contains("FAILED"));
    let _ = std::fs::remove_dir_all(&d);
}

#[test]
fn explicit_layouts_and_every_output_format() {
    let d = dir("layouts");
    let m = d.join("m.stl");
    block(&m, Vec3::new(300.0, 100.0, 50.0));
    for (i, extra) in [
        vec!["--layout", "grid", "--grid", "2,1,1"],
        vec!["--layout", "modular", "--spacing", "150,-,-"],
        vec!["--layout", "planes", "--plane", "100,0,0:1,0,0"],
        vec!["--layout", "grid", "--grid", "2,1,1", "--format", "3mf"],
        vec!["--layout", "grid", "--grid", "2,1,1", "--format", "obj", "--print-orientation"],
    ]
    .into_iter()
    .enumerate()
    {
        let out = d.join(format!("o{i}"));
        let mut args = vec!["split", m.to_str().unwrap(), "-o", out.to_str().unwrap()];
        args.extend(extra.iter());
        let (code, text, err) = run(&args);
        assert_eq!(code, 0, "{extra:?}: {text}{err}");
        assert_eq!(run(&["verify", out.to_str().unwrap()]).0, 0);
    }
    let _ = std::fs::remove_dir_all(&d);
}

#[test]
fn parts_that_cannot_fit_exit_with_the_problems_code() {
    let d = dir("nofit");
    let m = d.join("m.stl");
    block(&m, Vec3::new(300.0, 100.0, 50.0));
    // One cut cannot make a 300 mm block fit a 100 mm printer.
    let out = d.join("out");
    let (code, text, _) =
        run(&["split", m.to_str().unwrap(), "-o", out.to_str().unwrap(), "--volume", "100,100,100", "--layout", "grid", "--grid", "2,1,1"]);
    assert_eq!(code, 3, "{text}");
    assert!(text.contains("DO NOT fit") || text.contains("PROBLEM"), "{text}");
    let _ = std::fs::remove_dir_all(&d);
}

#[test]
fn info_repair_hollow_and_boolean() {
    let d = dir("tools");
    let m = d.join("cube.stl");
    block(&m, Vec3::new(60.0, 60.0, 60.0));
    let (code, text, _) = run(&["info", m.to_str().unwrap()]);
    assert_eq!(code, 0);
    assert!(text.contains("216.0 cm³") && text.contains("closed solid") && text.contains("fits in one piece"), "{text}");
    let (_, json, _) = run(&["info", "--json", m.to_str().unwrap()]);
    let v: serde_json::Value = serde_json::from_str(&json).unwrap();
    assert_eq!(v[0]["triangles"], 12);
    // Hollow: the result is a closed shell with less material.
    let h = d.join("hollow.stl");
    let (code, text, _) = run(&["hollow", m.to_str().unwrap(), h.to_str().unwrap(), "--wall", "4", "--drain-holes", "1"]);
    assert_eq!(code, 0, "{text}");
    let hm = load_mesh(&h).unwrap().stats();
    assert!(hm.signed_volume < 216_000.0 * 0.5 && hm.boundary_edges == 0, "{hm:?}");
    // Boolean: subtract an overlapping cube.
    let other = d.join("other.stl");
    let mut o = Mesh::cuboid(Vec3::splat(30.0));
    o.translate(Vec3::splat(60.0)); // spans 45..75, overlapping the cube by 15 mm per axis
    save_mesh(&other, &o).unwrap();
    let r = d.join("sub.stl");
    let (code, text, _) = run(&["boolean", "subtract", m.to_str().unwrap(), other.to_str().unwrap(), r.to_str().unwrap()]);
    assert_eq!(code, 0, "{text}");
    let removed = 216_000.0 - load_mesh(&r).unwrap().stats().signed_volume;
    assert!((removed - 15.0 * 15.0 * 15.0).abs() < 1.0, "{removed}");
    // Repair a mesh with a hole.
    let mut broken = Mesh::cuboid(Vec3::splat(20.0));
    broken.triangles.pop();
    let (bp, fp) = (d.join("broken.stl"), d.join("fixed.stl"));
    save_mesh(&bp, &broken).unwrap();
    assert_eq!(run(&["info", bp.to_str().unwrap()]).0, 3);
    let (code, text, _) = run(&["repair", bp.to_str().unwrap(), fp.to_str().unwrap()]);
    assert_eq!(code, 0, "{text}");
    assert!(load_mesh(&fp).unwrap().stats().is_watertight());
    let _ = std::fs::remove_dir_all(&d);
}

#[test]
fn fab_methods_and_image_tools_write_verified_output() {
    let d = dir("fab");
    let m = d.join("m.stl");
    block(&m, Vec3::new(100.0, 60.0, 40.0));
    let cases: Vec<Vec<&str>> = vec![
        vec!["fab", "stack", "--layer", "10"],
        vec!["fab", "interlock", "--spacing", "30"],
        vec!["fab", "blocks", "--cell", "20"],
        vec!["fab", "wireframe", "--spacing", "20"],
        vec!["fab", "relief", "--pixel", "2"],
    ];
    for (i, c) in cases.iter().enumerate() {
        let out = d.join(format!("f{i}"));
        let mut args = c.clone();
        args.extend([m.to_str().unwrap(), "-o", out.to_str().unwrap()]);
        let (code, text, err) = run(&args);
        assert!(code == 0, "{c:?} → {code}: {text}{err}");
        assert_eq!(run(&["verify", out.to_str().unwrap()]).0, 0, "{c:?}");
    }
    // Image: a small gradient PNG.
    let img = d.join("g.png");
    let buf = image_gradient();
    std::fs::write(&img, buf).unwrap();
    for (i, kind) in ["lithophane", "relief", "halftone"].into_iter().enumerate() {
        let out = d.join(format!("i{i}"));
        let (code, text, err) = run(&["image", kind, img.to_str().unwrap(), "-o", out.to_str().unwrap(), "--width", "40"]);
        assert_eq!(code, 0, "{kind}: {text}{err}");
        assert_eq!(run(&["verify", out.to_str().unwrap()]).0, 0, "{kind}");
    }
    let _ = std::fs::remove_dir_all(&d);
}

fn image_gradient() -> Vec<u8> {
    let mut png = Vec::new();
    let img = image::GrayImage::from_fn(32, 24, |x, y| image::Luma([((x * 8 + y * 3) % 256) as u8]));
    img.write_to(&mut std::io::Cursor::new(&mut png), image::ImageFormat::Png).unwrap();
    png
}

#[test]
fn usage_errors_and_listings() {
    let (code, _, err) = run(&["split"]);
    assert_eq!(code, 2, "{err}");
    let (code, _, err) = run(&["split", "x.stl", "-o", "o", "--connector", "banana"]);
    assert_eq!(code, 1);
    assert!(err.contains("unknown connector") || err.contains("cannot read") || err.contains("x.stl"), "{err}");
    let (code, text, _) = run(&["printers"]);
    assert_eq!(code, 0);
    assert!(text.contains("Ender-3") && text.contains("Voron"));
    assert_eq!(run(&["--version"]).0, 0);
}

#[test]
fn stock_models_can_be_listed_written_and_split() {
    let d = dir("stock");
    let (code, text, _) = run(&["stock"]);
    assert_eq!(code, 0);
    assert!(text.contains("dome") && text.contains("wing") && text.contains("panel"), "{text}");
    let wing = d.join("wing.stl");
    let (code, text, _) = run(&["stock", "wing", "-o", wing.to_str().unwrap()]);
    assert_eq!(code, 0, "{text}");
    assert_eq!(run(&["stock", "nope", "-o", d.join("x.stl").to_str().unwrap()]).0, 1);
    let out = d.join("out");
    let (code, text, _) =
        run(&["split", wing.to_str().unwrap(), "-o", out.to_str().unwrap(), "--printer", "prusa mk4", "--connector", "pin"]);
    assert_eq!(code, 0, "{text}");
    assert_eq!(run(&["verify", out.to_str().unwrap()]).0, 0);
    let _ = std::fs::remove_dir_all(&d);
}

#[test]
fn thin_shell_with_labels_writes_a_csv_and_verifies() {
    let d = dir("labels");
    let dome = d.join("dome.stl");
    assert_eq!(run(&["stock", "dome", "-o", dome.to_str().unwrap()]).0, 0);
    let out = d.join("out");
    let (code, text, _) = run(&[
        "split",
        dome.to_str().unwrap(),
        "-o",
        out.to_str().unwrap(),
        "--printer",
        "prusa mk4",
        "--connector",
        "pin",
        "--labels",
        "inset",
    ]);
    assert_eq!(code, 0, "thin walls are joined by finger joints, so nothing is left unconnected: {text}");
    let csv = std::fs::read_to_string(out.join("dome_assembly.csv")).unwrap();
    let lines: Vec<&str> = csv.lines().collect();
    assert!(lines.len() >= 3, "{csv}");
    assert!(lines[1].contains("P0") && lines[1].contains("-1"), "first step carries its label: {}", lines[1]);
    assert!(csv.contains("Finger joint"), "{csv}");
    assert_eq!(run(&["verify", out.to_str().unwrap()]).0, 0);
    // A bad mode is a usage-style failure, not a silent default.
    let (code, text, _) = run(&["split", dome.to_str().unwrap(), "-o", d.join("o2").to_str().unwrap(), "--labels", "sideways"]);
    assert_ne!(code, 0, "{text}");
    let _ = std::fs::remove_dir_all(&d);
}

#[test]
fn step_files_are_read_split_and_written() {
    let d = dir("step");
    let model = d.join("big.step");
    block(&model, Vec3::new(300.0, 100.0, 50.0));
    let (code, text, err) = run(&["info", model.to_str().unwrap()]);
    assert_eq!(code, 0, "{text}{err}");
    let out = d.join("out");
    let (code, text, err) = run(&[
        "split",
        model.to_str().unwrap(),
        "-o",
        out.to_str().unwrap(),
        "--layout",
        "grid",
        "--grid",
        "2,1,1",
        "--connector",
        "comb",
        "--format",
        "step",
    ]);
    assert_eq!(code, 0, "{text}{err}");
    let parts: Vec<_> =
        std::fs::read_dir(&out).unwrap().flatten().map(|e| e.path()).filter(|p| p.extension().is_some_and(|x| x == "step")).collect();
    assert_eq!(parts.len(), 2);
    let csv = std::fs::read_to_string(out.join("big_assembly.csv")).unwrap();
    assert!(
        csv.lines().next().unwrap().contains("needs_glue") && csv.contains(",yes,"),
        "finger joints are marked as needing glue:\n{csv}"
    );
    assert_eq!(run(&["verify", out.to_str().unwrap()]).0, 0);
    let _ = std::fs::remove_dir_all(&d);
}
