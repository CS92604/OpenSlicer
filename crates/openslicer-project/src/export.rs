//! Exporting a split result: one file per part, optional plate files, printable instructions and a
//! `manifest.json` with SHA-256 hashes. Every export is re-imported and checked before it is reported as done.

use crate::assembly::{instructions_html, plan_assembly, AssemblyStep, DocInput};
use crate::pipeline::SplitOutcome;
use crate::profile::PrinterProfile;
use glam::DVec3;
use openslicer_core::control::{Cancelled, Control};
use openslicer_core::Mesh;
use openslicer_io::{load_mesh, save_mesh, write_atomic, IoError};
use openslicer_split::exec::Piece;
use openslicer_split::fit::{apply_orientation, best_orientation, most_compact_orientation, Orientation};
use openslicer_split::nest::plate_mesh;
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::path::{Path, PathBuf};

pub const MANIFEST_NAME: &str = "manifest.json";
pub const MANIFEST_VERSION: u32 = 1;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum ExportFormat {
    Stl,
    Obj,
    #[serde(rename = "3mf")]
    ThreeMf,
}

impl ExportFormat {
    pub fn extension(self) -> &'static str {
        match self {
            ExportFormat::Stl => "stl",
            ExportFormat::Obj => "obj",
            ExportFormat::ThreeMf => "3mf",
        }
    }
    pub fn parse(s: &str) -> Option<Self> {
        match s.to_ascii_lowercase().as_str() {
            "stl" => Some(Self::Stl),
            "obj" => Some(Self::Obj),
            "3mf" => Some(Self::ThreeMf),
            _ => None,
        }
    }
}

#[derive(Clone, Debug)]
pub struct ExportOptions {
    pub format: ExportFormat,
    /// File name template; `{model}`, `{name}` (for example `P03` or `D01`) and `{id}` are replaced.
    pub name_template: String,
    pub model_name: String,
    /// Write each part in its print orientation (resting on a flat face where possible) instead of in assembly pose.
    pub print_orientation: bool,
    /// Also write one file per build plate when the outcome was nested.
    pub write_plates: bool,
    pub write_instructions: bool,
    /// Also write the assembly steps as a CSV table (for spreadsheets and kitting lists).
    pub write_csv: bool,
}

impl Default for ExportOptions {
    fn default() -> Self {
        Self {
            format: ExportFormat::Stl,
            name_template: "{model}_{name}".into(),
            model_name: "model".into(),
            print_orientation: false,
            write_plates: true,
            write_instructions: true,
            write_csv: true,
        }
    }
}

#[derive(Debug, thiserror::Error)]
pub enum ExportError {
    #[error("{0}")]
    Invalid(String),
    #[error(transparent)]
    Io(#[from] IoError),
    #[error("could not write {0}: {1}")]
    Fs(PathBuf, std::io::Error),
    #[error("the exported files did not pass verification: {}", .0.join("; "))]
    Verification(Vec<String>),
    #[error(transparent)]
    Cancelled(#[from] Cancelled),
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct PartEntry {
    pub id: usize,
    pub name: String,
    /// `"piece"` or `"loose"` (for example a dowel).
    pub kind: String,
    /// Descriptive label from fabrication methods, for example `Layer 4`.
    #[serde(default)]
    pub label: Option<String>,
    pub file: String,
    pub sha256: String,
    pub triangles: usize,
    pub volume_mm3: f64,
    pub bounds_min: [f64; 3],
    pub bounds_max: [f64; 3],
    pub size: [f64; 3],
    pub watertight: bool,
    /// Whether the part fits the printer in the pose it was exported in or, for assembly-pose files, in some orientation.
    pub fits_printer: bool,
    /// Row-major rotation applied to the assembly pose, present when `print_orientation` was used.
    pub print_rotation: Option<[[f64; 3]; 3]>,
    /// Translation applied after the rotation (so `exported = R * assembly + offset`).
    pub print_offset: Option<[f64; 3]>,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct PlateEntry {
    pub index: usize,
    pub file: String,
    pub sha256: String,
    pub triangles: usize,
    pub parts: Vec<String>,
    pub height_mm: f64,
}

/// A 2D cutting outline (SVG, millimetres) for flat stock.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct OutlineEntry {
    pub part: String,
    pub file: String,
    pub sha256: String,
    pub thickness_mm: f64,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct InterfaceEntry {
    pub between: [String; 2],
    pub area_mm2: f64,
    pub plane_normal: [f64; 3],
    pub plane_offset: f64,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ConnectorEntry {
    pub id: String,
    pub kind: String,
    pub male_part: Option<String>,
    pub female_part: String,
    pub position: [f64; 3],
    pub direction: [f64; 3],
    pub size_mm: f64,
    pub height_mm: f64,
    pub angle_deg: f64,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct StepEntry {
    pub step: usize,
    pub part: String,
    pub attaches_to: Vec<String>,
    pub connectors: Vec<String>,
    pub loose_parts: Vec<String>,
    pub starts_subassembly: bool,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct PrinterEntry {
    pub name: String,
    pub build_volume: [f64; 3],
    pub margin: f64,
    pub usable: [f64; 3],
    pub joint_clearance: f64,
    pub material: String,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Validation {
    /// True when the pipeline recorded no problems.
    pub valid: bool,
    pub problems: Vec<String>,
    pub notes: Vec<String>,
    pub source_volume_mm3: f64,
    pub source_watertight: bool,
    pub cut_count: usize,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Manifest {
    pub manifest_version: u32,
    pub generator: String,
    pub model_name: String,
    pub format: ExportFormat,
    pub print_orientation: bool,
    pub printer: PrinterEntry,
    pub parts: Vec<PartEntry>,
    pub plates: Vec<PlateEntry>,
    #[serde(default)]
    pub outlines: Vec<OutlineEntry>,
    /// Fabrication method (`stack`, `interlock`, ...) or `split`.
    #[serde(default)]
    pub method: String,
    pub interfaces: Vec<InterfaceEntry>,
    pub connectors: Vec<ConnectorEntry>,
    pub assembly: Vec<StepEntry>,
    pub instructions: Option<String>,
    /// The assembly steps as CSV.
    #[serde(default)]
    pub assembly_csv: Option<String>,
    pub validation: Validation,
}

#[derive(Clone, Debug)]
pub struct ExportReport {
    pub dir: PathBuf,
    pub manifest: Manifest,
    pub files: Vec<String>,
    /// Result of re-importing everything that was written.
    pub verification: VerifyReport,
}

#[derive(Clone, Debug, Default)]
pub struct VerifyReport {
    pub files_checked: usize,
    pub problems: Vec<String>,
}

impl VerifyReport {
    pub fn is_ok(&self) -> bool {
        self.problems.is_empty()
    }
}

fn sanitize(s: &str) -> String {
    s.chars().map(|c| if c.is_ascii_alphanumeric() || matches!(c, '-' | '_' | '.') { c } else { '_' }).collect()
}

fn hash_file(path: &Path) -> Result<String, ExportError> {
    let bytes = std::fs::read(path).map_err(|e| ExportError::Fs(path.to_path_buf(), e))?;
    Ok(format!("{:x}", Sha256::digest(&bytes)))
}

fn v3(v: DVec3) -> [f64; 3] {
    [v.x, v.y, v.z]
}

fn print_pose(piece: &Piece, usable: DVec3, down: &[DVec3]) -> Option<(Mesh, Orientation)> {
    let o = best_orientation(&piece.mesh, usable, down).or_else(|| most_compact_orientation(&piece.mesh, down))?;
    Some((apply_orientation(&piece.mesh, &o), o))
}

/// Writes the outcome into `dir` (created if needed) and verifies the result. `dir` must be empty or not exist,
/// so a previous export is never mixed with a new one.
pub fn export(
    outcome: &SplitOutcome,
    printer: &PrinterProfile,
    opts: &ExportOptions,
    dir: &Path,
    ctl: &Control,
) -> Result<ExportReport, ExportError> {
    let existed = dir.exists();
    let result = export_into(outcome, printer, opts, dir, ctl);
    if let Err(e) = &result {
        cleanup_after_failure(e, existed, dir);
    }
    result
}

fn prepare_dir(opts: &ExportOptions, dir: &Path) -> Result<(), ExportError> {
    if !opts.name_template.contains("{name}") && !opts.name_template.contains("{id}") {
        return Err(ExportError::Invalid("the file name template must contain {name} or {id} so each part gets its own file".into()));
    }
    if dir.exists() {
        let non_empty = std::fs::read_dir(dir).map_err(|e| ExportError::Fs(dir.to_path_buf(), e))?.next().is_some();
        if non_empty {
            return Err(ExportError::Invalid(format!("{} is not empty; choose an empty folder", dir.display())));
        }
    }
    std::fs::create_dir_all(dir).map_err(|e| ExportError::Fs(dir.to_path_buf(), e))
}

/// Writes one file per part; returns the manifest entries and the file names.
fn write_parts(
    all: &[(&Piece, &str)],
    fits: &dyn Fn(&Piece) -> bool,
    usable: DVec3,
    opts: &ExportOptions,
    dir: &Path,
    ctl: &Control,
) -> Result<(Vec<PartEntry>, Vec<String>), ExportError> {
    let model = sanitize(&opts.model_name);
    let ext = opts.format.extension();
    let mut names: Vec<String> = Vec::new();
    for (p, _) in all {
        let stem = opts.name_template.replace("{model}", &model).replace("{name}", &sanitize(&p.name)).replace("{id}", &p.id.to_string());
        let file = format!("{}.{ext}", sanitize(&stem));
        if names.contains(&file) {
            return Err(ExportError::Invalid(format!("the file name template produces the same name '{file}' for more than one part")));
        }
        names.push(file);
    }
    let mut files = Vec::new();
    let mut parts = Vec::new();
    for (i, ((piece, kind), file)) in all.iter().zip(&names).enumerate() {
        ctl.check()?;
        ctl.report(i as f32 / all.len() as f32 * 0.8, &format!("Writing {}", piece.name));
        let fits_in_some = fits(piece);
        let (mesh, rot, off) = if opts.print_orientation {
            let down: Vec<DVec3> = Vec::new();
            match print_pose(piece, usable, &down) {
                Some((m, o)) => {
                    // Offset that apply_orientation subtracted: recompute from the rotated bounds.
                    let rotated_min = piece.mesh.vertices.iter().fold(DVec3::splat(f64::MAX), |mn, v| mn.min(o.rotation * v.as_dvec3()));
                    let r = o.rotation;
                    let rows =
                        [[r.x_axis.x, r.y_axis.x, r.z_axis.x], [r.x_axis.y, r.y_axis.y, r.z_axis.y], [r.x_axis.z, r.y_axis.z, r.z_axis.z]];
                    (m, Some(rows), Some(v3(-rotated_min)))
                }
                None => (piece.mesh.clone(), None, None),
            }
        } else {
            (piece.mesh.clone(), None, None)
        };
        let path = dir.join(file);
        save_mesh(&path, &mesh)?;
        let stats = mesh.stats();
        let b = stats.bounds.ok_or_else(|| ExportError::Invalid(format!("{} is empty", piece.name)))?;
        parts.push(PartEntry {
            id: piece.id,
            name: piece.name.clone(),
            kind: (*kind).into(),
            label: None,
            file: file.clone(),
            sha256: hash_file(&path)?,
            triangles: stats.triangle_count,
            volume_mm3: stats.signed_volume,
            bounds_min: v3(b.min.as_dvec3()),
            bounds_max: v3(b.max.as_dvec3()),
            size: v3(b.size().as_dvec3()),
            watertight: stats.is_watertight(),
            fits_printer: fits_in_some,
            print_rotation: rot,
            print_offset: off,
        });
        files.push(file.clone());
    }
    Ok((parts, files))
}

fn export_into(
    outcome: &SplitOutcome,
    printer: &PrinterProfile,
    opts: &ExportOptions,
    dir: &Path,
    ctl: &Control,
) -> Result<ExportReport, ExportError> {
    if outcome.pieces.is_empty() {
        return Err(ExportError::Invalid("there are no parts to export".into()));
    }
    prepare_dir(opts, dir)?;
    let usable = printer.usable();
    let all: Vec<(&Piece, &str)> = outcome.pieces.iter().map(|p| (p, "piece")).chain(outcome.loose.iter().map(|p| (p, "loose"))).collect();
    let model = sanitize(&opts.model_name);
    let ext = opts.format.extension();
    let fits = |p: &Piece| outcome.fits.iter().find(|f| f.piece_id == p.id && f.name == p.name).is_none_or(|f| f.fits);
    let (mut parts, mut files) = write_parts(&all, &fits, usable, opts, dir, ctl)?;
    for p in &mut parts {
        p.label = outcome.labels.iter().find(|l| l.placed && l.piece_id == p.id && p.kind == "piece").map(|l| l.text.clone());
    }

    // Plates.
    let mut plates = Vec::new();
    if opts.write_plates {
        if let Some(n) = &outcome.nest {
            let all_pieces: Vec<Piece> = outcome.pieces.iter().chain(outcome.loose.iter()).cloned().collect();
            for (i, plate) in n.plates.iter().enumerate() {
                let m = plate_mesh(plate, &all_pieces);
                let file = format!("{}_plate_{:02}.{ext}", model, i + 1);
                let path = dir.join(&file);
                save_mesh(&path, &m)?;
                plates.push(PlateEntry {
                    index: i + 1,
                    sha256: hash_file(&path)?,
                    triangles: m.triangles.len(),
                    parts: plate
                        .placements
                        .iter()
                        .filter_map(|p| all_pieces.iter().find(|q| q.id == p.piece_id))
                        .map(|p| p.name.clone())
                        .collect(),
                    height_mm: plate.height,
                    file: file.clone(),
                });
                files.push(file);
            }
        }
    }

    // Assembly information.
    let steps = plan_assembly(&outcome.pieces, &outcome.interfaces, &outcome.connectors, &outcome.loose);
    let name_of = |id: usize| outcome.pieces.iter().find(|p| p.id == id).map_or_else(|| id.to_string(), |p| p.name.clone());
    let instructions = if opts.write_instructions {
        ctl.report(0.85, "Writing the assembly instructions");
        let html = instructions_html(&DocInput {
            title: &opts.model_name,
            pieces: &outcome.pieces,
            loose: &outcome.loose,
            connectors: &outcome.connectors,
            interfaces: &outcome.interfaces,
            steps: &steps,
            printer_name: &printer.name,
            notes: &outcome.problems,
        });
        let file = format!("{model}_instructions.html");
        write_atomic(&dir.join(&file), html.as_bytes())?;
        files.push(file.clone());
        Some(file)
    } else {
        None
    };

    let assembly_csv = if opts.write_csv {
        let file = format!("{model}_assembly.csv");
        write_atomic(&dir.join(&file), assembly_csv(outcome, &steps, &parts, &plates).as_bytes())?;
        files.push(file.clone());
        Some(file)
    } else {
        None
    };

    let manifest = Manifest {
        manifest_version: MANIFEST_VERSION,
        generator: format!("OpenSlicer {}", env!("CARGO_PKG_VERSION")),
        model_name: opts.model_name.clone(),
        format: opts.format,
        print_orientation: opts.print_orientation,
        printer: printer_entry(printer),
        parts,
        plates,
        outlines: Vec::new(),
        method: "split".into(),
        interfaces: outcome
            .interfaces
            .iter()
            .map(|i| InterfaceEntry {
                between: [name_of(i.a), name_of(i.b)],
                area_mm2: i.area,
                plane_normal: v3(i.plane.n),
                plane_offset: i.plane.d,
            })
            .collect(),
        connectors: outcome
            .connectors
            .iter()
            .map(|c| ConnectorEntry {
                id: c.id.clone(),
                kind: c.kind.label().into(),
                male_part: c.male_piece.map(name_of),
                female_part: name_of(c.female_piece),
                position: v3(c.position),
                direction: v3(c.normal),
                size_mm: c.size,
                height_mm: c.height,
                angle_deg: c.angle_deg,
            })
            .collect(),
        assembly: steps.iter().map(step_entry).collect(),
        instructions,
        assembly_csv,
        validation: Validation {
            valid: outcome.is_valid(),
            problems: outcome.problems.clone(),
            notes: outcome.notes.clone(),
            source_volume_mm3: outcome.source.signed_volume,
            source_watertight: outcome.source.is_watertight(),
            cut_count: outcome.cut_count,
        },
    };
    finish_export(manifest, files, dir, ctl)
}

fn csv_field(s: &str) -> String {
    if s.contains([',', '"', '\n', '\r']) {
        format!("\"{}\"", s.replace('"', "\"\""))
    } else {
        s.to_owned()
    }
}

/// The assembly sequence as CSV, one row per step (then one per loose part), in assembly order.
/// Lists the joints for each part, its size and volume, the build plate it is on and the label on it.
pub fn assembly_csv(outcome: &SplitOutcome, steps: &[AssemblyStep], parts: &[PartEntry], plates: &[PlateEntry]) -> String {
    let header = [
        "step",
        "part",
        "type",
        "file",
        "label_on_part",
        "label_status",
        "attaches_to",
        "contact_area_mm2",
        "connectors",
        "connector_types",
        "loose_parts",
        "starts_subassembly",
        "size_x_mm",
        "size_y_mm",
        "size_z_mm",
        "volume_cm3",
        "print_plate",
        "fits_printer",
    ];
    let mut out = header.join(",");
    out.push('\n');
    let plate_of = |name: &str| plates.iter().find(|p| p.parts.iter().any(|q| q == name)).map(|p| p.index.to_string()).unwrap_or_default();
    let kind_of = |id: &str| outcome.connectors.iter().find(|c| c.id == id).map_or("", |c| c.kind.label());
    let mut row = |step: String, name: &str, kind: &str, attach: String, area: String, conns: &[String], loose: &[String], sub: String| {
        let part = parts.iter().find(|p| p.name == name);
        let label = outcome.labels.iter().find(|l| outcome.pieces.iter().any(|p| p.name == name && p.id == l.piece_id));
        let mut kinds: Vec<&str> = conns.iter().map(|c| kind_of(c)).collect();
        kinds.sort_unstable();
        kinds.dedup();
        let fields = [
            step,
            name.to_owned(),
            kind.to_owned(),
            part.map(|p| p.file.clone()).unwrap_or_default(),
            label.filter(|l| l.placed).map(|l| l.text.clone()).unwrap_or_default(),
            label.map(|l| if l.placed { format!("placed: {}", l.note) } else { format!("not placed: {}", l.note) }).unwrap_or_default(),
            attach,
            area,
            conns.join(" "),
            kinds.join(" + "),
            loose.join(" "),
            sub,
            part.map(|p| format!("{:.1}", p.size[0])).unwrap_or_default(),
            part.map(|p| format!("{:.1}", p.size[1])).unwrap_or_default(),
            part.map(|p| format!("{:.1}", p.size[2])).unwrap_or_default(),
            part.map(|p| format!("{:.3}", p.volume_mm3 / 1000.0)).unwrap_or_default(),
            plate_of(name),
            part.map(|p| if p.fits_printer { "yes" } else { "no" }.to_owned()).unwrap_or_default(),
        ];
        out.push_str(&fields.iter().map(|f| csv_field(f)).collect::<Vec<_>>().join(","));
        out.push('\n');
    };
    for s in steps {
        let attach = s.attach_to.iter().map(|(_, n, _)| n.as_str()).collect::<Vec<_>>().join(" ");
        let area = s.attach_to.iter().map(|(_, _, a)| format!("{a:.0}")).collect::<Vec<_>>().join(" ");
        row(
            s.number.to_string(),
            &s.part_name,
            "piece",
            attach,
            area,
            &s.connectors,
            &s.loose,
            if s.starts_subassembly { "yes" } else { "no" }.into(),
        );
    }
    for l in &outcome.loose {
        row(String::new(), &l.name, "loose", String::new(), String::new(), &[], &[], String::new());
    }
    out
}

/// SVG of a flat part's outline (millimetres, even-odd fill so holes cut out).
pub fn outline_svg(o: &openslicer_fab::Outline) -> String {
    let (mut lo, mut hi) = (glam::DVec2::splat(f64::MAX), glam::DVec2::splat(f64::MIN));
    for p in o.polygons.iter().flat_map(|p| p.outer.iter()) {
        lo = lo.min(*p);
        hi = hi.max(*p);
    }
    let mut d = String::new();
    for poly in &o.polygons {
        for ring in std::iter::once(&poly.outer).chain(poly.holes.iter()) {
            for (i, p) in ring.iter().enumerate() {
                use std::fmt::Write;
                let _ = write!(d, "{}{:.3},{:.3} ", if i == 0 { "M" } else { "L" }, p.x - lo.x, hi.y - p.y);
            }
            d.push_str("Z ");
        }
    }
    let (w, h) = (hi.x - lo.x, hi.y - lo.y);
    format!(
        "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"{w:.3}mm\" height=\"{h:.3}mm\" viewBox=\"0 0 {w:.3} {h:.3}\">\
         <!-- sheet thickness {t:.2} mm -->\
         <path d=\"{d}\" fill=\"none\" stroke=\"#ff0000\" stroke-width=\"0.1\" fill-rule=\"evenodd\"/></svg>",
        t = o.thickness
    )
}

/// Exports the parts of a fabrication method (stack, interlock, blocks, plates, wireframe, relief, image tools)
/// with the same manifest and verification as a split. Flat parts also get an SVG outline for laser or CNC cutting.
pub fn export_fab(
    result: &openslicer_fab::FabResult,
    printer: &PrinterProfile,
    opts: &ExportOptions,
    dir: &Path,
    ctl: &Control,
) -> Result<ExportReport, ExportError> {
    let existed = dir.exists();
    let r = export_fab_into(result, printer, opts, dir, ctl);
    if let Err(e) = &r {
        cleanup_after_failure(e, existed, dir);
    }
    r
}

fn export_fab_into(
    result: &openslicer_fab::FabResult,
    printer: &PrinterProfile,
    opts: &ExportOptions,
    dir: &Path,
    ctl: &Control,
) -> Result<ExportReport, ExportError> {
    if result.parts.is_empty() {
        return Err(ExportError::Invalid("there are no parts to export".into()));
    }
    prepare_dir(opts, dir)?;
    let usable = printer.usable();
    let all: Vec<(&Piece, &str)> = result.parts.iter().map(|p| (&p.piece, "piece")).collect();
    let fits = |p: &Piece| best_orientation(&p.mesh, usable, &[]).is_some();
    let (mut parts, mut files) = write_parts(&all, &fits, usable, opts, dir, ctl)?;
    let model = sanitize(&opts.model_name);
    let mut outlines = Vec::new();
    for (entry, part) in parts.iter_mut().zip(&result.parts) {
        entry.label = Some(part.label.clone());
        if let Some(o) = &part.outline {
            let file = format!("{}_{}_outline.svg", model, sanitize(&part.piece.name));
            write_atomic(&dir.join(&file), outline_svg(o).as_bytes())?;
            outlines.push(OutlineEntry {
                part: part.piece.name.clone(),
                sha256: hash_file(&dir.join(&file))?,
                file: file.clone(),
                thickness_mm: o.thickness,
            });
            files.push(file);
        }
    }
    let rep = &result.report;
    let mut notes = rep.notes.clone();
    notes.push(format!("parts hold {:.1}% of the source volume", rep.fill_ratio * 100.0));
    if let Some(d) = rep.max_deviation {
        notes.push(format!("surface deviates from the source by at most {d:.2} mm"));
    }
    let manifest = Manifest {
        manifest_version: MANIFEST_VERSION,
        generator: format!("OpenSlicer {}", env!("CARGO_PKG_VERSION")),
        model_name: opts.model_name.clone(),
        format: opts.format,
        print_orientation: opts.print_orientation,
        printer: printer_entry(printer),
        parts,
        plates: Vec::new(),
        outlines,
        method: rep.method.clone(),
        interfaces: Vec::new(),
        connectors: Vec::new(),
        assembly: Vec::new(),
        instructions: None,
        assembly_csv: None,
        validation: Validation {
            valid: rep.is_valid(),
            problems: rep.problems.clone(),
            notes,
            source_volume_mm3: rep.source_volume,
            source_watertight: true,
            cut_count: 0,
        },
    };
    finish_export(manifest, files, dir, ctl)
}

fn printer_entry(printer: &PrinterProfile) -> PrinterEntry {
    PrinterEntry {
        name: printer.name.clone(),
        build_volume: printer.build_volume,
        margin: printer.margin,
        usable: v3(printer.usable()),
        joint_clearance: printer.joint_clearance,
        material: printer.material.clone(),
    }
}

fn cleanup_after_failure(e: &ExportError, existed: bool, dir: &Path) {
    // A failed or cancelled export leaves nothing behind, except a destination that was refused as non-empty.
    let refused = matches!(e, ExportError::Invalid(m) if m.contains("is not empty"));
    if refused {
        return;
    }
    if existed {
        if let Ok(rd) = std::fs::read_dir(dir) {
            for entry in rd.flatten() {
                let _ = std::fs::remove_file(entry.path());
            }
        }
    } else {
        let _ = std::fs::remove_dir_all(dir);
    }
}

/// Writes the manifest, verifies everything that was written, and removes the manifest again if verification fails.
fn finish_export(manifest: Manifest, mut files: Vec<String>, dir: &Path, ctl: &Control) -> Result<ExportReport, ExportError> {
    let json = serde_json::to_vec_pretty(&manifest).map_err(|e| ExportError::Invalid(e.to_string()))?;
    let manifest_path = dir.join(MANIFEST_NAME);
    write_atomic(&manifest_path, &json)?;
    files.push(MANIFEST_NAME.to_string());
    ctl.report(0.9, "Verifying the exported files");
    let verification = verify_export(dir);
    if !verification.is_ok() {
        // An export that fails verification must not look complete.
        let _ = std::fs::remove_file(&manifest_path);
        return Err(ExportError::Verification(verification.problems));
    }
    ctl.report(1.0, "Done");
    Ok(ExportReport { dir: dir.to_path_buf(), manifest, files, verification })
}

fn step_entry(s: &AssemblyStep) -> StepEntry {
    StepEntry {
        step: s.number,
        part: s.part_name.clone(),
        attaches_to: s.attach_to.iter().map(|(_, n, _)| n.clone()).collect(),
        connectors: s.connectors.clone(),
        loose_parts: s.loose.clone(),
        starts_subassembly: s.starts_subassembly,
    }
}

pub fn read_manifest(dir: &Path) -> Result<Manifest, String> {
    let bytes = std::fs::read(dir.join(MANIFEST_NAME)).map_err(|e| format!("cannot read {MANIFEST_NAME}: {e}"))?;
    serde_json::from_slice(&bytes).map_err(|e| format!("{MANIFEST_NAME} is not valid: {e}"))
}

/// Re-imports every file listed in the manifest and compares hash, triangle count, volume and bounds.
pub fn verify_export(dir: &Path) -> VerifyReport {
    let mut rep = VerifyReport::default();
    let manifest = match read_manifest(dir) {
        Ok(m) => m,
        Err(e) => {
            rep.problems.push(e);
            return rep;
        }
    };
    if manifest.manifest_version != MANIFEST_VERSION {
        rep.problems.push(format!("unsupported manifest version {}", manifest.manifest_version));
        return rep;
    }
    let usable = DVec3::from_array(manifest.printer.usable);
    for p in &manifest.parts {
        rep.files_checked += 1;
        let path = dir.join(&p.file);
        match hash_file(&path) {
            Ok(h) if h == p.sha256 => {}
            Ok(_) => {
                rep.problems.push(format!("{}: the file changed after it was exported (hash mismatch)", p.file));
                continue;
            }
            Err(e) => {
                rep.problems.push(e.to_string());
                continue;
            }
        }
        let mesh = match load_mesh(&path) {
            Ok(m) => m,
            Err(e) => {
                rep.problems.push(format!("{}: {e}", p.file));
                continue;
            }
        };
        let s = mesh.stats();
        if s.triangle_count != p.triangles {
            rep.problems.push(format!("{}: {} triangles after re-import, expected {}", p.file, s.triangle_count, p.triangles));
        }
        let tol = 1e-5 * p.volume_mm3.abs().max(1.0) + 1e-3;
        if (s.signed_volume - p.volume_mm3).abs() > tol {
            rep.problems.push(format!("{}: volume {:.3} mm³ after re-import, expected {:.3}", p.file, s.signed_volume, p.volume_mm3));
        }
        if s.is_watertight() != p.watertight {
            rep.problems.push(format!("{}: watertight state changed on re-import", p.file));
        }
        if p.kind == "piece" && !s.is_watertight() {
            rep.problems.push(format!(
                "{}: the part is not a closed solid (open edges {}, non-manifold edges {})",
                p.file, s.boundary_edges, s.non_manifold_edges
            ));
        }
        if s.signed_volume <= 0.0 {
            rep.problems.push(format!("{}: enclosed volume is not positive ({:.3} mm³)", p.file, s.signed_volume));
        }
        if let Some(b) = s.bounds {
            let (lo, hi) = (b.min.as_dvec3(), b.max.as_dvec3());
            let diag = (hi - lo).length().max(1.0);
            if (lo - DVec3::from_array(p.bounds_min)).abs().max_element() > 1e-4 * diag
                || (hi - DVec3::from_array(p.bounds_max)).abs().max_element() > 1e-4 * diag
            {
                rep.problems.push(format!("{}: bounds changed on re-import", p.file));
            }
            if manifest.print_orientation && p.fits_printer {
                let size = hi - lo;
                if size.x > usable.x + 1e-3 || size.y > usable.y + 1e-3 || size.z > usable.z + 1e-3 {
                    rep.problems.push(format!("{}: {:.1} × {:.1} × {:.1} mm exceeds the build volume", p.file, size.x, size.y, size.z));
                }
            }
        }
    }
    for pl in &manifest.plates {
        rep.files_checked += 1;
        let path = dir.join(&pl.file);
        match hash_file(&path) {
            Ok(h) if h == pl.sha256 => {}
            Ok(_) => {
                rep.problems.push(format!("{}: the file changed after it was exported (hash mismatch)", pl.file));
                continue;
            }
            Err(e) => {
                rep.problems.push(e.to_string());
                continue;
            }
        }
        match load_mesh(&path) {
            Ok(m) => {
                if m.triangles.len() != pl.triangles {
                    rep.problems.push(format!("{}: {} triangles after re-import, expected {}", pl.file, m.triangles.len(), pl.triangles));
                }
                if let Some(b) = m.bounds() {
                    let s = b.size().as_dvec3();
                    if s.x > usable.x + 1e-3 || s.y > usable.y + 1e-3 || s.z > usable.z + 1e-3 {
                        rep.problems.push(format!("{}: plate {:.1} × {:.1} × {:.1} mm exceeds the build volume", pl.file, s.x, s.y, s.z));
                    }
                }
            }
            Err(e) => rep.problems.push(format!("{}: {e}", pl.file)),
        }
    }
    for o in &manifest.outlines {
        rep.files_checked += 1;
        match hash_file(&dir.join(&o.file)) {
            Ok(h) if h == o.sha256 => {}
            Ok(_) => rep.problems.push(format!("{}: the file changed after it was exported (hash mismatch)", o.file)),
            Err(e) => rep.problems.push(e.to_string()),
        }
    }
    for f in manifest.instructions.iter().chain(manifest.assembly_csv.iter()) {
        rep.files_checked += 1;
        if !dir.join(f).is_file() {
            rep.problems.push(format!("{f}: missing"));
        }
    }
    rep
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::pipeline::run_split;
    use crate::settings::{Layout, SplitSettings};
    use glam::Vec3;
    use openslicer_joints::settings::{ConnectorKind, ConnectorSettings};

    fn temp_dir(tag: &str) -> PathBuf {
        let d = std::env::temp_dir().join(format!("openslicer-export-{tag}-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&d);
        d
    }

    fn outcome(nest: bool, connector: Option<ConnectorKind>) -> (SplitOutcome, PrinterProfile) {
        let mut m = Mesh::cuboid(Vec3::new(400.0, 120.0, 60.0));
        m.translate(Vec3::new(200.0, 60.0, 30.0));
        let printer = PrinterProfile::default();
        let settings = SplitSettings {
            layout: Layout::Grid { divisions: [2, 1, 1], yaw_deg: 0.0 },
            connectors: connector.map(ConnectorSettings::new),
            nest,
            ..Default::default()
        };
        (run_split(&m, &printer, &settings, &Control::none()).unwrap(), printer)
    }

    #[test]
    fn export_writes_parts_manifest_and_instructions_and_verifies() {
        let (out, printer) = outcome(true, Some(ConnectorKind::Pin));
        let dir = temp_dir("basic");
        let rep = export(&out, &printer, &ExportOptions { model_name: "Test Model".into(), ..Default::default() }, &dir, &Control::none())
            .unwrap();
        assert!(rep.verification.is_ok(), "{:?}", rep.verification);
        assert_eq!(rep.manifest.parts.len(), out.pieces.len() + out.loose.len());
        for p in &rep.manifest.parts {
            assert!(dir.join(&p.file).is_file());
            assert!(p.file.starts_with("Test_Model_"), "{}", p.file);
            assert_eq!(p.sha256.len(), 64);
        }
        assert!(dir.join(MANIFEST_NAME).is_file() && dir.join("Test_Model_instructions.html").is_file());
        assert!(!rep.manifest.plates.is_empty() && dir.join(&rep.manifest.plates[0].file).is_file());
        // The assembly CSV: header, one row per step, one per loose part; quoted where needed.
        let csv = std::fs::read_to_string(dir.join("Test_Model_assembly.csv")).unwrap();
        let lines: Vec<&str> = csv.lines().collect();
        assert!(lines[0].starts_with("step,part,type,file,label_on_part"), "{}", lines[0]);
        assert_eq!(lines.len(), 1 + out.pieces.len() + out.loose.len());
        assert!(lines[1].starts_with("1,"), "{}", lines[1]);
        assert_eq!(rep.manifest.assembly_csv.as_deref(), Some("Test_Model_assembly.csv"));
        assert_eq!(csv_field("a,b"), "\"a,b\"");
        assert_eq!(csv_field("say \"hi\""), "\"say \"\"hi\"\"\"");
        assert_eq!(rep.manifest.connectors.len(), out.connectors.len());
        assert_eq!(rep.manifest.assembly.len(), out.pieces.len());
        assert!(rep.manifest.validation.valid, "{:?}", rep.manifest.validation.problems);
        // The manifest on disk round-trips.
        let back = read_manifest(&dir).unwrap();
        assert_eq!(back.parts.len(), rep.manifest.parts.len());
        assert_eq!(back.printer.usable, [220.0, 220.0, 250.0]);
        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn every_format_round_trips_with_matching_volume() {
        let (out, printer) = outcome(false, None);
        for fmt in [ExportFormat::Stl, ExportFormat::Obj, ExportFormat::ThreeMf] {
            let dir = temp_dir(fmt.extension());
            let rep = export(
                &out,
                &printer,
                &ExportOptions { format: fmt, write_instructions: false, ..Default::default() },
                &dir,
                &Control::none(),
            )
            .unwrap();
            assert!(rep.verification.is_ok(), "{fmt:?}: {:?}", rep.verification);
            let total: f64 = rep.manifest.parts.iter().map(|p| p.volume_mm3).sum();
            assert!((total - 400.0 * 120.0 * 60.0).abs() < 1.0, "{fmt:?}: {total}");
            let _ = std::fs::remove_dir_all(&dir);
        }
    }

    #[test]
    fn tampering_with_a_file_is_detected() {
        let (out, printer) = outcome(false, None);
        let dir = temp_dir("tamper");
        let rep = export(&out, &printer, &ExportOptions::default(), &dir, &Control::none()).unwrap();
        let victim = dir.join(&rep.manifest.parts[0].file);
        let mut bytes = std::fs::read(&victim).unwrap();
        let n = bytes.len();
        bytes[n - 7] ^= 0x55;
        std::fs::write(&victim, bytes).unwrap();
        let v = verify_export(&dir);
        assert!(!v.is_ok() && v.problems.iter().any(|p| p.contains("hash")), "{:?}", v.problems);
        // A deleted file and a missing manifest are reported too.
        std::fs::remove_file(&victim).unwrap();
        assert!(!verify_export(&dir).is_ok());
        std::fs::remove_file(dir.join(MANIFEST_NAME)).unwrap();
        assert!(verify_export(&dir).problems[0].contains("manifest"));
        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn print_orientation_fits_the_build_volume_and_records_the_transform() {
        let (out, printer) = outcome(false, None);
        let dir = temp_dir("orient");
        let rep = export(
            &out,
            &printer,
            &ExportOptions { print_orientation: true, write_instructions: false, ..Default::default() },
            &dir,
            &Control::none(),
        )
        .unwrap();
        for p in &rep.manifest.parts {
            let (rot, off) = (p.print_rotation.unwrap(), DVec3::from_array(p.print_offset.unwrap()));
            assert!(p.bounds_min.iter().all(|v| v.abs() < 1e-3), "min corner sits at the origin: {:?}", p.bounds_min);
            assert!(p.size[0] <= 220.0 + 1e-3 && p.size[1] <= 220.0 + 1e-3 && p.size[2] <= 250.0 + 1e-3, "{:?}", p.size);
            // Applying the recorded transform to the assembly-pose part reproduces the exported bounds.
            let piece = out.pieces.iter().find(|q| q.id == p.id).unwrap();
            let r = glam::DMat3::from_cols(
                DVec3::new(rot[0][0], rot[1][0], rot[2][0]),
                DVec3::new(rot[0][1], rot[1][1], rot[2][1]),
                DVec3::new(rot[0][2], rot[1][2], rot[2][2]),
            );
            let (mut lo, mut hi) = (DVec3::splat(f64::MAX), DVec3::splat(f64::MIN));
            for v in &piece.mesh.vertices {
                let w = r * v.as_dvec3() + off;
                lo = lo.min(w);
                hi = hi.max(w);
            }
            assert!((hi - lo - DVec3::from_array(p.size)).abs().max_element() < 1e-3);
            assert!((lo - DVec3::from_array(p.bounds_min)).abs().max_element() < 1e-3);
        }
        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn refuses_unsafe_destinations_and_templates() {
        let (out, printer) = outcome(false, None);
        let dir = temp_dir("unsafe");
        std::fs::create_dir_all(&dir).unwrap();
        std::fs::write(dir.join("keep.txt"), b"x").unwrap();
        let e = export(&out, &printer, &ExportOptions::default(), &dir, &Control::none()).unwrap_err();
        assert!(e.to_string().contains("not empty"), "{e}");
        assert_eq!(std::fs::read(dir.join("keep.txt")).unwrap(), b"x");
        let bad = ExportOptions { name_template: "{model}".into(), ..Default::default() };
        assert!(export(&out, &printer, &bad, &temp_dir("tmpl"), &Control::none()).unwrap_err().to_string().contains("template"));
        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn file_names_are_sanitised_and_collisions_rejected() {
        assert_eq!(sanitize("a/b:c d"), "a_b_c_d");
        let (out, printer) = outcome(false, None);
        let dir = temp_dir("coll");
        let e = export(
            &out,
            &printer,
            &ExportOptions { name_template: "{model}_{name}".into(), model_name: "x".into(), ..Default::default() },
            &dir,
            &Control::none(),
        );
        assert!(e.is_ok());
        let _ = std::fs::remove_dir_all(&dir);
        // Two parts can never collide with {name}; an explicit constant prefix plus {id} stays unique as well.
        let dir2 = temp_dir("coll2");
        assert!(export(&out, &printer, &ExportOptions { name_template: "part{id}".into(), ..Default::default() }, &dir2, &Control::none())
            .is_ok());
        let _ = std::fs::remove_dir_all(&dir2);
    }

    #[test]
    fn cancellation_stops_the_export() {
        let (out, printer) = outcome(false, None);
        let flag = std::sync::atomic::AtomicBool::new(true);
        let ctl = Control::with_cancel(&flag);
        let r = export(&out, &printer, &ExportOptions::default(), &temp_dir("cancel"), &ctl);
        assert!(matches!(r, Err(ExportError::Cancelled(_))));
        assert!(!temp_dir("cancel").exists(), "a cancelled export leaves nothing behind");
    }

    #[test]
    fn fab_results_export_with_outlines_and_labels() {
        use openslicer_fab::stack::{stack, StackSettings};
        let mut m = Mesh::cuboid(Vec3::new(120.0, 80.0, 30.0));
        m.translate(Vec3::new(60.0, 40.0, 15.0));
        let printer = PrinterProfile::default();
        let res = stack(&m, &StackSettings { layer_thickness: 10.0, ..StackSettings::new(printer.usable()) }, &Control::none()).unwrap();
        let dir = temp_dir("fab");
        let rep =
            export_fab(&res, &printer, &ExportOptions { model_name: "box".into(), ..Default::default() }, &dir, &Control::none()).unwrap();
        assert!(rep.verification.is_ok(), "{:?}", rep.verification);
        assert_eq!(rep.manifest.method, res.report.method);
        assert_eq!(rep.manifest.parts.len(), res.parts.len());
        assert!(rep.manifest.parts.iter().all(|p| p.label.as_deref().is_some_and(|l| l.starts_with("Layer"))));
        assert_eq!(rep.manifest.outlines.len(), res.parts.len(), "every layer has a cutting outline");
        let svg = std::fs::read_to_string(dir.join(&rep.manifest.outlines[0].file)).unwrap();
        let doc = roxmltree::Document::parse(&svg).unwrap();
        let dim = |name: &str| doc.root_element().attribute(name).unwrap().trim_end_matches("mm").parse::<f64>().unwrap();
        let (mut d, e) = ([dim("width"), dim("height")], 1e-3);
        d.sort_by(f64::total_cmp);
        assert!((d[0] - 80.0).abs() < e && (d[1] - 120.0).abs() < e, "{d:?}");
        // Tampering with an outline is caught too.
        std::fs::write(dir.join(&rep.manifest.outlines[0].file), "<svg/>").unwrap();
        assert!(verify_export(&dir).problems.iter().any(|p| p.contains("hash")));
        let _ = std::fs::remove_dir_all(&dir);
    }
}
