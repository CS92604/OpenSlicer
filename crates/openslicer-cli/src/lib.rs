//! Command-line interface. The logic lives here (not in `main.rs`) so it can be tested end to end.
//!
//! Exit codes: `0` success, `1` failure, `2` usage error, `3` finished but the result has problems
//! (parts that do not fit, failed verification, open geometry). A result with problems is still written so it can
//! be inspected, but scripts can tell it apart from a clean one.

use clap::{Args, Parser, Subcommand, ValueEnum};
use glam::{DVec3, Vec3};
use openslicer_core::control::Control;
use openslicer_core::Mesh;
use openslicer_fab::{blocks, image_tools, interlock, plates, relief, stack, wireframe, FabResult};
use openslicer_geom::boolean::{boolean, BoolOp};
use openslicer_geom::hollow::{hollow, HollowOptions};
use openslicer_geom::repair::{repair, RepairOptions};
use openslicer_io::{load_mesh, save_mesh};
use openslicer_joints::settings::{ConnectorKind, ConnectorSettings};
use openslicer_project::export::{export, export_fab, verify_export, ExportFormat, ExportOptions};
use openslicer_project::labels::{LabelMode, LabelOptions};
use openslicer_project::pipeline::{run_split, SplitOutcome};
use openslicer_project::profile::PrinterProfile;
use openslicer_project::settings::{Layout, PlaneSpec, SplitSettings};
use openslicer_split::autotile::OrientationPolicy;
use openslicer_split::fit::best_orientation;
use std::io::Write;
use std::path::{Path, PathBuf};
use std::sync::atomic::AtomicBool;

pub const EXIT_OK: i32 = 0;
pub const EXIT_FAILED: i32 = 1;
pub const EXIT_PROBLEMS: i32 = 3;

#[derive(Parser)]
#[command(
    name = "openslicer-cli",
    version,
    about = "Split large 3D models into printable parts, connect them, and verify the result",
    propagate_version = true
)]
struct Cli {
    /// Print progress to standard error.
    #[arg(long, global = true)]
    progress: bool,
    #[command(subcommand)]
    command: Command,
}

#[derive(Subcommand)]
#[allow(clippy::large_enum_variant)]
enum Command {
    /// Show size, volume and mesh health of one or more models.
    Info(InfoArgs),
    /// Repair a mesh: weld, remove degenerate triangles, orient, fill small holes.
    Repair(RepairArgs),
    /// Split models to fit a printer, add connectors, and export the parts with a manifest.
    Split(SplitArgs),
    /// Build a model another way: stacked layers, interlocking ribs, blocks, plates, wireframe or relief.
    #[command(subcommand)]
    Fab(FabCommand),
    /// Make a lithophane, relief or halftone plate from an image.
    #[command(subcommand)]
    Image(ImageCommand),
    /// Hollow a solid model with a given wall thickness.
    Hollow(HollowArgs),
    /// Union, subtract or intersect two solids.
    Boolean(BooleanArgs),
    /// Re-import an exported folder and check hashes, geometry and printer fit.
    Verify { dir: PathBuf },
    /// List the built-in printer presets.
    Printers,
    /// Write a built-in sample model (dome, wing, panel) as an STL, or list them.
    Stock {
        /// Model name (dome, wing or panel). Omit to list them.
        name: Option<String>,
        /// Output file.
        #[arg(short, long)]
        output: Option<PathBuf>,
    },
}

#[derive(Args, Clone, Default)]
struct PrinterArgs {
    /// Printer preset (part of its name, see `printers`).
    #[arg(long)]
    printer: Option<String>,
    /// Build volume in mm, for example 220,220,250.
    #[arg(long, value_name = "X,Y,Z")]
    volume: Option<String>,
    /// Unusable margin on each side of the plate, mm.
    #[arg(long)]
    margin: Option<f64>,
    /// Lateral joint clearance, mm (0.2 suits most FDM printers).
    #[arg(long)]
    clearance: Option<f64>,
    /// Smallest feature the printer can reproduce, mm.
    #[arg(long)]
    min_feature: Option<f64>,
    /// Printer profile saved as JSON.
    #[arg(long, value_name = "FILE")]
    profile: Option<PathBuf>,
}

impl PrinterArgs {
    fn resolve(&self) -> Result<PrinterProfile, String> {
        let mut p = if let Some(f) = &self.profile {
            serde_json::from_slice(&std::fs::read(f).map_err(|e| format!("cannot read {}: {e}", f.display()))?)
                .map_err(|e| format!("{}: {e}", f.display()))?
        } else if let Some(name) = &self.printer {
            let lower = name.to_lowercase();
            let hits: Vec<PrinterProfile> =
                PrinterProfile::presets().into_iter().filter(|p| p.name.to_lowercase().contains(&lower)).collect();
            match hits.len() {
                1 => hits.into_iter().next().unwrap(),
                0 => return Err(format!("no printer preset matches '{name}'; run `printers` to list them")),
                _ => {
                    return Err(format!(
                        "'{name}' matches several presets: {}",
                        hits.iter().map(|p| p.name.as_str()).collect::<Vec<_>>().join(", ")
                    ))
                }
            }
        } else {
            PrinterProfile::default()
        };
        if let Some(v) = &self.volume {
            p.build_volume = parse_vec3(v)?;
            p.name = format!("Custom {} × {} × {}", p.build_volume[0], p.build_volume[1], p.build_volume[2]);
        }
        if let Some(m) = self.margin {
            p.margin = m;
        }
        if let Some(c) = self.clearance {
            p.joint_clearance = c;
        }
        if let Some(f) = self.min_feature {
            p.min_feature = f;
        }
        p.validate()?;
        Ok(p)
    }
}

fn parse_vec3(s: &str) -> Result<[f64; 3], String> {
    let v: Vec<f64> =
        s.split(',').map(|t| t.trim().parse::<f64>().map_err(|_| format!("'{t}' is not a number in '{s}'"))).collect::<Result<_, _>>()?;
    if v.len() != 3 || v.iter().any(|x| !x.is_finite()) {
        return Err(format!("expected three comma-separated numbers (X,Y,Z), got '{s}'"));
    }
    Ok([v[0], v[1], v[2]])
}

/// "100,100,-" gives `[Some(100), Some(100), None]`.
fn parse_opt_vec3(s: &str) -> Result<[Option<f64>; 3], String> {
    let parts: Vec<&str> = s.split(',').map(str::trim).collect();
    if parts.len() != 3 {
        return Err(format!("expected three comma-separated values, with '-' to skip an axis, got '{s}'"));
    }
    let mut out = [None; 3];
    for (o, p) in out.iter_mut().zip(parts) {
        if p != "-" && !p.eq_ignore_ascii_case("none") {
            *o = Some(p.parse::<f64>().map_err(|_| format!("'{p}' is not a number"))?);
        }
    }
    Ok(out)
}

#[derive(Args)]
struct InfoArgs {
    files: Vec<PathBuf>,
    #[command(flatten)]
    printer: PrinterArgs,
    /// Machine-readable output.
    #[arg(long)]
    json: bool,
}

#[derive(Args)]
struct RepairArgs {
    input: PathBuf,
    output: PathBuf,
    /// Fill holes with at most this many boundary edges (0 disables).
    #[arg(long, default_value_t = 64)]
    fill_holes: usize,
    /// Remove disconnected pieces smaller than this volume, mm³.
    #[arg(long)]
    remove_islands: Option<f64>,
    /// Skip self-intersection detection (faster on huge meshes).
    #[arg(long)]
    no_self_intersection_check: bool,
}

#[derive(Clone, Copy, ValueEnum)]
enum LayoutKind {
    Auto,
    Grid,
    Modular,
    Brick,
    Planes,
}

#[derive(Clone, Copy, ValueEnum)]
enum Format {
    Stl,
    Obj,
    #[value(name = "3mf")]
    ThreeMf,
    Step,
}

impl From<Format> for ExportFormat {
    fn from(f: Format) -> Self {
        match f {
            Format::Stl => ExportFormat::Stl,
            Format::Obj => ExportFormat::Obj,
            Format::ThreeMf => ExportFormat::ThreeMf,
            Format::Step => ExportFormat::Step,
        }
    }
}

#[derive(Args, Clone)]
struct OutputArgs {
    /// Output folder (must be empty or not exist).
    #[arg(short, long)]
    output: PathBuf,
    #[arg(long, value_enum, default_value = "stl")]
    format: Format,
    /// File name template: {model}, {name} and {id} are replaced.
    #[arg(long, default_value = "{model}_{name}")]
    name_template: String,
}

impl OutputArgs {
    fn options(&self, model: &str) -> ExportOptions {
        ExportOptions {
            format: self.format.into(),
            name_template: self.name_template.clone(),
            model_name: model.to_string(),
            ..Default::default()
        }
    }
}

#[derive(Args)]
struct SplitArgs {
    /// One or more models. With several, each gets its own sub-folder of the output.
    #[arg(required = true)]
    inputs: Vec<PathBuf>,
    #[command(flatten)]
    out: OutputArgs,
    #[command(flatten)]
    printer: PrinterArgs,
    #[arg(long, value_enum, default_value = "auto")]
    layout: LayoutKind,
    /// Cells along each axis for the grid layout, for example 3,2,1.
    #[arg(long, value_name = "X,Y,Z")]
    grid: Option<String>,
    /// Cut spacing in mm per axis for the modular layout; '-' skips an axis, for example 100,100,-.
    #[arg(long, value_name = "X,Y,Z")]
    spacing: Option<String>,
    /// Rotate the cut grid about Z by this many degrees (grid and modular layouts).
    #[arg(long, default_value_t = 0.0)]
    yaw: f64,
    /// Brick layout: row_axis,along_axis,row_height,brick_length,stagger (axes are 0=X, 1=Y, 2=Z).
    #[arg(long, value_name = "R,A,H,L,S")]
    brick: Option<String>,
    /// A cut plane as px,py,pz:nx,ny,nz. Repeat for more planes (planes layout).
    #[arg(long = "plane", value_name = "P:N")]
    planes: Vec<String>,
    /// Auto layout: keep the grid aligned to the axes instead of searching rotations.
    #[arg(long)]
    no_yaw_search: bool,
    #[arg(long, default_value_t = 15.0)]
    yaw_step: f64,
    /// Auto layout: allow parts to stand on end if that reduces the piece count.
    #[arg(long)]
    allow_standing: bool,
    /// Connector type: pin, dowel, dovetail, tab, puzzle, lip, lap, terrace, pyramid, claw.
    #[arg(long)]
    connector: Option<String>,
    /// Connector width or diameter, mm.
    #[arg(long)]
    connector_size: Option<f64>,
    /// Connector protrusion or depth, mm.
    #[arg(long)]
    connector_height: Option<f64>,
    /// Merge pieces smaller than this volume (mm³) into neighbours.
    #[arg(long)]
    merge_below: Option<f64>,
    /// Do not repair the model before cutting.
    #[arg(long)]
    no_repair: bool,
    /// Arrange the parts on build plates and write a file per plate.
    #[arg(long)]
    nest: bool,
    /// Run the (slow) interpenetration check on the finished assembly.
    #[arg(long)]
    verify_assembly: bool,
    /// Write each part in its print orientation.
    #[arg(long)]
    print_orientation: bool,
    /// Skip the HTML assembly instructions.
    #[arg(long)]
    no_instructions: bool,
    /// Skip the assembly CSV table.
    #[arg(long)]
    no_csv: bool,
    /// Do not move automatic cuts away from tiny slivers (by default a cut may move a few millimetres).
    #[arg(long)]
    keep_cuts: bool,
    /// Put the part number and assembly step on each part: `inset` (engraved) or `outset` (raised).
    #[arg(long, value_name = "MODE")]
    labels: Option<String>,
    /// Label character height in mm.
    #[arg(long, default_value_t = 8.0)]
    label_height: f64,
    /// Label relief depth in mm.
    #[arg(long, default_value_t = 0.6)]
    label_depth: f64,
    /// Split settings saved as JSON; replaces the layout and connector options above.
    #[arg(long, value_name = "FILE")]
    settings: Option<PathBuf>,
}

impl SplitArgs {
    fn settings(&self, printer: &PrinterProfile) -> Result<SplitSettings, String> {
        if let Some(f) = &self.settings {
            return serde_json::from_slice(&std::fs::read(f).map_err(|e| format!("cannot read {}: {e}", f.display()))?)
                .map_err(|e| format!("{}: {e}", f.display()));
        }
        let layout = match self.layout {
            LayoutKind::Auto => Layout::Auto { search_yaw: !self.no_yaw_search, yaw_step_deg: self.yaw_step },
            LayoutKind::Grid => {
                let g = self.grid.as_deref().ok_or("--grid X,Y,Z is required for the grid layout")?;
                let v = parse_vec3(g)?;
                if v.iter().any(|x| x.fract() != 0.0 || *x < 1.0) {
                    return Err("grid cell counts must be whole numbers of at least 1".into());
                }
                Layout::Grid { divisions: [v[0] as usize, v[1] as usize, v[2] as usize], yaw_deg: self.yaw }
            }
            LayoutKind::Modular => {
                let s = self.spacing.as_deref().ok_or("--spacing X,Y,Z is required for the modular layout")?;
                Layout::Modular { spacing: parse_opt_vec3(s)?, offset: [0.0; 3], yaw_deg: self.yaw }
            }
            LayoutKind::Brick => {
                let b = self.brick.as_deref().ok_or("--brick R,A,H,L,S is required for the brick layout")?;
                let v: Vec<f64> = b
                    .split(',')
                    .map(|t| t.trim().parse::<f64>().map_err(|_| format!("'{t}' is not a number")))
                    .collect::<Result<_, _>>()?;
                if v.len() != 5 {
                    return Err("--brick needs five values: row_axis,along_axis,row_height,brick_length,stagger".into());
                }
                Layout::Brick { row_axis: v[0] as usize, along_axis: v[1] as usize, row_height: v[2], brick_length: v[3], stagger: v[4] }
            }
            LayoutKind::Planes => {
                if self.planes.is_empty() {
                    return Err("give at least one --plane px,py,pz:nx,ny,nz for the planes layout".into());
                }
                let mut specs = Vec::new();
                for p in &self.planes {
                    let (pt, n) = p.split_once(':').ok_or_else(|| format!("plane '{p}' must look like px,py,pz:nx,ny,nz"))?;
                    specs.push(PlaneSpec { point: parse_vec3(pt)?, normal: parse_vec3(n)? });
                }
                Layout::Planes(specs)
            }
        };
        let connectors = match self.connector.as_deref() {
            None | Some("none") => None,
            Some(k) => {
                let kind = ConnectorKind::from_slug(k).ok_or_else(|| {
                    format!(
                        "unknown connector '{k}'; choose from {}",
                        ConnectorKind::ALL.iter().map(|k| k.slug()).collect::<Vec<_>>().join(", ")
                    )
                })?;
                let mut c = ConnectorSettings::new(kind);
                c.clearance = printer.joint_clearance;
                if let Some(s) = self.connector_size {
                    c.size = s;
                }
                if let Some(h) = self.connector_height {
                    c.height = h;
                }
                c.validate().map_err(|e| format!("connector settings: {e}"))?;
                Some(c)
            }
        };
        Ok(SplitSettings {
            layout,
            orientation: if self.allow_standing { OrientationPolicy::Any } else { OrientationPolicy::KeepUpright },
            connectors,
            merge_pieces_below: self.merge_below,
            repair_first: !self.no_repair,
            nest: self.nest,
            verify_assembly: self.verify_assembly,
            avoid_slivers: !self.keep_cuts,
            labels: LabelOptions {
                mode: match self.labels.as_deref().map(str::to_ascii_lowercase).as_deref() {
                    None | Some("off") | Some("none") => LabelMode::Off,
                    Some("inset") | Some("engrave") | Some("engraved") => LabelMode::Inset,
                    Some("outset") | Some("raised") | Some("emboss") => LabelMode::Outset,
                    Some(other) => return Err(format!("unknown label mode '{other}'; use inset, outset or off")),
                },
                height: self.label_height,
                depth: self.label_depth,
            },
        })
    }
}

#[derive(Subcommand)]
enum FabCommand {
    /// Stack flat layers (cross-sections) cut from the model.
    Stack {
        input: PathBuf,
        #[command(flatten)]
        out: OutputArgs,
        #[command(flatten)]
        printer: PrinterArgs,
        /// Axis the layers stack along: x, y or z.
        #[arg(long, default_value = "z")]
        axis: String,
        /// Sheet thickness, mm.
        #[arg(long, default_value_t = 3.0)]
        layer: f64,
        /// Registration holes through every layer (0 for none).
        #[arg(long, default_value_t = 0)]
        holes: usize,
        #[arg(long, default_value_t = 4.0)]
        hole_diameter: f64,
    },
    /// Slotted ribs in two directions that slot together like an egg-crate.
    Interlock {
        input: PathBuf,
        #[command(flatten)]
        out: OutputArgs,
        #[command(flatten)]
        printer: PrinterArgs,
        /// Rib spacing in the two horizontal directions, mm.
        #[arg(long, default_value_t = 40.0)]
        spacing: f64,
        #[arg(long, default_value_t = 3.0)]
        thickness: f64,
        /// Vertical axis: 0 = X, 1 = Y, 2 = Z.
        #[arg(long, default_value_t = 2)]
        up_axis: usize,
    },
    /// Build the model from rectangular blocks on a grid.
    Blocks {
        input: PathBuf,
        #[command(flatten)]
        out: OutputArgs,
        #[command(flatten)]
        printer: PrinterArgs,
        /// Block size, mm.
        #[arg(long, default_value_t = 20.0)]
        cell: f64,
        #[arg(long, value_enum, default_value = "nearest")]
        fill: FillKind,
    },
    /// Flat plates fitted to the model's surface regions.
    Plates {
        input: PathBuf,
        #[command(flatten)]
        out: OutputArgs,
        #[command(flatten)]
        printer: PrinterArgs,
        #[arg(long, default_value_t = 3.0)]
        thickness: f64,
        /// Largest angle between a plate and the surface it follows, degrees.
        #[arg(long, default_value_t = 12.0)]
        angle: f64,
        #[arg(long, default_value_t = 100.0)]
        min_area: f64,
    },
    /// Struts following the model's cross-section contours.
    Wireframe {
        input: PathBuf,
        #[command(flatten)]
        out: OutputArgs,
        #[command(flatten)]
        printer: PrinterArgs,
        #[arg(long, default_value_t = 40.0)]
        spacing: f64,
        #[arg(long, default_value_t = 1.5)]
        radius: f64,
        /// Axes to take sections along, for example xz.
        #[arg(long, default_value = "z")]
        axes: String,
        /// Union crossing hoops into single solids.
        #[arg(long)]
        merge: bool,
    },
    /// Depth relief of the model seen from one side.
    Relief {
        input: PathBuf,
        #[command(flatten)]
        out: OutputArgs,
        #[command(flatten)]
        printer: PrinterArgs,
        /// Direction the viewer looks along: x, y, z, -x, -y or -z.
        #[arg(long, default_value = "-z")]
        view: String,
        /// Sample spacing, mm.
        #[arg(long, default_value_t = 0.5)]
        pixel: f64,
        #[arg(long, default_value_t = 2.0)]
        base: f64,
        #[arg(long, default_value_t = 6.0)]
        height: f64,
        /// Bas-relief compression strength (0 keeps heights linear).
        #[arg(long, default_value_t = 0.0)]
        compress: f64,
    },
}

#[derive(Clone, Copy, ValueEnum)]
enum FillKind {
    Nearest,
    Cover,
    Shell,
}

#[derive(Subcommand)]
enum ImageCommand {
    /// Thickness varies with brightness so light shines through the image.
    Lithophane {
        image: PathBuf,
        #[command(flatten)]
        out: OutputArgs,
        #[command(flatten)]
        printer: PrinterArgs,
        /// Width of the finished piece, mm.
        #[arg(long, default_value_t = 100.0)]
        width: f64,
        #[arg(long, default_value_t = 0.8)]
        min_thickness: f64,
        #[arg(long, default_value_t = 3.2)]
        max_thickness: f64,
        #[arg(long, default_value_t = 0.4)]
        pixel: f64,
        /// Dark areas thin instead of bright ones.
        #[arg(long)]
        invert: bool,
    },
    /// Heightfield relief: bright areas rise.
    Relief {
        image: PathBuf,
        #[command(flatten)]
        out: OutputArgs,
        #[command(flatten)]
        printer: PrinterArgs,
        #[arg(long, default_value_t = 100.0)]
        width: f64,
        #[arg(long, default_value_t = 2.0)]
        base: f64,
        #[arg(long, default_value_t = 6.0)]
        height: f64,
        #[arg(long, default_value_t = 0.5)]
        pixel: f64,
        #[arg(long)]
        invert: bool,
    },
    /// A plate of raised dots whose size follows the image's darkness.
    Halftone {
        image: PathBuf,
        #[command(flatten)]
        out: OutputArgs,
        #[command(flatten)]
        printer: PrinterArgs,
        #[arg(long, default_value_t = 100.0)]
        width: f64,
        /// Distance between dot centres, mm.
        #[arg(long, default_value_t = 2.0)]
        pitch: f64,
        #[arg(long, default_value_t = 1.5)]
        base: f64,
        #[arg(long, default_value_t = 1.0)]
        dot_height: f64,
        #[arg(long)]
        invert: bool,
    },
}

#[derive(Args)]
struct HollowArgs {
    input: PathBuf,
    output: PathBuf,
    /// Wall thickness, mm.
    #[arg(long, default_value_t = 3.0)]
    wall: f64,
    /// Number of drain holes (0 leaves a sealed cavity).
    #[arg(long, default_value_t = 0)]
    drain_holes: usize,
    #[arg(long, default_value_t = 4.0)]
    drain_diameter: f64,
    /// Voxel size, mm (default: a third of the wall).
    #[arg(long)]
    resolution: Option<f64>,
}

#[derive(Clone, Copy, ValueEnum)]
enum BoolKind {
    Union,
    Subtract,
    Intersect,
}

#[derive(Args)]
struct BooleanArgs {
    #[arg(value_enum)]
    op: BoolKind,
    a: PathBuf,
    b: PathBuf,
    output: PathBuf,
}

type Res = Result<i32, String>;

struct Io<'a> {
    out: &'a mut dyn Write,
    progress: bool,
}

impl Io<'_> {
    fn say(&mut self, s: impl AsRef<str>) {
        let _ = writeln!(self.out, "{}", s.as_ref());
    }

    fn with_control<T>(&self, f: impl FnOnce(&Control) -> T) -> T {
        if self.progress {
            let flag = AtomicBool::new(false);
            let last = std::sync::Mutex::new((String::new(), -1i32));
            let report = move |frac: f32, what: &str| {
                let pct = (frac * 100.0) as i32;
                let mut l = last.lock().unwrap();
                if l.0 != what || pct / 10 != l.1 / 10 {
                    eprintln!("[{pct:3}%] {what}");
                    *l = (what.to_string(), pct);
                }
            };
            f(&Control::new(&flag, &report))
        } else {
            f(&Control::none())
        }
    }
}

/// Runs the CLI. Returns the process exit code.
pub fn run<I, S>(args: I, out: &mut dyn Write, err: &mut dyn Write) -> i32
where
    I: IntoIterator<Item = S>,
    S: Into<String>,
{
    let argv: Vec<String> = args.into_iter().map(Into::into).collect();
    let cli = match Cli::try_parse_from(&argv) {
        Ok(c) => c,
        Err(e) => {
            let text = e.render().to_string();
            let _ = if e.use_stderr() { write!(err, "{text}") } else { write!(out, "{text}") };
            return e.exit_code();
        }
    };
    let mut io = Io { out, progress: cli.progress };
    match dispatch(cli.command, &mut io) {
        Ok(code) => code,
        Err(msg) => {
            let _ = writeln!(err, "error: {msg}");
            EXIT_FAILED
        }
    }
}

fn dispatch(cmd: Command, io: &mut Io) -> Res {
    match cmd {
        Command::Info(a) => info(a, io),
        Command::Repair(a) => repair_cmd(a, io),
        Command::Split(a) => split(a, io),
        Command::Fab(c) => fab(c, io),
        Command::Image(c) => image(c, io),
        Command::Hollow(a) => hollow_cmd(a, io),
        Command::Boolean(a) => boolean_cmd(a, io),
        Command::Verify { dir } => verify(&dir, io),
        Command::Stock { name, output } => {
            use openslicer_project::stock;
            let (Some(name), Some(output)) = (name, output) else {
                for m in &stock::STOCK {
                    io.say(format!("{:<6} {} - {}", m.id, m.name, m.description));
                }
                return Ok(EXIT_OK);
            };
            let mesh = stock::build(&name).ok_or_else(|| format!("no stock model '{name}'; run `stock` to list them"))?;
            save_mesh(&output, &mesh).map_err(|e| format!("{}: {e}", output.display()))?;
            let size = mesh.stats().bounds.map(|b| b.size()).unwrap_or_default();
            io.say(format!(
                "wrote {} ({} triangles, {:.0} × {:.0} × {:.0} mm)",
                output.display(),
                mesh.triangles.len(),
                size.x,
                size.y,
                size.z
            ));
            Ok(EXIT_OK)
        }
        Command::Printers => {
            for p in PrinterProfile::presets() {
                io.say(format!("{:<34} {} × {} × {} mm", p.name, p.build_volume[0], p.build_volume[1], p.build_volume[2]));
            }
            Ok(EXIT_OK)
        }
    }
}

fn load(path: &Path) -> Result<Mesh, String> {
    load_mesh(path).map_err(|e| format!("{}: {e}", path.display()))
}

fn stem(path: &Path) -> String {
    path.file_stem().map_or_else(|| "model".to_string(), |s| s.to_string_lossy().into_owned())
}

fn size_text(s: Vec3) -> String {
    format!("{:.1} × {:.1} × {:.1} mm", s.x, s.y, s.z)
}

fn info(a: InfoArgs, io: &mut Io) -> Res {
    if a.files.is_empty() {
        return Err("give at least one model file".into());
    }
    let printer = a.printer.resolve()?;
    let usable = printer.usable();
    let mut json = Vec::new();
    let mut code = EXIT_OK;
    for f in &a.files {
        let mesh = load(f)?;
        let s = mesh.stats();
        let size = s.bounds.map_or(Vec3::ZERO, |b| b.size());
        let fits = best_orientation(&mesh, usable, &[]).is_some();
        let clean = s.is_watertight() && !s.is_inverted();
        if !clean {
            code = EXIT_PROBLEMS;
        }
        if a.json {
            json.push(serde_json::json!({
                "file": f.display().to_string(),
                "triangles": s.triangle_count, "vertices": s.vertex_count,
                "size_mm": [size.x, size.y, size.z],
                "volume_mm3": s.signed_volume, "surface_area_mm2": s.surface_area,
                "boundary_edges": s.boundary_edges, "non_manifold_edges": s.non_manifold_edges,
                "degenerate_triangles": s.degenerate_triangles,
                "watertight": s.is_watertight(), "inverted": s.is_inverted(),
                "fits_printer": fits, "printer": printer.name,
            }));
        } else {
            io.say(format!("{}", f.display()));
            io.say(format!("  {} triangles, {} vertices, {}", s.triangle_count, s.vertex_count, size_text(size)));
            io.say(format!("  volume {:.1} cm³, surface {:.1} cm²", s.signed_volume / 1000.0, s.surface_area / 100.0));
            io.say(format!(
                "  mesh: {} (open edges {}, non-manifold edges {}, degenerate triangles {}{})",
                if clean { "closed solid" } else { "NOT a clean solid" },
                s.boundary_edges,
                s.non_manifold_edges,
                s.degenerate_triangles,
                if s.is_inverted() { ", inside-out" } else { "" }
            ));
            io.say(format!(
                "  {}: {}",
                printer.name,
                if fits { "fits in one piece".to_string() } else { "does not fit; needs splitting".to_string() }
            ));
        }
    }
    if a.json {
        io.say(serde_json::to_string_pretty(&json).map_err(|e| e.to_string())?);
    }
    Ok(code)
}

fn repair_cmd(a: RepairArgs, io: &mut Io) -> Res {
    let mesh = load(&a.input)?;
    let opts = RepairOptions {
        fill_holes_up_to: a.fill_holes,
        remove_islands_below: a.remove_islands,
        detect_self_intersections: !a.no_self_intersection_check,
        ..Default::default()
    };
    let (fixed, r) = repair(&mesh, &opts);
    save_mesh(&a.output, &fixed).map_err(|e| format!("{}: {e}", a.output.display()))?;
    io.say(format!(
        "welded {} vertices, removed {} degenerate and {} duplicate triangles, flipped {} triangles, filled {} holes ({} left open), removed {} islands",
        r.vertices_welded, r.degenerate_removed, r.duplicates_removed, r.triangles_flipped, r.holes_filled, r.holes_unfilled, r.islands_removed
    ));
    if let Some(n) = r.self_intersections {
        io.say(format!("self-intersecting triangle pairs: {n}"));
    }
    io.say(format!("wrote {}", a.output.display()));
    if r.is_clean() {
        io.say("result is a clean closed solid");
        Ok(EXIT_OK)
    } else {
        for p in &r.remaining {
            io.say(format!("remaining problem: {p}"));
        }
        Ok(EXIT_PROBLEMS)
    }
}

fn summarize_split(name: &str, o: &SplitOutcome, io: &mut Io) {
    io.say(format!("{name}: {} parts from {} cuts", o.pieces.len(), o.cut_count));
    if let Some(t) = &o.tiling {
        io.say(format!("  grid {}×{}×{} rotated {:.0}°", t.divisions[0], t.divisions[1], t.divisions[2], t.yaw_deg));
    }
    if !o.connectors.is_empty() {
        io.say(format!("  {} connectors, {} loose parts", o.connectors.len(), o.loose.len()));
    }
    let unfit = o.fits.iter().filter(|f| !f.fits).count();
    io.say(if unfit == 0 {
        format!("  all {} parts fit the printer", o.fits.len())
    } else {
        format!("  {unfit} part(s) DO NOT fit the printer")
    });
    if let Some(n) = &o.nest {
        io.say(format!("  {} build plate(s), {:.0}% of the plate area used", n.plates.len(), n.utilisation * 100.0));
    }
    for n in &o.notes {
        io.say(format!("  note: {n}"));
    }
    for p in &o.problems {
        io.say(format!("  PROBLEM: {p}"));
    }
}

fn split(a: SplitArgs, io: &mut Io) -> Res {
    let printer = a.printer.resolve()?;
    let settings = a.settings(&printer)?;
    let batch = a.inputs.len() > 1;
    let mut code = EXIT_OK;
    for input in &a.inputs {
        let name = stem(input);
        let dir = if batch { a.out.output.join(&name) } else { a.out.output.clone() };
        let result = (|| -> Result<i32, String> {
            let mesh = load(input)?;
            let outcome = io.with_control(|c| run_split(&mesh, &printer, &settings, c)).map_err(|e| format!("{name}: {e}"))?;
            summarize_split(&name, &outcome, io);
            let opts = ExportOptions {
                print_orientation: a.print_orientation,
                write_instructions: !a.no_instructions,
                write_csv: !a.no_csv,
                ..a.out.options(&name)
            };
            let report = io.with_control(|c| export(&outcome, &printer, &opts, &dir, c)).map_err(|e| format!("{name}: {e}"))?;
            io.say(format!("  wrote {} files to {} (verified by re-import)", report.files.len(), dir.display()));
            Ok(if outcome.is_valid() { EXIT_OK } else { EXIT_PROBLEMS })
        })();
        match result {
            Ok(c) => code = code.max(c),
            Err(e) if batch => {
                io.say(format!("{name}: FAILED: {e}"));
                code = EXIT_FAILED;
            }
            Err(e) => return Err(e),
        }
    }
    Ok(code)
}

fn axis_vec(s: &str) -> Result<DVec3, String> {
    Ok(match s.to_lowercase().as_str() {
        "x" | "+x" => DVec3::X,
        "y" | "+y" => DVec3::Y,
        "z" | "+z" => DVec3::Z,
        "-x" => -DVec3::X,
        "-y" => -DVec3::Y,
        "-z" => -DVec3::Z,
        other => return Err(format!("'{other}' is not an axis; use x, y, z, -x, -y or -z")),
    })
}

fn finish_fab(
    res: Result<FabResult, openslicer_fab::FabError>,
    input: &Path,
    out: &OutputArgs,
    printer: &PrinterProfile,
    io: &mut Io,
) -> Res {
    let res = res.map_err(|e| e.to_string())?;
    let r = &res.report;
    io.say(format!("{}: {} parts, {:.1}% of the source volume", r.method, res.parts.len(), r.fill_ratio * 100.0));
    if let Some(d) = r.max_deviation {
        io.say(format!("  surface within {d:.2} mm of the source"));
    }
    for n in &r.notes {
        io.say(format!("  note: {n}"));
    }
    for p in &r.problems {
        io.say(format!("  PROBLEM: {p}"));
    }
    let report = io.with_control(|c| export_fab(&res, printer, &out.options(&stem(input)), &out.output, c)).map_err(|e| e.to_string())?;
    io.say(format!("  wrote {} files to {} (verified by re-import)", report.files.len(), out.output.display()));
    Ok(if r.is_valid() { EXIT_OK } else { EXIT_PROBLEMS })
}

fn fab(c: FabCommand, io: &mut Io) -> Res {
    match c {
        FabCommand::Stack { input, out, printer, axis, layer, holes, hole_diameter } => {
            let p = printer.resolve()?;
            let mesh = load(&input)?;
            let s = stack::StackSettings {
                axis: axis_vec(&axis)?,
                layer_thickness: layer,
                registration_holes: holes,
                hole_diameter,
                ..stack::StackSettings::new(p.usable())
            };
            let r = io.with_control(|c| stack::stack(&mesh, &s, c));
            finish_fab(r, &input, &out, &p, io)
        }
        FabCommand::Interlock { input, out, printer, spacing, thickness, up_axis } => {
            let p = printer.resolve()?;
            let mesh = load(&input)?;
            let s = interlock::InterlockSettings {
                up_axis,
                spacing_u: spacing,
                spacing_v: spacing,
                thickness,
                slot_clearance: p.joint_clearance,
                ..interlock::InterlockSettings::new(p.usable())
            };
            let r = io.with_control(|c| interlock::interlock(&mesh, &s, c));
            finish_fab(r, &input, &out, &p, io)
        }
        FabCommand::Blocks { input, out, printer, cell, fill } => {
            let p = printer.resolve()?;
            let mesh = load(&input)?;
            let s = blocks::BlockSettings {
                fill: match fill {
                    FillKind::Nearest => blocks::BlockFill::Nearest,
                    FillKind::Cover => blocks::BlockFill::Cover,
                    FillKind::Shell => blocks::BlockFill::Shell,
                },
                ..blocks::BlockSettings::new(cell, p.usable())
            };
            let r = io.with_control(|c| blocks::blocks(&mesh, &s, c));
            finish_fab(r, &input, &out, &p, io)
        }
        FabCommand::Plates { input, out, printer, thickness, angle, min_area } => {
            let p = printer.resolve()?;
            let mesh = load(&input)?;
            let s = plates::PlateSettings { thickness, max_normal_angle_deg: angle, min_area, ..plates::PlateSettings::new(p.usable()) };
            let r = io.with_control(|c| plates::plates(&mesh, &s, c));
            finish_fab(r, &input, &out, &p, io)
        }
        FabCommand::Wireframe { input, out, printer, spacing, radius, axes, merge } => {
            let p = printer.resolve()?;
            let mesh = load(&input)?;
            let lower = axes.to_lowercase();
            if lower.is_empty() || lower.chars().any(|c| !"xyz".contains(c)) {
                return Err(format!("--axes takes letters from x, y and z, got '{axes}'"));
            }
            let s = wireframe::WireframeSettings {
                spacing,
                strut_radius: radius,
                axes: [lower.contains('x'), lower.contains('y'), lower.contains('z')],
                merge,
                ..wireframe::WireframeSettings::new(p.usable())
            };
            let r = io.with_control(|c| wireframe::wireframe(&mesh, &s, c));
            finish_fab(r, &input, &out, &p, io)
        }
        FabCommand::Relief { input, out, printer, view, pixel, base, height, compress } => {
            let p = printer.resolve()?;
            let mesh = load(&input)?;
            let s = relief::ReliefSettings {
                view_direction: axis_vec(&view)?,
                pixel,
                base_thickness: base,
                relief_height: height,
                compression: if compress > 0.0 { relief::Compression::Log { strength: compress } } else { relief::Compression::Linear },
                ..relief::ReliefSettings::new(p.usable())
            };
            let r = io.with_control(|c| relief::relief_from_mesh(&mesh, &s, c));
            finish_fab(r, &input, &out, &p, io)
        }
    }
}

fn image(c: ImageCommand, io: &mut Io) -> Res {
    let read = |p: &Path| std::fs::read(p).map_err(|e| format!("{}: {e}", p.display()));
    match c {
        ImageCommand::Lithophane { image, out, printer, width, min_thickness, max_thickness, pixel, invert } => {
            let p = printer.resolve()?;
            let s = image_tools::LithophaneSettings {
                width_mm: width,
                min_thickness,
                max_thickness,
                pixel_mm: pixel,
                invert,
                ..image_tools::LithophaneSettings::new(p.usable())
            };
            let bytes = read(&image)?;
            let r = io.with_control(|c| image_tools::lithophane(&bytes, &s, c));
            finish_fab(r, &image, &out, &p, io)
        }
        ImageCommand::Relief { image, out, printer, width, base, height, pixel, invert } => {
            let p = printer.resolve()?;
            let s = image_tools::ImageReliefSettings {
                width_mm: width,
                base_thickness: base,
                relief_height: height,
                pixel_mm: pixel,
                invert,
                ..image_tools::ImageReliefSettings::new(p.usable())
            };
            let bytes = read(&image)?;
            let r = io.with_control(|c| image_tools::relief_from_image(&bytes, &s, c));
            finish_fab(r, &image, &out, &p, io)
        }
        ImageCommand::Halftone { image, out, printer, width, pitch, base, dot_height, invert } => {
            let p = printer.resolve()?;
            let s = image_tools::HalftoneSettings {
                width_mm: width,
                pitch_mm: pitch,
                base_thickness: base,
                dot_height,
                invert,
                ..image_tools::HalftoneSettings::new(p.usable())
            };
            let bytes = read(&image)?;
            let r = io.with_control(|c| image_tools::halftone(&bytes, &s, c));
            finish_fab(r, &image, &out, &p, io)
        }
    }
}

fn hollow_cmd(a: HollowArgs, io: &mut Io) -> Res {
    let mesh = load(&a.input)?;
    let opts = HollowOptions {
        wall: a.wall,
        resolution: a.resolution,
        drain_holes: a.drain_holes,
        drain_diameter: a.drain_diameter,
        ..Default::default()
    };
    let res = io.with_control(|c| hollow(&mesh, &opts, c)).map_err(|e| e.to_string())?;
    let stats = res.mesh.stats();
    let r = &res.report;
    io.say(format!(
        "material {:.1} cm³ (was {:.1}), cavity {:.1} cm³, wall {:.2}–{:.2} mm, {} drain holes",
        r.material_volume / 1000.0,
        mesh.stats().signed_volume / 1000.0,
        r.cavity_volume / 1000.0,
        r.wall_min,
        r.wall_mean,
        r.drain_holes
    ));
    for n in &r.notes {
        io.say(format!("  note: {n}"));
    }
    let closed = stats.boundary_edges == 0 && stats.non_manifold_edges == 0;
    save_mesh(&a.output, &res.mesh).map_err(|e| format!("{}: {e}", a.output.display()))?;
    io.say(format!("wrote {}", a.output.display()));
    if closed {
        Ok(EXIT_OK)
    } else {
        io.say(format!("PROBLEM: the result has {} open and {} non-manifold edges", stats.boundary_edges, stats.non_manifold_edges));
        Ok(EXIT_PROBLEMS)
    }
}

fn boolean_cmd(a: BooleanArgs, io: &mut Io) -> Res {
    let (ma, mb) = (load(&a.a)?, load(&a.b)?);
    let op = match a.op {
        BoolKind::Union => BoolOp::Union,
        BoolKind::Subtract => BoolOp::Difference,
        BoolKind::Intersect => BoolOp::Intersection,
    };
    let (mesh, r) = boolean(&ma, &mb, op).map_err(|e| e.to_string())?;
    save_mesh(&a.output, &mesh).map_err(|e| format!("{}: {e}", a.output.display()))?;
    io.say(format!("volumes: {:.1} and {:.1} mm³ → result {:.1} mm³", r.volume_a, r.volume_b, r.volume_result));
    io.say(format!("wrote {}", a.output.display()));
    Ok(EXIT_OK)
}

fn verify(dir: &Path, io: &mut Io) -> Res {
    let r = verify_export(dir);
    io.say(format!("checked {} files in {}", r.files_checked, dir.display()));
    if r.is_ok() {
        io.say("all files match the manifest and are valid");
        Ok(EXIT_OK)
    } else {
        for p in &r.problems {
            io.say(format!("PROBLEM: {p}"));
        }
        Ok(EXIT_FAILED)
    }
}
