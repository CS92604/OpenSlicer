use crate::camera::{ray_plane_z, Camera, ViewPreset};
use crate::cutplanes::{plane_quad, section_segments, CutPlane};
use crate::gl_view::{self, DrawModel, FlatBatch, Renderer};
use crate::icons::Icon;
use crate::jobs::{self, Job, JobEvent};
use crate::model::{Model, Transform};
use crate::session::{self, Autosaver, Snapshot};
use crate::settings::{Fit, Preferences, PrinterProfile};
use crate::split_ui::{self, Action, SplitPanel};
use crate::theme;
use crate::widgets::{self, caption, chip, model_row, ribbon_button, section_header, vec3_row};
use eframe::egui::{
    self, vec2, Align2, Color32, CornerRadius, FontId, Key, KeyboardShortcut, Modifiers, PointerButton, Rect, RichText, Sense, Stroke, Ui,
};
use glam::{Vec2, Vec3};
use openslicer_core::{Mesh, MeshStats};
use openslicer_project::stock;
use std::collections::{HashMap, HashSet, VecDeque};
use std::hash::{Hash, Hasher};
use std::path::PathBuf;
use std::sync::Arc;

const PREFS_KEY: &str = "openslicer.preferences.v1";
const SPLIT_KEY: &str = "openslicer.split.v1";

#[derive(Clone, Copy, PartialEq, Eq)]
enum Tool {
    Select,
    Move,
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum Level {
    Info,
    Ok,
    Warn,
    Error,
}

struct ErrorInfo {
    title: String,
    detail: String,
}

struct MoveDrag {
    id: u64,
    start_hit: Vec3,
    start_pos: Vec3,
    plane_z: f32,
}

/// A model as the undo history remembers it.
#[derive(Clone)]
struct SavedModel {
    id: u64,
    name: String,
    mesh: Arc<Mesh>,
    stats: Arc<MeshStats>,
    source: Option<PathBuf>,
    transform: Transform,
}

/// The editable scene (the models the user added; split parts are regenerated, not remembered).
#[derive(Clone, Default)]
struct SceneState {
    models: Vec<SavedModel>,
    selected: Option<u64>,
}

fn scene_signature(s: &SceneState) -> u64 {
    let mut h = std::collections::hash_map::DefaultHasher::new();
    for m in &s.models {
        m.id.hash(&mut h);
        m.name.hash(&mut h);
        (Arc::as_ptr(&m.mesh) as usize).hash(&mut h);
        for v in [m.transform.position, m.transform.rotation_deg, m.transform.scale] {
            [v.x.to_bits(), v.y.to_bits(), v.z.to_bits()].hash(&mut h);
        }
    }
    h.finish()
}

/// A manual plane, the mesh it was cut against (by address), and the outline segments found.
type PlaneOutline = (CutPlane, usize, Vec<(Vec3, Vec3)>);

const UNDO_LIMIT: usize = 100;
/// Highlight for the part being placed during assembly playback.
const STEP_COLOR: [f32; 3] = [1.0, 0.85, 0.25];

pub struct OpenSlicerApp {
    prefs: Preferences,
    camera: Camera,
    renderer: Renderer,
    models: Vec<Model>,
    selected: Option<u64>,
    next_id: u64,
    tool: Tool,
    job: Option<Job>,
    queue: VecDeque<PathBuf>,
    status: (String, Level),
    error: Option<ErrorInfo>,
    show_printer: bool,
    show_about: bool,
    lock_scale: bool,
    move_drag: Option<MoveDrag>,
    pending_fit: bool,
    split: SplitPanel,
    /// Models created from the latest split, with the assembly-pose position of each.
    parts: Vec<(u64, Vec3)>,
    /// Source model hidden while its parts are shown.
    hidden_source: Option<u64>,
    autosaver: Autosaver,
    /// An autosave from the previous session that the user has not yet restored or discarded.
    restore_prompt: Option<PathBuf>,
    project_path: Option<PathBuf>,
    show_stock: bool,
    undo: Vec<SceneState>,
    redo: Vec<SceneState>,
    /// The scene as last recorded; a change that has settled is pushed onto `undo`.
    hist_state: SceneState,
    hist_sig: u64,
    /// World-space copy of the model being cut, for the outline preview of manual planes.
    cut_mesh: Option<(u64, Transform, Arc<Mesh>)>,
    plane_outline: HashMap<usize, PlaneOutline>,
    plane_drag: Option<usize>,
}

impl OpenSlicerApp {
    pub fn new(cc: &eframe::CreationContext<'_>, initial: Vec<PathBuf>) -> Self {
        theme::apply(&cc.egui_ctx);
        let prefs = cc.storage.and_then(|s| eframe::get_value::<Preferences>(s, PREFS_KEY)).unwrap_or_default();
        let mut split = SplitPanel::default();
        if let Some(saved) = cc.storage.and_then(|s| eframe::get_value::<openslicer_project::settings::SplitSettings>(s, SPLIT_KEY)) {
            split.apply_settings(&saved);
        }
        let restore_prompt = if initial.is_empty() { session::autosave_path().filter(|p| p.is_file()) } else { None };
        Self {
            prefs,
            camera: Camera::default(),
            renderer: Renderer::default(),
            models: Vec::new(),
            selected: None,
            next_id: 1,
            tool: Tool::Select,
            job: None,
            queue: initial.into(),
            status: ("Ready. Open an STL, OBJ or 3MF file or drop one onto the window.".into(), Level::Info),
            error: None,
            show_printer: false,
            show_about: false,
            lock_scale: true,
            move_drag: None,
            pending_fit: false,
            split,
            parts: Vec::new(),
            hidden_source: None,
            autosaver: Autosaver::default(),
            restore_prompt,
            project_path: None,
            show_stock: false,
            undo: Vec::new(),
            redo: Vec::new(),
            hist_sig: scene_signature(&SceneState::default()),
            hist_state: SceneState::default(),
            cut_mesh: None,
            plane_outline: HashMap::new(),
            plane_drag: None,
        }
    }

    fn set_status(&mut self, text: impl Into<String>, level: Level) {
        self.status = (text.into(), level);
    }

    fn selected_index(&self) -> Option<usize> {
        let id = self.selected?;
        self.models.iter().position(|m| m.id == id)
    }

    // ---- actions -------------------------------------------------------------

    fn start_next_load(&mut self, ctx: &egui::Context) {
        if self.job.is_some() {
            return;
        }
        if let Some(path) = self.queue.pop_front() {
            if path.extension().is_some_and(|e| e.eq_ignore_ascii_case(openslicer_project::project::EXTENSION)) {
                self.open_project_file(path);
                return;
            }
            if let Some(dir) = path.parent() {
                self.prefs.last_directory = Some(dir.to_string_lossy().into_owned());
            }
            self.job = Some(jobs::spawn_load(ctx, path));
        }
    }

    fn dialog(&self) -> rfd::FileDialog {
        let mut d = rfd::FileDialog::new();
        if let Some(dir) = &self.prefs.last_directory {
            d = d.set_directory(dir);
        }
        d
    }

    fn open_dialog(&mut self, ctx: &egui::Context) {
        if let Some(files) = self.dialog().add_filter("3D models", &["stl", "obj", "3mf"]).set_title("Open model").pick_files() {
            self.queue.extend(files);
            self.start_next_load(ctx);
        }
    }

    fn export_dialog(&mut self, ctx: &egui::Context) {
        if self.job.is_some() {
            return;
        }
        let Some(idx) = self.selected_index() else {
            self.set_status("Select a model to export.", Level::Warn);
            return;
        };
        let name = format!("{}.stl", self.models[idx].name.trim_end_matches(".stl"));
        let Some(path) =
            self.dialog().add_filter("Binary STL", &["stl"]).set_file_name(&name).set_title("Export selected model").save_file()
        else {
            return;
        };
        let mesh = self.models[idx].transformed_mesh();
        self.job = Some(jobs::spawn_export(ctx, mesh, path));
    }

    fn remove_selected(&mut self) {
        if let Some(idx) = self.selected_index() {
            let m = self.models.remove(idx);
            self.renderer.remove(m.id);
            self.selected = self.models.last().map(|m| m.id);
            self.set_status(format!("Removed {}", m.name), Level::Info);
        }
    }

    /// Puts a mesh in the scene. With `announce`, reports its health in the status bar. Returns false for empty meshes.
    fn add_model(
        &mut self,
        name: String,
        mesh: Arc<Mesh>,
        stats: Arc<MeshStats>,
        source: Option<PathBuf>,
        transform: Option<Transform>,
        announce: bool,
    ) -> bool {
        let id = self.next_id;
        let Some(mut model) = Model::new(id, name.clone(), mesh.clone(), stats.clone(), source) else {
            self.error = Some(ErrorInfo { title: format!("Could not open {name}"), detail: "The file contains no geometry.".into() });
            return false;
        };
        if let Some(t) = transform {
            model.transform = t;
        }
        self.next_id += 1;
        self.renderer.upload(id, mesh);
        self.models.push(model);
        self.selected = Some(id);
        self.pending_fit = true;
        if announce {
            let (msg, level) = if stats.is_inverted() {
                (
                    format!(
                        "Opened {name}: {} triangles. Normals are INVERTED (negative volume); slicing would treat the model as empty space",
                        stats.triangle_count
                    ),
                    Level::Warn,
                )
            } else if stats.is_watertight() {
                (format!("Opened {name}: {} triangles, watertight", stats.triangle_count), Level::Ok)
            } else {
                (
                    format!(
                        "Opened {name}: {} triangles. Mesh is NOT watertight ({} open edges, {} non-manifold edges, {} degenerate triangles)",
                        stats.triangle_count, stats.boundary_edges, stats.non_manifold_edges, stats.degenerate_triangles
                    ),
                    Level::Warn,
                )
            };
            self.set_status(msg, level);
        }
        true
    }

    fn import_stock(&mut self, id: &str) {
        let Some(mesh) = stock::build(id) else { return };
        let name = stock::STOCK.iter().find(|s| s.id == id).map_or(id, |s| s.name).to_owned();
        let stats = Arc::new(mesh.stats());
        self.add_model(name, Arc::new(mesh), stats, None, None, true);
    }

    // ---- projects and autosave -----------------------------------------------

    fn snapshot(&self) -> Snapshot {
        let name = self
            .project_path
            .as_ref()
            .and_then(|p| p.file_stem())
            .map_or_else(|| "Untitled".to_owned(), |s| s.to_string_lossy().into_owned());
        let part_ids: Vec<u64> = self.parts.iter().map(|p| p.0).collect();
        let split = self.split.settings().unwrap_or_default();
        Snapshot::capture(&name, &self.prefs.printer, split, self.models.iter().filter(|m| !part_ids.contains(&m.id)))
    }

    fn autosave_tick(&mut self) {
        let snap = self.snapshot();
        self.autosaver.tick(snap, session::autosave_path());
        if let Some(e) = self.autosaver.last_error.take() {
            self.set_status(format!("Autosave failed: {e}"), Level::Warn);
        }
    }

    fn apply_restored(&mut self, r: session::Restored) {
        self.clear_split();
        for m in self.models.drain(..) {
            self.renderer.remove(m.id);
        }
        self.selected = None;
        self.prefs.printer = r.printer;
        self.split.apply_settings(&r.split);
        let count = r.models.len();
        for (name, mesh, transform) in r.models {
            let stats = Arc::new(mesh.stats());
            self.add_model(name, mesh, stats, None, transform, false);
        }
        self.set_status(format!("Restored {count} model{}", if count == 1 { "" } else { "s" }), Level::Ok);
    }

    fn open_project_file(&mut self, path: PathBuf) {
        match session::read(&path) {
            Ok(r) => {
                self.apply_restored(r);
                let name = path.file_name().map(|n| n.to_string_lossy().into_owned()).unwrap_or_default();
                self.set_status(format!("Opened project {name}"), Level::Ok);
                self.project_path = Some(path);
            }
            Err(e) => {
                self.error = Some(ErrorInfo { title: "Could not open the project".into(), detail: e.to_string() });
            }
        }
    }

    fn open_project_dialog(&mut self) {
        if let Some(path) = self.dialog().add_filter("OpenSlicer projects", &["osproj"]).set_title("Open project").pick_file() {
            self.open_project_file(path);
        }
    }

    fn save_project(&mut self, save_as: bool) {
        if self.models.is_empty() {
            self.set_status("Nothing to save: the scene is empty.", Level::Warn);
            return;
        }
        let path = match (&self.project_path, save_as) {
            (Some(p), false) => p.clone(),
            _ => match self
                .dialog()
                .add_filter("OpenSlicer projects", &["osproj"])
                .set_title("Save project")
                .set_file_name("project.osproj")
                .save_file()
            {
                Some(p) => p.with_extension(openslicer_project::project::EXTENSION),
                None => return,
            },
        };
        match self.snapshot().write(&path) {
            Ok(()) => {
                let name = path.file_name().map(|n| n.to_string_lossy().into_owned()).unwrap_or_default();
                self.set_status(format!("Saved {name}"), Level::Ok);
                self.project_path = Some(path);
            }
            Err(e) => self.error = Some(ErrorInfo { title: "Could not save the project".into(), detail: e.to_string() }),
        }
    }

    fn handle_job(&mut self, ctx: &egui::Context) {
        let Some(job) = self.job.as_mut() else {
            self.start_next_load(ctx);
            return;
        };
        let Some(event) = job.poll() else { return };
        self.job = None;
        match event {
            JobEvent::Loaded { path, mesh, stats } => {
                let name = path.file_name().map(|n| n.to_string_lossy().into_owned()).unwrap_or_else(|| "model".into());
                self.add_model(name, mesh, stats, Some(path), None, true);
            }
            JobEvent::Exported { path, summary } => {
                let name = path.file_name().map(|n| n.to_string_lossy().into_owned()).unwrap_or_default();
                self.set_status(format!("Exported {name}: {summary}"), Level::Ok);
            }
            JobEvent::Split { source, outcome } => self.show_split_result(source, *outcome),
            JobEvent::Cancelled => self.set_status("Operation cancelled. No changes were made.", Level::Info),
            JobEvent::Failed { title, detail } => {
                self.set_status(title.clone(), Level::Error);
                self.error = Some(ErrorInfo { title, detail });
            }
            JobEvent::Progress(_) => {}
        }
        self.start_next_load(ctx);
    }

    // ---- splitting -----------------------------------------------------------

    fn start_split(&mut self, ctx: &egui::Context) {
        if self.job.is_some() {
            return;
        }
        let Some(idx) = self.selected_index() else {
            self.set_status("Select a model to split.", Level::Warn);
            return;
        };
        let settings = match self.split.settings() {
            Ok(s) => s,
            Err(e) => {
                self.set_status(e, Level::Warn);
                return;
            }
        };
        // Splitting a previous result again would cut the parts, not the model.
        if self.parts.iter().any(|(id, _)| *id == self.models[idx].id) {
            self.set_status("That is a part of the previous split. Select the original model, or clear the result first.", Level::Warn);
            return;
        }
        let printer = split_ui::project_printer(&self.prefs.printer, self.split.clearance);
        let model = &self.models[idx];
        self.split.source_name = model.name.clone();
        self.job = Some(jobs::spawn_split(ctx, model.name.clone(), model.transformed_mesh(), printer, settings));
    }

    fn clear_split(&mut self) {
        for (id, _) in std::mem::take(&mut self.parts) {
            self.renderer.remove(id);
            self.models.retain(|m| m.id != id);
        }
        if let Some(src) = self.hidden_source.take() {
            if let Some(m) = self.models.iter_mut().find(|m| m.id == src) {
                m.visible = true;
            }
            self.selected = Some(src);
        }
        self.split.result = None;
        self.split.explode = 0.0;
        self.split.steps.clear();
        self.split.step = 0;
        self.split.playing = false;
    }

    fn show_split_result(&mut self, source: String, outcome: openslicer_project::pipeline::SplitOutcome) {
        let src_id = self.selected;
        self.clear_split();
        let mut first = None;
        for piece in outcome.pieces.iter().chain(outcome.loose.iter()) {
            let mesh = std::sync::Arc::new(piece.mesh.clone());
            let stats = std::sync::Arc::new(mesh.stats());
            let id = self.next_id;
            let Some(mut model) = Model::new(id, piece.name.clone(), mesh.clone(), stats, None) else { continue };
            model.color = Some(gl_view::PART_PALETTE[self.parts.len() % gl_view::PART_PALETTE.len()]);
            let centre = piece.bounds.center();
            model.transform.position = centre;
            self.next_id += 1;
            self.renderer.upload(id, mesh);
            self.models.push(model);
            self.parts.push((id, centre));
            first.get_or_insert(id);
        }
        if let Some(src) = src_id {
            if let Some(m) = self.models.iter_mut().find(|m| m.id == src) {
                m.visible = false;
                self.hidden_source = Some(src);
            }
        }
        self.selected = first;
        self.pending_fit = true;
        let problems = outcome.problems.len();
        let msg = format!(
            "Split {source} into {} parts{}",
            outcome.pieces.len(),
            if problems == 0 { String::new() } else { format!(" with {problems} problem(s); see the Split window") }
        );
        self.set_status(msg, if problems == 0 { Level::Ok } else { Level::Warn });
        self.split.explode = 0.0;
        self.split.steps =
            openslicer_project::assembly::plan_assembly(&outcome.pieces, &outcome.interfaces, &outcome.connectors, &outcome.loose);
        self.split.step = 0;
        self.split.playing = false;
        self.split.result = Some(std::sync::Arc::new(outcome));
        self.split.open = true;
    }

    fn apply_explode(&mut self) {
        let Some(r) = &self.split.result else { return };
        let total: f32 = r.pieces.iter().map(|p| p.volume as f32).sum::<f32>().max(1e-6);
        let centre = r.pieces.iter().fold(Vec3::ZERO, |acc, p| acc + p.bounds.center() * (p.volume as f32 / total));
        let factor = self.split.explode * 0.6;
        for (id, base) in &self.parts {
            if let Some(m) = self.models.iter_mut().find(|m| m.id == *id) {
                m.transform.position = *base + split_ui::explode_offset(*base, centre, factor);
            }
        }
    }

    /// Shows the parts that exist at the current playback step and highlights the one being placed.
    fn apply_playback(&mut self) {
        let Some(r) = self.split.result.clone() else { return };
        let step = self.split.step;
        if step != 0 {
            // The highlight colour marks the part being placed; a selection would tint it.
            self.selected = None;
        }
        let pieces = r.pieces.len();
        for (idx, (id, _)) in self.parts.iter().enumerate() {
            let own = gl_view::PART_PALETTE[idx % gl_view::PART_PALETTE.len()];
            let (number, current) = if idx < pieces {
                let n = self.split.steps.iter().find(|s| s.part_id == r.pieces[idx].id).map_or(0, |s| s.number);
                (n, step != 0 && n == step)
            } else {
                let name = r.loose.get(idx - pieces).map(|l| l.name.as_str()).unwrap_or_default();
                let n = self.split.steps.iter().find(|s| s.loose.iter().any(|l| l == name)).map_or(0, |s| s.number);
                (n, false)
            };
            if let Some(m) = self.models.iter_mut().find(|m| m.id == *id) {
                m.visible = step == 0 || number <= step;
                m.color = Some(if current { STEP_COLOR } else { own });
            }
        }
    }

    // ---- manual cutting planes ------------------------------------------------

    /// The model the manual planes act on: the selection, or else the first visible model that is not a part.
    fn cut_target(&self) -> Option<usize> {
        let is_part = |id: u64| self.parts.iter().any(|p| p.0 == id);
        self.selected_index()
            .filter(|&i| !is_part(self.models[i].id))
            .or_else(|| self.models.iter().position(|m| !is_part(m.id) && m.visible))
    }

    fn cut_preview_active(&self) -> bool {
        self.split.open && self.split.layout == split_ui::LayoutChoice::Manual && self.split.show_planes && self.split.result.is_none()
    }

    /// Centre and half-size of the preview squares.
    fn cut_extent(&mut self) -> Option<(Vec3, f32)> {
        let i = self.cut_target()?;
        let b = self.models[i].world_bounds();
        Some((b.center(), (b.max - b.min).length() * 0.5))
    }

    fn pick_plane(&mut self, origin: Vec3, dir: Vec3) -> Option<usize> {
        let (centre, half) = self.cut_extent()?;
        let mut best: Option<(f32, usize)> = None;
        for (i, p) in self.split.planes.iter().enumerate() {
            let (n, d) = (p.normal().as_vec3(), p.distance() as f32);
            let denom = n.dot(dir);
            if denom.abs() < 1e-6 {
                continue;
            }
            let t = (d - n.dot(origin)) / denom;
            if t <= 0.0 {
                continue;
            }
            let hit = origin + dir * t;
            let on_plane_centre = centre - n * (n.dot(centre) - d);
            if (hit - on_plane_centre).length() <= half && best.is_none_or(|(bt, _)| t < bt) {
                best = Some((t, i));
            }
        }
        best.map(|(_, i)| i)
    }

    /// Moves plane `i` along its normal by a mouse movement of `delta` pixels.
    fn drag_plane(&mut self, i: usize, delta: Vec2, rect: Rect, aspect: f32) {
        let Some((centre, _)) = self.cut_extent() else { return };
        let Some(p) = self.split.planes.get(i).copied() else { return };
        let (n, d) = (p.normal().as_vec3(), p.distance() as f32);
        let on_plane = centre - n * (n.dot(centre) - d);
        let vp = self.camera.projection(aspect) * self.camera.view();
        let to_px = |q: Vec3| {
            let h = vp * q.extend(1.0);
            let ndc = Vec2::new(h.x, h.y) / h.w;
            Vec2::new((ndc.x + 1.0) * 0.5 * rect.width(), (1.0 - ndc.y) * 0.5 * rect.height())
        };
        let step = (self.camera.distance * 0.2).max(1.0);
        let dir = to_px(on_plane + n * step) - to_px(on_plane);
        let len2 = dir.length_squared();
        if len2 < 1e-6 || !len2.is_finite() {
            return;
        }
        let mm = delta.dot(dir) / len2 * step;
        let cos = p.tilt_deg.to_radians().cos().max(0.5);
        self.split.planes[i].offset += f64::from(mm) / cos;
    }

    /// Translucent squares and the outline where each plane cuts the model.
    fn cut_preview(&mut self, tris: &mut Vec<gl_view::FlatVertex>, lines: &mut Vec<gl_view::FlatVertex>) {
        if !self.cut_preview_active() {
            return;
        }
        let (Some(ti), Some((centre, half))) = (self.cut_target(), self.cut_extent()) else { return };
        let (id, tr) = (self.models[ti].id, self.models[ti].transform);
        if !matches!(&self.cut_mesh, Some((i, t, _)) if *i == id && *t == tr) {
            self.cut_mesh = Some((id, tr, Arc::new(self.models[ti].transformed_mesh())));
            self.plane_outline.clear();
        }
        let Some((_, _, mesh)) = self.cut_mesh.clone() else { return };
        let ptr = Arc::as_ptr(&mesh) as usize;
        self.plane_outline.retain(|i, _| *i < self.split.planes.len());
        for (i, p) in self.split.planes.iter().enumerate() {
            let (n, d) = (p.normal().as_vec3(), p.distance() as f32);
            let active = i == self.split.active_plane;
            let fill = if active { [1.0, 0.78, 0.2, 0.22] } else { [0.30, 0.76, 1.0, 0.16] };
            let edge = if active { [1.0, 0.85, 0.3, 1.0] } else { [0.30, 0.76, 1.0, 0.8] };
            let q = plane_quad(n, d, centre, half);
            for k in [0, 1, 2, 0, 2, 3] {
                tris.push(gl_view::FlatVertex::new(q[k], fill));
            }
            for k in 0..4 {
                lines.push(gl_view::FlatVertex::new(q[k], edge));
                lines.push(gl_view::FlatVertex::new(q[(k + 1) % 4], edge));
            }
            let stale = !matches!(self.plane_outline.get(&i), Some((cp, cptr, _)) if cp == p && *cptr == ptr);
            if stale {
                self.plane_outline.insert(i, (*p, ptr, section_segments(&mesh, n, d)));
            }
            if let Some((_, _, segs)) = self.plane_outline.get(&i) {
                for (a, b) in segs {
                    lines.push(gl_view::FlatVertex::new(*a, [1.0, 0.85, 0.2, 1.0]));
                    lines.push(gl_view::FlatVertex::new(*b, [1.0, 0.85, 0.2, 1.0]));
                }
            }
        }
    }

    // ---- undo and redo ---------------------------------------------------------

    fn scene_state(&self) -> SceneState {
        let is_part = |id: u64| self.parts.iter().any(|p| p.0 == id);
        SceneState {
            models: self
                .models
                .iter()
                .filter(|m| !is_part(m.id))
                .map(|m| SavedModel {
                    id: m.id,
                    name: m.name.clone(),
                    mesh: m.mesh.clone(),
                    stats: m.stats.clone(),
                    source: m.source.clone(),
                    transform: m.transform,
                })
                .collect(),
            selected: self.selected,
        }
    }

    /// Records the scene once an edit has settled (the mouse is up and no field is being typed in), so a whole
    /// drag or a typed value is one undo step.
    fn history_tick(&mut self, ctx: &egui::Context) {
        if ctx.input(|i| i.pointer.any_down()) || ctx.egui_wants_keyboard_input() {
            return;
        }
        let now = self.scene_state();
        let sig = scene_signature(&now);
        if sig != self.hist_sig {
            let previous = std::mem::replace(&mut self.hist_state, now);
            self.undo.push(previous);
            if self.undo.len() > UNDO_LIMIT {
                self.undo.remove(0);
            }
            self.redo.clear();
            self.hist_sig = sig;
        }
    }

    fn restore_scene(&mut self, st: &SceneState) {
        self.clear_split();
        let keep: HashSet<u64> = st.models.iter().map(|m| m.id).collect();
        let mut old: Vec<Model> = std::mem::take(&mut self.models);
        for m in old.iter().filter(|m| !keep.contains(&m.id)) {
            self.renderer.remove(m.id);
        }
        let mut restored = Vec::new();
        for sm in &st.models {
            if let Some(pos) = old.iter().position(|m| m.id == sm.id) {
                let mut m = old.remove(pos);
                m.name = sm.name.clone();
                m.transform = sm.transform;
                restored.push(m);
            } else if let Some(mut m) = Model::new(sm.id, sm.name.clone(), sm.mesh.clone(), sm.stats.clone(), sm.source.clone()) {
                m.transform = sm.transform;
                self.renderer.upload(sm.id, sm.mesh.clone());
                restored.push(m);
            }
        }
        self.models = restored;
        self.selected = st.selected.filter(|id| keep.contains(id));
        self.hist_state = st.clone();
        self.hist_sig = scene_signature(st);
    }

    fn undo(&mut self) {
        let Some(prev) = self.undo.pop() else { return };
        let current = self.hist_state.clone();
        self.restore_scene(&prev);
        self.redo.push(current);
        self.set_status(format!("Undo ({} more)", self.undo.len()), Level::Info);
    }

    fn redo(&mut self) {
        let Some(next) = self.redo.pop() else { return };
        let current = self.hist_state.clone();
        self.restore_scene(&next);
        self.undo.push(current);
        self.set_status(format!("Redo ({} more)", self.redo.len()), Level::Info);
    }

    fn export_split(&mut self, ctx: &egui::Context) {
        let (Some(result), true) = (self.split.result.clone(), self.job.is_none()) else { return };
        let Some(dir) = self.dialog().set_title("Choose an empty folder for the parts").pick_folder() else { return };
        let printer = split_ui::project_printer(&self.prefs.printer, self.split.clearance);
        let name = self.split.source_name.trim_end_matches(".stl").to_string();
        self.job = Some(jobs::spawn_export_split(ctx, result, printer, name, dir));
    }

    fn split_window(&mut self, ctx: &egui::Context) {
        let can_run = self.selected.is_some();
        let busy = self.job.is_some();
        let bounds = self.cut_target().map(|i| {
            let b = self.models[i].world_bounds();
            (b.min, b.max)
        });
        match self.split.show(ctx, can_run, busy, bounds) {
            Action::PlaybackChanged => self.apply_playback(),
            Action::Run => self.start_split(ctx),
            Action::Export => self.export_split(ctx),
            Action::Clear => self.clear_split(),
            Action::ExplodeChanged => self.apply_explode(),
            Action::None => {}
        }
    }

    fn handle_shortcuts(&mut self, ctx: &egui::Context) {
        let mut open = false;
        let mut export = false;
        let (mut save, mut open_project) = (false, false);
        ctx.input_mut(|i| {
            // The more specific shortcut first, so Ctrl+Shift+O is not taken as Ctrl+O.
            open_project = i.consume_shortcut(&KeyboardShortcut::new(Modifiers::COMMAND | Modifiers::SHIFT, Key::O));
            save = i.consume_shortcut(&KeyboardShortcut::new(Modifiers::COMMAND, Key::S));
            open = i.consume_shortcut(&KeyboardShortcut::new(Modifiers::COMMAND, Key::O));
            export = i.consume_shortcut(&KeyboardShortcut::new(Modifiers::COMMAND, Key::E));
        });
        if open_project {
            self.open_project_dialog();
        }
        if save {
            self.save_project(false);
        }
        if open {
            self.open_dialog(ctx);
        }
        if export {
            self.export_dialog(ctx);
        }
        if ctx.egui_wants_keyboard_input() {
            return;
        }
        let (undo, redo) = ctx.input_mut(|i| {
            let redo = i.consume_shortcut(&KeyboardShortcut::new(Modifiers::COMMAND | Modifiers::SHIFT, Key::Z))
                || i.consume_shortcut(&KeyboardShortcut::new(Modifiers::COMMAND, Key::Y));
            (i.consume_shortcut(&KeyboardShortcut::new(Modifiers::COMMAND, Key::Z)), redo)
        });
        if undo {
            self.undo();
        }
        if redo {
            self.redo();
        }
        let (del, fit, v1, v2, v3, v4, esc) = ctx.input(|i| {
            (
                i.key_pressed(Key::Delete),
                i.key_pressed(Key::F),
                i.key_pressed(Key::Num1),
                i.key_pressed(Key::Num2),
                i.key_pressed(Key::Num3),
                i.key_pressed(Key::Num4),
                i.key_pressed(Key::Escape),
            )
        });
        if del {
            self.remove_selected();
        }
        if fit {
            self.fit_view();
        }
        for (pressed, preset) in [(v1, ViewPreset::Isometric), (v2, ViewPreset::Top), (v3, ViewPreset::Front), (v4, ViewPreset::Right)] {
            if pressed {
                self.camera.set_preset(preset);
            }
        }
        if esc {
            self.tool = Tool::Select;
        }
    }

    /// Frames the selected model, or the build plate and all models when nothing is selected.
    fn fit_view(&mut self) {
        let sel = self.selected;
        let mut bounds: Option<openslicer_core::Aabb> = None;
        for m in self.models.iter_mut().filter(|m| m.visible && sel == Some(m.id)) {
            bounds = Some(m.world_bounds());
        }
        let bounds = bounds.unwrap_or_else(|| {
            let [bx, by, _] = self.prefs.printer.build_volume;
            let mut b = openslicer_core::Aabb { min: Vec3::new(-bx / 2.0, -by / 2.0, 0.0), max: Vec3::new(bx / 2.0, by / 2.0, 0.0) };
            for m in self.models.iter_mut().filter(|m| m.visible) {
                let mb = m.world_bounds();
                b.min = b.min.min(mb.min);
                b.max = b.max.max(mb.max);
            }
            b
        });
        self.camera.fit(bounds);
    }

    // ---- panels --------------------------------------------------------------

    fn menu_bar(&mut self, ui: &mut Ui) {
        let ctx = ui.ctx().clone();
        egui::MenuBar::new().ui(ui, |ui| {
            ui.menu_button("File", |ui| {
                if ui.add(egui::Button::new("Open…").shortcut_text("Ctrl+O")).clicked() {
                    ui.close();
                    self.open_dialog(&ctx);
                }
                if ui.button("Import stock model…").clicked() {
                    ui.close();
                    self.show_stock = true;
                }
                ui.separator();
                if ui.add(egui::Button::new("Open project…").shortcut_text("Ctrl+Shift+O")).clicked() {
                    ui.close();
                    self.open_project_dialog();
                }
                if ui.add_enabled(!self.models.is_empty(), egui::Button::new("Save project").shortcut_text("Ctrl+S")).clicked() {
                    ui.close();
                    self.save_project(false);
                }
                if ui.add_enabled(!self.models.is_empty(), egui::Button::new("Save project as…")).clicked() {
                    ui.close();
                    self.save_project(true);
                }
                ui.separator();
                let can_export = self.selected.is_some() && self.job.is_none();
                if ui.add_enabled(can_export, egui::Button::new("Export selected as STL…").shortcut_text("Ctrl+E")).clicked() {
                    ui.close();
                    self.export_dialog(&ctx);
                }
                ui.separator();
                if ui.button("Exit").clicked() {
                    ctx.send_viewport_cmd(egui::ViewportCommand::Close);
                }
            });
            ui.menu_button("Edit", |ui| {
                if ui.add_enabled(!self.undo.is_empty(), egui::Button::new("Undo").shortcut_text("Ctrl+Z")).clicked() {
                    ui.close();
                    self.undo();
                }
                if ui.add_enabled(!self.redo.is_empty(), egui::Button::new("Redo").shortcut_text("Ctrl+Y")).clicked() {
                    ui.close();
                    self.redo();
                }
            });
            ui.menu_button("View", |ui| {
                ui.checkbox(&mut self.prefs.show_scene_panel, "Scene panel");
                ui.checkbox(&mut self.prefs.show_properties_panel, "Properties panel");
                ui.separator();
                ui.checkbox(&mut self.prefs.show_grid, "Grid");
                ui.checkbox(&mut self.prefs.show_build_volume, "Build volume");
                ui.separator();
                for (label, key, preset) in [
                    ("Isometric", "1", ViewPreset::Isometric),
                    ("Top", "2", ViewPreset::Top),
                    ("Front", "3", ViewPreset::Front),
                    ("Right", "4", ViewPreset::Right),
                ] {
                    if ui.add(egui::Button::new(label).shortcut_text(key)).clicked() {
                        self.camera.set_preset(preset);
                        ui.close();
                    }
                }
                if ui.add(egui::Button::new("Fit to view").shortcut_text("F")).clicked() {
                    self.fit_view();
                    ui.close();
                }
            });
            ui.menu_button("Printer", |ui| {
                if ui.button("Printer profile…").clicked() {
                    self.show_printer = true;
                    ui.close();
                }
            });
            ui.menu_button("Help", |ui| {
                if ui.button("About OpenSlicer").clicked() {
                    self.show_about = true;
                    ui.close();
                }
            });
        });
    }

    fn ribbon(&mut self, ui: &mut Ui) {
        let ctx = ui.ctx().clone();
        let has_sel = self.selected.is_some();
        let busy = self.job.is_some();
        ui.horizontal_centered(|ui| {
            // Centre the command groups.
            let group_w = 78.0 * 13.0 + 8.0 * 12.0;
            ui.add_space(((ui.available_width() - group_w) / 2.0).max(8.0));
            ui.spacing_mut().item_spacing.x = 4.0;
            if ribbon_button(ui, Icon::Open, "Open", !busy, false, "Open a model (Ctrl+O)") {
                self.open_dialog(&ctx);
            }
            if ribbon_button(ui, Icon::Cube, "Stock", !busy, self.show_stock, "Import a built-in sample model") {
                self.show_stock = !self.show_stock;
            }
            if ribbon_button(ui, Icon::Export, "Export", has_sel && !busy, false, "Export the selected model as binary STL (Ctrl+E)") {
                self.export_dialog(&ctx);
            }
            ribbon_sep(ui);
            if ribbon_button(ui, Icon::Select, "Select", true, self.tool == Tool::Select, "Click a model to select it (Esc)") {
                self.tool = Tool::Select;
            }
            if ribbon_button(ui, Icon::Move, "Move", has_sel, self.tool == Tool::Move, "Drag the selected model across the build plate") {
                self.tool = Tool::Move;
            }
            if ribbon_button(ui, Icon::Drop, "To plate", has_sel, false, "Rest the model on the build plate") {
                if let Some(i) = self.selected_index() {
                    self.models[i].drop_to_plate();
                }
            }
            ribbon_sep(ui);
            if ribbon_button(ui, Icon::Fit, "Fit view", true, false, "Frame the model and build plate (F)") {
                self.fit_view();
            }
            if ribbon_button(ui, Icon::Printer, "Printer", true, self.show_printer, "Edit the printer profile and build volume") {
                self.show_printer = !self.show_printer;
            }
            ribbon_sep(ui);
            let can_split = !busy && self.models.iter().any(|m| m.visible);
            if ribbon_button(
                ui,
                Icon::Split,
                "Split",
                can_split || self.split.result.is_some(),
                self.split.open,
                "Split the selected model to fit the printer, with connectors",
            ) {
                self.split.open = !self.split.open;
            }
            if ribbon_button(
                ui,
                Icon::Connector,
                "Connectors",
                can_split || self.split.result.is_some(),
                false,
                "Choose connector type, size and clearance (in the Split window)",
            ) {
                self.split.open = true;
            }
            let have_result = self.split.result.is_some();
            if ribbon_button(
                ui,
                Icon::Assembly,
                "Export parts",
                have_result && !busy,
                false,
                "Write all parts, assembly instructions and a verified manifest to a folder",
            ) {
                self.export_split(&ctx);
            }
        });
    }

    fn status_bar(&mut self, ui: &mut Ui) {
        ui.horizontal_centered(|ui| {
            ui.add_space(8.0);
            let (text, level) = &self.status;
            let color = match level {
                Level::Info => theme::TEXT_SECONDARY,
                Level::Ok => theme::OK,
                Level::Warn => theme::WARN,
                Level::Error => theme::ERROR,
            };
            if let Some(job) = &self.job {
                ui.add(egui::Spinner::new().size(14.0).color(theme::ACCENT));
                ui.label(RichText::new(format!("{}  {}", job.label, job.progress)).size(12.5).color(theme::TEXT));
                if ui.small_button("Cancel").clicked() {
                    job.cancel();
                }
            } else {
                ui.label(RichText::new(text).size(12.5).color(color));
            }
            ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                ui.add_space(8.0);
                let p = &self.prefs.printer;
                caption(ui, format!("{}  ·  {:.0} × {:.0} × {:.0} mm", p.name, p.build_volume[0], p.build_volume[1], p.build_volume[2]));
            });
        });
    }

    fn scene_panel(&mut self, ui: &mut Ui) {
        ui.add_space(4.0);
        ui.label(RichText::new("Scene").size(16.0).strong());
        ui.add_space(6.0);
        if self.models.is_empty() {
            caption(ui, "No models yet.");
            return;
        }
        let printer = self.prefs.printer.clone();
        let mut select = None;
        egui::ScrollArea::vertical().show(ui, |ui| {
            for m in &mut self.models {
                let size = m.world_bounds().size();
                let detail = format!("{:.0} × {:.0} × {:.0} mm", size.x, size.y, size.z);
                let color = match printer.check_fit(size) {
                    Fit::Fits => theme::TEXT_SECONDARY,
                    Fit::Exceeds(_) => theme::WARN, // amber size = does not fit the printer
                };
                let (resp, eye) = model_row(ui, &m.name, &detail, color, self.selected == Some(m.id), m.visible);
                if eye {
                    m.visible = !m.visible;
                } else if resp.clicked() {
                    select = Some(m.id);
                }
            }
        });
        if select.is_some() {
            self.selected = select;
        }
    }

    fn properties_panel(&mut self, ui: &mut Ui) {
        ui.add_space(4.0);
        ui.label(RichText::new("Properties").size(16.0).strong());
        let printer = self.prefs.printer.clone();
        let Some(idx) = self.selected_index() else {
            ui.add_space(6.0);
            caption(ui, "Select a model to see its properties.");
            return;
        };
        let lock = &mut self.lock_scale;
        let mut remove = false;
        let m = &mut self.models[idx];
        egui::ScrollArea::vertical().show(ui, |ui| {
            section_header(ui, "Name");
            ui.add(egui::TextEdit::singleline(&mut m.name).desired_width(f32::INFINITY));

            section_header(ui, "Transform");
            let mut pos = m.transform.position;
            if vec3_row(ui, "Position", &mut pos, 0.5, " mm", 2, true).is_some() {
                m.transform.position = pos;
            }
            let mut rot = m.transform.rotation_deg;
            if vec3_row(ui, "Rotation", &mut rot, 0.5, "°", 1, true).is_some() {
                m.transform.rotation_deg = rot;
            }
            let mut scale_pct = m.transform.scale * 100.0;
            if let Some(axis) = vec3_row(ui, "Scale", &mut scale_pct, 0.5, " %", 1, true) {
                let s = (scale_pct / 100.0).to_array();
                if *lock {
                    m.transform.scale = Vec3::splat(s[axis]);
                } else {
                    m.transform.scale = Vec3::from(s);
                }
            }
            ui.checkbox(lock, "Lock proportions");

            let rotated = m.transform.rotation_deg != Vec3::ZERO;
            let mut size = m.world_bounds().size();
            if vec3_row(ui, "Size", &mut size, 0.5, " mm", 2, !rotated).is_some() {
                if *lock {
                    // Keep proportions: scale uniformly by the changed axis' ratio.
                    let cur = m.world_bounds().size();
                    let ratio = (0..3).find(|&i| (size[i] - cur[i]).abs() > 1e-6).map(|i| size[i] / cur[i]).unwrap_or(1.0);
                    m.set_world_size(cur * ratio);
                } else {
                    m.set_world_size(size);
                }
            }
            if rotated {
                caption(ui, "Size is the rotated bounding box; reset rotation to edit it directly.");
            }
            ui.horizontal(|ui| {
                if ui.button("Drop to plate").clicked() {
                    m.drop_to_plate();
                }
                if ui.button("Centre").clicked() {
                    m.centre_on_plate();
                }
                if ui.button("Reset").clicked() {
                    let z = m.transform.position.z;
                    m.transform = crate::model::Transform { position: Vec3::new(0.0, 0.0, z), ..Default::default() };
                    m.drop_to_plate();
                }
            });

            section_header(ui, "Printer fit");
            match printer.check_fit(m.world_bounds().size()) {
                Fit::Fits => chip(ui, "Fits the build volume", theme::OK),
                Fit::Exceeds(over) => {
                    chip(ui, "Exceeds the build volume", theme::WARN);
                    caption(ui, format!("Over by {:.1} × {:.1} × {:.1} mm", over.x, over.y, over.z));
                    caption(ui, "Use Split in the command bar to cut it into parts that fit.");
                }
            }

            section_header(ui, "Mesh");
            let s = m.stats.clone();
            if s.is_inverted() {
                chip(ui, "Inverted normals", theme::WARN);
            } else if s.is_watertight() {
                chip(ui, "Watertight", theme::OK);
            } else {
                chip(ui, "Not watertight", theme::WARN);
            }
            egui::Grid::new("mesh-stats").num_columns(2).spacing([12.0, 2.0]).min_row_height(20.0).show(ui, |ui| {
                let mut row = |k: &str, v: String, warn: bool| {
                    caption(ui, k);
                    ui.label(RichText::new(v).color(if warn { theme::WARN } else { theme::TEXT }));
                    ui.end_row();
                };
                row("Triangles", group(s.triangle_count as u64), false);
                row("Vertices", group(s.vertex_count as u64), false);
                row("Volume", format!("{:.2} cm³", s.signed_volume / 1000.0), s.signed_volume < 0.0);
                row("Surface area", format!("{:.2} cm²", s.surface_area / 100.0), false);
                row("Open edges", group(s.boundary_edges as u64), s.boundary_edges > 0);
                row("Non-manifold edges", group(s.non_manifold_edges as u64), s.non_manifold_edges > 0);
                row("Degenerate triangles", group(s.degenerate_triangles as u64), s.degenerate_triangles > 0);
            });
            if s.signed_volume < 0.0 {
                caption(ui, "Negative volume: the triangle winding is inverted (faces point inward).");
            }
            if let Some(src) = &m.source {
                caption(ui, format!("Source: {}", src.display()));
            }

            ui.add_space(10.0);
            if ui.add(egui::Button::new(RichText::new("Remove model").color(theme::ERROR))).clicked() {
                remove = true;
            }
        });
        if remove {
            self.remove_selected();
        }
    }

    fn printer_window(&mut self, ctx: &egui::Context) {
        if !self.show_printer {
            return;
        }
        let mut open = true;
        egui::Window::new("Printer profile")
            .open(&mut open)
            .collapsible(false)
            .resizable(false)
            .anchor(Align2::RIGHT_TOP, vec2(-16.0, 56.0))
            .show(ctx, |ui| {
                let p = &mut self.prefs.printer;
                ui.horizontal(|ui| {
                    ui.label("Name");
                    ui.add(egui::TextEdit::singleline(&mut p.name).desired_width(220.0));
                });
                ui.add_space(4.0);
                egui::Grid::new("printer-grid").num_columns(2).spacing([12.0, 6.0]).show(ui, |ui| {
                    for (label, v) in [("Build X", 0usize), ("Build Y", 1), ("Build Z", 2)] {
                        ui.label(label);
                        ui.add(egui::DragValue::new(&mut p.build_volume[v]).range(10.0..=5000.0).speed(1.0).suffix(" mm"));
                        ui.end_row();
                    }
                    ui.label("Margin");
                    ui.add(egui::DragValue::new(&mut p.margin).range(0.0..=100.0).speed(0.5).suffix(" mm"));
                    ui.end_row();
                });
                ui.add_space(4.0);
                let u = p.usable();
                caption(ui, format!("Usable volume: {:.0} × {:.0} × {:.0} mm", u.x, u.y, u.z));
                ui.add_space(6.0);
                ui.label(RichText::new("Presets").color(theme::TEXT_SECONDARY));
                let current = p.name.clone();
                egui::ScrollArea::vertical().max_height(260.0).id_salt("printer-presets").show(ui, |ui| {
                    for preset in openslicer_project::profile::PrinterProfile::presets() {
                        let [x, y, z] = preset.build_volume;
                        let label = format!("{}  ({x:.0} × {y:.0} × {z:.0})", preset.name);
                        if ui.selectable_label(current == preset.name, label).clicked() {
                            *p =
                                PrinterProfile { name: preset.name, build_volume: preset.build_volume.map(|v| v as f32), margin: p.margin };
                        }
                    }
                });
                caption(ui, "Common printers, from published specifications. Check yours: bed clips and firmware change the usable area.");
            });
        self.show_printer = open;
    }

    fn stock_window(&mut self, ctx: &egui::Context) {
        if !self.show_stock {
            return;
        }
        let mut open = true;
        let mut chosen: Option<&'static str> = None;
        egui::Window::new("Import stock model")
            .open(&mut open)
            .collapsible(false)
            .resizable(false)
            .anchor(Align2::CENTER_CENTER, vec2(0.0, 0.0))
            .show(ctx, |ui| {
                ui.set_width(420.0);
                caption(ui, "Sample models larger than a print bed, generated inside the app. Use them to try the split, connector and export workflow.");
                for model in &stock::STOCK {
                    ui.add_space(8.0);
                    ui.separator();
                    ui.add_space(4.0);
                    ui.horizontal(|ui| {
                        ui.label(RichText::new(model.name).size(16.0).strong());
                        ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                            if widgets::primary_button(ui, "Import").clicked() {
                                chosen = Some(model.id);
                            }
                        });
                    });
                    ui.label(model.description);
                }
            });
        if let Some(id) = chosen {
            self.import_stock(id);
            open = false;
        }
        self.show_stock = open;
    }

    fn restore_window(&mut self, ctx: &egui::Context) {
        let Some(path) = self.restore_prompt.clone() else { return };
        let age = std::fs::metadata(&path).and_then(|m| m.modified()).ok().and_then(|t| t.elapsed().ok());
        let mut decision: Option<bool> = None;
        egui::Modal::new(egui::Id::new("restore-modal")).show(ctx, |ui| {
            ui.set_width(420.0);
            ui.label(RichText::new("Restore your last session?").size(18.0).strong());
            ui.add_space(8.0);
            let when = match age {
                Some(a) if a.as_secs() < 90 => "less than a minute ago".to_owned(),
                Some(a) if a.as_secs() < 5400 => format!("{} minutes ago", a.as_secs() / 60),
                Some(a) if a.as_secs() < 172_800 => format!("{} hours ago", a.as_secs() / 3600),
                Some(a) => format!("{} days ago", a.as_secs() / 86_400),
                None => "earlier".to_owned(),
            };
            ui.label(format!("OpenSlicer autosaved your models, their positions, the printer and the split settings {when}. Split results are not saved; run the split again."));
            ui.add_space(14.0);
            ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                if widgets::primary_button(ui, "Restore").clicked() {
                    decision = Some(true);
                }
                if ui.button("Start empty").clicked() {
                    decision = Some(false);
                }
            });
        });
        if let Some(restore) = decision {
            self.restore_prompt = None;
            if restore {
                self.open_project_file(path);
                // The autosave is not the user's own project file.
                self.project_path = None;
                self.set_status("Restored your last session from the autosave", Level::Ok);
            } else {
                let _ = std::fs::remove_file(&path);
            }
        }
    }

    fn dialogs(&mut self, ctx: &egui::Context) {
        self.printer_window(ctx);
        self.stock_window(ctx);
        self.restore_window(ctx);
        self.split_window(ctx);
        let mut close_error = false;
        if let Some(err) = &self.error {
            egui::Modal::new(egui::Id::new("error-modal")).show(ctx, |ui| {
                ui.set_width(420.0);
                ui.label(RichText::new(&err.title).size(18.0).strong().color(theme::ERROR));
                ui.add_space(8.0);
                ui.label(&err.detail);
                ui.add_space(14.0);
                ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                    if widgets::primary_button(ui, "OK").clicked() {
                        close_error = true;
                    }
                });
            });
        }
        if close_error {
            self.error = None;
        }
        if self.show_about {
            let mut open = true;
            egui::Window::new("About OpenSlicer")
                .open(&mut open)
                .collapsible(false)
                .resizable(false)
                .anchor(Align2::CENTER_CENTER, vec2(0.0, 0.0))
                .show(ctx, |ui| {
                    ui.label(RichText::new("OpenSlicer").size(22.0).strong());
                    caption(ui, format!("Version {}  ·  MIT OR Apache-2.0", env!("CARGO_PKG_VERSION")));
                    ui.add_space(8.0);
                    ui.label("Open-source tool for splitting oversized models into printable parts.");
                    ui.add_space(6.0);
                    caption(ui, "Splitting, connectors, labels, assembly playback, verified export, projects and autosave.");
                    caption(ui, "Alternative construction methods and image tools are available in openslicer-cli.");
                });
            self.show_about = open;
        }
        if let Some(e) = self.renderer.take_error() {
            self.error = Some(ErrorInfo {
                title: "3D viewport error".into(),
                detail: format!("{e}\n\nOpenSlicer needs OpenGL 3.3 or newer. Update your graphics driver and try again."),
            });
        }
    }

    // ---- viewport ------------------------------------------------------------

    fn viewport(&mut self, ui: &mut Ui) {
        let rect = ui.available_rect_before_wrap();
        let resp = ui.allocate_rect(rect, Sense::click_and_drag());
        let aspect = rect.width() / rect.height().max(1.0);

        self.handle_viewport_input(ui, &resp, rect, aspect);

        if self.pending_fit {
            self.pending_fit = false;
            self.fit_view();
        }

        // Backdrop gradient.
        {
            let (top, bottom) = (Color32::from_rgb(0x30, 0x33, 0x38), Color32::from_rgb(0x1B, 0x1C, 0x1F));
            let mut mesh = egui::Mesh::default();
            mesh.colored_vertex(rect.left_top(), top);
            mesh.colored_vertex(rect.right_top(), top);
            mesh.colored_vertex(rect.right_bottom(), bottom);
            mesh.colored_vertex(rect.left_bottom(), bottom);
            mesh.add_triangle(0, 1, 2);
            mesh.add_triangle(0, 2, 3);
            ui.painter().add(egui::Shape::mesh(mesh));
        }

        // 3D scene.
        let p = self.prefs.printer.clone();
        let [bx, by, bz] = p.build_volume;
        let sel = self.selected;
        let mut overlay_lines = Vec::new();
        if self.prefs.show_build_volume {
            gl_view::push_box_lines(
                &mut overlay_lines,
                Vec3::new(-bx / 2.0, -by / 2.0, 0.0),
                Vec3::new(bx / 2.0, by / 2.0, bz),
                [0.30, 0.76, 1.0, 0.30],
            );
        }
        let mut models = Vec::new();
        for m in self.models.iter_mut() {
            if !m.visible {
                continue;
            }
            let is_sel = sel == Some(m.id);
            if is_sel {
                let b = m.world_bounds();
                let color = match p.check_fit(b.size()) {
                    Fit::Fits => [0.30, 0.76, 1.0, 0.9],
                    Fit::Exceeds(_) => [1.0, 0.35, 0.35, 0.95],
                };
                gl_view::push_box_lines(&mut overlay_lines, b.min, b.max, color);
            }
            models.push(DrawModel { id: m.id, matrix: m.matrix(), selected: is_sel, mirrored: m.is_mirrored(), color: m.color });
        }
        let mut plane_tris = Vec::new();
        self.cut_preview(&mut plane_tris, &mut overlay_lines);
        let frame = gl_view::Frame {
            view: self.camera.view(),
            proj: self.camera.projection(aspect),
            underlay: gl_view::plate_batches((bx / 2.0, by / 2.0), self.prefs.show_grid),
            models,
            overlay: vec![FlatBatch { triangles: true, verts: plane_tris }, FlatBatch { triangles: false, verts: overlay_lines }],
        };
        ui.painter().add(egui::PaintCallback { rect, callback: self.renderer.callback(frame) });

        self.viewport_overlays(ui, rect);
    }

    fn handle_viewport_input(&mut self, ui: &mut Ui, resp: &egui::Response, rect: Rect, aspect: f32) {
        let to_ndc = |pos: egui::Pos2| {
            Vec2::new(((pos.x - rect.left()) / rect.width()) * 2.0 - 1.0, 1.0 - ((pos.y - rect.top()) / rect.height()) * 2.0)
        };
        let (shift, middle_down) = ui.input(|i| (i.modifiers.shift, i.pointer.middle_down()));

        // Zoom.
        if resp.hovered() {
            let (scroll, pinch) = ui.input(|i| (i.smooth_scroll_delta.y, i.zoom_delta()));
            if scroll != 0.0 {
                self.camera.zoom(scroll);
            }
            if pinch != 1.0 {
                self.camera.distance = (self.camera.distance / pinch).clamp(1.0, 1.0e6);
            }
        }

        // Manual cutting planes take the drag when it starts on one.
        if resp.drag_started_by(PointerButton::Primary) && !shift && self.cut_preview_active() {
            if let Some(pos) = resp.interact_pointer_pos() {
                let (o, d) = self.camera.ray(to_ndc(pos), aspect);
                if let Some(i) = self.pick_plane(o, d) {
                    self.plane_drag = Some(i);
                    self.split.active_plane = i;
                }
            }
        }

        // Start of a primary drag: decide between moving a model and orbiting.
        if resp.drag_started_by(PointerButton::Primary) && self.tool == Tool::Move && !shift {
            if let Some(pos) = resp.interact_pointer_pos() {
                let (o, d) = self.camera.ray(to_ndc(pos), aspect);
                if let Some(id) = self.pick(o, d) {
                    self.selected = Some(id);
                    if let Some(idx) = self.selected_index() {
                        let plane_z = self.models[idx].world_bounds().center().z;
                        if let Some(hit) = ray_plane_z(o, d, plane_z) {
                            self.move_drag = Some(MoveDrag { id, start_hit: hit, start_pos: self.models[idx].transform.position, plane_z });
                        }
                    }
                }
            }
        }

        if resp.dragged() {
            let delta = resp.drag_delta();
            let d = Vec2::new(delta.x, delta.y);
            let pan = resp.dragged_by(PointerButton::Secondary) || middle_down || (resp.dragged_by(PointerButton::Primary) && shift);
            if let (Some(i), true) = (self.plane_drag, resp.dragged_by(PointerButton::Primary)) {
                self.drag_plane(i, d, rect, aspect);
            } else if let (Some(md), true) = (&self.move_drag, resp.dragged_by(PointerButton::Primary)) {
                if let (Some(pos), Some(idx)) = (resp.interact_pointer_pos(), self.models.iter().position(|m| m.id == md.id)) {
                    let (o, dir) = self.camera.ray(to_ndc(pos), aspect);
                    if let Some(hit) = ray_plane_z(o, dir, md.plane_z) {
                        let off = hit - md.start_hit;
                        self.models[idx].transform.position = md.start_pos + Vec3::new(off.x, off.y, 0.0);
                    }
                }
            } else if pan {
                self.camera.pan(d, rect.height());
            } else if resp.dragged_by(PointerButton::Primary) {
                self.camera.orbit(d);
            }
        }
        if resp.drag_stopped() {
            self.move_drag = None;
            self.plane_drag = None;
        }

        if resp.clicked_by(PointerButton::Primary) {
            if let Some(pos) = resp.interact_pointer_pos() {
                let (o, d) = self.camera.ray(to_ndc(pos), aspect);
                self.selected = self.pick(o, d);
            }
        }
        if resp.double_clicked_by(PointerButton::Primary) {
            self.fit_view();
        }
    }

    /// Nearest visible model hit by a world-space ray.
    fn pick(&mut self, origin: Vec3, dir: Vec3) -> Option<u64> {
        self.models
            .iter_mut()
            .filter(|m| m.visible)
            .filter_map(|m| m.pick(origin, dir).map(|t| (t, m.id)))
            .min_by(|a, b| a.0.total_cmp(&b.0))
            .map(|(_, id)| id)
    }

    fn viewport_overlays(&mut self, ui: &mut Ui, rect: Rect) {
        let painter = ui.painter_at(rect);

        // Axis triad.
        {
            let origin = rect.left_bottom() + vec2(54.0, -54.0);
            let view = self.camera.view();
            let mut axes = [
                (Vec3::X, "X", Color32::from_rgb(0xF2, 0x59, 0x59)),
                (Vec3::Y, "Y", Color32::from_rgb(0x73, 0xD9, 0x66)),
                (Vec3::Z, "Z", Color32::from_rgb(0x59, 0xA6, 0xF2)),
            ]
            .map(|(a, l, c)| (view.transform_vector3(a), l, c));
            axes.sort_by(|a, b| a.0.z.total_cmp(&b.0.z));
            for (v, label, color) in axes {
                let end = origin + vec2(v.x, -v.y) * 30.0;
                painter.line_segment([origin, end], Stroke::new(2.0, color));
                painter.circle_filled(end, 8.0, color.gamma_multiply(0.85));
                painter.text(end, Align2::CENTER_CENTER, label, FontId::proportional(11.0), Color32::BLACK);
            }
        }

        // View buttons.
        let bar = Rect::from_min_size(rect.right_top() + vec2(-8.0 - 4.0 * 30.0 - 8.0, 10.0), vec2(4.0 * 30.0 + 8.0, 36.0));
        painter.rect(
            bar,
            CornerRadius::same(8),
            theme::SURFACE.gamma_multiply(0.92),
            Stroke::new(1.0, theme::STROKE),
            egui::StrokeKind::Inside,
        );
        let mut child = ui.new_child(
            egui::UiBuilder::new().max_rect(bar.shrink2(vec2(4.0, 4.0))).layout(egui::Layout::left_to_right(egui::Align::Center)),
        );
        child.spacing_mut().item_spacing.x = 0.0;
        for (label, preset, tip) in [
            ("ISO", ViewPreset::Isometric, "Isometric view (1)"),
            ("TOP", ViewPreset::Top, "Top view (2)"),
            ("FRT", ViewPreset::Front, "Front view (3)"),
            ("RGT", ViewPreset::Right, "Right view (4)"),
        ] {
            let (r, resp) = child.allocate_exact_size(vec2(30.0, 28.0), Sense::click());
            if resp.hovered() {
                child.painter().rect_filled(r, CornerRadius::same(4), theme::CONTROL_HOVER);
            }
            child.painter().text(r.center(), Align2::CENTER_CENTER, label, FontId::proportional(10.5), theme::TEXT_SECONDARY);
            if resp.on_hover_text(tip).clicked() {
                self.camera.set_preset(preset);
            }
        }

        if self.models.is_empty() && self.job.is_none() {
            painter.text(
                rect.center() + vec2(0.0, -70.0),
                Align2::CENTER_CENTER,
                "Drop an STL file here, or press Ctrl+O",
                FontId::proportional(16.0),
                theme::TEXT_SECONDARY.gamma_multiply(0.9),
            );
        }

        // Warn banner for a selected model that does not fit.
        if let Some(idx) = self.selected_index() {
            let size = self.models[idx].world_bounds().size();
            if let Fit::Exceeds(over) = self.prefs.printer.check_fit(size) {
                let text = format!("Too large for {}: over by {:.0} × {:.0} × {:.0} mm", self.prefs.printer.name, over.x, over.y, over.z);
                let galley = painter.layout_no_wrap(text, FontId::proportional(13.0), theme::WARN);
                let r = Rect::from_center_size(egui::pos2(rect.center().x, rect.top() + 28.0), galley.size() + vec2(28.0, 14.0));
                painter.rect(
                    r,
                    CornerRadius::same(8),
                    theme::SURFACE.gamma_multiply(0.92),
                    Stroke::new(1.0, theme::WARN.gamma_multiply(0.6)),
                    egui::StrokeKind::Inside,
                );
                painter.galley(r.center() - galley.size() * 0.5, galley, theme::WARN);
            }
        }
    }
}

fn ribbon_sep(ui: &mut Ui) {
    ui.add_space(4.0);
    let (r, _) = ui.allocate_exact_size(vec2(1.0, 44.0), Sense::hover());
    ui.painter().rect_filled(r, 0.0, theme::STROKE);
    ui.add_space(4.0);
}

/// Formats an integer with thousands separators.
fn group(n: u64) -> String {
    let s = n.to_string();
    let mut out = String::new();
    for (i, c) in s.chars().enumerate() {
        if i > 0 && (s.len() - i) % 3 == 0 {
            out.push(',');
        }
        out.push(c);
    }
    out
}

impl eframe::App for OpenSlicerApp {
    fn logic(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        let dropped: Vec<PathBuf> = ctx.input(|i| i.raw.dropped_files.iter().map(|f| f.path().to_path_buf()).collect());
        self.queue.extend(dropped);
        self.handle_job(ctx);
    }

    fn ui(&mut self, ui: &mut Ui, _frame: &mut eframe::Frame) {
        let ctx = ui.ctx().clone();
        self.handle_shortcuts(&ctx);
        let bar_frame = |fill| egui::Frame::new().fill(fill).inner_margin(egui::Margin::symmetric(8, 0));

        egui::Panel::top("menu").exact_size(32.0).frame(bar_frame(theme::BG)).show(ui, |ui| self.menu_bar(ui));
        egui::Panel::bottom("status").exact_size(28.0).frame(bar_frame(theme::BG)).show(ui, |ui| self.status_bar(ui));
        egui::Panel::bottom("ribbon").exact_size(82.0).frame(bar_frame(theme::SURFACE)).show(ui, |ui| self.ribbon(ui));
        let side = |fill| egui::Frame::new().fill(fill).inner_margin(egui::Margin::same(12));
        if self.prefs.show_scene_panel {
            egui::Panel::left("scene")
                .default_size(250.0)
                .size_range(200.0..=420.0)
                .frame(side(theme::SURFACE))
                .show(ui, |ui| self.scene_panel(ui));
        }
        if self.prefs.show_properties_panel {
            egui::Panel::right("properties")
                .default_size(340.0)
                .size_range(280.0..=520.0)
                .frame(side(theme::SURFACE))
                .show(ui, |ui| self.properties_panel(ui));
        }
        egui::CentralPanel::default().frame(egui::Frame::NONE).show(ui, |ui| self.viewport(ui));
        self.dialogs(&ctx);
        self.history_tick(&ctx);

        if self.job.is_some() {
            ctx.request_repaint_after(std::time::Duration::from_millis(100));
        }
    }

    fn save(&mut self, storage: &mut dyn eframe::Storage) {
        eframe::set_value(storage, PREFS_KEY, &self.prefs);
        if let Ok(settings) = self.split.settings() {
            eframe::set_value(storage, SPLIT_KEY, &settings);
        }
        // Do not overwrite the previous session's autosave until the user has decided about it.
        if self.restore_prompt.is_none() {
            self.autosave_tick();
        }
    }

    fn auto_save_interval(&self) -> std::time::Duration {
        std::time::Duration::from_secs(10)
    }

    fn on_exit(&mut self, _gl: Option<&eframe::glow::Context>) {
        if self.restore_prompt.is_none() {
            self.autosaver.finish();
            self.autosave_tick();
            self.autosaver.finish();
        }
    }
}

#[cfg(test)]
mod tests {
    use super::group;
    use super::{scene_signature, SavedModel, SceneState};
    use crate::model::Transform;
    use glam::Vec3;
    use openslicer_core::Mesh;
    use std::sync::Arc;

    fn saved(id: u64) -> SavedModel {
        let mesh = Arc::new(Mesh::cuboid(Vec3::splat(10.0)));
        SavedModel { id, name: format!("M{id}"), stats: Arc::new(mesh.stats()), mesh, source: None, transform: Transform::default() }
    }

    #[test]
    fn the_scene_signature_changes_with_edits_and_not_without() {
        let mut s = SceneState { models: vec![saved(1), saved(2)], selected: Some(1) };
        let base = scene_signature(&s);
        assert_eq!(base, scene_signature(&s.clone()));
        s.selected = Some(2);
        assert_eq!(base, scene_signature(&s), "selection alone is not an edit");
        s.models[0].transform.position.x += 0.5;
        assert_ne!(base, scene_signature(&s));
        let moved = scene_signature(&s);
        s.models[0].name.push('!');
        assert_ne!(moved, scene_signature(&s));
        s.models.pop();
        assert_ne!(moved, scene_signature(&s));
    }

    #[test]
    fn thousands_grouping() {
        assert_eq!(group(0), "0");
        assert_eq!(group(999), "999");
        assert_eq!(group(1_000), "1,000");
        assert_eq!(group(12_345_678), "12,345,678");
    }
}
