use crate::camera::{ray_plane_z, Camera, ViewPreset};
use crate::gl_view::{self, DrawModel, FlatBatch, Renderer};
use crate::icons::Icon;
use crate::jobs::{self, Job, JobEvent};
use crate::model::Model;
use crate::settings::{Fit, Preferences, PrinterProfile};
use crate::theme;
use crate::widgets::{self, caption, chip, model_row, ribbon_button, section_header, vec3_row};
use eframe::egui::{
    self, vec2, Align2, Color32, CornerRadius, FontId, Key, KeyboardShortcut, Modifiers, PointerButton, Rect, RichText, Sense, Stroke, Ui,
};
use glam::{Vec2, Vec3};
use std::collections::VecDeque;
use std::path::PathBuf;

const PREFS_KEY: &str = "openslicer.preferences.v1";

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
}

impl OpenSlicerApp {
    pub fn new(cc: &eframe::CreationContext<'_>, initial: Vec<PathBuf>) -> Self {
        theme::apply(&cc.egui_ctx);
        let prefs = cc.storage.and_then(|s| eframe::get_value::<Preferences>(s, PREFS_KEY)).unwrap_or_default();
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
            status: ("Ready. Open an STL file or drop one onto the window.".into(), Level::Info),
            error: None,
            show_printer: false,
            show_about: false,
            lock_scale: true,
            move_drag: None,
            pending_fit: false,
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
        if let Some(files) = self.dialog().add_filter("3D models", &["stl"]).set_title("Open model").pick_files() {
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
                let id = self.next_id;
                match Model::new(id, name.clone(), mesh.clone(), stats.clone(), Some(path)) {
                    Some(model) => {
                        self.next_id += 1;
                        self.renderer.upload(id, mesh);
                        self.models.push(model);
                        self.selected = Some(id);
                        self.pending_fit = true;
                        let (msg, level) = if stats.is_inverted() {
                            (format!("Opened {name}: {} triangles. Normals are INVERTED (negative volume); slicing would treat the model as empty space", stats.triangle_count), Level::Warn)
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
                    None => {
                        self.error =
                            Some(ErrorInfo { title: format!("Could not open {name}"), detail: "The file contains no geometry.".into() });
                    }
                }
            }
            JobEvent::Exported { path, summary } => {
                let name = path.file_name().map(|n| n.to_string_lossy().into_owned()).unwrap_or_default();
                self.set_status(format!("Exported {name}: {summary}"), Level::Ok);
            }
            JobEvent::Cancelled => self.set_status("Operation cancelled. No changes were made.", Level::Info),
            JobEvent::Failed { title, detail } => {
                self.set_status(title.clone(), Level::Error);
                self.error = Some(ErrorInfo { title, detail });
            }
            JobEvent::Progress(_) => {}
        }
        self.start_next_load(ctx);
    }

    fn handle_shortcuts(&mut self, ctx: &egui::Context) {
        let mut open = false;
        let mut export = false;
        ctx.input_mut(|i| {
            open = i.consume_shortcut(&KeyboardShortcut::new(Modifiers::COMMAND, Key::O));
            export = i.consume_shortcut(&KeyboardShortcut::new(Modifiers::COMMAND, Key::E));
        });
        if open {
            self.open_dialog(ctx);
        }
        if export {
            self.export_dialog(ctx);
        }
        if ctx.egui_wants_keyboard_input() {
            return;
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
            let group_w = 78.0 * 12.0 + 8.0 * 11.0;
            ui.add_space(((ui.available_width() - group_w) / 2.0).max(8.0));
            ui.spacing_mut().item_spacing.x = 4.0;
            if ribbon_button(ui, Icon::Open, "Open", !busy, false, "Open a model (Ctrl+O)") {
                self.open_dialog(&ctx);
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
            let todo = "Not implemented yet";
            ribbon_button(ui, Icon::Split, "Split", false, false, todo);
            ribbon_button(ui, Icon::Connector, "Connectors", false, false, todo);
            ribbon_button(ui, Icon::Assembly, "Assembly", false, false, todo);
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
                    caption(ui, "Splitting is not implemented yet.");
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
                for (name, v) in [
                    ("Creality Ender-3 (220 × 220 × 250)", [220.0, 220.0, 250.0]),
                    ("Prusa MK4 (250 × 210 × 220)", [250.0, 210.0, 220.0]),
                    ("Bambu Lab X1 / P1 (256 × 256 × 256)", [256.0, 256.0, 256.0]),
                    ("Voron 2.4 350 (350 × 350 × 330)", [350.0, 350.0, 330.0]),
                ] {
                    if ui.button(name).clicked() {
                        *p = PrinterProfile { name: name.split(" (").next().unwrap().to_owned(), build_volume: v, margin: p.margin };
                    }
                }
            });
        self.show_printer = open;
    }

    fn dialogs(&mut self, ctx: &egui::Context) {
        self.printer_window(ctx);
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
                    caption(ui, "Early skeleton: model viewing, transforms, build-volume checks and STL export work.");
                    caption(ui, "Splitting, connectors and assembly planning are not implemented yet.");
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
            models.push(DrawModel { id: m.id, matrix: m.matrix(), selected: is_sel, mirrored: m.is_mirrored() });
        }
        let frame = gl_view::Frame {
            view: self.camera.view(),
            proj: self.camera.projection(aspect),
            underlay: gl_view::plate_batches((bx / 2.0, by / 2.0), self.prefs.show_grid),
            models,
            overlay: vec![FlatBatch { triangles: false, verts: overlay_lines }],
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
            if let (Some(md), true) = (&self.move_drag, resp.dragged_by(PointerButton::Primary)) {
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

        if self.job.is_some() {
            ctx.request_repaint_after(std::time::Duration::from_millis(100));
        }
    }

    fn save(&mut self, storage: &mut dyn eframe::Storage) {
        eframe::set_value(storage, PREFS_KEY, &self.prefs);
    }
}

#[cfg(test)]
mod tests {
    use super::group;

    #[test]
    fn thousands_grouping() {
        assert_eq!(group(0), "0");
        assert_eq!(group(999), "999");
        assert_eq!(group(1_000), "1,000");
        assert_eq!(group(12_345_678), "12,345,678");
    }
}
