//! The Split window: choose a layout and connectors, run the pipeline, review the outcome, export.

use crate::cutplanes::{CutPlane, MAX_TILT};
use crate::settings::PrinterProfile;
use crate::theme;
use crate::widgets::caption;
use eframe::egui::{self, vec2, Align2, RichText};
use glam::Vec3;
use openslicer_joints::settings::InterfaceOverride;
use openslicer_joints::settings::{ConnectorKind, ConnectorSettings};
use openslicer_project::assembly::AssemblyStep;
use openslicer_project::export::ExportFormat;
use openslicer_project::labels::{LabelMode, LabelOptions};
use openslicer_project::pipeline::SplitOutcome;
use openslicer_project::profile::PrinterProfile as ProjectPrinter;
use openslicer_project::settings::{Layout, SplitSettings};
use openslicer_split::autotile::OrientationPolicy;
use std::sync::Arc;

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum LayoutChoice {
    Auto,
    Grid,
    Modular,
    /// Planes the user places and moves, with a live preview in the viewport.
    Manual,
}

pub struct SplitPanel {
    pub open: bool,
    pub layout: LayoutChoice,
    pub grid: [usize; 3],
    pub spacing: [f64; 3],
    pub spacing_on: [bool; 3],
    pub search_yaw: bool,
    pub allow_standing: bool,
    pub connector: Option<ConnectorKind>,
    pub conn_size: f64,
    pub conn_height: f64,
    pub clearance: f64,
    pub repair: bool,
    pub nest: bool,
    pub verify_assembly: bool,
    pub avoid_slivers: bool,
    /// Joins whose connector was chosen by hand (keyed by piece ids of the current cut).
    pub overrides: Vec<InterfaceOverride>,
    pub label_mode: LabelMode,
    pub export_format: ExportFormat,
    pub label_height: f64,
    pub label_depth: f64,
    /// Manual cutting planes (world coordinates).
    pub planes: Vec<CutPlane>,
    pub active_plane: usize,
    pub show_planes: bool,
    /// True while the next click on the model should place a plane on the face that was clicked.
    pub pick_face: bool,
    /// Assembly sequence of the latest result, and the playback position (0 = all parts shown).
    pub steps: Vec<AssemblyStep>,
    pub step: usize,
    pub playing: bool,
    pub last_tick: Option<std::time::Instant>,
    /// Latest finished split.
    pub result: Option<Arc<SplitOutcome>>,
    /// Name of the model that was split.
    pub source_name: String,
    pub explode: f32,
}

impl Default for SplitPanel {
    fn default() -> Self {
        Self {
            open: false,
            layout: LayoutChoice::Auto,
            grid: [2, 2, 1],
            spacing: [150.0, 150.0, 150.0],
            spacing_on: [true, true, false],
            search_yaw: true,
            allow_standing: false,
            connector: Some(ConnectorKind::Pin),
            conn_size: 6.0,
            conn_height: 8.0,
            clearance: 0.2,
            repair: true,
            nest: false,
            verify_assembly: false,
            avoid_slivers: true,
            overrides: Vec::new(),
            label_mode: LabelMode::Off,
            export_format: ExportFormat::Stl,
            label_height: 8.0,
            label_depth: 0.6,
            planes: Vec::new(),
            active_plane: 0,
            show_planes: true,
            pick_face: false,
            steps: Vec::new(),
            step: 0,
            playing: false,
            last_tick: None,
            result: None,
            source_name: String::new(),
            explode: 0.0,
        }
    }
}

pub enum Action {
    None,
    PlaybackChanged,
    Run,
    Export,
    Clear,
    ExplodeChanged,
}

/// The engine's printer profile for the GUI's simple one.
pub fn project_printer(p: &PrinterProfile, clearance: f64) -> ProjectPrinter {
    ProjectPrinter {
        name: p.name.clone(),
        build_volume: p.build_volume.map(f64::from),
        margin: f64::from(p.margin),
        joint_clearance: clearance,
        ..Default::default()
    }
}

impl SplitPanel {
    pub fn settings(&self) -> Result<SplitSettings, String> {
        let layout = match self.layout {
            LayoutChoice::Auto => Layout::Auto { search_yaw: self.search_yaw, yaw_step_deg: 15.0 },
            LayoutChoice::Grid => Layout::Grid { divisions: self.grid, yaw_deg: 0.0 },
            LayoutChoice::Manual => {
                if self.planes.is_empty() {
                    return Err("add at least one cutting plane".into());
                }
                Layout::Planes(self.planes.iter().map(|p| p.to_spec()).collect())
            }
            LayoutChoice::Modular => {
                if !self.spacing_on.iter().any(|&b| b) {
                    return Err("enable the cut spacing on at least one axis".into());
                }
                let spacing = std::array::from_fn(|i| self.spacing_on[i].then_some(self.spacing[i]));
                Layout::Modular { spacing, offset: [0.0; 3], yaw_deg: 0.0 }
            }
        };
        let connectors = match self.connector {
            None => None,
            Some(kind) => {
                let mut c = ConnectorSettings::new(kind);
                c.size = if kind == ConnectorKind::Comb { self.conn_size.max(10.0) } else { self.conn_size };
                c.height = self.conn_height;
                c.clearance = self.clearance;
                c.overrides = self.overrides.clone();
                c.validate().map_err(|e| format!("connector settings: {e}"))?;
                Some(c)
            }
        };
        Ok(SplitSettings {
            layout,
            orientation: if self.allow_standing { OrientationPolicy::Any } else { OrientationPolicy::KeepUpright },
            connectors,
            merge_pieces_below: None,
            repair_first: self.repair,
            nest: self.nest,
            verify_assembly: self.verify_assembly,
            avoid_slivers: self.avoid_slivers,
            labels: LabelOptions { mode: self.label_mode, height: self.label_height, depth: self.label_depth },
        })
    }

    fn planes_ui(&mut self, ui: &mut egui::Ui, bounds: Option<(Vec3, Vec3)>) {
        caption(
            ui,
            "Place planes yourself. Drag a plane in the viewport, or type its position. The outline shows where it cuts the model.",
        );
        let mut remove: Option<usize> = None;
        for i in 0..self.planes.len() {
            let range = bounds.map_or(-5000.0..=5000.0, |(lo, hi)| {
                let a = self.planes[i].axis;
                (f64::from(lo[a]) - 1.0)..=(f64::from(hi[a]) + 1.0)
            });
            ui.horizontal(|ui| {
                let active = self.active_plane == i;
                if ui.selectable_label(active, format!("Plane {}", i + 1)).clicked() {
                    self.active_plane = i;
                }
                let p = &mut self.planes[i];
                egui::ComboBox::from_id_salt(("plane-axis", i)).width(44.0).selected_text(["X", "Y", "Z"][p.axis % 3]).show_ui(ui, |ui| {
                    for (k, name) in ["X", "Y", "Z"].into_iter().enumerate() {
                        ui.selectable_value(&mut p.axis, k, name);
                    }
                });
                ui.add(egui::DragValue::new(&mut p.offset).range(range).speed(0.5).suffix(" mm"));
                ui.add(egui::DragValue::new(&mut p.tilt_deg).range(-MAX_TILT..=MAX_TILT).speed(0.5).suffix("°"))
                    .on_hover_text("Tilt towards the next axis (X→Y, Y→Z, Z→X)");
                ui.add(egui::DragValue::new(&mut p.tilt2_deg).range(-MAX_TILT..=MAX_TILT).speed(0.5).suffix("°"))
                    .on_hover_text("Second tilt, towards the remaining axis. With both tilts the plane can face almost any direction.");
                if ui.small_button("Remove").clicked() {
                    remove = Some(i);
                }
            });
        }
        if let Some(i) = remove {
            self.planes.remove(i);
            self.active_plane = self.active_plane.min(self.planes.len().saturating_sub(1));
        }
        ui.horizontal(|ui| {
            if ui.button("Add plane").clicked() {
                let axis = self.planes.len() % 3;
                let offset = bounds.map_or(0.0, |(lo, hi)| f64::from((lo[axis] + hi[axis]) * 0.5));
                self.planes.push(CutPlane::new(axis, offset));
                self.active_plane = self.planes.len() - 1;
            }
            ui.checkbox(&mut self.show_planes, "Show in viewport");
        });
        ui.horizontal(|ui| {
            let label = if self.pick_face { "Click a face on the model…" } else { "Plane from a face" };
            if ui
                .selectable_label(self.pick_face, label)
                .on_hover_text("Click a face of the model: a plane is placed on it, flush with the face.")
                .clicked()
            {
                self.pick_face = !self.pick_face;
            }
            if self.pick_face {
                caption(ui, "Esc cancels.");
            }
        });
    }

    /// Per-join connector choices: each seam can keep the automatic connector, stay plain, or use another type.
    fn joins_ui(&mut self, ui: &mut egui::Ui, r: &SplitOutcome) {
        if self.connector.is_none() || r.interfaces.is_empty() {
            return;
        }
        let name = |id: usize| r.pieces.iter().find(|p| p.id == id).map_or_else(|| id.to_string(), |p| p.name.clone());
        let title = if self.overrides.is_empty() { "Joins".to_owned() } else { format!("Joins ({} changed)", self.overrides.len()) };
        egui::CollapsingHeader::new(title).id_salt("split-joins").show(ui, |ui| {
            caption(ui, "Choose the connector for each seam, or leave it plain. Changes apply the next time you press Split, and are forgotten by Clear result.");
            egui::ScrollArea::vertical().max_height(150.0).id_salt("split-joins-scroll").show(ui, |ui| {
                for (n, iface) in r.interfaces.iter().enumerate() {
                    let existing = self.overrides.iter().position(|o| (o.a == iface.a && o.b == iface.b) || (o.a == iface.b && o.b == iface.a));
                    let current = existing.map(|k| self.overrides[k].choice);
                    let text = match current {
                        None => "Automatic",
                        Some(None) => "No connector",
                        Some(Some(k)) => k.label(),
                    };
                    let mut pick = current;
                    ui.horizontal(|ui| {
                        ui.label(format!("{} – {}", name(iface.a), name(iface.b)));
                        egui::ComboBox::from_id_salt(("join", n)).selected_text(text).width(150.0).show_ui(ui, |ui| {
                            ui.selectable_value(&mut pick, None, "Automatic");
                            ui.selectable_value(&mut pick, Some(None), "No connector");
                            for k in ConnectorKind::ALL {
                                ui.selectable_value(&mut pick, Some(Some(k)), k.label());
                            }
                        });
                    });
                    if pick != current {
                        if let Some(k) = existing {
                            self.overrides.remove(k);
                        }
                        if let Some(choice) = pick {
                            self.overrides.push(InterfaceOverride { a: iface.a, b: iface.b, choice });
                        }
                    }
                }
            });
            if !self.overrides.is_empty() && ui.small_button("Reset all joins to automatic").clicked() {
                self.overrides.clear();
            }
        });
    }

    /// Loads saved settings into the panel. Layouts the panel cannot edit (bricks, explicit planes) leave it unchanged.
    pub fn apply_settings(&mut self, s: &SplitSettings) {
        match &s.layout {
            Layout::Auto { search_yaw, .. } => {
                self.layout = LayoutChoice::Auto;
                self.search_yaw = *search_yaw;
            }
            Layout::Grid { divisions, .. } => {
                self.layout = LayoutChoice::Grid;
                self.grid = divisions.map(|d| d.clamp(1, 20));
            }
            Layout::Modular { spacing, .. } => {
                self.layout = LayoutChoice::Modular;
                for (i, axis) in spacing.iter().enumerate() {
                    self.spacing_on[i] = axis.is_some();
                    if let Some(v) = axis {
                        self.spacing[i] = *v;
                    }
                }
            }
            Layout::Planes(specs) => {
                let planes: Vec<CutPlane> = specs.iter().filter_map(CutPlane::from_spec).collect();
                if !planes.is_empty() {
                    self.layout = LayoutChoice::Manual;
                    self.planes = planes;
                    self.active_plane = 0;
                }
            }
            Layout::Brick { .. } => {}
        }
        self.allow_standing = s.orientation == OrientationPolicy::Any;
        match &s.connectors {
            None => self.connector = None,
            Some(c) => {
                self.connector = Some(c.kind);
                self.conn_size = c.size;
                self.conn_height = c.height;
                self.clearance = c.clearance;
                self.overrides = c.overrides.clone();
            }
        }
        self.avoid_slivers = s.avoid_slivers;
        self.repair = s.repair_first;
        self.nest = s.nest;
        self.verify_assembly = s.verify_assembly;
        self.label_mode = s.labels.mode;
        self.label_height = s.labels.height;
        self.label_depth = s.labels.depth;
    }

    /// `bounds` is the world-space box of the model being split (min, max), used for plane defaults and ranges.
    pub fn show(&mut self, ctx: &egui::Context, can_run: bool, busy: bool, bounds: Option<(Vec3, Vec3)>) -> Action {
        if !self.open {
            return Action::None;
        }
        let mut action = Action::None;
        if self.playing && !self.steps.is_empty() {
            let due = self.last_tick.is_none_or(|t| t.elapsed() >= std::time::Duration::from_millis(900));
            if due {
                self.last_tick = Some(std::time::Instant::now());
                if self.step >= self.steps.len() {
                    self.playing = false;
                } else {
                    self.step += 1;
                    action = Action::PlaybackChanged;
                }
            }
            ctx.request_repaint_after(std::time::Duration::from_millis(250));
        }
        let mut open = true;
        egui::Window::new("Split model")
            .open(&mut open)
            .collapsible(false)
            .resizable(false)
            .anchor(Align2::RIGHT_TOP, vec2(-16.0, 56.0))
            .show(ctx, |ui| {
                ui.set_width(340.0);
                let max_h = (ctx.content_rect().height() - 260.0).max(240.0);
                egui::ScrollArea::vertical().max_height(max_h).auto_shrink([false, true]).show(ui, |ui| {
                ui.label(RichText::new("Layout").color(theme::TEXT_SECONDARY));
                ui.horizontal(|ui| {
                    ui.selectable_value(&mut self.layout, LayoutChoice::Auto, "Automatic");
                    ui.selectable_value(&mut self.layout, LayoutChoice::Grid, "Grid");
                    ui.selectable_value(&mut self.layout, LayoutChoice::Modular, "Every N mm");
                    ui.selectable_value(&mut self.layout, LayoutChoice::Manual, "Manual planes");
                });
                match self.layout {
                    LayoutChoice::Auto => {
                        caption(ui, "Fewest pieces that all fit the printer, including connector allowance.");
                        ui.checkbox(&mut self.search_yaw, "Try rotating the grid");
                        ui.checkbox(&mut self.allow_standing, "Allow parts to stand on end");
                    }
                    LayoutChoice::Grid => {
                        egui::Grid::new("split-grid").num_columns(2).show(ui, |ui| {
                            for (i, axis) in ["X", "Y", "Z"].into_iter().enumerate() {
                                ui.label(format!("Cells along {axis}"));
                                ui.add(egui::DragValue::new(&mut self.grid[i]).range(1..=50));
                                ui.end_row();
                            }
                        });
                    }
                    LayoutChoice::Manual => self.planes_ui(ui, bounds),
                    LayoutChoice::Modular => {
                        egui::Grid::new("split-mod").num_columns(2).show(ui, |ui| {
                            for (i, axis) in ["X", "Y", "Z"].into_iter().enumerate() {
                                ui.checkbox(&mut self.spacing_on[i], format!("Cut along {axis} every"));
                                ui.add_enabled(
                                    self.spacing_on[i],
                                    egui::DragValue::new(&mut self.spacing[i]).range(5.0..=5000.0).speed(1.0).suffix(" mm"),
                                );
                                ui.end_row();
                            }
                        });
                    }
                }
                ui.add_space(8.0);
                ui.label(RichText::new("Connectors").color(theme::TEXT_SECONDARY));
                egui::ComboBox::from_id_salt("conn-kind")
                    .selected_text(self.connector.map_or("None (plain cut faces)", |k| k.label()))
                    .show_ui(ui, |ui| {
                        ui.selectable_value(&mut self.connector, None, "None (plain cut faces)");
                        for k in ConnectorKind::ALL {
                            ui.selectable_value(&mut self.connector, Some(k), k.label());
                        }
                    });
                if self.connector.is_some() {
                    egui::Grid::new("split-conn").num_columns(2).show(ui, |ui| {
                        ui.label("Size");
                        ui.add(egui::DragValue::new(&mut self.conn_size).range(2.0..=60.0).speed(0.1).suffix(" mm"));
                        ui.end_row();
                        ui.label("Height / depth");
                        ui.add(egui::DragValue::new(&mut self.conn_height).range(1.0..=60.0).speed(0.1).suffix(" mm"));
                        ui.end_row();
                        ui.label("Clearance");
                        ui.add(egui::DragValue::new(&mut self.clearance).range(0.0..=1.0).speed(0.01).suffix(" mm"));
                        ui.end_row();
                    });
                    caption(ui, "0.2 mm suits most FDM printers. Print one test joint first.");
                }
                ui.add_space(8.0);
                ui.checkbox(&mut self.repair, "Repair the model first");
                ui.checkbox(&mut self.nest, "Arrange parts on build plates");
                ui.checkbox(&mut self.verify_assembly, "Check the assembly for overlaps (slow)");
                ui.checkbox(&mut self.avoid_slivers, "Nudge automatic cuts away from tiny slivers")
                    .on_hover_text("Moves an automatic cut up to a few millimetres when that avoids tiny floating islands in its cross-section. Manual planes are never moved.");
                ui.add_space(4.0);
                ui.label(RichText::new("Part labels").color(theme::TEXT_SECONDARY));
                ui.horizontal(|ui| {
                    for mode in [LabelMode::Off, LabelMode::Inset, LabelMode::Outset] {
                        ui.selectable_value(&mut self.label_mode, mode, mode.label());
                    }
                });
                if self.label_mode != LabelMode::Off {
                    egui::Grid::new("label-grid").num_columns(2).spacing([12.0, 6.0]).show(ui, |ui| {
                        ui.label("Text height");
                        ui.add(egui::DragValue::new(&mut self.label_height).range(3.0..=40.0).speed(0.2).suffix(" mm"));
                        ui.end_row();
                        ui.label("Relief depth");
                        ui.add(egui::DragValue::new(&mut self.label_depth).range(0.2..=3.0).speed(0.05).suffix(" mm"));
                        ui.end_row();
                    });
                    caption(ui, "Part number and assembly step, for example P07-3 (part 7, third to assemble). Parts with no flat area for it are listed.");
                }
                if let Some(r) = self.result.clone() {
                    ui.separator();
                    self.results(ui, &r, busy, &mut action);
                }
                });
                // Always visible, below the scrolling settings.
                ui.add_space(6.0);
                ui.horizontal(|ui| {
                    if ui.add_enabled(can_run && !busy, egui::Button::new("Split")).on_disabled_hover_text("Select a model first").clicked()
                    {
                        action = Action::Run;
                    }
                    if self.result.is_some() && ui.add_enabled(!busy, egui::Button::new("Clear result")).clicked() {
                        action = Action::Clear;
                    }
                });
            });
        if !open {
            self.open = false;
        }
        action
    }

    fn results(&mut self, ui: &mut egui::Ui, r: &SplitOutcome, busy: bool, action: &mut Action) {
        let unfit = r.fits.iter().filter(|f| !f.fits).count();
        ui.label(RichText::new(format!("{}: {} parts from {} cuts", self.source_name, r.pieces.len(), r.cut_count)).strong());
        if !r.connectors.is_empty() {
            caption(ui, format!("{} connectors, {} loose parts (print these too)", r.connectors.len(), r.loose.len()));
        }
        if unfit == 0 {
            ui.colored_label(theme::OK, format!("All {} parts fit the printer", r.fits.len()));
        } else {
            ui.colored_label(theme::ERROR, format!("{unfit} part(s) do not fit the printer"));
        }
        if let Some(n) = &r.nest {
            caption(ui, format!("{} build plate(s), {:.0}% of plate area used", n.plates.len(), n.utilisation * 100.0));
        }
        if r.is_valid() {
            ui.colored_label(theme::OK, "Every check passed");
        } else {
            ui.colored_label(theme::ERROR, format!("{} problem(s):", r.problems.len()));
            egui::ScrollArea::vertical().max_height(110.0).id_salt("split-problems").show(ui, |ui| {
                for p in &r.problems {
                    ui.label(RichText::new(p).color(theme::ERROR).size(12.0));
                }
            });
        }
        if !r.notes.is_empty() {
            egui::CollapsingHeader::new("Notes").id_salt("split-notes").show(ui, |ui| {
                for n in &r.notes {
                    caption(ui, n.clone());
                }
            });
        }
        self.joins_ui(ui, r);
        if !self.steps.is_empty() {
            ui.add_space(4.0);
            ui.label(RichText::new("Assembly playback").color(theme::TEXT_SECONDARY));
            let n = self.steps.len();
            ui.horizontal(|ui| {
                let mut changed = false;
                if ui.add_enabled(self.step > 0, egui::Button::new("Back")).clicked() {
                    self.step -= 1;
                    self.playing = false;
                    changed = true;
                }
                if ui.add_enabled(self.step < n, egui::Button::new("Next")).clicked() {
                    self.step += 1;
                    self.playing = false;
                    changed = true;
                }
                let label = if self.playing { "Pause" } else { "Play" };
                if ui.button(label).clicked() {
                    self.playing = !self.playing;
                    if self.playing && self.step >= n {
                        self.step = 0;
                    }
                    self.last_tick = None;
                    changed = true;
                }
                if ui.button("All").clicked() {
                    self.step = 0;
                    self.playing = false;
                    changed = true;
                }
                if changed {
                    *action = Action::PlaybackChanged;
                }
            });
            if ui.add(egui::Slider::new(&mut self.step, 0..=n).show_value(false)).changed() {
                self.playing = false;
                *action = Action::PlaybackChanged;
            }
            let text = match self.steps.get(self.step.wrapping_sub(1)) {
                Some(s) => describe_step(s, n),
                None => format!("All {n} parts shown. Press Play or Next to build the assembly step by step."),
            };
            caption(ui, text);
        }
        ui.add_space(4.0);
        ui.horizontal(|ui| {
            ui.label("Exploded view");
            if ui.add(egui::Slider::new(&mut self.explode, 0.0..=1.0).show_value(false)).changed() {
                *action = Action::ExplodeChanged;
            }
        });
        ui.add_space(4.0);
        ui.horizontal(|ui| {
            ui.label("File format");
            for (f, name) in
                [(ExportFormat::Stl, "STL"), (ExportFormat::Obj, "OBJ"), (ExportFormat::ThreeMf, "3MF"), (ExportFormat::Step, "STEP")]
            {
                ui.selectable_value(&mut self.export_format, f, name);
            }
        });
        if ui.add_enabled(!busy, egui::Button::new("Export parts…")).clicked() {
            *action = Action::Export;
        }
        caption(ui, "Writes one file per part, assembly instructions (HTML and CSV) and manifest.json, then re-imports and verifies them. STEP is a faceted solid, not a parametric one.");
    }
}

/// One line describing an assembly step.
pub fn describe_step(s: &AssemblyStep, total: usize) -> String {
    let mut t = format!("Step {} of {total}: place {}", s.number, s.part_name);
    if !s.attach_to.is_empty() {
        t += &format!(" against {}", s.attach_to.iter().map(|(_, n, _)| n.as_str()).collect::<Vec<_>>().join(", "));
    }
    if !s.connectors.is_empty() {
        t += &format!("; joints {}", s.connectors.join(", "));
    }
    if s.needs_glue {
        t += " (glue the finger joint)";
    }
    if !s.loose.is_empty() {
        t += &format!("; insert {}", s.loose.join(", "));
    }
    if s.starts_subassembly {
        t += " (starts a new sub-assembly)";
    }
    t
}

/// World position of a piece's centre, used as the base for exploded offsets.
pub fn explode_offset(piece_centre: Vec3, assembly_centre: Vec3, factor: f32) -> Vec3 {
    (piece_centre - assembly_centre) * factor
}

#[cfg(test)]
mod tests {
    #[test]
    fn steps_are_described_in_one_line() {
        let step = AssemblyStep {
            number: 3,
            part_id: 7,
            part_name: "P07".into(),
            attach_to: vec![(2, "P02".into(), 100.0), (4, "P04".into(), 50.0)],
            connectors: vec!["C05".into()],
            loose: vec!["D01".into()],
            needs_glue: false,
            starts_subassembly: false,
        };
        assert_eq!(describe_step(&step, 8), "Step 3 of 8: place P07 against P02, P04; joints C05; insert D01");
        let glued = AssemblyStep { needs_glue: true, ..step };
        assert!(describe_step(&glued, 8).contains("glue the finger joint"));
    }

    /// Manual planes drive the real engine: two parallel planes and one tilted plane cut a block into the expected pieces.
    #[test]
    fn manual_planes_cut_the_model_where_the_user_put_them() {
        use openslicer_core::{control::Control, Mesh};
        let mut m = Mesh::cuboid(Vec3::new(300.0, 100.0, 50.0));
        m.translate(Vec3::new(0.0, 0.0, 25.0));
        let panel = SplitPanel {
            layout: LayoutChoice::Manual,
            planes: vec![CutPlane::new(0, -50.0), CutPlane::new(0, 70.0)],
            connector: None,
            ..Default::default()
        };
        let out = openslicer_project::pipeline::run_split(
            &m,
            &project_printer(&PrinterProfile::default(), 0.2),
            &panel.settings().unwrap(),
            &Control::none(),
        )
        .unwrap();
        assert_eq!(out.pieces.len(), 3, "{:?}", out.problems);
        let mut widths: Vec<f32> = out.pieces.iter().map(|p| p.bounds.size().x).collect();
        widths.sort_by(f32::total_cmp);
        assert!(widths.iter().zip([80.0, 100.0, 120.0]).all(|(w, e)| (w - e).abs() < 0.01), "{widths:?}");
        let sum: f64 = out.pieces.iter().map(|p| p.volume).sum();
        assert!((sum - out.source.signed_volume).abs() < 1.0, "pieces add up to the model");
    }

    #[test]
    fn manual_planes_become_engine_planes_and_back() {
        let mut panel = SplitPanel { layout: LayoutChoice::Manual, ..Default::default() };
        assert!(panel.settings().is_err(), "no planes yet");
        panel.planes = vec![CutPlane::new(0, 100.0), CutPlane { axis: 2, offset: 40.0, tilt_deg: 15.0, tilt2_deg: -22.0 }];
        let s = panel.settings().unwrap();
        assert!(matches!(&s.layout, Layout::Planes(p) if p.len() == 2));
        let mut other = SplitPanel::default();
        other.apply_settings(&s);
        assert_eq!(other.layout, LayoutChoice::Manual);
        assert_eq!(other.planes.len(), 2);
        assert!((other.planes[1].tilt_deg - 15.0).abs() < 1e-9 && other.planes[1].axis == 2);
        assert!((other.planes[1].tilt2_deg + 22.0).abs() < 1e-9);
    }

    #[test]
    fn settings_round_trip_through_the_panel() {
        let panel = SplitPanel {
            layout: LayoutChoice::Grid,
            grid: [3, 2, 1],
            connector: Some(ConnectorKind::Dovetail),
            conn_size: 9.0,
            clearance: 0.25,
            nest: true,
            ..Default::default()
        };
        let saved = panel.settings().unwrap();
        let mut other = SplitPanel::default();
        other.apply_settings(&saved);
        assert_eq!(other.settings().unwrap(), saved);
        assert_eq!(
            (other.layout, other.grid, other.connector, other.nest),
            (LayoutChoice::Grid, [3, 2, 1], Some(ConnectorKind::Dovetail), true)
        );
    }

    use super::*;

    #[test]
    fn default_panel_produces_valid_settings() {
        let s = SplitPanel::default().settings().unwrap();
        assert!(matches!(s.layout, Layout::Auto { .. }));
        assert_eq!(s.connectors.unwrap().kind, ConnectorKind::Pin);
    }

    #[test]
    fn grid_and_modular_layouts_map_through() {
        let p = SplitPanel { layout: LayoutChoice::Grid, grid: [3, 2, 1], ..Default::default() };
        assert_eq!(p.settings().unwrap().layout, Layout::Grid { divisions: [3, 2, 1], yaw_deg: 0.0 });
        let p = SplitPanel {
            layout: LayoutChoice::Modular,
            spacing: [100.0, 80.0, 60.0],
            spacing_on: [true, false, true],
            ..Default::default()
        };
        match p.settings().unwrap().layout {
            Layout::Modular { spacing, .. } => assert_eq!(spacing, [Some(100.0), None, Some(60.0)]),
            other => panic!("{other:?}"),
        }
        let none = SplitPanel { layout: LayoutChoice::Modular, spacing_on: [false; 3], ..Default::default() };
        assert!(none.settings().is_err());
    }

    #[test]
    fn printer_conversion_keeps_volume_margin_and_clearance() {
        let p = PrinterProfile { name: "X".into(), build_volume: [250.0, 210.0, 220.0], margin: 5.0 };
        let q = project_printer(&p, 0.3);
        assert_eq!(q.build_volume, [250.0, 210.0, 220.0]);
        assert_eq!(q.usable().x, 240.0);
        assert_eq!(q.joint_clearance, 0.3);
    }

    #[test]
    fn exploded_offsets_scale_with_distance_from_centre() {
        let o = explode_offset(Vec3::new(30.0, 0.0, 0.0), Vec3::ZERO, 0.5);
        assert_eq!(o, Vec3::new(15.0, 0.0, 0.0));
    }

    /// The same path the Split button takes: panel settings → engine → valid, fitting parts.
    #[test]
    fn panel_settings_drive_the_engine_end_to_end() {
        use openslicer_core::{control::Control, Mesh};
        let mut m = Mesh::cuboid(Vec3::new(400.0, 100.0, 50.0));
        m.translate(Vec3::new(0.0, 0.0, 25.0));
        let panel = SplitPanel::default();
        let out = openslicer_project::pipeline::run_split(
            &m,
            &project_printer(&PrinterProfile::default(), panel.clearance),
            &panel.settings().unwrap(),
            &Control::none(),
        )
        .unwrap();
        assert!(out.is_valid(), "{:?}", out.problems);
        assert!(out.pieces.len() >= 2 && out.fits.iter().all(|f| f.fits));
    }
}
