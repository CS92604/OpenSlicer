//! The Split window: choose a layout and connectors, run the pipeline, review the outcome, export.

use crate::settings::PrinterProfile;
use crate::theme;
use crate::widgets::caption;
use eframe::egui::{self, vec2, Align2, RichText};
use glam::Vec3;
use openslicer_joints::settings::{ConnectorKind, ConnectorSettings};
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
            result: None,
            source_name: String::new(),
            explode: 0.0,
        }
    }
}

pub enum Action {
    None,
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
                c.size = self.conn_size;
                c.height = self.conn_height;
                c.clearance = self.clearance;
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
        })
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
            Layout::Brick { .. } | Layout::Planes(_) => {}
        }
        self.allow_standing = s.orientation == OrientationPolicy::Any;
        match &s.connectors {
            None => self.connector = None,
            Some(c) => {
                self.connector = Some(c.kind);
                self.conn_size = c.size;
                self.conn_height = c.height;
                self.clearance = c.clearance;
            }
        }
        self.repair = s.repair_first;
        self.nest = s.nest;
        self.verify_assembly = s.verify_assembly;
    }

    pub fn show(&mut self, ctx: &egui::Context, can_run: bool, busy: bool) -> Action {
        if !self.open {
            return Action::None;
        }
        let mut action = Action::None;
        let mut open = true;
        egui::Window::new("Split model")
            .open(&mut open)
            .collapsible(false)
            .resizable(false)
            .anchor(Align2::RIGHT_TOP, vec2(-16.0, 56.0))
            .show(ctx, |ui| {
                ui.set_width(340.0);
                ui.label(RichText::new("Layout").color(theme::TEXT_SECONDARY));
                ui.horizontal(|ui| {
                    ui.selectable_value(&mut self.layout, LayoutChoice::Auto, "Automatic");
                    ui.selectable_value(&mut self.layout, LayoutChoice::Grid, "Grid");
                    ui.selectable_value(&mut self.layout, LayoutChoice::Modular, "Every N mm");
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
                ui.add_space(8.0);
                ui.horizontal(|ui| {
                    if ui.add_enabled(can_run && !busy, egui::Button::new("Split")).on_disabled_hover_text("Select a model first").clicked()
                    {
                        action = Action::Run;
                    }
                    if self.result.is_some() && ui.add_enabled(!busy, egui::Button::new("Clear result")).clicked() {
                        action = Action::Clear;
                    }
                });
                if let Some(r) = self.result.clone() {
                    ui.separator();
                    self.results(ui, &r, busy, &mut action);
                }
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
        ui.add_space(4.0);
        ui.horizontal(|ui| {
            ui.label("Exploded view");
            if ui.add(egui::Slider::new(&mut self.explode, 0.0..=1.0).show_value(false)).changed() {
                *action = Action::ExplodeChanged;
            }
        });
        ui.add_space(4.0);
        if ui.add_enabled(!busy, egui::Button::new("Export parts…")).clicked() {
            *action = Action::Export;
        }
        caption(ui, "Writes STL files, assembly instructions and manifest.json, then re-imports and verifies them.");
    }
}

/// World position of a piece's centre, used as the base for exploded offsets.
pub fn explode_offset(piece_centre: Vec3, assembly_centre: Vec3, factor: f32) -> Vec3 {
    (piece_centre - assembly_centre) * factor
}

#[cfg(test)]
mod tests {
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
