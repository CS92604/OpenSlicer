//! Fluent-style building blocks shared by the panels.

use crate::icons::{self, Icon};
use crate::theme;
use eframe::egui::{self, vec2, Align2, Color32, CornerRadius, FontId, Rect, Response, RichText, Sense, Stroke, StrokeKind, Ui};
use glam::Vec3;

/// A large icon-over-label command button, as on the 3D Builder command bar.
/// Returns true when clicked (never when disabled).
pub fn ribbon_button(ui: &mut Ui, icon: Icon, label: &str, enabled: bool, selected: bool, tooltip: &str) -> bool {
    let size = vec2(78.0, 64.0);
    let (rect, resp) = ui.allocate_exact_size(size, Sense::click());
    let hovered = resp.hovered() && enabled;
    let bg = if selected {
        theme::ACCENT_FILL
    } else if resp.is_pointer_button_down_on() && enabled {
        theme::CONTROL_PRESSED
    } else if hovered {
        theme::CONTROL_HOVER
    } else {
        Color32::TRANSPARENT
    };
    let painter = ui.painter();
    painter.rect_filled(rect, CornerRadius::same(6), bg);
    if selected {
        let bar = Rect::from_center_size(egui::pos2(rect.center().x, rect.bottom() - 3.0), vec2(20.0, 3.0));
        painter.rect_filled(bar, CornerRadius::same(2), theme::ACCENT);
    }
    let fg = if !enabled {
        theme::TEXT_DISABLED
    } else if selected {
        theme::ACCENT
    } else {
        theme::TEXT
    };
    let icon_rect = Rect::from_center_size(egui::pos2(rect.center().x, rect.top() + 23.0), vec2(30.0, 30.0));
    icons::paint(painter, icon_rect, icon, fg);
    painter.text(egui::pos2(rect.center().x, rect.bottom() - 14.0), Align2::CENTER_CENTER, label, FontId::proportional(12.5), fg);
    let resp = resp.on_hover_text(tooltip);
    enabled && resp.clicked()
}

pub fn section_header(ui: &mut Ui, text: &str) {
    ui.add_space(6.0);
    ui.label(RichText::new(text).size(13.0).strong().color(theme::TEXT));
    ui.add_space(2.0);
}

pub fn caption(ui: &mut Ui, text: impl Into<String>) {
    ui.label(RichText::new(text.into()).size(12.0).color(theme::TEXT_SECONDARY));
}

/// A rounded status pill.
pub fn chip(ui: &mut Ui, text: &str, color: Color32) {
    let galley = ui.painter().layout_no_wrap(text.to_owned(), FontId::proportional(12.0), color);
    let (rect, _) = ui.allocate_exact_size(galley.size() + vec2(16.0, 6.0), Sense::hover());
    ui.painter().rect(
        rect,
        CornerRadius::same(10),
        color.gamma_multiply(0.16),
        Stroke::new(1.0, color.gamma_multiply(0.5)),
        StrokeKind::Inside,
    );
    ui.painter().galley(rect.center() - galley.size() * 0.5, galley, color);
}

/// Three labelled numeric fields (X/Y/Z). Returns the index of the field that changed, if any.
pub fn vec3_row(ui: &mut Ui, label: &str, v: &mut Vec3, speed: f64, suffix: &str, decimals: usize, enabled: bool) -> Option<usize> {
    let mut changed = None;
    ui.horizontal(|ui| {
        ui.add_sized([58.0, theme::CONTROL_HEIGHT], egui::Label::new(RichText::new(label).color(theme::TEXT_SECONDARY)));
        let w = ((ui.available_width() - 2.0 * ui.spacing().item_spacing.x) / 3.0).max(40.0);
        for (i, axis) in ["X", "Y", "Z"].iter().enumerate() {
            let mut val = v[i] as f64;
            let drag = egui::DragValue::new(&mut val).speed(speed).prefix(format!("{axis}  ")).suffix(suffix).fixed_decimals(decimals);
            if ui.add_enabled_ui(enabled, |ui| ui.add_sized([w, theme::CONTROL_HEIGHT], drag)).inner.changed() {
                v[i] = val as f32;
                changed = Some(i);
            }
        }
    });
    changed
}

/// Selectable row used for the scene list. Returns (row response, eye clicked).
pub fn model_row(ui: &mut Ui, name: &str, detail: &str, detail_color: Color32, selected: bool, visible: bool) -> (Response, bool) {
    let (rect, resp) = ui.allocate_exact_size(vec2(ui.available_width(), 46.0), Sense::click());
    let bg = if selected {
        theme::ACCENT_FILL
    } else if resp.hovered() {
        theme::CONTROL_HOVER
    } else {
        Color32::TRANSPARENT
    };
    let painter = ui.painter();
    painter.rect_filled(rect, CornerRadius::same(6), bg);
    if selected {
        painter.rect_filled(
            Rect::from_min_size(rect.min + vec2(0.0, 10.0), vec2(3.0, rect.height() - 20.0)),
            CornerRadius::same(2),
            theme::ACCENT,
        );
    }
    icons::paint(
        painter,
        Rect::from_center_size(rect.left_center() + vec2(24.0, 0.0), vec2(26.0, 26.0)),
        Icon::Cube,
        theme::TEXT_SECONDARY,
    );
    let text_color = if visible { theme::TEXT } else { theme::TEXT_DISABLED };
    let eye_rect = Rect::from_center_size(rect.right_center() - vec2(20.0, 0.0), vec2(28.0, 28.0));
    let text_painter = painter.with_clip_rect(Rect::from_min_max(rect.min, egui::pos2(eye_rect.left() - 4.0, rect.max.y)));
    text_painter.text(rect.left_top() + vec2(46.0, 9.0), Align2::LEFT_TOP, name, FontId::proportional(14.0), text_color);
    text_painter.text(rect.left_top() + vec2(46.0, 27.0), Align2::LEFT_TOP, detail, FontId::proportional(11.5), detail_color);
    let eye = ui.interact(eye_rect, resp.id.with("eye"), Sense::click());
    if eye.hovered() {
        ui.painter().rect_filled(eye_rect, CornerRadius::same(4), theme::CONTROL);
    }
    let icon = if visible { Icon::Eye } else { Icon::EyeOff };
    icons::paint(ui.painter(), eye_rect.shrink(4.0), icon, theme::TEXT_SECONDARY);
    let eye = eye.on_hover_text(if visible { "Hide model" } else { "Show model" });
    (resp, eye.clicked())
}

/// Accent-filled primary button.
pub fn primary_button(ui: &mut Ui, text: &str) -> Response {
    ui.add(egui::Button::new(RichText::new(text).color(Color32::BLACK)).fill(theme::ACCENT).min_size(vec2(88.0, theme::CONTROL_HEIGHT)))
}
