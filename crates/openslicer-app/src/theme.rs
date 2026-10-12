//! Windows 11 dark ("Fluent") look for egui.

use eframe::egui::{self, Color32, CornerRadius, FontData, FontDefinitions, FontFamily, FontId, Margin, Stroke, TextStyle};

pub const ACCENT: Color32 = Color32::from_rgb(0x4C, 0xC2, 0xFF);
pub const ACCENT_DIM: Color32 = Color32::from_rgb(0x00, 0x67, 0xC0);
pub const ACCENT_FILL: Color32 = Color32::from_rgb(0x1C, 0x4A, 0x6E);
/// Window background ("Mica" base layer in dark mode).
pub const BG: Color32 = Color32::from_rgb(0x20, 0x20, 0x20);
pub const SURFACE: Color32 = Color32::from_rgb(0x2B, 0x2B, 0x2B);
pub const CONTROL: Color32 = Color32::from_rgb(0x38, 0x38, 0x38);
pub const CONTROL_HOVER: Color32 = Color32::from_rgb(0x3D, 0x3D, 0x3D);
pub const CONTROL_PRESSED: Color32 = Color32::from_rgb(0x30, 0x30, 0x30);
pub const STROKE: Color32 = Color32::from_rgb(0x3A, 0x3A, 0x3A);
pub const TEXT: Color32 = Color32::from_rgb(0xFF, 0xFF, 0xFF);
pub const TEXT_SECONDARY: Color32 = Color32::from_rgb(0xC5, 0xC5, 0xC5);
pub const TEXT_DISABLED: Color32 = Color32::from_rgb(0x78, 0x78, 0x78);
pub const OK: Color32 = Color32::from_rgb(0x6C, 0xCB, 0x5F);
pub const WARN: Color32 = Color32::from_rgb(0xFC, 0xE1, 0x00);
pub const ERROR: Color32 = Color32::from_rgb(0xFF, 0x99, 0xA4);

pub const CONTROL_HEIGHT: f32 = 32.0;

/// Installs fonts, visuals and spacing. Always dark; the app has no light mode.
pub fn apply(ctx: &egui::Context) {
    ctx.set_theme(egui::ThemePreference::Dark);
    install_fonts(ctx);

    let mut v = egui::Visuals::dark();
    v.panel_fill = BG;
    v.window_fill = SURFACE;
    v.extreme_bg_color = Color32::from_rgb(0x1A, 0x1A, 0x1A);
    v.faint_bg_color = Color32::from_rgb(0x27, 0x27, 0x27);
    v.window_stroke = Stroke::new(1.0, STROKE);
    v.window_corner_radius = CornerRadius::same(8);
    v.menu_corner_radius = CornerRadius::same(8);
    v.override_text_color = None;
    v.selection.bg_fill = ACCENT_FILL;
    v.selection.stroke = Stroke::new(1.0, ACCENT);
    v.hyperlink_color = ACCENT;
    v.warn_fg_color = WARN;
    v.error_fg_color = ERROR;

    let radius = CornerRadius::same(4);
    let w = &mut v.widgets;
    w.noninteractive.bg_fill = SURFACE;
    w.noninteractive.weak_bg_fill = SURFACE;
    w.noninteractive.bg_stroke = Stroke::new(1.0, STROKE);
    w.noninteractive.fg_stroke = Stroke::new(1.0, TEXT_SECONDARY);
    w.noninteractive.corner_radius = radius;
    w.inactive.bg_fill = CONTROL;
    w.inactive.weak_bg_fill = CONTROL;
    w.inactive.bg_stroke = Stroke::new(1.0, Color32::from_rgb(0x44, 0x44, 0x44));
    w.inactive.fg_stroke = Stroke::new(1.0, TEXT);
    w.inactive.corner_radius = radius;
    w.hovered.bg_fill = CONTROL_HOVER;
    w.hovered.weak_bg_fill = CONTROL_HOVER;
    w.hovered.bg_stroke = Stroke::new(1.0, Color32::from_rgb(0x50, 0x50, 0x50));
    w.hovered.fg_stroke = Stroke::new(1.0, TEXT);
    w.hovered.corner_radius = radius;
    w.active.bg_fill = CONTROL_PRESSED;
    w.active.weak_bg_fill = CONTROL_PRESSED;
    w.active.bg_stroke = Stroke::new(1.0, ACCENT_DIM);
    w.active.fg_stroke = Stroke::new(1.0, TEXT_SECONDARY);
    w.active.corner_radius = radius;
    w.open = w.active;
    ctx.set_visuals(v);

    ctx.global_style_mut(|style| {
        style.spacing.item_spacing = egui::vec2(8.0, 6.0);
        style.spacing.button_padding = egui::vec2(12.0, 6.0);
        style.spacing.interact_size.y = CONTROL_HEIGHT;
        style.spacing.window_margin = Margin::same(16);
        style.spacing.menu_margin = Margin::same(6);
        style.spacing.scroll.bar_width = 8.0;
        style.text_styles.insert(TextStyle::Heading, FontId::new(20.0, FontFamily::Proportional));
        style.text_styles.insert(TextStyle::Body, FontId::new(14.0, FontFamily::Proportional));
        style.text_styles.insert(TextStyle::Button, FontId::new(14.0, FontFamily::Proportional));
        style.text_styles.insert(TextStyle::Small, FontId::new(12.0, FontFamily::Proportional));
        style.text_styles.insert(TextStyle::Monospace, FontId::new(13.0, FontFamily::Monospace));
    });
}

/// Uses Segoe UI (the Windows 11 system font) when it is installed; otherwise egui's bundled font.
fn install_fonts(ctx: &egui::Context) {
    let windir = std::env::var_os("WINDIR").map(std::path::PathBuf::from);
    let Some(dir) = windir.map(|w| w.join("Fonts")) else { return };
    let Ok(bytes) = std::fs::read(dir.join("segoeui.ttf")) else { return };
    let mut fonts = FontDefinitions::default();
    fonts.font_data.insert("segoe-ui".into(), std::sync::Arc::new(FontData::from_owned(bytes)));
    fonts.families.entry(FontFamily::Proportional).or_default().insert(0, "segoe-ui".into());
    ctx.set_fonts(fonts);
}
