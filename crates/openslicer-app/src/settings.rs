//! Persisted user preferences and printer profiles.

use glam::Vec3;
use serde::{Deserialize, Serialize};

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct PrinterProfile {
    pub name: String,
    /// Full build volume in mm (X, Y, Z).
    pub build_volume: [f32; 3],
    /// Unusable margin on each side of the plate (X and Y), in mm.
    pub margin: f32,
}

impl Default for PrinterProfile {
    fn default() -> Self {
        Self { name: "Generic 220 × 220 × 250".into(), build_volume: [220.0, 220.0, 250.0], margin: 0.0 }
    }
}

/// Result of checking a part's size against a printer profile.
#[derive(Clone, Debug, PartialEq)]
pub enum Fit {
    Fits,
    /// Per-axis overshoot in mm (0 for axes that fit).
    Exceeds(Vec3),
}

impl PrinterProfile {
    /// Usable volume after margins.
    pub fn usable(&self) -> Vec3 {
        let [x, y, z] = self.build_volume;
        Vec3::new((x - 2.0 * self.margin).max(0.0), (y - 2.0 * self.margin).max(0.0), z)
    }

    pub fn check_fit(&self, size: Vec3) -> Fit {
        const EPS: f32 = 1e-3;
        let over = (size - self.usable()).max(Vec3::ZERO);
        if over.max_element() > EPS {
            Fit::Exceeds(over)
        } else {
            Fit::Fits
        }
    }
}

#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(default)]
pub struct Preferences {
    pub printer: PrinterProfile,
    pub show_grid: bool,
    pub show_build_volume: bool,
    pub show_scene_panel: bool,
    pub show_properties_panel: bool,
    pub last_directory: Option<String>,
}

impl Default for Preferences {
    fn default() -> Self {
        Self {
            printer: PrinterProfile::default(),
            show_grid: true,
            show_build_volume: true,
            show_scene_panel: true,
            show_properties_panel: true,
            last_directory: None,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn fit_check_reports_overshoot_per_axis() {
        let p = PrinterProfile { margin: 10.0, ..Default::default() };
        assert_eq!(p.usable(), Vec3::new(200.0, 200.0, 250.0));
        assert_eq!(p.check_fit(Vec3::new(200.0, 100.0, 250.0)), Fit::Fits);
        assert_eq!(p.check_fit(Vec3::new(210.0, 100.0, 300.0)), Fit::Exceeds(Vec3::new(10.0, 0.0, 50.0)));
    }

    #[test]
    fn preferences_round_trip_through_ron() {
        let mut prefs = Preferences { show_grid: false, ..Preferences::default() };
        prefs.printer.margin = 7.5;
        let text = ron::to_string(&prefs).unwrap();
        let back: Preferences = ron::from_str(&text).unwrap();
        assert!(!back.show_grid);
        assert_eq!(back.printer.margin, 7.5);
    }

    #[test]
    fn preferences_from_older_versions_get_defaults_for_new_fields() {
        let back: Preferences = ron::from_str("(show_grid: false)").unwrap();
        assert!(!back.show_grid);
        assert!(back.show_build_volume);
        assert_eq!(back.printer, PrinterProfile::default());
    }
}
