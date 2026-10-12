//! Printer profiles: build volume, margins and manufacturing constraints.

use glam::DVec3;
use serde::{Deserialize, Serialize};

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct PrinterProfile {
    pub name: String,
    /// Full build volume in mm (X, Y, Z).
    pub build_volume: [f64; 3],
    /// Unusable margin on each side of the plate (X and Y), in mm.
    pub margin: f64,
    /// Smallest feature the printer can reproduce reliably (mm). Connectors are never smaller.
    pub min_feature: f64,
    /// Lateral clearance for joints that must fit together (mm).
    pub joint_clearance: f64,
    /// Free-text material note (for example "PLA").
    pub material: String,
    /// Allow parts to be printed standing on a side face when that is the only way they fit.
    pub allow_standing_parts: bool,
}

impl Default for PrinterProfile {
    fn default() -> Self {
        Self {
            name: "Generic 220 × 220 × 250".into(),
            build_volume: [220.0, 220.0, 250.0],
            margin: 0.0,
            min_feature: 1.5,
            joint_clearance: 0.2,
            material: "PLA".into(),
            allow_standing_parts: true,
        }
    }
}

/// Result of checking a size against a profile.
#[derive(Clone, Debug, PartialEq)]
pub enum Fit {
    Fits,
    /// Per-axis overshoot in mm (0 for axes that fit).
    Exceeds(DVec3),
}

impl PrinterProfile {
    /// Usable volume after margins.
    pub fn usable(&self) -> DVec3 {
        let [x, y, z] = self.build_volume;
        DVec3::new((x - 2.0 * self.margin).max(0.0), (y - 2.0 * self.margin).max(0.0), z)
    }

    pub fn check_fit(&self, size: DVec3) -> Fit {
        const EPS: f64 = 1e-3;
        let over = (size - self.usable()).max(DVec3::ZERO);
        if over.max_element() > EPS {
            Fit::Exceeds(over)
        } else {
            Fit::Fits
        }
    }

    pub fn validate(&self) -> Result<(), String> {
        if self.build_volume.iter().any(|v| !v.is_finite() || *v < 10.0) {
            return Err("every build volume dimension must be at least 10 mm".into());
        }
        if !self.margin.is_finite() || self.margin < 0.0 || 2.0 * self.margin >= self.build_volume[0].min(self.build_volume[1]) {
            return Err("the margin must be non-negative and smaller than half the plate".into());
        }
        if !(self.min_feature.is_finite() && self.min_feature > 0.0) || !(self.joint_clearance.is_finite() && self.joint_clearance >= 0.0) {
            return Err("the minimum feature must be positive and the joint clearance not negative".into());
        }
        Ok(())
    }

    /// Common printers as starting points.
    pub fn presets() -> Vec<PrinterProfile> {
        let mk = |name: &str, v: [f64; 3]| PrinterProfile { name: name.into(), build_volume: v, ..Default::default() };
        vec![
            mk("Creality Ender-3", [220.0, 220.0, 250.0]),
            mk("Prusa MK4", [250.0, 210.0, 220.0]),
            mk("Bambu Lab X1 / P1", [256.0, 256.0, 256.0]),
            mk("Voron 2.4 350", [350.0, 350.0, 330.0]),
            mk("Creality CR-10 S5", [500.0, 500.0, 500.0]),
        ]
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn usable_volume_and_fit() {
        let p = PrinterProfile { margin: 10.0, ..Default::default() };
        assert_eq!(p.usable(), DVec3::new(200.0, 200.0, 250.0));
        assert_eq!(p.check_fit(DVec3::new(200.0, 100.0, 250.0)), Fit::Fits);
        assert_eq!(p.check_fit(DVec3::new(210.0, 100.0, 300.0)), Fit::Exceeds(DVec3::new(10.0, 0.0, 50.0)));
    }

    #[test]
    fn validation_rejects_nonsense() {
        assert!(PrinterProfile::default().validate().is_ok());
        assert!(PrinterProfile { build_volume: [5.0, 200.0, 200.0], ..Default::default() }.validate().is_err());
        assert!(PrinterProfile { margin: 120.0, ..Default::default() }.validate().is_err());
        assert!(PrinterProfile { min_feature: 0.0, ..Default::default() }.validate().is_err());
        assert!(PrinterProfile { joint_clearance: f64::NAN, ..Default::default() }.validate().is_err());
        assert!(PrinterProfile::presets().iter().all(|p| p.validate().is_ok()));
    }

    #[test]
    fn profiles_survive_json_and_old_files_get_defaults() {
        let p = PrinterProfile { name: "Mine".into(), margin: 5.0, ..Default::default() };
        let back: PrinterProfile = serde_json::from_str(&serde_json::to_string(&p).unwrap()).unwrap();
        assert_eq!(back, p);
        let old: PrinterProfile = serde_json::from_str(r#"{"name":"Old","build_volume":[300,300,300]}"#).unwrap();
        assert_eq!(old.build_volume, [300.0; 3]);
        assert_eq!(old.joint_clearance, 0.2);
    }
}
