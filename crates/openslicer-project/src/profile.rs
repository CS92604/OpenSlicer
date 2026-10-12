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

    /// The 20 most widely used desktop FDM printers, as starting points.
    ///
    /// Build volumes are the manufacturers' published figures (X × Y × Z, mm), entered from memory
    /// and not re-checked against each datasheet. Several models share a volume across variants.
    /// Check yours in the Printer window, because firmware and bed clips change the usable area.
    pub fn presets() -> Vec<PrinterProfile> {
        PRESETS.iter().map(|&(name, v)| PrinterProfile { name: name.into(), build_volume: v, ..Default::default() }).collect()
    }
}

/// (name, build volume X × Y × Z in mm)
const PRESETS: [(&str, [f64; 3]); 20] = [
    ("Creality Ender-3 / V2 / V3 SE", [220.0, 220.0, 250.0]),
    ("Creality Ender-3 S1 / S1 Pro", [220.0, 220.0, 270.0]),
    ("Creality Ender-3 V3 KE", [220.0, 220.0, 240.0]),
    ("Creality K1 / K1C", [220.0, 220.0, 250.0]),
    ("Creality K1 Max", [300.0, 300.0, 300.0]),
    ("Creality CR-10 / CR-10 V3", [300.0, 300.0, 400.0]),
    ("Creality Ender-5 Plus", [350.0, 350.0, 400.0]),
    ("Prusa MK3S+", [250.0, 210.0, 210.0]),
    ("Prusa MK4 / MK4S", [250.0, 210.0, 220.0]),
    ("Prusa MINI+", [180.0, 180.0, 180.0]),
    ("Prusa XL (single tool)", [360.0, 360.0, 360.0]),
    ("Bambu Lab A1 mini", [180.0, 180.0, 180.0]),
    ("Bambu Lab A1", [256.0, 256.0, 256.0]),
    ("Bambu Lab P1P / P1S / X1C", [256.0, 256.0, 256.0]),
    ("Elegoo Neptune 4 / 4 Pro", [225.0, 225.0, 265.0]),
    ("Elegoo Neptune 4 Max", [420.0, 420.0, 480.0]),
    ("Anycubic Kobra 3", [250.0, 250.0, 260.0]),
    ("Sovol SV06", [220.0, 220.0, 250.0]),
    ("Artillery Sidewinder X2", [300.0, 300.0, 400.0]),
    ("Voron 2.4 (350 mm)", [350.0, 350.0, 330.0]),
];

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
        let presets = PrinterProfile::presets();
        assert_eq!(presets.len(), 20);
        assert!(presets.iter().all(|p| p.validate().is_ok()));
        let mut names: Vec<_> = presets.iter().map(|p| p.name.as_str()).collect();
        names.sort();
        names.dedup();
        assert_eq!(names.len(), 20, "preset names must be unique");
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
