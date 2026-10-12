//! Connector settings and per-kind defaults.

#[derive(Clone, Copy, Debug, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum ConnectorKind {
    /// Round pin on one piece, matching socket in the other.
    Pin,
    /// Separate dowel (loose part) seated in sockets in both pieces.
    Dowel,
    /// Sliding dovetail: tenon on one piece, open-ended groove in the other.
    Dovetail,
    /// Rectangular tab and slot.
    TabSlot,
    /// Keyed bulb-and-neck knob (jigsaw style).
    Puzzle,
    /// Ridge following the cut boundary on one piece, groove in the other (native-shaped).
    Lip,
    /// Half-lap: one half of the cut face is stepped up, the other recessed.
    LapJoint,
    /// Stepped (terraced) face: strips of increasing height.
    Terrace,
    /// Truncated square pyramid peg and socket (self-centring).
    Pyramid,
    /// Hook that is inserted and slid sideways to lock under a ledge.
    Claw,
}

impl ConnectorKind {
    pub const ALL: [ConnectorKind; 10] = [
        ConnectorKind::Pin,
        ConnectorKind::Dowel,
        ConnectorKind::Dovetail,
        ConnectorKind::TabSlot,
        ConnectorKind::Puzzle,
        ConnectorKind::Lip,
        ConnectorKind::LapJoint,
        ConnectorKind::Terrace,
        ConnectorKind::Pyramid,
        ConnectorKind::Claw,
    ];

    pub fn label(self) -> &'static str {
        match self {
            ConnectorKind::Pin => "Pin and socket",
            ConnectorKind::Dowel => "Dowel",
            ConnectorKind::Dovetail => "Dovetail",
            ConnectorKind::TabSlot => "Tab and slot",
            ConnectorKind::Puzzle => "Puzzle",
            ConnectorKind::Lip => "Lip (follows cut boundary)",
            ConnectorKind::LapJoint => "Lap joint",
            ConnectorKind::Terrace => "Terrace",
            ConnectorKind::Pyramid => "Pyramid",
            ConnectorKind::Claw => "Claw",
        }
    }

    /// Short lowercase name used on the command line and in files.
    pub fn slug(self) -> &'static str {
        match self {
            ConnectorKind::Pin => "pin",
            ConnectorKind::Dowel => "dowel",
            ConnectorKind::Dovetail => "dovetail",
            ConnectorKind::TabSlot => "tab",
            ConnectorKind::Puzzle => "puzzle",
            ConnectorKind::Lip => "lip",
            ConnectorKind::LapJoint => "lap",
            ConnectorKind::Terrace => "terrace",
            ConnectorKind::Pyramid => "pyramid",
            ConnectorKind::Claw => "claw",
        }
    }

    pub fn from_slug(s: &str) -> Option<Self> {
        Self::ALL.into_iter().find(|k| k.slug().eq_ignore_ascii_case(s))
    }

    /// Placed at discrete points (as opposed to spanning the whole interface).
    pub fn is_point_based(self) -> bool {
        !matches!(self, ConnectorKind::Lip | ConnectorKind::LapJoint | ConnectorKind::Terrace)
    }
}

#[derive(Clone, Copy, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub enum AngleMode {
    /// Aligned to the cut boundary (tabs along it, dovetail grooves opening towards the nearest edge).
    Auto,
    /// Fixed rotation about the interface normal, in degrees.
    Fixed(f64),
}

/// What to do when the interface is too thin for the requested connector size.
#[derive(Clone, Copy, Debug, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum ThinWallPolicy {
    /// Shrink the connector to fit (down to `min_feature`), or skip the interface with an explanation.
    ShrinkToFit,
    /// Never resize: report the interface as unconnectable.
    Skip,
}

#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(default)]
pub struct ConnectorSettings {
    pub kind: ConnectorKind,
    /// Main width/diameter in mm (pin diameter, tab width, dovetail width, puzzle head, pyramid base, claw width).
    pub size: f64,
    /// Protrusion of the male part / depth of the pockets, in mm (lap: step height; terrace: height per step; lip: ridge height).
    pub height: f64,
    /// Length along the main axis for tabs, dovetails (0 = automatic, twice the width).
    pub length: f64,
    /// Lateral gap between male and female parts, in mm. Depends on printer and material.
    pub clearance: f64,
    /// Extra pocket depth beyond the male part, in mm.
    pub clearance_z: f64,
    /// 45° chamfer on pin tips, in mm.
    pub chamfer: f64,
    /// Pyramid taper: top side = (1 - taper) * base side.
    pub taper: f64,
    /// Dovetail flank angle, in degrees.
    pub flank_angle: f64,
    /// Number of steps for terraces.
    pub steps: usize,
    /// Claw hook overhang and lock travel, in mm.
    pub overhang: f64,
    pub slide: f64,
    /// Lip ridge width and distance from the boundary (0 = automatic, centred in the wall).
    pub lip_width: f64,
    pub lip_inset: f64,
    /// Target spacing between connectors on large interfaces, in mm.
    pub spacing: f64,
    pub min_count: usize,
    pub max_count: usize,
    /// Minimum material kept around any pocket, in mm.
    pub min_wall: f64,
    /// Smallest feature the printer can reproduce, in mm.
    pub min_feature: f64,
    pub angle: AngleMode,
    /// Alternate which piece carries the male part from one connector to the next (tabs on both sides).
    pub alternate: bool,
    pub thin_wall: ThinWallPolicy,
    /// Circle resolution.
    pub segments: usize,
}

impl Default for ConnectorSettings {
    fn default() -> Self {
        Self::new(ConnectorKind::Pin)
    }
}

impl ConnectorSettings {
    pub fn new(kind: ConnectorKind) -> Self {
        let base = Self {
            kind,
            size: 6.0,
            height: 8.0,
            length: 0.0,
            clearance: 0.2,
            clearance_z: 0.4,
            chamfer: 0.6,
            taper: 0.4,
            flank_angle: 15.0,
            steps: 3,
            overhang: 2.0,
            slide: 3.0,
            lip_width: 0.0,
            lip_inset: 0.0,
            spacing: 60.0,
            min_count: 1,
            max_count: 12,
            min_wall: 1.2,
            min_feature: 1.5,
            angle: AngleMode::Auto,
            alternate: false,
            thin_wall: ThinWallPolicy::ShrinkToFit,
            segments: 32,
        };
        match kind {
            ConnectorKind::Pin => base,
            ConnectorKind::Dowel => Self { height: 8.0, chamfer: 0.5, ..base },
            ConnectorKind::Dovetail => Self { size: 10.0, height: 6.0, ..base },
            ConnectorKind::TabSlot => Self { size: 8.0, height: 6.0, ..base },
            ConnectorKind::Puzzle => Self { size: 10.0, height: 6.0, ..base },
            ConnectorKind::Lip => Self { height: 3.0, ..base },
            ConnectorKind::LapJoint => Self { height: 4.0, ..base },
            ConnectorKind::Terrace => Self { height: 2.0, ..base },
            ConnectorKind::Pyramid => Self { size: 10.0, height: 6.0, ..base },
            ConnectorKind::Claw => Self { size: 8.0, height: 8.0, ..base },
        }
    }

    /// Checks that the values describe a buildable connector.
    pub fn validate(&self) -> Result<(), String> {
        let positive =
            |v: f64, name: &str| if v.is_finite() && v > 0.0 { Ok(()) } else { Err(format!("{name} must be a positive number")) };
        positive(self.size, "size")?;
        positive(self.height, "height")?;
        positive(self.spacing, "spacing")?;
        positive(self.min_feature, "minimum feature size")?;
        for (v, n) in [
            (self.clearance, "clearance"),
            (self.clearance_z, "depth clearance"),
            (self.min_wall, "minimum wall"),
            (self.chamfer, "chamfer"),
        ] {
            if !v.is_finite() || v < 0.0 {
                return Err(format!("{n} must not be negative"));
            }
        }
        if self.size < self.min_feature {
            return Err(format!("size {:.2} mm is below the minimum feature size {:.2} mm", self.size, self.min_feature));
        }
        if self.chamfer * 2.0 >= self.size.min(self.height * 2.0) {
            return Err("the chamfer is too large for the connector".into());
        }
        if !(0.0..0.9).contains(&self.taper) {
            return Err("taper must be between 0 and 0.9".into());
        }
        if !(1.0..=40.0).contains(&self.flank_angle) {
            return Err("the dovetail flank angle must be between 1 and 40 degrees".into());
        }
        if self.max_count == 0 || self.min_count > self.max_count {
            return Err("connector counts are inconsistent".into());
        }
        if self.segments < 8 {
            return Err("at least 8 segments are needed for round features".into());
        }
        if self.kind == ConnectorKind::Terrace && self.steps < 2 {
            return Err("a terrace needs at least 2 steps".into());
        }
        Ok(())
    }

    /// Radius of the smallest circle around the male footprint (centre at the placement point).
    pub fn footprint_radius(&self) -> f64 {
        let w = self.size;
        match self.kind {
            ConnectorKind::Pin | ConnectorKind::Dowel => w / 2.0,
            ConnectorKind::Pyramid => w * std::f64::consts::FRAC_1_SQRT_2,
            ConnectorKind::TabSlot | ConnectorKind::Dovetail => {
                let l = self.length_or_auto();
                0.5 * (l * l + w * w).sqrt()
            }
            ConnectorKind::Puzzle => w / 2.0 * 1.75,
            ConnectorKind::Claw => 0.5 * ((w + self.overhang * 2.0).powi(2) + w * w).sqrt() + self.slide,
            _ => 0.0,
        }
    }

    pub fn length_or_auto(&self) -> f64 {
        if self.length > 0.0 {
            self.length
        } else {
            self.size * 2.0
        }
    }
}

/// Typical joint clearances (lateral, in mm) for common printer/material combinations. Starting points only:
/// every printer differs, so print a test joint and adjust.
pub fn clearance_presets() -> &'static [(&'static str, f64)] {
    &[
        ("FDM, PLA, 0.4 mm nozzle (tight)", 0.15),
        ("FDM, PLA / PETG, 0.4 mm nozzle (standard)", 0.20),
        ("FDM, ABS / ASA (standard)", 0.25),
        ("FDM, loose / easy assembly", 0.30),
        ("Resin (SLA/MSLA)", 0.10),
    ]
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn defaults_are_valid_for_every_kind() {
        for k in ConnectorKind::ALL {
            let s = ConnectorSettings::new(k);
            assert_eq!(s.validate(), Ok(()), "{k:?}");
            assert!(!k.label().is_empty());
        }
    }

    #[test]
    fn invalid_values_are_rejected_with_a_message() {
        let mut s = ConnectorSettings::new(ConnectorKind::Pin);
        s.size = 0.5;
        assert!(s.validate().unwrap_err().contains("minimum feature"));
        let mut s = ConnectorSettings::new(ConnectorKind::Pin);
        s.clearance = -0.1;
        assert!(s.validate().is_err());
        let mut s = ConnectorSettings::new(ConnectorKind::Pin);
        s.chamfer = 5.0;
        assert!(s.validate().is_err());
        let mut s = ConnectorSettings::new(ConnectorKind::Terrace);
        s.steps = 1;
        assert!(s.validate().is_err());
        let mut s = ConnectorSettings::new(ConnectorKind::Pin);
        s.size = f64::NAN;
        assert!(s.validate().is_err());
    }

    #[test]
    fn point_based_kinds_have_a_footprint() {
        for k in ConnectorKind::ALL {
            let s = ConnectorSettings::new(k);
            assert_eq!(k.is_point_based(), s.footprint_radius() > 0.0, "{k:?}");
        }
    }
}
