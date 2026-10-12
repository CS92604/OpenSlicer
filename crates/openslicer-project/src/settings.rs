//! Serializable description of what to do with a model.

use glam::DVec3;
use openslicer_geom::Plane;
use openslicer_joints::settings::ConnectorSettings;
use openslicer_split::autotile::OrientationPolicy;
use serde::{Deserialize, Serialize};

/// A plane given by a point and a normal.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct PlaneSpec {
    pub point: [f64; 3],
    pub normal: [f64; 3],
}

impl PlaneSpec {
    pub fn to_plane(&self) -> Option<Plane> {
        Plane::from_point_normal(DVec3::from(self.point), DVec3::from(self.normal))
    }
}

/// How the model is divided.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub enum Layout {
    /// Fewest pieces that all fit the printer (searches grid orientation and axis assignment).
    Auto { search_yaw: bool, yaw_step_deg: f64 },
    /// Equal division into `divisions` cells along each axis of a grid rotated by `yaw_deg`.
    Grid { divisions: [usize; 3], yaw_deg: f64 },
    /// Cuts every `spacing[i]` mm from `offset[i]` (None disables an axis).
    Modular { spacing: [Option<f64>; 3], offset: [f64; 3], yaw_deg: f64 },
    /// Staggered brick pattern.
    Brick { row_axis: usize, along_axis: usize, row_height: f64, brick_length: f64, stagger: f64 },
    /// Cuts along explicitly placed planes.
    Planes(Vec<PlaneSpec>),
}

impl Default for Layout {
    fn default() -> Self {
        Layout::Auto { search_yaw: true, yaw_step_deg: 15.0 }
    }
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct SplitSettings {
    pub layout: Layout,
    pub orientation: OrientationPolicy,
    /// `None` leaves the cut faces plain.
    pub connectors: Option<ConnectorSettings>,
    /// Merge pieces smaller than this volume (mm³) into neighbours when the result still fits.
    pub merge_pieces_below: Option<f64>,
    /// Repair the model first (weld, orientation, small holes). Problems that remain are reported.
    pub repair_first: bool,
    /// Arrange the parts on build plates after splitting.
    pub nest: bool,
    /// Run the (slow) interpenetration check on the finished assembly.
    pub verify_assembly: bool,
}

impl Default for SplitSettings {
    fn default() -> Self {
        Self {
            layout: Layout::default(),
            orientation: OrientationPolicy::KeepUpright,
            connectors: None,
            merge_pieces_below: None,
            repair_first: true,
            nest: false,
            verify_assembly: false,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use openslicer_joints::settings::ConnectorKind;

    #[test]
    fn settings_round_trip_through_json() {
        let s = SplitSettings {
            layout: Layout::Planes(vec![PlaneSpec { point: [0.0, 0.0, 50.0], normal: [0.0, 0.0, 1.0] }]),
            connectors: Some(ConnectorSettings::new(ConnectorKind::Dovetail)),
            merge_pieces_below: Some(500.0),
            nest: true,
            ..Default::default()
        };
        let back: SplitSettings = serde_json::from_str(&serde_json::to_string_pretty(&s).unwrap()).unwrap();
        assert_eq!(back, s);
        let grid = Layout::Grid { divisions: [3, 2, 1], yaw_deg: 15.0 };
        let back: Layout = serde_json::from_str(&serde_json::to_string(&grid).unwrap()).unwrap();
        assert_eq!(back, grid);
    }

    #[test]
    fn missing_fields_take_defaults() {
        let s: SplitSettings = serde_json::from_str(r#"{"nest": true}"#).unwrap();
        assert!(s.nest && s.repair_first);
        assert_eq!(s.layout, Layout::default());
    }

    #[test]
    fn plane_spec_rejects_zero_normals() {
        assert!(PlaneSpec { point: [0.0; 3], normal: [0.0; 3] }.to_plane().is_none());
        assert!(PlaneSpec { point: [1.0, 2.0, 3.0], normal: [0.0, 0.0, 2.0] }.to_plane().is_some());
    }
}
