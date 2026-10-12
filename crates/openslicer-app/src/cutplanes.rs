//! Manual cutting planes: their geometry, conversion to engine settings, and the live preview
//! (a translucent plane and the outline where it meets the model).

use glam::{DVec3, Vec3};
use openslicer_core::Mesh;
use openslicer_project::settings::PlaneSpec;

/// A plane that passes through `offset` on `axis`, tilted towards the next axis by `tilt_deg` and then towards the
/// third axis by `tilt2_deg`, so any orientation within about 60° of an axis can be reached.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CutPlane {
    /// 0 = X, 1 = Y, 2 = Z.
    pub axis: usize,
    /// World coordinate (mm) where the plane crosses the axis.
    pub offset: f64,
    /// Tilt in degrees, from the axis towards the next one.
    pub tilt_deg: f64,
    /// Second tilt in degrees, towards the axis after the next one.
    pub tilt2_deg: f64,
}

pub const MAX_TILT: f64 = 60.0;

impl CutPlane {
    pub fn new(axis: usize, offset: f64) -> Self {
        Self { axis: axis % 3, offset, tilt_deg: 0.0, tilt2_deg: 0.0 }
    }

    pub fn normal(&self) -> DVec3 {
        let (a, c, e) = (self.axis % 3, (self.axis + 1) % 3, (self.axis + 2) % 3);
        let (t, u) = (self.tilt_deg.to_radians(), self.tilt2_deg.to_radians());
        let mut n = DVec3::ZERO;
        n[a] = t.cos() * u.cos();
        n[c] = t.sin();
        n[e] = t.cos() * u.sin();
        n
    }

    pub fn point(&self) -> DVec3 {
        let mut p = DVec3::ZERO;
        p[self.axis % 3] = self.offset;
        p
    }

    /// `n · x = distance` for points x on the plane.
    pub fn distance(&self) -> f64 {
        self.normal().dot(self.point())
    }

    pub fn to_spec(self) -> PlaneSpec {
        PlaneSpec { point: self.point().to_array(), normal: self.normal().to_array() }
    }

    /// Recovers a plane saved by [`CutPlane::to_spec`], using the axis the normal points along most. Only the plane's
    /// normal and the offset where it crosses that axis are kept, which describes the same plane.
    pub fn from_spec(s: &PlaneSpec) -> Option<Self> {
        let n = DVec3::from(s.normal).try_normalize()?;
        Self::through(n, DVec3::from(s.point))
    }

    /// The plane through `point` with the given normal (which need not be unit length).
    pub fn through(normal: DVec3, point: DVec3) -> Option<Self> {
        let n = normal.try_normalize()?;
        let axis = (0..3).max_by(|&i, &j| n[i].abs().total_cmp(&n[j].abs()))?;
        let (c, e) = ((axis + 1) % 3, (axis + 2) % 3);
        // A normal pointing against the axis describes the same plane as its negation.
        let n = if n[axis] < 0.0 { -n } else { n };
        let tilt = n[c].clamp(-1.0, 1.0).asin().to_degrees();
        let tilt2 = n[e].atan2(n[axis]).to_degrees();
        let offset = n.dot(point) / n[axis];
        Some(Self { axis, offset, tilt_deg: tilt.clamp(-MAX_TILT, MAX_TILT), tilt2_deg: tilt2.clamp(-MAX_TILT, MAX_TILT) })
    }
}

/// Line segments where the plane `n · x = d` cuts the surface of `mesh`.
pub fn section_segments(mesh: &Mesh, n: Vec3, d: f32) -> Vec<(Vec3, Vec3)> {
    let mut out = Vec::new();
    for t in &mesh.triangles {
        let p = t.map(|i| mesh.vertices[i as usize]);
        let s = p.map(|v| n.dot(v) - d);
        let below: Vec<usize> = (0..3).filter(|&i| s[i] < 0.0).collect();
        if below.is_empty() || below.len() == 3 {
            continue;
        }
        let cross = |a: usize, b: usize| p[a] + (p[b] - p[a]) * (s[a] / (s[a] - s[b]));
        // One vertex alone on its side: its two edges are the crossing ones.
        let lone = if below.len() == 1 { below[0] } else { (0..3).find(|i| !below.contains(i)).unwrap() };
        let (b, c) = ((lone + 1) % 3, (lone + 2) % 3);
        out.push((cross(lone, b), cross(lone, c)));
    }
    out
}

/// Corners of a square on the plane, centred on the point of the plane nearest `around` and `half` wide.
pub fn plane_quad(n: Vec3, d: f32, around: Vec3, half: f32) -> [Vec3; 4] {
    let centre = around - n * (n.dot(around) - d);
    let helper = if n.z.abs() < 0.9 { Vec3::Z } else { Vec3::X };
    let u = n.cross(helper).normalize();
    let v = n.cross(u);
    [centre - u * half - v * half, centre + u * half - v * half, centre + u * half + v * half, centre - u * half + v * half]
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn normals_are_unit_and_tilt_towards_the_next_axis() {
        for axis in 0..3 {
            for tilt in [-45.0, 0.0, 30.0] {
                let p = CutPlane { axis, offset: 10.0, tilt_deg: tilt, tilt2_deg: 17.0 };
                assert!((p.normal().length() - 1.0).abs() < 1e-12);
            }
        }
        let p = CutPlane { axis: 0, offset: 0.0, tilt_deg: 90.0, tilt2_deg: 0.0 };
        assert!((p.normal() - DVec3::Y).length() < 1e-12, "X tilts towards Y");
        let z = CutPlane { axis: 2, offset: 5.0, tilt_deg: 0.0, tilt2_deg: 0.0 };
        assert_eq!(z.normal(), DVec3::Z);
        assert_eq!(z.point(), DVec3::new(0.0, 0.0, 5.0));
    }

    #[test]
    fn planes_round_trip_through_engine_specs() {
        for p in [
            CutPlane::new(0, 12.5),
            CutPlane { axis: 1, offset: -30.0, tilt_deg: 20.0, tilt2_deg: 0.0 },
            CutPlane { axis: 2, offset: 80.0, tilt_deg: -35.0, tilt2_deg: 25.0 },
            CutPlane { axis: 0, offset: 5.0, tilt_deg: 10.0, tilt2_deg: -40.0 },
        ] {
            let back = CutPlane::from_spec(&p.to_spec()).unwrap();
            assert_eq!(back.axis, p.axis);
            assert!(
                (back.offset - p.offset).abs() < 1e-9
                    && (back.tilt_deg - p.tilt_deg).abs() < 1e-9
                    && (back.tilt2_deg - p.tilt2_deg).abs() < 1e-9,
                "{p:?} -> {back:?}"
            );
        }
        assert!(CutPlane::from_spec(&PlaneSpec { point: [0.0; 3], normal: [0.0; 3] }).is_none());
    }

    #[test]
    fn any_plane_can_be_recovered_from_a_clicked_face() {
        // A face normal pointing in an arbitrary direction, with the clicked point on the plane.
        let n = DVec3::new(0.3, -0.5, 0.8).normalize();
        let p = DVec3::new(12.0, 7.0, -3.0);
        let plane = CutPlane::through(n, p).unwrap();
        let (pn, pd) = (plane.normal(), plane.distance());
        assert!(pn.dot(n).abs() > 1.0 - 1e-9, "same orientation (up to sign): {pn} vs {n}");
        assert!((pn.dot(p) - pd).abs() < 1e-9, "the clicked point lies on the plane");
        // Flipped normals describe the same plane.
        let flipped = CutPlane::through(-n, p).unwrap();
        assert!((flipped.normal() - pn).length() < 1e-9 && (flipped.offset - plane.offset).abs() < 1e-9);
        assert!(CutPlane::through(DVec3::ZERO, p).is_none());
    }

    #[test]
    fn the_section_of_a_cube_is_a_closed_loop_of_the_right_length() {
        let cube = Mesh::cuboid(Vec3::new(40.0, 40.0, 40.0));
        let seg = section_segments(&cube, Vec3::Z, 5.0);
        let total: f32 = seg.iter().map(|(a, b)| (*a - *b).length()).sum();
        assert!((total - 160.0).abs() < 1e-3, "{total}");
        assert!(seg.iter().all(|(a, b)| (a.z - 5.0).abs() < 1e-4 && (b.z - 5.0).abs() < 1e-4));
        // A plane that misses the model has no outline.
        assert!(section_segments(&cube, Vec3::Z, 50.0).is_empty());
        // Every endpoint appears twice: the outline is closed.
        let mut ends: Vec<[i64; 3]> = seg
            .iter()
            .flat_map(|(a, b)| [*a, *b])
            .map(|v| [(v.x * 100.0).round() as i64, (v.y * 100.0).round() as i64, (v.z * 100.0).round() as i64])
            .collect();
        ends.sort();
        assert!(ends.chunks(2).all(|c| c.len() == 2 && c[0] == c[1]));
    }

    #[test]
    fn the_preview_quad_lies_on_the_plane() {
        let n = Vec3::new(1.0, 1.0, 0.0).normalize();
        let q = plane_quad(n, 30.0, Vec3::new(10.0, -5.0, 7.0), 100.0);
        for c in q {
            assert!((n.dot(c) - 30.0).abs() < 1e-3);
        }
        assert!(((q[1] - q[0]).length() - 200.0).abs() < 1e-3);
    }
}
