//! Orbit camera (Z-up, right-handed, OpenGL clip space).

use glam::{Mat4, Vec2, Vec3};
use openslicer_core::Aabb;

#[derive(Clone, Copy, Debug)]
pub struct Camera {
    pub target: Vec3,
    pub distance: f32,
    /// Rotation about +Z in radians (0 looks along +Y from -Y).
    pub yaw: f32,
    /// Elevation above the XY plane in radians.
    pub pitch: f32,
    pub fov_y: f32,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ViewPreset {
    Isometric,
    Top,
    Front,
    Right,
}

impl Default for Camera {
    fn default() -> Self {
        let mut c = Self { target: Vec3::new(0.0, 0.0, 40.0), distance: 450.0, yaw: 0.0, pitch: 0.0, fov_y: 45f32.to_radians() };
        c.set_preset(ViewPreset::Isometric);
        c
    }
}

impl Camera {
    const MAX_PITCH: f32 = 89.0 * std::f32::consts::PI / 180.0;

    pub fn eye(&self) -> Vec3 {
        let (sy, cy) = self.yaw.sin_cos();
        let (sp, cp) = self.pitch.sin_cos();
        // yaw = 0, pitch = 0 → eye on -Y looking toward +Y.
        self.target + self.distance * Vec3::new(sy * cp, -cy * cp, sp)
    }

    pub fn view(&self) -> Mat4 {
        Mat4::look_at_rh(self.eye(), self.target, Vec3::Z)
    }

    pub fn projection(&self, aspect: f32) -> Mat4 {
        let near = (self.distance * 0.01).max(0.05);
        let far = self.distance * 20.0 + 10_000.0;
        Mat4::perspective_rh_gl(self.fov_y, aspect.max(1e-3), near, far)
    }

    pub fn orbit(&mut self, delta_px: Vec2) {
        self.yaw -= delta_px.x * 0.008;
        self.pitch = (self.pitch + delta_px.y * 0.008).clamp(-Self::MAX_PITCH, Self::MAX_PITCH);
    }

    /// Pans so the scene follows the cursor; `viewport_height_px` scales pixels to world units.
    pub fn pan(&mut self, delta_px: Vec2, viewport_height_px: f32) {
        let view = self.view();
        let right = view.row(0).truncate();
        let up = view.row(1).truncate();
        let world_per_px = 2.0 * self.distance * (self.fov_y * 0.5).tan() / viewport_height_px.max(1.0);
        self.target += (-right * delta_px.x + up * delta_px.y) * world_per_px;
    }

    /// Zooms toward the target; positive `amount` zooms in.
    pub fn zoom(&mut self, amount: f32) {
        self.distance = (self.distance * (-amount * 0.0015).exp()).clamp(1.0, 1.0e6);
    }

    pub fn set_preset(&mut self, preset: ViewPreset) {
        let (yaw, pitch) = match preset {
            ViewPreset::Isometric => (-35f32.to_radians(), 30f32.to_radians()),
            ViewPreset::Top => (0.0, Self::MAX_PITCH),
            ViewPreset::Front => (0.0, 0.0),
            ViewPreset::Right => (90f32.to_radians(), 0.0),
        };
        self.yaw = yaw;
        self.pitch = pitch;
    }

    /// Frames a bounding box.
    pub fn fit(&mut self, b: Aabb) {
        let radius = (b.size().length() * 0.5).max(1.0);
        self.target = b.center();
        self.distance = radius / (self.fov_y * 0.5).sin() * 1.05;
    }

    /// World-space ray through a point given in normalised device coordinates (-1..1, +y up).
    pub fn ray(&self, ndc: Vec2, aspect: f32) -> (Vec3, Vec3) {
        let inv = (self.projection(aspect) * self.view()).inverse();
        let near = inv.project_point3(Vec3::new(ndc.x, ndc.y, -1.0));
        let far = inv.project_point3(Vec3::new(ndc.x, ndc.y, 1.0));
        (near, (far - near).normalize())
    }
}

/// Intersects a ray with the horizontal plane `z = height`.
pub fn ray_plane_z(origin: Vec3, dir: Vec3, height: f32) -> Option<Vec3> {
    if dir.z.abs() < 1e-6 {
        return None;
    }
    let t = (height - origin.z) / dir.z;
    (t > 0.0).then(|| origin + dir * t)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn front_view_looks_along_plus_y() {
        let mut c = Camera { target: Vec3::ZERO, ..Camera::default() };
        c.set_preset(ViewPreset::Front);
        let dir = (c.target - c.eye()).normalize();
        assert!((dir - Vec3::Y).length() < 1e-5, "{dir:?}");
    }

    #[test]
    fn centre_ray_passes_through_target() {
        let c = Camera::default();
        let (o, d) = c.ray(Vec2::ZERO, 1.5);
        let to_target = c.target - o;
        let off = to_target - d * to_target.dot(d);
        assert!(off.length() < 1e-2, "{off:?}");
    }

    #[test]
    fn fit_makes_box_visible() {
        let mut c = Camera::default();
        let b = Aabb { min: Vec3::new(-500.0, -200.0, 0.0), max: Vec3::new(500.0, 200.0, 300.0) };
        c.fit(b);
        let clip = c.projection(1.0) * c.view();
        for x in [b.min.x, b.max.x] {
            for y in [b.min.y, b.max.y] {
                for z in [b.min.z, b.max.z] {
                    let p = clip.project_point3(Vec3::new(x, y, z));
                    assert!(p.x.abs() <= 1.0 && p.y.abs() <= 1.0, "corner outside view: {p:?}");
                }
            }
        }
    }

    #[test]
    fn ray_plane_hits_floor() {
        let p = ray_plane_z(Vec3::new(1.0, 2.0, 10.0), Vec3::new(0.0, 0.0, -1.0), 0.0).unwrap();
        assert!((p - Vec3::new(1.0, 2.0, 0.0)).length() < 1e-6);
        assert!(ray_plane_z(Vec3::new(0.0, 0.0, 10.0), Vec3::new(0.0, 0.0, 1.0), 0.0).is_none());
    }

    #[test]
    fn pan_moves_target_opposite_to_drag() {
        let mut c = Camera::default();
        c.set_preset(ViewPreset::Front);
        c.target = Vec3::ZERO;
        c.pan(Vec2::new(100.0, 0.0), 800.0);
        assert!(c.target.x < 0.0);
    }
}
