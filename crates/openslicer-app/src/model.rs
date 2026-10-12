//! A model in the scene: shared immutable mesh plus an editable transform.

use glam::{Mat4, Vec3};
use openslicer_core::{Aabb, Mesh, MeshStats};
use std::path::PathBuf;
use std::sync::Arc;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Transform {
    /// World position of the model's pivot (its original bounding-box centre).
    pub position: Vec3,
    /// Euler angles in degrees, applied X, then Y, then Z.
    pub rotation_deg: Vec3,
    pub scale: Vec3,
}

impl Default for Transform {
    fn default() -> Self {
        Self { position: Vec3::ZERO, rotation_deg: Vec3::ZERO, scale: Vec3::ONE }
    }
}

impl Transform {
    pub fn rotation_matrix(&self) -> Mat4 {
        let r = self.rotation_deg * std::f32::consts::PI / 180.0;
        Mat4::from_rotation_z(r.z) * Mat4::from_rotation_y(r.y) * Mat4::from_rotation_x(r.x)
    }

    /// Local mesh space → world space, rotating and scaling about `pivot`.
    pub fn matrix(&self, pivot: Vec3) -> Mat4 {
        Mat4::from_translation(self.position) * self.rotation_matrix() * Mat4::from_scale(self.scale) * Mat4::from_translation(-pivot)
    }
}

pub struct Model {
    pub id: u64,
    pub name: String,
    pub mesh: Arc<Mesh>,
    pub stats: Arc<MeshStats>,
    pub source: Option<PathBuf>,
    pub visible: bool,
    pub transform: Transform,
    pivot: Vec3,
    bounds_cache: Option<(Transform, Aabb)>,
}

impl Model {
    /// Creates a model resting on the build plate, centred on the origin.
    /// Returns `None` for meshes with no geometry.
    pub fn new(id: u64, name: String, mesh: Arc<Mesh>, stats: Arc<MeshStats>, source: Option<PathBuf>) -> Option<Self> {
        let local_bounds = mesh.bounds()?;
        let pivot = local_bounds.center();
        let transform = Transform { position: Vec3::new(0.0, 0.0, local_bounds.size().z * 0.5), ..Default::default() };
        Some(Self { id, name, mesh, stats, source, visible: true, transform, pivot, bounds_cache: None })
    }

    pub fn matrix(&self) -> Mat4 {
        self.transform.matrix(self.pivot)
    }

    /// Exact world-space bounds, cached until the transform changes.
    pub fn world_bounds(&mut self) -> Aabb {
        if let Some((t, b)) = self.bounds_cache {
            if t == self.transform {
                return b;
            }
        }
        let m = self.matrix();
        let first = m.transform_point3(self.mesh.vertices[0]);
        let (min, max) = self.mesh.vertices.iter().fold((first, first), |(lo, hi), v| {
            let p = m.transform_point3(*v);
            (lo.min(p), hi.max(p))
        });
        let b = Aabb { min, max };
        self.bounds_cache = Some((self.transform, b));
        b
    }

    /// Lowers or raises the model so its lowest point rests on the build plate (Z = 0).
    pub fn drop_to_plate(&mut self) {
        let b = self.world_bounds();
        self.transform.position.z -= b.min.z;
    }

    /// Centres the model on the plate in X/Y.
    pub fn centre_on_plate(&mut self) {
        let b = self.world_bounds();
        let c = b.center();
        self.transform.position.x -= c.x;
        self.transform.position.y -= c.y;
    }

    /// Sets the world-space size along each axis by adjusting scale (relative to the current size).
    pub fn set_world_size(&mut self, size: Vec3) {
        let cur = self.world_bounds().size();
        for i in 0..3 {
            if cur[i] > 1e-9 && size[i] > 1e-6 {
                self.transform.scale[i] *= size[i] / cur[i];
            }
        }
    }

    /// The mesh with the transform baked in, as used for export. Winding is
    /// flipped when the transform mirrors the model so normals stay outward.
    pub fn transformed_mesh(&self) -> Mesh {
        let m = self.matrix();
        let vertices = self.mesh.vertices.iter().map(|v| m.transform_point3(*v)).collect();
        let mut triangles = self.mesh.triangles.clone();
        if m.determinant() < 0.0 {
            for t in &mut triangles {
                t.swap(1, 2);
            }
        }
        Mesh::new(vertices, triangles)
    }

    pub fn is_mirrored(&self) -> bool {
        self.matrix().determinant() < 0.0
    }

    /// Nearest hit distance along a world-space ray, if the model is hit.
    pub fn pick(&mut self, origin: Vec3, dir: Vec3) -> Option<f32> {
        // Cheap reject against the world AABB (slab test).
        let b = self.world_bounds();
        let inv = Vec3::ONE / dir;
        let (t1, t2) = ((b.min - origin) * inv, (b.max - origin) * inv);
        let (tmin, tmax) = (t1.min(t2).max_element(), t1.max(t2).min_element());
        if tmax < 0.0 || tmin > tmax {
            return None;
        }
        // Intersect in local space; the matrix is affine so `t` carries over unchanged.
        let inv_m = self.matrix().inverse();
        let o = inv_m.transform_point3(origin);
        let d = inv_m.transform_vector3(dir);
        self.mesh.ray_intersect(o, d)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn cube_model(size: Vec3) -> Model {
        let mesh = Arc::new(Mesh::cuboid(size));
        let stats = Arc::new(mesh.stats());
        Model::new(1, "cube".into(), mesh, stats, None).unwrap()
    }

    #[test]
    fn new_model_rests_on_plate_and_is_centred() {
        let mut m = cube_model(Vec3::new(10.0, 20.0, 30.0));
        let b = m.world_bounds();
        assert!((b.min.z).abs() < 1e-5);
        assert!((b.center().x).abs() < 1e-5 && (b.center().y).abs() < 1e-5);
        assert!((b.size() - Vec3::new(10.0, 20.0, 30.0)).length() < 1e-4);
    }

    #[test]
    fn rotation_by_90_degrees_swaps_extents() {
        let mut m = cube_model(Vec3::new(10.0, 20.0, 30.0));
        m.transform.rotation_deg = Vec3::new(0.0, 0.0, 90.0);
        let s = m.world_bounds().size();
        assert!((s - Vec3::new(20.0, 10.0, 30.0)).length() < 1e-3, "{s:?}");
        m.drop_to_plate();
        assert!(m.world_bounds().min.z.abs() < 1e-4);
    }

    #[test]
    fn set_world_size_scales_to_target() {
        let mut m = cube_model(Vec3::splat(10.0));
        m.set_world_size(Vec3::new(50.0, 10.0, 10.0));
        let s = m.world_bounds().size();
        assert!((s - Vec3::new(50.0, 10.0, 10.0)).length() < 1e-3);
    }

    #[test]
    fn exported_mesh_has_transform_baked_and_correct_volume() {
        let mut m = cube_model(Vec3::new(10.0, 20.0, 30.0));
        m.transform.scale = Vec3::new(2.0, 1.0, 1.0);
        m.transform.rotation_deg = Vec3::new(0.0, 0.0, 90.0);
        let out = m.transformed_mesh();
        let s = out.stats();
        assert!(s.is_watertight());
        assert!((s.signed_volume - 12000.0).abs() < 1.0, "{}", s.signed_volume);
        let b = out.bounds().unwrap().size();
        // Scale is applied in model space before rotation: 20 × 20 × 30 (rotate-then-scale would give 40 × 10 × 30).
        assert!((b - Vec3::new(20.0, 20.0, 30.0)).length() < 1e-3, "{b:?}");
    }

    #[test]
    fn mirrored_model_keeps_outward_winding_on_export() {
        let mut m = cube_model(Vec3::splat(10.0));
        m.transform.scale = Vec3::new(-1.0, 1.0, 1.0);
        assert!(m.is_mirrored());
        assert!(m.transformed_mesh().stats().signed_volume > 0.0);
    }

    #[test]
    fn picking_uses_transform() {
        let mut m = cube_model(Vec3::splat(10.0));
        m.transform.position = Vec3::new(100.0, 0.0, 5.0);
        let down = Vec3::new(0.0, 0.0, -1.0);
        assert!(m.pick(Vec3::new(100.0, 0.0, 50.0), down).is_some());
        assert!(m.pick(Vec3::new(0.0, 0.0, 50.0), down).is_none());
    }
}
