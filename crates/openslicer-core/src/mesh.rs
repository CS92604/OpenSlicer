use glam::Vec3;
use std::collections::HashMap;

/// Axis-aligned bounding box.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Aabb {
    pub min: Vec3,
    pub max: Vec3,
}

impl Aabb {
    pub fn size(&self) -> Vec3 {
        self.max - self.min
    }
    pub fn center(&self) -> Vec3 {
        (self.min + self.max) * 0.5
    }
}

/// Indexed triangle mesh. Units are millimetres, +Z is up.
#[derive(Clone, Debug, Default)]
pub struct Mesh {
    pub vertices: Vec<Vec3>,
    pub triangles: Vec<[u32; 3]>,
}

/// Basic measurements and topology counts for a mesh.
#[derive(Clone, Debug, PartialEq)]
pub struct MeshStats {
    pub vertex_count: usize,
    pub triangle_count: usize,
    pub bounds: Option<Aabb>,
    /// Surface area in mm².
    pub surface_area: f64,
    /// Signed enclosed volume in mm³ (negative when the winding is inverted).
    pub signed_volume: f64,
    /// Edges used by exactly one triangle.
    pub boundary_edges: usize,
    /// Edges used by more than two triangles.
    pub non_manifold_edges: usize,
    /// Triangles with zero (or near-zero) area.
    pub degenerate_triangles: usize,
}

impl MeshStats {
    /// Closed surface whose triangles face inward (negative enclosed volume).
    pub fn is_inverted(&self) -> bool {
        self.boundary_edges == 0 && self.non_manifold_edges == 0 && self.signed_volume < 0.0
    }

    /// True when every edge is shared by exactly two triangles and no triangle is degenerate.
    pub fn is_watertight(&self) -> bool {
        self.triangle_count > 0 && self.boundary_edges == 0 && self.non_manifold_edges == 0 && self.degenerate_triangles == 0
    }
}

impl Mesh {
    pub fn new(vertices: Vec<Vec3>, triangles: Vec<[u32; 3]>) -> Self {
        Self { vertices, triangles }
    }

    /// Builds a mesh from an unindexed triangle soup, merging vertices that
    /// coincide within `tolerance` (mm). With `tolerance <= 0` only bit-identical
    /// vertices merge.
    ///
    /// Merging snaps candidate vertices to a grid of cell size `tolerance`; two
    /// vertices closer than the tolerance can occasionally land in different
    /// cells and stay separate, which shows up as an open edge in mesh
    /// validation rather than as silent damage. The first vertex seen in a
    /// cell is kept with its original coordinates, so geometry is never snapped.
    pub fn from_triangle_soup(tris: impl IntoIterator<Item = [Vec3; 3]>, tolerance: f32) -> Self {
        let mut map: HashMap<[i64; 3], u32> = HashMap::new();
        let mut mesh = Mesh::default();
        let cell = |x: f32| -> i64 {
            if tolerance > 0.0 {
                (x as f64 / tolerance as f64).round() as i64
            } else {
                // -0.0 and 0.0 must weld together.
                (x + 0.0).to_bits() as i64
            }
        };
        for tri in tris {
            let mut idx = [0u32; 3];
            for (k, v) in tri.iter().enumerate() {
                let key = [cell(v.x), cell(v.y), cell(v.z)];
                idx[k] = *map.entry(key).or_insert_with(|| {
                    mesh.vertices.push(*v);
                    (mesh.vertices.len() - 1) as u32
                });
            }
            mesh.triangles.push(idx);
        }
        mesh
    }

    pub fn is_empty(&self) -> bool {
        self.triangles.is_empty()
    }

    pub fn triangle(&self, i: usize) -> [Vec3; 3] {
        let t = self.triangles[i];
        [self.vertices[t[0] as usize], self.vertices[t[1] as usize], self.vertices[t[2] as usize]]
    }

    pub fn bounds(&self) -> Option<Aabb> {
        let first = *self.vertices.first()?;
        let (min, max) = self.vertices.iter().fold((first, first), |(lo, hi), v| (lo.min(*v), hi.max(*v)));
        Some(Aabb { min, max })
    }

    pub fn translate(&mut self, offset: Vec3) {
        for v in &mut self.vertices {
            *v += offset;
        }
    }

    /// Computes measurements and topology counts.
    pub fn stats(&self) -> MeshStats {
        let mut area = 0.0f64;
        let mut volume = 0.0f64;
        let mut degenerate = 0usize;
        let mut edges: HashMap<(u32, u32), u32> = HashMap::new();
        for t in &self.triangles {
            let [a, b, c] =
                [self.vertices[t[0] as usize].as_dvec3(), self.vertices[t[1] as usize].as_dvec3(), self.vertices[t[2] as usize].as_dvec3()];
            let cross = (b - a).cross(c - a);
            let tri_area = cross.length() * 0.5;
            area += tri_area;
            volume += a.dot(b.cross(c)) / 6.0;
            if tri_area < 1e-12 || t[0] == t[1] || t[1] == t[2] || t[0] == t[2] {
                degenerate += 1;
            }
            for (u, v) in [(t[0], t[1]), (t[1], t[2]), (t[2], t[0])] {
                *edges.entry((u.min(v), u.max(v))).or_insert(0) += 1;
            }
        }
        MeshStats {
            vertex_count: self.vertices.len(),
            triangle_count: self.triangles.len(),
            bounds: self.bounds(),
            surface_area: area,
            signed_volume: volume,
            boundary_edges: edges.values().filter(|&&n| n == 1).count(),
            non_manifold_edges: edges.values().filter(|&&n| n > 2).count(),
            degenerate_triangles: degenerate,
        }
    }

    /// Distance along the ray to the nearest triangle hit (Möller–Trumbore), if any.
    /// `dir` need not be normalised; the returned `t` is in units of `dir`.
    pub fn ray_intersect(&self, origin: Vec3, dir: Vec3) -> Option<f32> {
        let mut best: Option<f32> = None;
        for t in &self.triangles {
            let a = self.vertices[t[0] as usize];
            let e1 = self.vertices[t[1] as usize] - a;
            let e2 = self.vertices[t[2] as usize] - a;
            let p = dir.cross(e2);
            let det = e1.dot(p);
            if det.abs() < 1e-12 {
                continue;
            }
            let inv = 1.0 / det;
            let s = origin - a;
            let u = s.dot(p) * inv;
            if !(0.0..=1.0).contains(&u) {
                continue;
            }
            let q = s.cross(e1);
            let v = dir.dot(q) * inv;
            if v < 0.0 || u + v > 1.0 {
                continue;
            }
            let dist = e2.dot(q) * inv;
            if dist > 0.0 && best.is_none_or(|b| dist < b) {
                best = Some(dist);
            }
        }
        best
    }

    /// Axis-aligned box centred on the origin, mainly for tests and the sample model.
    pub fn cuboid(size: Vec3) -> Self {
        let h = size * 0.5;
        let v = |x: f32, y: f32, z: f32| Vec3::new(x * h.x, y * h.y, z * h.z);
        let vertices = vec![
            v(-1., -1., -1.),
            v(1., -1., -1.),
            v(1., 1., -1.),
            v(-1., 1., -1.),
            v(-1., -1., 1.),
            v(1., -1., 1.),
            v(1., 1., 1.),
            v(-1., 1., 1.),
        ];
        let triangles = vec![
            [0, 2, 1],
            [0, 3, 2], // bottom
            [4, 5, 6],
            [4, 6, 7], // top
            [0, 1, 5],
            [0, 5, 4], // -y
            [2, 3, 7],
            [2, 7, 6], // +y
            [1, 2, 6],
            [1, 6, 5], // +x
            [3, 0, 4],
            [3, 4, 7], // -x
        ];
        Self { vertices, triangles }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn cuboid_is_watertight_with_correct_measurements() {
        let m = Mesh::cuboid(Vec3::new(10.0, 20.0, 30.0));
        let s = m.stats();
        assert!(s.is_watertight(), "{s:?}");
        assert!((s.signed_volume - 6000.0).abs() < 1e-6);
        assert!((s.surface_area - 2.0 * (200.0 + 300.0 + 600.0)).abs() < 1e-6);
        assert_eq!(m.bounds().unwrap().size(), Vec3::new(10.0, 20.0, 30.0));
    }

    #[test]
    fn flipped_winding_is_detected_as_inverted() {
        let mut m = Mesh::cuboid(Vec3::splat(10.0));
        assert!(!m.stats().is_inverted());
        for t in &mut m.triangles {
            t.swap(1, 2);
        }
        let s = m.stats();
        assert!(s.is_inverted());
        assert!(s.signed_volume < 0.0);
    }

    #[test]
    fn missing_triangle_is_reported_as_open_boundary() {
        let mut m = Mesh::cuboid(Vec3::ONE);
        m.triangles.pop();
        let s = m.stats();
        assert!(!s.is_watertight());
        assert_eq!(s.boundary_edges, 3);
    }

    #[test]
    fn tolerance_welding_closes_float_noise_gaps_that_exact_welding_leaves() {
        let c = Mesh::cuboid(Vec3::splat(10.0));
        // Perturb every re-used corner by float noise, as CAD exporters do.
        let soup: Vec<_> = (0..c.triangles.len())
            .map(|i| {
                let mut t = c.triangle(i);
                for (k, v) in t.iter_mut().enumerate() {
                    *v += Vec3::splat(1e-6 * ((i * 3 + k) % 5) as f32);
                }
                t
            })
            .collect();
        let exact = Mesh::from_triangle_soup(soup.clone(), 0.0);
        assert!(!exact.stats().is_watertight());
        let welded = Mesh::from_triangle_soup(soup, 1e-3);
        assert_eq!(welded.vertices.len(), 8);
        assert!(welded.stats().is_watertight());
    }

    #[test]
    fn welding_keeps_original_coordinates_and_distinct_vertices() {
        let a = Vec3::new(0.0, 0.0, 0.0);
        let b = Vec3::new(1.0, 0.0, 0.0);
        let c = Vec3::new(0.0, 1.0, 0.0);
        let m = Mesh::from_triangle_soup([[a, b, c], [a + Vec3::splat(1e-7), b, Vec3::new(1.0, 1.0, 0.0)]], 1e-3);
        assert_eq!(m.vertices.len(), 4);
        assert_eq!(m.vertices[0], a, "first-seen vertex must keep its exact coordinates");
        // Features larger than the tolerance stay distinct.
        let m2 = Mesh::from_triangle_soup([[a, b, c], [a + Vec3::splat(0.01), b, c]], 1e-3);
        assert_eq!(m2.vertices.len(), 4);
    }

    #[test]
    fn ray_hits_nearest_face_and_misses_outside() {
        let m = Mesh::cuboid(Vec3::splat(10.0));
        let hit = m.ray_intersect(Vec3::new(0.0, 0.0, 50.0), Vec3::new(0.0, 0.0, -1.0)).unwrap();
        assert!((hit - 45.0).abs() < 1e-4);
        assert!(m.ray_intersect(Vec3::new(20.0, 0.0, 50.0), Vec3::new(0.0, 0.0, -1.0)).is_none());
        assert!(m.ray_intersect(Vec3::new(0.0, 0.0, 50.0), Vec3::new(0.0, 0.0, 1.0)).is_none());
    }

    #[test]
    fn soup_welding_merges_shared_corners() {
        let c = Mesh::cuboid(Vec3::ONE);
        let soup: Vec<_> = (0..c.triangles.len()).map(|i| c.triangle(i)).collect();
        let welded = Mesh::from_triangle_soup(soup, 0.0);
        assert_eq!(welded.vertices.len(), 8);
        assert!(welded.stats().is_watertight());
    }
}
