//! Grid occupancy: which cells of a regular grid contain material.
//!
//! A cell is occupied when the surface passes through it (exact triangle/box
//! overlap) or when its centre lies inside the solid (ray-crossing parity along
//! the first grid axis). Used to count the pieces a tiling would produce
//! without actually cutting the mesh.

use crate::layout::GridFrame;
use glam::DVec3;
use openslicer_core::Mesh;
use rayon::prelude::*;

pub struct Grid {
    pub origin: [f64; 3],
    pub cell: [f64; 3],
    pub n: [usize; 3],
}

impl Grid {
    pub fn index(&self, i: usize, j: usize, k: usize) -> usize {
        (i * self.n[1] + j) * self.n[2] + k
    }
    pub fn cell_count(&self) -> usize {
        self.n[0] * self.n[1] * self.n[2]
    }
}

/// Separating-axis triangle / axis-aligned box overlap (Akenine-Möller). `c` is the box centre, `h` its half extents.
pub fn tri_box_overlap(c: DVec3, h: DVec3, tri: [DVec3; 3]) -> bool {
    let v = [tri[0] - c, tri[1] - c, tri[2] - c];
    // Box axes.
    for axis in 0..3 {
        let (mn, mx) = (v[0][axis].min(v[1][axis]).min(v[2][axis]), v[0][axis].max(v[1][axis]).max(v[2][axis]));
        if mn > h[axis] || mx < -h[axis] {
            return false;
        }
    }
    // Triangle plane.
    let e = [v[1] - v[0], v[2] - v[1], v[0] - v[2]];
    let n = e[0].cross(e[1]);
    let r = h.x * n.x.abs() + h.y * n.y.abs() + h.z * n.z.abs();
    if n.dot(v[0]).abs() > r {
        return false;
    }
    // Nine edge cross-product axes.
    for ed in &e {
        for axis in [DVec3::X, DVec3::Y, DVec3::Z] {
            let a = axis.cross(*ed);
            if a.length_squared() < 1e-30 {
                continue;
            }
            let p = [v[0].dot(a), v[1].dot(a), v[2].dot(a)];
            let r = h.x * a.x.abs() + h.y * a.y.abs() + h.z * a.z.abs();
            if p[0].min(p[1]).min(p[2]) > r || p[0].max(p[1]).max(p[2]) < -r {
                return false;
            }
        }
    }
    true
}

/// Per-cell classification of a grid against a mesh.
pub struct Occupancy {
    /// The surface passes through the cell.
    pub surface: Vec<bool>,
    /// The cell centre lies inside the solid.
    pub centre_inside: Vec<bool>,
}

impl Occupancy {
    /// Cells that contain any material: touched by the surface or with the centre inside.
    pub fn any_material(&self) -> Vec<bool> {
        self.surface.iter().zip(&self.centre_inside).map(|(a, b)| *a || *b).collect()
    }
}

/// Occupancy of `grid` (cells in frame coordinates) for `mesh`. Returns one flag per cell, indexed by [`Grid::index`].
pub fn occupancy(mesh: &Mesh, frame: &GridFrame, grid: &Grid) -> Vec<bool> {
    occupancy_detail(mesh, frame, grid).any_material()
}

/// Like [`occupancy`], but keeps the surface and centre-inside classifications apart.
pub fn occupancy_detail(mesh: &Mesh, frame: &GridFrame, grid: &Grid) -> Occupancy {
    let axes = frame.axes();
    let to_grid = |p: DVec3| -> DVec3 {
        DVec3::new(p.dot(axes[0]) - grid.origin[0], p.dot(axes[1]) - grid.origin[1], p.dot(axes[2]) - grid.origin[2])
    };
    let q: Vec<DVec3> = mesh.vertices.par_iter().map(|v| to_grid(v.as_dvec3())).collect();
    let cell = DVec3::from(grid.cell);
    let n = grid.n;
    let ncell = grid.cell_count();
    let range = |lo: f64, hi: f64, axis: usize| -> (usize, usize) {
        let a = (lo / grid.cell[axis]).floor().max(0.0) as usize;
        let b = ((hi / grid.cell[axis]).floor().max(0.0) as usize).min(n[axis] - 1);
        (a.min(n[axis] - 1), b)
    };

    // Surface cells, in parallel chunks of triangles.
    let surface: Vec<bool> = mesh
        .triangles
        .par_chunks(4096)
        .map(|chunk| {
            let mut local = vec![false; ncell];
            for t in chunk {
                let tri = [q[t[0] as usize], q[t[1] as usize], q[t[2] as usize]];
                let (lo, hi) = (tri[0].min(tri[1]).min(tri[2]), tri[0].max(tri[1]).max(tri[2]));
                let (i0, i1) = range(lo.x, hi.x, 0);
                let (j0, j1) = range(lo.y, hi.y, 1);
                let (k0, k1) = range(lo.z, hi.z, 2);
                for i in i0..=i1 {
                    for j in j0..=j1 {
                        for k in k0..=k1 {
                            let idx = grid.index(i, j, k);
                            if local[idx] {
                                continue;
                            }
                            let centre = DVec3::new((i as f64 + 0.5) * cell.x, (j as f64 + 0.5) * cell.y, (k as f64 + 0.5) * cell.z);
                            if tri_box_overlap(centre, cell * 0.5, tri) {
                                local[idx] = true;
                            }
                        }
                    }
                }
            }
            local
        })
        .reduce(
            || vec![false; ncell],
            |mut a, b| {
                for (x, y) in a.iter_mut().zip(b) {
                    *x |= y;
                }
                a
            },
        );

    // Interior cells: ray parity along the first axis through each column's centre (slightly offset so
    // rays do not graze triangle edges of axis-aligned geometry).
    let mut hits: Vec<Vec<f64>> = vec![Vec::new(); n[1] * n[2]];
    let (off_y, off_z) = (cell.y * 1.318e-6, cell.z * 2.718e-6);
    for t in &mesh.triangles {
        let tri = [q[t[0] as usize], q[t[1] as usize], q[t[2] as usize]];
        let (lo, hi) = (tri[0].min(tri[1]).min(tri[2]), tri[0].max(tri[1]).max(tri[2]));
        let (j0, j1) = range(lo.y - cell.y, hi.y + cell.y, 1);
        let (k0, k1) = range(lo.z - cell.z, hi.z + cell.z, 2);
        // Plane x = (d - ny*y - nz*z) / nx
        let nrm = (tri[1] - tri[0]).cross(tri[2] - tri[0]);
        if nrm.x.abs() < 1e-18 {
            continue;
        }
        let d = nrm.dot(tri[0]);
        for j in j0..=j1 {
            let y = (j as f64 + 0.5) * cell.y + off_y;
            for k in k0..=k1 {
                let z = (k as f64 + 0.5) * cell.z + off_z;
                // Barycentric test in the (y, z) projection.
                let (a, b, c) = ((tri[0].y, tri[0].z), (tri[1].y, tri[1].z), (tri[2].y, tri[2].z));
                let det = (b.0 - a.0) * (c.1 - a.1) - (c.0 - a.0) * (b.1 - a.1);
                if det.abs() < 1e-18 {
                    continue;
                }
                let u = ((y - a.0) * (c.1 - a.1) - (c.0 - a.0) * (z - a.1)) / det;
                let v = ((b.0 - a.0) * (z - a.1) - (y - a.0) * (b.1 - a.1)) / det;
                if u >= 0.0 && v >= 0.0 && u + v < 1.0 {
                    hits[j * n[2] + k].push((d - nrm.y * y - nrm.z * z) / nrm.x);
                }
            }
        }
    }
    let mut inside = vec![false; ncell];
    for j in 0..n[1] {
        for k in 0..n[2] {
            let col = &mut hits[j * n[2] + k];
            col.sort_by(|a, b| a.total_cmp(b));
            let mut crossed = 0usize;
            for i in 0..n[0] {
                let xc = (i as f64 + 0.5) * cell.x;
                while crossed < col.len() && col[crossed] < xc {
                    crossed += 1;
                }
                if crossed % 2 == 1 {
                    inside[grid.index(i, j, k)] = true;
                }
            }
        }
    }
    Occupancy { surface, centre_inside: inside }
}

#[cfg(test)]
mod tests {
    use super::*;
    use glam::Vec3;

    fn grid_for(mesh: &Mesh, n: [usize; 3]) -> Grid {
        let b = mesh.bounds().unwrap();
        let size = b.size().as_dvec3();
        Grid { origin: b.min.as_dvec3().to_array(), cell: [size.x / n[0] as f64, size.y / n[1] as f64, size.z / n[2] as f64], n }
    }

    #[test]
    fn tri_box_overlap_cases() {
        let h = DVec3::splat(1.0);
        let tri = |a: [f64; 3], b: [f64; 3], c: [f64; 3]| [DVec3::from(a), DVec3::from(b), DVec3::from(c)];
        assert!(tri_box_overlap(DVec3::ZERO, h, tri([-0.5, -0.5, 0.0], [0.5, -0.5, 0.0], [0.0, 0.5, 0.0])));
        assert!(!tri_box_overlap(DVec3::ZERO, h, tri([2.0, 2.0, 2.0], [3.0, 2.0, 2.0], [2.0, 3.0, 2.0])));
        // large triangle slicing through the box
        assert!(tri_box_overlap(DVec3::ZERO, h, tri([-10.0, -10.0, 0.0], [10.0, -10.0, 0.0], [0.0, 10.0, 0.0])));
        // Near miss: every bounding-box axis overlaps, but the triangle's plane x + y >= 2.2 stays clear of the box (x + y <= 2).
        assert!(!tri_box_overlap(DVec3::ZERO, h, tri([2.2, 0.0, 0.0], [0.0, 2.2, 0.0], [2.2, 2.2, 3.0])));
        // The same triangle pulled in to x + y >= 1.6 does touch the box along its edge.
        assert!(tri_box_overlap(DVec3::ZERO, h, tri([1.6, 0.0, 0.0], [0.0, 1.6, 0.0], [1.6, 1.6, 3.0])));
    }

    #[test]
    fn solid_cube_fills_every_cell() {
        let m = Mesh::cuboid(Vec3::splat(30.0));
        let g = grid_for(&m, [3, 3, 3]);
        assert!(occupancy(&m, &GridFrame::WORLD, &g).iter().all(|&c| c));
    }

    #[test]
    fn hollow_shell_has_empty_core_cells() {
        // 5x5x5 grid over a thin sphere shell: the central cell is empty (cavity), outer corner cells are empty too.
        let m = crate::tests_shapes::sphere_shell(50.0, 48.0, 64, 32);
        let g = grid_for(&m, [5, 5, 5]);
        let occ = occupancy(&m, &GridFrame::WORLD, &g);
        assert!(!occ[g.index(2, 2, 2)], "centre of a hollow shell is empty");
        assert!(!occ[g.index(0, 0, 0)], "corner outside the sphere is empty");
        assert!(occ[g.index(2, 2, 0)], "bottom pole cell has material");
    }

    #[test]
    fn solid_sphere_has_a_filled_core_and_empty_corners() {
        let m = crate::tests_shapes::sphere(50.0, 48, 24);
        let g = grid_for(&m, [5, 5, 5]);
        let occ = occupancy(&m, &GridFrame::WORLD, &g);
        assert!(occ[g.index(2, 2, 2)]);
        assert!(!occ[g.index(0, 0, 0)]);
        assert!(!occ[g.index(4, 4, 4)]);
    }

    #[test]
    fn yawed_frame_changes_which_cells_are_used() {
        // A bar along the world diagonal: in a frame yawed 45 degrees it occupies one row of cells.
        let mut bar = Mesh::cuboid(Vec3::new(400.0, 40.0, 20.0));
        let rot = glam::Mat3::from_rotation_z(std::f32::consts::FRAC_PI_4);
        for v in &mut bar.vertices {
            *v = rot * *v;
        }
        let f = GridFrame { yaw: std::f64::consts::FRAC_PI_4 };
        let ext = crate::layout::extents(&bar, &f);
        let g = Grid {
            origin: [ext[0].0, ext[1].0, ext[2].0],
            cell: [(ext[0].1 - ext[0].0) / 4.0, (ext[1].1 - ext[1].0) / 2.0, ext[2].1 - ext[2].0],
            n: [4, 2, 1],
        };
        let occ = occupancy(&bar, &f, &g);
        assert!(occ.iter().filter(|&&c| c).count() >= 4 && occ.iter().filter(|&&c| c).count() <= 8);
        let g0 = grid_for(&bar, [4, 4, 1]);
        let occ0 = occupancy(&bar, &GridFrame::WORLD, &g0);
        assert!(occ0.iter().filter(|&&c| c).count() < 16, "a diagonal bar cannot fill the whole axis-aligned grid");
    }
}
