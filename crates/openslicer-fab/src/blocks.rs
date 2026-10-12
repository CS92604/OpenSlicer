//! Blocks: the model approximated by a stepped assembly of rectangular bricks.
//!
//! The model is sampled on a regular grid. Cells whose centre is inside the solid become material (or,
//! conservatively, any cell the surface touches). Neighbouring cells are merged greedily into larger bricks
//! (no larger than the machine allows), so far fewer parts than cells are produced. A hollow option keeps
//! only the cells the surface passes through, giving a one-brick-thick shell.

use crate::{ensure_fit, FabError, FabReport, FabResult, Part};
use glam::DVec3;
use openslicer_core::control::Control;
use openslicer_core::Mesh;
use openslicer_geom::build;
use openslicer_split::exec::Piece;
use openslicer_split::layout::{extents, GridFrame};
use openslicer_split::occupancy::{occupancy_detail, Grid};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum BlockFill {
    /// Cells whose centre is inside the solid: unbiased, the volume error averages out.
    Nearest,
    /// Every cell the solid touches: the blocks always cover the model (volume is over-estimated).
    Cover,
    /// Only cells the surface passes through: a hollow shell of blocks.
    Shell,
}

#[derive(Clone, Debug)]
pub struct BlockSettings {
    /// Cell size along each grid axis (mm).
    pub cell: DVec3,
    /// Rotation of the grid about Z (radians).
    pub yaw: f64,
    pub fill: BlockFill,
    pub usable: DVec3,
    /// Refuse grids with more cells than this.
    pub max_cells: usize,
}

impl BlockSettings {
    pub fn new(cell_size: f64, usable: DVec3) -> Self {
        Self { cell: DVec3::splat(cell_size), yaw: 0.0, fill: BlockFill::Nearest, usable, max_cells: 400_000 }
    }
}

pub fn blocks(mesh: &Mesh, s: &BlockSettings, ctl: &Control) -> Result<FabResult, FabError> {
    if mesh.is_empty() {
        return Err(FabError::Invalid("the mesh is empty".into()));
    }
    if !(s.cell.is_finite() && s.cell.min_element() > 0.0) {
        return Err(FabError::Invalid("the block size must be positive".into()));
    }
    let stats = mesh.stats();
    if !stats.is_watertight() && s.fill != BlockFill::Shell {
        return Err(FabError::Invalid("the mesh must be watertight to tell inside from outside; repair it or use the shell fill".into()));
    }
    let frame = GridFrame { yaw: s.yaw };
    let ext = extents(mesh, &frame);
    let n = [0, 1, 2].map(|i| (((ext[i].1 - ext[i].0) / s.cell[i]).ceil() as usize).max(1));
    let cells = n[0] * n[1] * n[2];
    if cells > s.max_cells {
        return Err(FabError::Invalid(format!("{cells} cells would be needed; use a larger block size")));
    }
    ctl.report(0.1, "Sampling the model on the grid");
    let grid = Grid { origin: [ext[0].0, ext[1].0, ext[2].0], cell: [s.cell.x, s.cell.y, s.cell.z], n };
    let occ = occupancy_detail(mesh, &frame, &grid);
    ctl.check()?;
    let material: Vec<bool> = match s.fill {
        BlockFill::Nearest => occ.centre_inside.clone(),
        BlockFill::Cover => occ.any_material(),
        BlockFill::Shell => occ.surface.clone(),
    };
    // Largest brick (in cells) the machine can print along each axis.
    let max_brick = [0, 1, 2].map(|i| ((s.usable[i] / s.cell[i]).floor() as usize).max(1));

    // Greedy merge into boxes.
    let mut used = vec![false; cells];
    let mut boxes: Vec<([usize; 3], [usize; 3])> = Vec::new(); // (min cell, size in cells)
    let free = |i: usize, j: usize, k: usize, used: &[bool]| {
        let idx = grid.index(i, j, k);
        material[idx] && !used[idx]
    };
    for i in 0..n[0] {
        for j in 0..n[1] {
            for k in 0..n[2] {
                if !free(i, j, k, &used) {
                    continue;
                }
                let mut size = [1usize, 1, 1];
                while size[0] < max_brick[0] && i + size[0] < n[0] && free(i + size[0], j, k, &used) {
                    size[0] += 1;
                }
                'y: while size[1] < max_brick[1] && j + size[1] < n[1] {
                    for di in 0..size[0] {
                        if !free(i + di, j + size[1], k, &used) {
                            break 'y;
                        }
                    }
                    size[1] += 1;
                }
                'z: while size[2] < max_brick[2] && k + size[2] < n[2] {
                    for di in 0..size[0] {
                        for dj in 0..size[1] {
                            if !free(i + di, j + dj, k + size[2], &used) {
                                break 'z;
                            }
                        }
                    }
                    size[2] += 1;
                }
                for di in 0..size[0] {
                    for dj in 0..size[1] {
                        for dk in 0..size[2] {
                            used[grid.index(i + di, j + dj, k + dk)] = true;
                        }
                    }
                }
                boxes.push(([i, j, k], size));
            }
        }
    }
    if boxes.is_empty() {
        return Err(FabError::Invalid("no cell contains material at this block size; use smaller blocks".into()));
    }

    let (sin, cos) = s.yaw.sin_cos();
    let origin = DVec3::from(grid.origin);
    let to_world = |p: DVec3| -> DVec3 {
        // Grid coordinates (along the yawed axes) back to world coordinates.
        let g = origin + p;
        DVec3::new(g.x * cos - g.y * sin, g.x * sin + g.y * cos, g.z)
    };
    let mut parts = Vec::with_capacity(boxes.len());
    for (idx, (min, size)) in boxes.iter().enumerate() {
        let lo = DVec3::new(min[0] as f64 * s.cell.x, min[1] as f64 * s.cell.y, min[2] as f64 * s.cell.z);
        let hi = lo + DVec3::new(size[0] as f64 * s.cell.x, size[1] as f64 * s.cell.y, size[2] as f64 * s.cell.z);
        let local = build::cuboid(lo, hi);
        let m = Mesh::new(local.vertices.iter().map(|v| to_world(v.as_dvec3()).as_vec3()).collect(), local.triangles);
        if let Some(piece) = Piece::new(m) {
            parts.push(Part { piece, label: format!("Block {} ({}×{}×{})", idx + 1, size[0], size[1], size[2]), outline: None });
        }
    }
    let mut report = FabReport { method: "Blocks".into(), source_volume: stats.signed_volume, ..Default::default() };
    report.max_deviation = Some(s.cell.length());
    report.notes.push(format!("{} cells merged into {} bricks", material.iter().filter(|&&m| m).count(), parts.len()));
    let parts = ensure_fit(parts, s.usable, &mut report, ctl)?;
    Ok(FabResult::finish(parts, report))
}

#[cfg(test)]
mod tests {
    use super::*;
    use glam::Vec3;

    fn block(size: Vec3) -> Mesh {
        let mut m = Mesh::cuboid(size);
        m.translate(size * 0.5);
        m
    }

    fn sphere(r: f32) -> Mesh {
        let (nu, nv) = (64usize, 32usize);
        let mut verts = vec![Vec3::new(0.0, 0.0, -r), Vec3::new(0.0, 0.0, r)];
        for j in 1..nv {
            let th = std::f32::consts::PI * j as f32 / nv as f32;
            for i in 0..nu {
                let ph = std::f32::consts::TAU * i as f32 / nu as f32;
                verts.push(Vec3::new(r * th.sin() * ph.cos(), r * th.sin() * ph.sin(), -r * th.cos()));
            }
        }
        let ring = |j: usize, i: usize| (2 + (j - 1) * nu + i % nu) as u32;
        let mut tris = Vec::new();
        for i in 0..nu {
            tris.push([0, ring(1, i + 1), ring(1, i)]);
            tris.push([1, ring(nv - 1, i), ring(nv - 1, i + 1)]);
            for j in 1..nv - 1 {
                let (a, b, c, d) = (ring(j, i), ring(j, i + 1), ring(j + 1, i + 1), ring(j + 1, i));
                tris.push([a, b, c]);
                tris.push([a, c, d]);
            }
        }
        Mesh::new(verts, tris)
    }

    #[test]
    fn an_aligned_box_becomes_one_brick_when_the_machine_allows() {
        let r = blocks(&block(Vec3::new(40.0, 30.0, 20.0)), &BlockSettings::new(5.0, DVec3::splat(250.0)), &Control::none()).unwrap();
        assert!(r.report.is_valid(), "{:?}", r.report);
        assert_eq!(r.parts.len(), 1);
        assert!((r.report.fill_ratio - 1.0).abs() < 1e-6);
    }

    #[test]
    fn brick_size_is_limited_by_the_machine() {
        let r = blocks(&block(Vec3::new(100.0, 20.0, 20.0)), &BlockSettings::new(10.0, DVec3::new(35.0, 250.0, 250.0)), &Control::none())
            .unwrap();
        assert!(r.report.is_valid(), "{:?}", r.report);
        // 10 cells along X, at most 3 per brick -> 4 bricks (3+3+3+1) in the single row
        assert_eq!(r.parts.len(), 4, "{:?}", r.parts.iter().map(|p| p.label.clone()).collect::<Vec<_>>());
        assert!(r.parts.iter().all(|p| p.piece.bounds.size().x <= 30.0 + 1e-3));
    }

    #[test]
    fn a_sphere_is_approximated_with_bounded_error_and_cover_is_a_superset() {
        let m = sphere(30.0);
        let nearest = blocks(&m, &BlockSettings::new(4.0, DVec3::splat(250.0)), &Control::none()).unwrap();
        assert!(nearest.report.is_valid(), "{:?}", nearest.report);
        assert!((nearest.report.fill_ratio - 1.0).abs() < 0.08, "{}", nearest.report.fill_ratio);
        assert!(nearest.parts.len() < 400, "merging must reduce the part count: {}", nearest.parts.len());
        let cover = blocks(&m, &BlockSettings { fill: BlockFill::Cover, ..BlockSettings::new(4.0, DVec3::splat(250.0)) }, &Control::none())
            .unwrap();
        assert!(cover.report.fill_ratio > nearest.report.fill_ratio);
        assert!(cover.report.fill_ratio > 1.0, "the cover fill always contains the model");
    }

    #[test]
    fn shell_fill_is_hollow() {
        let m = sphere(30.0);
        let solid = blocks(&m, &BlockSettings::new(4.0, DVec3::splat(250.0)), &Control::none()).unwrap();
        let shell = blocks(&m, &BlockSettings { fill: BlockFill::Shell, ..BlockSettings::new(4.0, DVec3::splat(250.0)) }, &Control::none())
            .unwrap();
        assert!(shell.report.is_valid(), "{:?}", shell.report);
        assert!(shell.report.parts_volume < solid.report.parts_volume * 0.6);
    }

    #[test]
    fn rotated_grid_produces_rotated_bricks() {
        let r = blocks(
            &block(Vec3::new(40.0, 40.0, 10.0)),
            &BlockSettings { yaw: 0.5, ..BlockSettings::new(10.0, DVec3::splat(250.0)) },
            &Control::none(),
        )
        .unwrap();
        assert!(r.report.is_valid(), "{:?}", r.report);
        assert!(!r.parts.is_empty());
        // Bricks are valid solids; a yawed grid over an axis-aligned block cannot be a single exact box.
        assert!(r.parts.iter().all(|p| p.piece.mesh.stats().is_watertight()));
    }

    #[test]
    fn invalid_input_is_rejected() {
        let m = block(Vec3::splat(20.0));
        assert!(blocks(&m, &BlockSettings::new(0.0, DVec3::splat(250.0)), &Control::none()).is_err());
        assert!(blocks(&m, &BlockSettings { max_cells: 4, ..BlockSettings::new(2.0, DVec3::splat(250.0)) }, &Control::none()).is_err());
        let mut open = m.clone();
        open.triangles.pop();
        assert!(blocks(&open, &BlockSettings::new(5.0, DVec3::splat(250.0)), &Control::none()).is_err());
        assert!(blocks(&Mesh::default(), &BlockSettings::new(5.0, DVec3::splat(250.0)), &Control::none()).is_err());
    }
}
