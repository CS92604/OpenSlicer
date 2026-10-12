//! Hollowing: remove the interior of a solid, leaving a wall of constant thickness.
//!
//! A signed-distance field is sampled on a voxel grid: inside/outside by ray parity, distances by a fast
//! Euclidean distance transform, refined with exact point-to-mesh distances in the narrow band around the
//! cavity surface. The cavity surface (where the distance to the outside equals the wall thickness) is extracted
//! with marching tetrahedra and added to the model as an inner shell. Optional drain holes connect the cavity to
//! the outside so resin or powder can escape.

use crate::boolean::{boolean, BoolOp};
use crate::build;
use crate::bvh::Bvh;
use glam::{DVec3, Vec3};
use openslicer_core::control::{Cancelled, Control};
use openslicer_core::Mesh;
use rayon::prelude::*;
use rustc_hash::FxHashMap;

#[derive(Clone, Debug)]
pub struct HollowOptions {
    /// Wall thickness in mm.
    pub wall: f64,
    /// Voxel size in mm. `None` uses `wall / 3` (limited by `max_voxels`).
    pub resolution: Option<f64>,
    /// Number of drain holes (placed at the lowest points of the cavity); 0 for a sealed cavity.
    pub drain_holes: usize,
    pub drain_diameter: f64,
    pub max_voxels: usize,
}

impl Default for HollowOptions {
    fn default() -> Self {
        Self { wall: 3.0, resolution: None, drain_holes: 0, drain_diameter: 4.0, max_voxels: 30_000_000 }
    }
}

#[derive(Clone, Debug, Default)]
pub struct HollowReport {
    pub voxel_size: f64,
    pub material_volume: f64,
    pub cavity_volume: f64,
    /// Smallest and mean wall thickness measured from cavity vertices to the outer surface.
    pub wall_min: f64,
    pub wall_mean: f64,
    pub drain_holes: usize,
    pub notes: Vec<String>,
}

#[derive(Debug, thiserror::Error)]
pub enum HollowError {
    #[error("{0}")]
    Invalid(String),
    #[error(transparent)]
    Cancelled(#[from] Cancelled),
}

#[derive(Debug)]
pub struct HollowResult {
    /// The outer surface plus the cavity as an inner shell (and drain holes, if requested).
    pub mesh: Mesh,
    pub report: HollowReport,
}

struct Field {
    n: [usize; 3],
}

impl Field {
    fn idx(&self, i: usize, j: usize, k: usize) -> usize {
        (i * self.n[1] + j) * self.n[2] + k
    }
}

/// "Infinite" squared distance for non-source nodes; large against any grid distance, small enough for exact f32.
const FAR: f32 = 1.0e7;

/// Squared Euclidean distance transform of one line `f` (0 at sources, `FAR` elsewhere), after Felzenszwalb &
/// Huttenlocher: the lower envelope of the parabolas rooted at each sample.
fn edt_line(f: &[f32], out: &mut [f32]) {
    let n = f.len();
    let mut v = vec![0usize; n];
    let mut z = vec![0.0f64; n + 1];
    let mut k = 0usize;
    z[0] = f64::NEG_INFINITY;
    z[1] = f64::INFINITY;
    for q in 1..n {
        let fq = f[q] as f64 + (q * q) as f64;
        let mut s;
        loop {
            let p = v[k];
            s = (fq - (f[p] as f64 + (p * p) as f64)) / (2.0 * (q as f64 - p as f64));
            if s <= z[k] && k > 0 {
                k -= 1;
            } else {
                break;
            }
        }
        // For k == 0 the new parabola always starts at s (z[0] is -inf, so `s <= z[0]` never holds).
        k += 1;
        v[k] = q;
        z[k] = s;
        z[k + 1] = f64::INFINITY;
    }
    let mut k = 0;
    for (q, o) in out.iter_mut().enumerate() {
        while z[k + 1] < q as f64 {
            k += 1;
        }
        let d = q as f64 - v[k] as f64;
        *o = (d * d + f[v[k]] as f64).min(FAR as f64) as f32;
    }
}

fn edt_3d(src: &mut [f32], n: [usize; 3]) {
    let fld = Field { n };
    // along k (contiguous)
    src.par_chunks_mut(n[2]).for_each(|line| {
        let mut out = vec![0.0; line.len()];
        edt_line(line, &mut out);
        line.copy_from_slice(&out);
    });
    // along j
    for i in 0..n[0] {
        for k in 0..n[2] {
            let line: Vec<f32> = (0..n[1]).map(|j| src[fld.idx(i, j, k)]).collect();
            let mut out = vec![0.0; n[1]];
            edt_line(&line, &mut out);
            for j in 0..n[1] {
                src[fld.idx(i, j, k)] = out[j];
            }
        }
    }
    // along i
    for j in 0..n[1] {
        for k in 0..n[2] {
            let line: Vec<f32> = (0..n[0]).map(|i| src[fld.idx(i, j, k)]).collect();
            let mut out = vec![0.0; n[0]];
            edt_line(&line, &mut out);
            for i in 0..n[0] {
                src[fld.idx(i, j, k)] = out[i];
            }
        }
    }
}

/// Freudenthal decomposition of a cube into six tetrahedra sharing the main diagonal (consistent across neighbours).
const TETS: [[usize; 4]; 6] = [[0, 1, 3, 7], [0, 1, 5, 7], [0, 2, 3, 7], [0, 2, 6, 7], [0, 4, 5, 7], [0, 4, 6, 7]];

pub fn hollow(mesh: &Mesh, opts: &HollowOptions, ctl: &Control) -> Result<HollowResult, HollowError> {
    let stats = mesh.stats();
    if !stats.is_watertight() || stats.signed_volume <= 0.0 {
        return Err(HollowError::Invalid("the mesh must be a closed solid with outward normals; repair it first".into()));
    }
    if !(opts.wall.is_finite() && opts.wall > 0.0) {
        return Err(HollowError::Invalid("the wall thickness must be positive".into()));
    }
    let b = mesh.bounds().unwrap();
    let size = b.size().as_dvec3();
    let mut h = opts.resolution.unwrap_or(opts.wall / 3.0);
    if !(h.is_finite() && h > 0.0) {
        return Err(HollowError::Invalid("the resolution must be positive".into()));
    }
    let count = |h: f64| ((size.x / h + 6.0) * (size.y / h + 6.0) * (size.z / h + 6.0)) as usize;
    if opts.resolution.is_none() {
        // Coarsen automatically if the default grid is too large, but never below wall/1.5.
        while count(h) > opts.max_voxels && h < opts.wall / 1.5 {
            h *= 1.15;
        }
    }
    if count(h) > opts.max_voxels {
        return Err(HollowError::Invalid(format!(
            "the grid would have {} million voxels (limit {}); use thicker walls or a coarser resolution",
            count(h) / 1_000_000,
            opts.max_voxels / 1_000_000
        )));
    }
    if size.min_element() < 2.0 * opts.wall + 2.0 * h {
        return Err(HollowError::Invalid(format!(
            "the model is only {:.1} mm thick at its thinnest extent; the wall ({:.1} mm) leaves no room for a cavity",
            size.min_element(),
            opts.wall
        )));
    }
    let origin = b.min.as_dvec3() - DVec3::splat(3.0 * h);
    let n = [0, 1, 2].map(|i| (size[i] / h).ceil() as usize + 7);
    let total = n[0] * n[1] * n[2];
    let fld = Field { n };
    ctl.report(0.05, "Classifying voxels");

    // Inside/outside by ray parity along x through each (y, z) column of nodes.
    let verts: Vec<DVec3> = mesh.vertices.iter().map(|v| v.as_dvec3() - origin).collect();
    let mut hits: Vec<Vec<f64>> = vec![Vec::new(); n[1] * n[2]];
    let (oy, oz) = (h * 1.318e-6, h * 2.718e-6);
    for t in &mesh.triangles {
        let (a, bb, c) = (verts[t[0] as usize], verts[t[1] as usize], verts[t[2] as usize]);
        let nrm = (bb - a).cross(c - a);
        if nrm.x.abs() < 1e-18 {
            continue;
        }
        let (lo, hi) = (a.min(bb).min(c), a.max(bb).max(c));
        let (j0, j1) = (((lo.y / h).floor().max(0.0)) as usize, ((hi.y / h).ceil() as usize).min(n[1] - 1));
        let (k0, k1) = (((lo.z / h).floor().max(0.0)) as usize, ((hi.z / h).ceil() as usize).min(n[2] - 1));
        let d = nrm.dot(a);
        let det = (bb.y - a.y) * (c.z - a.z) - (c.y - a.y) * (bb.z - a.z);
        if det.abs() < 1e-18 {
            continue;
        }
        for j in j0..=j1 {
            let y = j as f64 * h + oy;
            for k in k0..=k1 {
                let z = k as f64 * h + oz;
                let u = ((y - a.y) * (c.z - a.z) - (c.y - a.y) * (z - a.z)) / det;
                let v = ((bb.y - a.y) * (z - a.z) - (y - a.y) * (bb.z - a.z)) / det;
                if u >= 0.0 && v >= 0.0 && u + v < 1.0 {
                    hits[j * n[2] + k].push((d - nrm.y * y - nrm.z * z) / nrm.x);
                }
            }
        }
    }
    ctl.check()?;
    let mut inside = vec![false; total];
    for j in 0..n[1] {
        for k in 0..n[2] {
            let col = &mut hits[j * n[2] + k];
            col.sort_by(|a, b| a.total_cmp(b));
            let mut crossed = 0usize;
            for i in 0..n[0] {
                let x = i as f64 * h;
                while crossed < col.len() && col[crossed] < x {
                    crossed += 1;
                }
                inside[fld.idx(i, j, k)] = crossed % 2 == 1;
            }
        }
    }
    drop(hits);

    ctl.report(0.25, "Computing distances");
    // Squared distance (in voxels) from every node to the nearest outside node.
    let mut d2: Vec<f32> = inside.iter().map(|&i| if i { FAR } else { 0.0 }).collect();
    edt_3d(&mut d2, n);
    ctl.check()?;
    let bvh = Bvh::build(mesh);
    let wall = opts.wall;
    let band = 2.0 * h;
    // Signed field: negative inside. Estimated distance to the surface is (d_voxels - 0.5) * h.
    let field: Vec<f32> = (0..total)
        .into_par_iter()
        .map(|idx| {
            if !inside[idx] {
                return h as f32;
            }
            let est = (d2[idx].sqrt() as f64 - 0.5).max(0.0) * h;
            if (est - wall).abs() < band {
                let k = idx % n[2];
                let j = (idx / n[2]) % n[1];
                let i = idx / (n[2] * n[1]);
                let p = DVec3::new(i as f64 * h, j as f64 * h, k as f64 * h) + origin;
                -(bvh.distance_to(mesh, p, wall + 4.0 * h) as f32)
            } else {
                -(est as f32)
            }
        })
        .collect();
    drop(d2);
    ctl.check()?;

    ctl.report(0.6, "Extracting the cavity");
    // Nudge the iso-level off exact multiples of the grid spacing, where axis-aligned shapes would otherwise
    // put the surface exactly on grid nodes and collapse triangles.
    let iso = -((wall + 0.0137 * h) as f32);
    let mut cavity = Mesh::default();
    let mut edge_vertex: FxHashMap<(u32, u32), u32> = FxHashMap::default();
    let pos = |idx: usize| -> DVec3 {
        let k = idx % n[2];
        let j = (idx / n[2]) % n[1];
        let i = idx / (n[2] * n[1]);
        DVec3::new(i as f64 * h, j as f64 * h, k as f64 * h) + origin
    };
    for i in 0..n[0] - 1 {
        for j in 0..n[1] - 1 {
            for k in 0..n[2] - 1 {
                let corner = |c: usize| fld.idx(i + (c & 1), j + ((c >> 1) & 1), k + ((c >> 2) & 1));
                let ids: [usize; 8] = std::array::from_fn(corner);
                let vals: [f32; 8] = std::array::from_fn(|c| field[ids[c]]);
                if vals.iter().all(|&v| v >= iso) || vals.iter().all(|&v| v < iso) {
                    continue;
                }
                for tet in TETS {
                    let t = tet.map(|c| ids[c]);
                    let s = tet.map(|c| vals[c] - iso);
                    let ins: Vec<usize> = (0..4).filter(|&q| s[q] < 0.0).collect();
                    let outs: Vec<usize> = (0..4).filter(|&q| s[q] >= 0.0).collect();
                    if ins.is_empty() || outs.is_empty() {
                        continue;
                    }
                    let mut edge = |a: usize, c: usize| -> u32 {
                        let key = (t[a].min(t[c]) as u32, t[a].max(t[c]) as u32);
                        *edge_vertex.entry(key).or_insert_with(|| {
                            let (pa, pc) = (pos(t[a]), pos(t[c]));
                            let f = s[a] as f64 / (s[a] as f64 - s[c] as f64);
                            cavity.vertices.push((pa + (pc - pa) * f).as_vec3());
                            (cavity.vertices.len() - 1) as u32
                        })
                    };
                    let mut tris: Vec<[u32; 3]> = Vec::new();
                    // Orientation is decided from the tetrahedron's corners (never degenerate), so triangles that
                    // collapse to a point or line still get a consistent winding.
                    let cp: [DVec3; 4] = std::array::from_fn(|q| pos(t[q]));
                    let flip = match ins.len() {
                        1 => {
                            let a = ins[0];
                            tris.push([edge(a, outs[0]), edge(a, outs[1]), edge(a, outs[2])]);
                            // Normal of the triangle should point towards the inside corner `a`.
                            let n = (cp[outs[1]] - cp[outs[0]]).cross(cp[outs[2]] - cp[outs[0]]);
                            n.dot(cp[a] - cp[outs[0]]) < 0.0
                        }
                        3 => {
                            let a = outs[0];
                            tris.push([edge(a, ins[0]), edge(a, ins[1]), edge(a, ins[2])]);
                            // Towards the inside corners, i.e. away from the single outside corner.
                            let n = (cp[ins[1]] - cp[ins[0]]).cross(cp[ins[2]] - cp[ins[0]]);
                            n.dot(cp[a] - cp[ins[0]]) > 0.0
                        }
                        _ => {
                            let (a, bq, c, d) = (ins[0], ins[1], outs[0], outs[1]);
                            let (e_ac, e_ad, e_bd, e_bc) = (edge(a, c), edge(a, d), edge(bq, d), edge(bq, c));
                            tris.push([e_ac, e_ad, e_bd]);
                            tris.push([e_ac, e_bd, e_bc]);
                            let (m_ac, m_ad, m_bd) = ((cp[a] + cp[c]) * 0.5, (cp[a] + cp[d]) * 0.5, (cp[bq] + cp[d]) * 0.5);
                            let n = (m_ad - m_ac).cross(m_bd - m_ac);
                            n.dot((cp[a] + cp[bq]) * 0.5 - m_ac) < 0.0
                        }
                    };
                    for mut tri in tris {
                        if flip {
                            tri.swap(1, 2);
                        }
                        cavity.triangles.push(tri);
                    }
                }
            }
        }
        if i % 16 == 0 {
            ctl.check()?;
        }
    }
    if cavity.is_empty() {
        return Err(HollowError::Invalid("no part of the model is thick enough for a cavity with this wall thickness".into()));
    }
    let cs = cavity.stats();
    if !cs.is_watertight() {
        return Err(HollowError::Invalid(format!(
            "the cavity surface could not be closed ({} open edges); try a different resolution",
            cs.boundary_edges
        )));
    }
    if cs.signed_volume >= 0.0 {
        return Err(HollowError::Invalid("the cavity came out with the wrong orientation".into()));
    }
    let cavity_volume = -cs.signed_volume;

    // Measure the wall.
    let samples: Vec<f64> =
        cavity.vertices.iter().step_by((cavity.vertices.len() / 400).max(1)).map(|v| bvh.distance_to(mesh, v.as_dvec3(), 1e9)).collect();
    let wall_min = samples.iter().cloned().fold(f64::MAX, f64::min);
    let wall_mean = samples.iter().sum::<f64>() / samples.len() as f64;

    let mut combined = build::merge(&[mesh.clone(), cavity]);
    let mut report = HollowReport {
        voxel_size: h,
        material_volume: stats.signed_volume - cavity_volume,
        cavity_volume,
        wall_min,
        wall_mean,
        ..Default::default()
    };
    report.notes.push(format!("the wall is accurate to about ±{:.2} mm (voxel size {h:.2} mm)", h / 2.0));

    if opts.drain_holes > 0 {
        ctl.report(0.9, "Cutting drain holes");
        if !(opts.drain_diameter > 0.0 && opts.drain_diameter < size.min_element() / 2.0) {
            return Err(HollowError::Invalid("the drain hole diameter is invalid for this model".into()));
        }
        let cav: Vec<Vec3> = combined.vertices[mesh.vertices.len()..].to_vec();
        let mut chosen: Vec<Vec3> = Vec::new();
        let mut by_z: Vec<&Vec3> = cav.iter().collect();
        by_z.sort_by(|a, b| a.z.total_cmp(&b.z));
        for v in by_z {
            if chosen.iter().all(|c| ((c.x - v.x).powi(2) + (c.y - v.y).powi(2)).sqrt() > 4.0 * opts.drain_diameter as f32) {
                chosen.push(*v);
                if chosen.len() == opts.drain_holes {
                    break;
                }
            }
        }
        for v in &chosen {
            let r = opts.drain_diameter / 2.0;
            let (z0, z1) = (b.min.z as f64 - 1.0, v.z as f64 + opts.drain_diameter + 1.0);
            let hole = build::cylinder(r, z0, z1, 0.0, 0.0, 24);
            let moved = Mesh::new(hole.vertices.iter().map(|p| *p + Vec3::new(v.x, v.y, 0.0)).collect(), hole.triangles);
            let (m, _) = boolean(&combined, &moved, BoolOp::Difference)
                .map_err(|e| HollowError::Invalid(format!("could not cut a drain hole: {e}")))?;
            combined = m;
        }
        report.drain_holes = chosen.len();
        if chosen.len() < opts.drain_holes {
            report.notes.push(format!("only {} of {} drain holes could be spaced apart", chosen.len(), opts.drain_holes));
        }
    }
    let fs = combined.stats();
    if !fs.is_watertight() || fs.signed_volume <= 0.0 {
        return Err(HollowError::Invalid("the hollowed model is not a valid solid".into()));
    }
    report.material_volume = fs.signed_volume;
    ctl.report(1.0, "Done");
    Ok(HollowResult { mesh: combined, report })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::tests_support::*;

    #[test]
    fn edt_matches_brute_force_on_a_small_grid() {
        let n = [7, 6, 8];
        let sources = [(1usize, 1usize, 2usize), (5, 4, 6), (3, 0, 7)];
        let mut d: Vec<f32> = vec![FAR; n[0] * n[1] * n[2]];
        let fld = Field { n };
        for &(i, j, k) in &sources {
            d[fld.idx(i, j, k)] = 0.0;
        }
        edt_3d(&mut d, n);
        for i in 0..n[0] {
            for j in 0..n[1] {
                for k in 0..n[2] {
                    let brute = sources
                        .iter()
                        .map(|&(a, b, c)| (i as f32 - a as f32).powi(2) + (j as f32 - b as f32).powi(2) + (k as f32 - c as f32).powi(2))
                        .fold(f32::MAX, f32::min);
                    assert!((d[fld.idx(i, j, k)] - brute).abs() < 1e-3, "{i} {j} {k}: {} vs {brute}", d[fld.idx(i, j, k)]);
                }
            }
        }
    }

    #[test]
    fn hollow_sphere_has_the_requested_wall_and_correct_volumes() {
        let m = sphere(30.0, 64, 32, false);
        let outer = m.stats().signed_volume;
        let r = hollow(&m, &HollowOptions { wall: 4.0, resolution: Some(1.0), ..Default::default() }, &Control::none()).unwrap();
        let s = r.mesh.stats();
        assert!(s.is_watertight() && s.signed_volume > 0.0, "{s:?}");
        // Cavity is a sphere of radius ~26: volume 4/3 pi 26^3 = 73,622.
        let expected_cavity = 4.0 / 3.0 * std::f64::consts::PI * 26.0f64.powi(3);
        assert!(
            (r.report.cavity_volume - expected_cavity).abs() / expected_cavity < 0.03,
            "{} vs {expected_cavity}",
            r.report.cavity_volume
        );
        assert!((s.signed_volume - (outer - r.report.cavity_volume)).abs() < 1.0);
        // Measured wall is within about half a voxel of the request.
        assert!((r.report.wall_mean - 4.0).abs() < 0.6 && r.report.wall_min > 3.3, "{:?}", r.report);
        assert_eq!(r.report.drain_holes, 0);
    }

    #[test]
    fn hollow_cube_is_still_a_valid_solid_for_cutting() {
        let mut m = Mesh::cuboid(Vec3::splat(40.0));
        m.translate(Vec3::splat(20.0));
        let r = hollow(&m, &HollowOptions { wall: 5.0, resolution: Some(1.0), ..Default::default() }, &Control::none()).unwrap();
        // Cavity ~ 30^3 = 27,000 (rounded corners shave a little off).
        assert!((r.report.cavity_volume - 27_000.0).abs() / 27_000.0 < 0.05, "{}", r.report.cavity_volume);
        let cut = crate::cut::cut_plane(&r.mesh, &crate::Plane::axis(2, 20.0), &crate::cut::CutOptions::default());
        assert!(cut.report.is_valid(), "{:?}", cut.report);
        // The cut section is a square annulus: outer 40x40 minus the cavity section.
        assert_eq!(cut.section.polygons.len(), 1);
        assert_eq!(cut.section.polygons[0].holes.len(), 1);
    }

    #[test]
    fn drain_holes_open_the_cavity_to_the_outside() {
        let m = sphere(30.0, 64, 32, false);
        let sealed = hollow(&m, &HollowOptions { wall: 4.0, resolution: Some(1.0), ..Default::default() }, &Control::none()).unwrap();
        let drained = hollow(
            &m,
            &HollowOptions { wall: 4.0, resolution: Some(1.0), drain_holes: 1, drain_diameter: 5.0, ..Default::default() },
            &Control::none(),
        )
        .unwrap();
        assert_eq!(drained.report.drain_holes, 1);
        let ds = drained.mesh.stats();
        assert!(ds.is_watertight());
        // The hole removes a few cylinders' worth of wall: ~ pi * 2.5^2 * 4 = 78 mm^3.
        let removed = sealed.report.material_volume - drained.report.material_volume;
        assert!(removed > 30.0 && removed < 200.0, "{removed}");
    }

    #[test]
    fn thin_models_and_bad_options_are_refused_with_reasons() {
        let m = sphere(5.0, 32, 16, false);
        assert!(hollow(&m, &HollowOptions { wall: 4.0, ..Default::default() }, &Control::none())
            .unwrap_err()
            .to_string()
            .contains("no room"));
        let big = sphere(30.0, 32, 16, false);
        assert!(hollow(&big, &HollowOptions { wall: -1.0, ..Default::default() }, &Control::none()).is_err());
        let e = hollow(&big, &HollowOptions { wall: 1.0, resolution: Some(0.05), ..Default::default() }, &Control::none()).unwrap_err();
        assert!(e.to_string().contains("million voxels"), "{e}");
        let mut open = big.clone();
        open.triangles.pop();
        assert!(hollow(&open, &HollowOptions::default(), &Control::none()).is_err());
    }

    #[test]
    fn cancellation_is_honoured() {
        use std::sync::atomic::AtomicBool;
        let flag = AtomicBool::new(true);
        let m = sphere(30.0, 32, 16, false);
        assert!(matches!(hollow(&m, &HollowOptions::default(), &Control::with_cancel(&flag)), Err(HollowError::Cancelled(_))));
    }
}
