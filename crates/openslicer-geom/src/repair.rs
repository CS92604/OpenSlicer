//! Mesh repair.
//!
//! Repairs are applied in a fixed order and each one is counted, so the report states exactly what changed:
//! weld coincident vertices, drop degenerate and duplicate triangles, make winding consistent within each
//! connected surface, flip surfaces that are inside-out (but keep genuine cavities), fill small holes, and
//! optionally remove tiny floating pieces. Self-intersections are detected and reported, not altered.
//! Nothing is claimed repaired unless the result is verified watertight.

use crate::bvh::{triangles_intersect, Bvh};
use crate::components::point_inside;
use glam::{DVec2, DVec3, Vec3};
use openslicer_core::{Mesh, MeshStats};
use rustc_hash::{FxHashMap, FxHashSet};

#[derive(Clone, Debug)]
pub struct RepairOptions {
    /// Merge vertices closer than this (mm); `None` uses 1e-6 of the bounding-box diagonal.
    pub weld_tolerance: Option<f64>,
    pub remove_degenerate: bool,
    pub remove_duplicates: bool,
    /// Make winding consistent and flip inside-out surfaces.
    pub orient: bool,
    /// Fill holes whose boundary has at most this many edges (0 disables hole filling).
    pub fill_holes_up_to: usize,
    /// Remove disconnected pieces smaller than this volume (mm³); `None` keeps them all.
    pub remove_islands_below: Option<f64>,
    /// Look for self-intersections after repairing (can be slow on very large meshes).
    pub detect_self_intersections: bool,
}

impl Default for RepairOptions {
    fn default() -> Self {
        Self {
            weld_tolerance: None,
            remove_degenerate: true,
            remove_duplicates: true,
            orient: true,
            fill_holes_up_to: 200,
            remove_islands_below: None,
            detect_self_intersections: true,
        }
    }
}

#[derive(Clone, Debug, Default)]
pub struct RepairReport {
    pub vertices_welded: usize,
    pub degenerate_removed: usize,
    pub duplicates_removed: usize,
    pub triangles_flipped: usize,
    pub components_flipped: usize,
    pub holes_filled: usize,
    pub holes_unfilled: usize,
    pub islands_removed: usize,
    /// `None` when detection was not requested.
    pub self_intersections: Option<usize>,
    pub before: Option<MeshStats>,
    pub after: Option<MeshStats>,
    /// Problems that remain after repair. Empty means the mesh is a clean closed solid.
    pub remaining: Vec<String>,
}

impl RepairReport {
    pub fn is_clean(&self) -> bool {
        self.remaining.is_empty()
    }

    pub fn changed(&self) -> bool {
        self.vertices_welded
            + self.degenerate_removed
            + self.duplicates_removed
            + self.triangles_flipped
            + self.holes_filled
            + self.islands_removed
            > 0
    }
}

fn edge_key(a: u32, b: u32) -> (u32, u32) {
    (a.min(b), a.max(b))
}

/// Repairs `mesh` and returns the new mesh and a report.
pub fn repair(mesh: &Mesh, opts: &RepairOptions) -> (Mesh, RepairReport) {
    let mut report = RepairReport { before: Some(mesh.stats()), ..Default::default() };
    let diag = mesh.bounds().map_or(0.0, |b| b.size().length() as f64);
    // 1. Weld.
    let tol = opts.weld_tolerance.unwrap_or(diag * 1e-6).max(0.0) as f32;
    let soup: Vec<[Vec3; 3]> = (0..mesh.triangles.len()).map(|i| mesh.triangle(i)).collect();
    let mut m = Mesh::from_triangle_soup(soup, tol);
    report.vertices_welded = mesh.vertices.len().saturating_sub(m.vertices.len());
    // Vertices not referenced by any triangle disappear in the weld; do not count them as merged duplicates.
    let used: FxHashSet<u32> = mesh.triangles.iter().flatten().copied().collect();
    report.vertices_welded = used.len().saturating_sub(m.vertices.len());

    // 2. Degenerate triangles.
    if opts.remove_degenerate {
        let min_area = (diag * diag) * 1e-14;
        let before = m.triangles.len();
        let verts = m.vertices.clone();
        m.triangles.retain(|t| {
            if t[0] == t[1] || t[1] == t[2] || t[0] == t[2] {
                return false;
            }
            let (a, b, c) = (verts[t[0] as usize].as_dvec3(), verts[t[1] as usize].as_dvec3(), verts[t[2] as usize].as_dvec3());
            (b - a).cross(c - a).length() * 0.5 > min_area
        });
        report.degenerate_removed = before - m.triangles.len();
    }
    // 3. Duplicate triangles (same three vertices, any order).
    if opts.remove_duplicates {
        let mut seen: FxHashSet<[u32; 3]> = FxHashSet::default();
        let before = m.triangles.len();
        m.triangles.retain(|t| {
            let mut k = *t;
            k.sort_unstable();
            seen.insert(k)
        });
        report.duplicates_removed = before - m.triangles.len();
    }
    // 4. Orientation.
    if opts.orient {
        let (flipped, comps_flipped) = orient(&mut m);
        report.triangles_flipped = flipped;
        report.components_flipped = comps_flipped;
    }
    // 5. Holes.
    if opts.fill_holes_up_to > 0 {
        let (filled, left) = fill_holes(&mut m, opts.fill_holes_up_to);
        report.holes_filled = filled;
        report.holes_unfilled = left;
    }
    // 6. Islands.
    if let Some(min_volume) = opts.remove_islands_below {
        report.islands_removed = remove_islands(&mut m, min_volume);
    }
    // Drop vertices no triangle uses.
    m = compact(m);

    let after = m.stats();
    // 7. Self-intersections.
    if opts.detect_self_intersections && !m.is_empty() {
        report.self_intersections = Some(count_self_intersections(&m));
    }
    if after.triangle_count == 0 {
        report.remaining.push("the mesh is empty".into());
    }
    if after.boundary_edges > 0 {
        report.remaining.push(format!("{} open edges remain (holes too large or irregular to fill)", after.boundary_edges));
    }
    if after.non_manifold_edges > 0 {
        report.remaining.push(format!("{} non-manifold edges remain (more than two faces meet at an edge)", after.non_manifold_edges));
    }
    if after.degenerate_triangles > 0 {
        report.remaining.push(format!("{} degenerate triangles remain", after.degenerate_triangles));
    }
    if after.is_inverted() {
        report.remaining.push("the normals still point inward".into());
    }
    if let Some(n) = report.self_intersections.filter(|&n| n > 0) {
        report.remaining.push(format!("{n} pairs of triangles intersect each other; Boolean operations and splitting may be unreliable"));
    }
    report.after = Some(after);
    (m, report)
}

fn compact(m: Mesh) -> Mesh {
    let mut map = vec![u32::MAX; m.vertices.len()];
    let mut verts = Vec::new();
    let tris = m
        .triangles
        .iter()
        .map(|t| {
            let mut o = [0u32; 3];
            for k in 0..3 {
                let v = t[k] as usize;
                if map[v] == u32::MAX {
                    map[v] = verts.len() as u32;
                    verts.push(m.vertices[v]);
                }
                o[k] = map[v];
            }
            o
        })
        .collect();
    Mesh::new(verts, tris)
}

/// Makes winding consistent across each manifold-connected surface, then flips surfaces that are inside-out unless they
/// are cavities inside another surface. Returns (triangles flipped, whole surfaces flipped).
fn orient(m: &mut Mesh) -> (usize, usize) {
    let n = m.triangles.len();
    if n == 0 {
        return (0, 0);
    }
    let mut edge_tris: FxHashMap<(u32, u32), Vec<u32>> = FxHashMap::default();
    for (i, t) in m.triangles.iter().enumerate() {
        for k in 0..3 {
            edge_tris.entry(edge_key(t[k], t[(k + 1) % 3])).or_default().push(i as u32);
        }
    }
    let directed = |t: &[u32; 3], a: u32, b: u32| -> bool { (0..3).any(|k| t[k] == a && t[(k + 1) % 3] == b) };
    let mut comp = vec![u32::MAX; n];
    let mut flip = vec![false; n];
    let mut comps = 0u32;
    for start in 0..n {
        if comp[start] != u32::MAX {
            continue;
        }
        comp[start] = comps;
        let mut stack = vec![start];
        while let Some(i) = stack.pop() {
            let t = m.triangles[i];
            for k in 0..3 {
                let (a, b) = (t[k], t[(k + 1) % 3]);
                let list = &edge_tris[&edge_key(a, b)];
                if list.len() != 2 {
                    continue; // boundary or non-manifold edge: do not propagate across it
                }
                let j = if list[0] as usize == i { list[1] } else { list[0] } as usize;
                if comp[j] != u32::MAX {
                    continue;
                }
                // Triangle i (after its own flip) traverses a->b if !flip[i]. A consistent neighbour traverses b->a.
                let i_forward = !flip[i];
                let j_has_ab = directed(&m.triangles[j], a, b);
                let j_forward_ab = if flip[i] { !j_has_ab } else { j_has_ab };
                // j must run opposite to i along the shared edge.
                flip[j] = if i_forward { j_has_ab } else { !j_has_ab && false || j_forward_ab };
                comp[j] = comps;
                stack.push(j);
            }
        }
        comps += 1;
    }
    let mut flipped = 0;
    for (t, f) in m.triangles.iter_mut().zip(&flip) {
        if *f {
            t.swap(1, 2);
            flipped += 1;
        }
    }
    // Inside-out surfaces.
    let mut volume = vec![0.0f64; comps as usize];
    let mut closed = vec![true; comps as usize];
    for (i, t) in m.triangles.iter().enumerate() {
        let (a, b, c) = (m.vertices[t[0] as usize].as_dvec3(), m.vertices[t[1] as usize].as_dvec3(), m.vertices[t[2] as usize].as_dvec3());
        volume[comp[i] as usize] += a.dot(b.cross(c)) / 6.0;
        for k in 0..3 {
            if edge_tris[&edge_key(t[k], t[(k + 1) % 3])].len() != 2 {
                closed[comp[i] as usize] = false;
            }
        }
    }
    let mut flipped_components = 0;
    let negatives: Vec<u32> = (0..comps).filter(|&c| closed[c as usize] && volume[c as usize] < 0.0).collect();
    if !negatives.is_empty() {
        let positives: Vec<u32> = (0..comps).filter(|&c| closed[c as usize] && volume[c as usize] > 0.0).collect();
        let surface = |c: u32| -> Mesh {
            let tris: Vec<u32> = (0..n as u32).filter(|&i| comp[i as usize] == c).collect();
            crate::components::extract(m, &tris)
        };
        let mut to_flip = Vec::new();
        for &c in &negatives {
            let probe = {
                let s = surface(c);
                s.vertices[0]
            };
            // A negative surface lying inside a positive one is a cavity and is correct as it is.
            let is_cavity = positives.iter().any(|&p| point_inside(&surface(p), probe));
            if !is_cavity {
                to_flip.push(c);
            }
        }
        for (t, &c) in m.triangles.iter_mut().zip(&comp) {
            if to_flip.contains(&c) {
                t.swap(1, 2);
                flipped += 1;
            }
        }
        flipped_components = to_flip.len();
    }
    (flipped, flipped_components)
}

/// Fills closed boundary loops with at most `max_edges` edges. Returns (filled, remaining open loops).
fn fill_holes(m: &mut Mesh, max_edges: usize) -> (usize, usize) {
    // Directed boundary edges: an edge used once.
    let mut directed: FxHashMap<(u32, u32), u32> = FxHashMap::default();
    let mut undirected: FxHashMap<(u32, u32), u32> = FxHashMap::default();
    for t in &m.triangles {
        for k in 0..3 {
            let (a, b) = (t[k], t[(k + 1) % 3]);
            *directed.entry((a, b)).or_insert(0) += 1;
            *undirected.entry(edge_key(a, b)).or_insert(0) += 1;
        }
    }
    let boundary: Vec<(u32, u32)> =
        directed.iter().filter(|(&(a, b), &n)| n == 1 && undirected[&edge_key(a, b)] == 1).map(|(&e, _)| e).collect();
    if boundary.is_empty() {
        return (0, 0);
    }
    let mut next: FxHashMap<u32, Vec<u32>> = FxHashMap::default();
    for &(a, b) in &boundary {
        next.entry(a).or_default().push(b);
    }
    let mut sorted = boundary.clone();
    sorted.sort_unstable();
    let mut used: FxHashSet<(u32, u32)> = FxHashSet::default();
    let (mut filled, mut remaining) = (0, 0);
    for &(a0, b0) in &sorted {
        if used.contains(&(a0, b0)) {
            continue;
        }
        // Walk the loop a0 -> b0 -> ...
        let mut ring = vec![a0];
        let (mut a, mut b) = (a0, b0);
        let mut closed = false;
        for _ in 0..=max_edges + 1 {
            used.insert((a, b));
            if b == a0 {
                closed = true;
                break;
            }
            ring.push(b);
            let Some(cands) = next.get(&b) else { break };
            // At a pinch vertex take an unused continuation.
            let Some(&c) = cands.iter().find(|&&c| !used.contains(&(b, c))) else { break };
            a = b;
            b = c;
        }
        if !closed || ring.len() < 3 || ring.len() > max_edges {
            remaining += 1;
            continue;
        }
        match triangulate_loop(m, &ring) {
            Some(tris) => {
                m.triangles.extend(tris);
                filled += 1;
            }
            None => remaining += 1,
        }
    }
    (filled, remaining)
}

/// Triangulates a boundary loop (vertex ids in the order the mesh traverses its boundary). The new triangles
/// run opposite to the existing edges so they close the hole consistently.
fn triangulate_loop(m: &Mesh, ring: &[u32]) -> Option<Vec<[u32; 3]>> {
    use spade::{ConstrainedDelaunayTriangulation, Point2, Triangulation};
    let pts: Vec<DVec3> = ring.iter().map(|&v| m.vertices[v as usize].as_dvec3()).collect();
    // Newell normal of the loop.
    let mut normal = DVec3::ZERO;
    for i in 0..pts.len() {
        let (p, q) = (pts[i], pts[(i + 1) % pts.len()]);
        normal += DVec3::new((p.y - q.y) * (p.z + q.z), (p.z - q.z) * (p.x + q.x), (p.x - q.x) * (p.y + q.y));
    }
    let n = normal.try_normalize()?;
    let helper = if n.z.abs() < 0.9 { DVec3::Z } else { DVec3::X };
    let u = helper.cross(n).normalize();
    let v = n.cross(u);
    let p2: Vec<DVec2> = pts.iter().map(|p| DVec2::new(p.dot(u), p.dot(v))).collect();
    let mut cdt: ConstrainedDelaunayTriangulation<Point2<f64>> = ConstrainedDelaunayTriangulation::new();
    let mut handles = Vec::with_capacity(ring.len());
    let mut id_of: Vec<usize> = Vec::new(); // handle index -> ring index
    for (i, p) in p2.iter().enumerate() {
        let h = cdt.insert(Point2::new(p.x, p.y)).ok()?;
        if h.index() == id_of.len() {
            id_of.push(i);
        }
        handles.push(h);
    }
    for i in 0..ring.len() {
        let (a, b) = (handles[i], handles[(i + 1) % ring.len()]);
        if a == b {
            continue;
        }
        if !cdt.can_add_constraint(a, b) {
            return None;
        }
        cdt.add_constraint(a, b);
    }
    // Topological inside test: flood from the outer face, toggling at constraint edges.
    let mut depth: Vec<u32> = vec![u32::MAX; cdt.num_all_faces()];
    let mut queue = std::collections::VecDeque::new();
    for face in cdt.inner_faces() {
        for e in face.adjacent_edges() {
            if e.rev().face().as_inner().is_none() {
                let d = u32::from(e.is_constraint_edge());
                let slot = &mut depth[face.fix().index()];
                if *slot == u32::MAX || d < *slot {
                    *slot = d;
                    queue.push_back(face.fix());
                }
            }
        }
    }
    while let Some(f) = queue.pop_front() {
        let d = depth[f.index()];
        for e in cdt.face(f).adjacent_edges() {
            if let Some(nf) = e.rev().face().as_inner() {
                let nd = d + u32::from(e.is_constraint_edge());
                let slot = &mut depth[nf.fix().index()];
                if *slot == u32::MAX {
                    *slot = nd;
                    queue.push_back(nf.fix());
                }
            }
        }
    }
    let loop_area: f64 = (0..p2.len()).map(|i| p2[i].perp_dot(p2[(i + 1) % p2.len()])).sum::<f64>() * 0.5;
    let mut out = Vec::new();
    let mut area = 0.0;
    for face in cdt.inner_faces() {
        if depth[face.fix().index()] % 2 == 0 {
            continue;
        }
        let ids = face.vertices().map(|h| ring[id_of[h.fix().index()]]);
        let q = ids.map(|v| {
            let i = ring.iter().position(|&r| r == v).unwrap();
            p2[i]
        });
        let signed = (q[1] - q[0]).perp_dot(q[2] - q[0]) * 0.5;
        area += signed.abs();
        // The loop is traversed in the mesh's boundary direction; a CCW triangle (w.r.t. the loop's own orientation) would
        // repeat those directions, so reverse it. `loop_area` tells which way "CCW" is for this ring.
        let ccw_for_ring = if loop_area >= 0.0 { signed >= 0.0 } else { signed < 0.0 };
        out.push(if ccw_for_ring { [ids[0], ids[2], ids[1]] } else { [ids[0], ids[1], ids[2]] });
    }
    if out.is_empty() || (area - loop_area.abs()).abs() > 1e-6 * loop_area.abs().max(1e-12) {
        return None;
    }
    Some(out)
}

/// Removes closed, self-contained surfaces whose volume is below `min_volume`. Returns how many were removed.
fn remove_islands(m: &mut Mesh, min_volume: f64) -> usize {
    let comps = crate::components::split_components(m);
    if comps.len() <= 1 {
        return 0;
    }
    let keep: Vec<&Mesh> = comps.iter().filter(|c| c.stats().signed_volume.abs() >= min_volume).collect();
    let removed = comps.len() - keep.len();
    if removed > 0 && !keep.is_empty() {
        let owned: Vec<Mesh> = keep.into_iter().cloned().collect();
        *m = crate::build::merge(&owned);
    }
    removed
}

/// Number of triangle pairs that intersect without sharing a vertex.
pub fn count_self_intersections(m: &Mesh) -> usize {
    let bvh = Bvh::build(m);
    let mut count = 0;
    bvh.overlapping_pairs(|i, j| {
        let (a, b) = (m.triangles[i as usize], m.triangles[j as usize]);
        if a.iter().any(|v| b.contains(v)) {
            return;
        }
        let to = |t: [u32; 3]| t.map(|v| m.vertices[v as usize].as_dvec3());
        if triangles_intersect(to(a), to(b)) {
            count += 1;
        }
    });
    count
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::tests_support::*;

    fn cube() -> Mesh {
        let mut m = Mesh::cuboid(Vec3::splat(10.0));
        m.translate(Vec3::splat(5.0));
        m
    }

    fn opts() -> RepairOptions {
        RepairOptions { remove_islands_below: None, ..Default::default() }
    }

    #[test]
    fn a_clean_mesh_is_left_alone() {
        let (m, r) = repair(&cube(), &opts());
        assert!(r.is_clean(), "{r:?}");
        assert!(!r.changed(), "{r:?}");
        assert_eq!(m.triangles.len(), 12);
        assert_eq!(r.self_intersections, Some(0));
    }

    #[test]
    fn unwelded_soup_is_welded() {
        let c = cube();
        let soup = Mesh::new((0..12).flat_map(|i| c.triangle(i)).collect(), (0..12).map(|i| [3 * i, 3 * i + 1, 3 * i + 2]).collect());
        assert_eq!(soup.vertices.len(), 36);
        let (m, r) = repair(&soup, &opts());
        assert!(r.is_clean(), "{r:?}");
        assert_eq!(m.vertices.len(), 8);
        assert_eq!(r.vertices_welded, 28);
    }

    #[test]
    fn degenerate_and_duplicate_triangles_are_removed() {
        let mut m = cube();
        m.triangles.push([0, 0, 1]);
        m.triangles.push(m.triangles[0]);
        let mut rev = m.triangles[3];
        rev.swap(0, 1);
        m.triangles.push(rev); // same vertices, other order
        let (out, r) = repair(&m, &opts());
        assert_eq!(r.degenerate_removed, 1);
        assert_eq!(r.duplicates_removed, 2);
        assert!(r.is_clean(), "{r:?}");
        assert_eq!(out.triangles.len(), 12);
    }

    #[test]
    fn inconsistent_winding_is_fixed_and_inside_out_surfaces_are_flipped() {
        let mut m = cube();
        // Flip three scattered triangles: inconsistent winding.
        for i in [1, 6, 9] {
            m.triangles[i].swap(1, 2);
        }
        let (out, r) = repair(&m, &opts());
        assert!(r.is_clean(), "{r:?}");
        assert!(r.triangles_flipped >= 3);
        assert!((out.stats().signed_volume - 1000.0).abs() < 1e-3);
        // A fully inverted cube.
        let mut inv = cube();
        for t in &mut inv.triangles {
            t.swap(1, 2);
        }
        let (out, r) = repair(&inv, &opts());
        assert!(r.is_clean(), "{r:?}");
        assert_eq!(r.components_flipped, 1);
        assert!((out.stats().signed_volume - 1000.0).abs() < 1e-3);
    }

    #[test]
    fn a_genuine_cavity_is_not_flipped() {
        let shell = sphere_shell(25.0, 20.0, 32, 16);
        let before = shell.stats().signed_volume;
        let (out, r) = repair(&shell, &opts());
        assert!(r.is_clean(), "{r:?}");
        assert_eq!(r.components_flipped, 0);
        assert!((out.stats().signed_volume - before).abs() < 1e-6);
    }

    #[test]
    fn holes_are_filled_with_matching_geometry() {
        let mut m = cube();
        m.triangles.remove(4); // opens a 2-triangle quad hole on one face... (removes one triangle: triangular hole)
        let (out, r) = repair(&m, &opts());
        assert!(r.is_clean(), "{r:?}");
        assert_eq!(r.holes_filled, 1);
        assert!((out.stats().signed_volume - 1000.0).abs() < 1e-3);
        // A bigger hole in a sphere (a cap of faces removed).
        let mut s = sphere(20.0, 32, 16, false);
        let before = s.stats().signed_volume;
        let keep: Vec<[u32; 3]> =
            s.triangles.iter().copied().filter(|t| t.iter().map(|&v| s.vertices[v as usize].z).sum::<f32>() / 3.0 < 15.0).collect();
        s.triangles = keep;
        assert!(!s.stats().is_watertight());
        let (out, r) = repair(&s, &opts());
        assert!(r.is_clean(), "{r:?}");
        assert_eq!(r.holes_filled, 1);
        // The flat cap closes a spherical cap region from outside: volume is slightly below the full sphere.
        let v = out.stats().signed_volume;
        assert!(v > 0.95 * before && v < before, "{v} vs {before}");
    }

    #[test]
    fn holes_larger_than_the_limit_are_reported_not_hidden() {
        let mut m = cube();
        m.triangles.truncate(10); // removes one whole face (2 triangles)
        let (out, r) = repair(&m, &RepairOptions { fill_holes_up_to: 3, ..opts() });
        assert!(!r.is_clean());
        assert!(r.remaining.iter().any(|s| s.contains("open edges")), "{:?}", r.remaining);
        assert_eq!(r.holes_unfilled, 1);
        assert!(!out.stats().is_watertight());
    }

    #[test]
    fn tiny_islands_can_be_removed() {
        let mut m = cube();
        let mut speck = Mesh::cuboid(Vec3::splat(0.5));
        speck.translate(Vec3::new(50.0, 0.0, 0.0));
        let base = m.vertices.len() as u32;
        m.vertices.extend(speck.vertices);
        m.triangles.extend(speck.triangles.iter().map(|t| [t[0] + base, t[1] + base, t[2] + base]));
        let (out, r) = repair(&m, &RepairOptions { remove_islands_below: Some(1.0), ..opts() });
        assert_eq!(r.islands_removed, 1);
        assert_eq!(out.triangles.len(), 12);
        // Without the option the island stays.
        let (out2, r2) = repair(&m, &opts());
        assert_eq!(r2.islands_removed, 0);
        assert_eq!(out2.triangles.len(), 24);
    }

    #[test]
    fn self_intersections_are_detected_and_reported() {
        let mut a = cube();
        let mut b = Mesh::cuboid(Vec3::splat(10.0));
        b.translate(Vec3::new(9.0, 8.0, 7.0));
        let base = a.vertices.len() as u32;
        a.vertices.extend(b.vertices);
        a.triangles.extend(b.triangles.iter().map(|t| [t[0] + base, t[1] + base, t[2] + base]));
        let (_, r) = repair(&a, &opts());
        assert!(r.self_intersections.unwrap() > 0);
        assert!(!r.is_clean());
        assert!(r.remaining.iter().any(|s| s.contains("intersect")), "{:?}", r.remaining);
        // Two separate cubes do not intersect.
        let mut c = cube();
        let mut d = Mesh::cuboid(Vec3::splat(10.0));
        d.translate(Vec3::new(60.0, 0.0, 0.0));
        let base = c.vertices.len() as u32;
        c.vertices.extend(d.vertices);
        c.triangles.extend(d.triangles.iter().map(|t| [t[0] + base, t[1] + base, t[2] + base]));
        assert_eq!(repair(&c, &opts()).1.self_intersections, Some(0));
    }

    #[test]
    fn repaired_meshes_can_be_cut_and_booleaned() {
        // A cube with a hole and flipped triangles becomes usable by the cutter.
        let mut m = cube();
        m.triangles.remove(7);
        m.triangles[2].swap(1, 2);
        let (fixed, r) = repair(&m, &opts());
        assert!(r.is_clean(), "{r:?}");
        let cut = crate::cut::cut_plane(&fixed, &crate::Plane::axis(2, 5.0), &crate::cut::CutOptions::default());
        assert!(cut.report.is_valid(), "{:?}", cut.report);
    }
}
