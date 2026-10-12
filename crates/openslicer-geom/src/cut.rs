//! Planar cutting with cap generation.
//!
//! # Method
//! * Vertices within `tolerance` of the plane are snapped onto it.
//! * Triangles are classified, and straddling triangles are split. The crossing
//!   point of each edge is created once and shared by both neighbouring
//!   triangles, so the two halves stay topologically consistent.
//! * The cut loops are read from the boundary of the lower half (boundary edges
//!   whose endpoints both lie on the plane). This is exact even when the plane
//!   passes through vertices or along edges, because it relies on mesh topology
//!   rather than on geometric tests.
//! * Loops are triangulated into caps (earcut) using the *same* vertices, so no
//!   gaps can appear between the caps and the walls.
//! * The result is verified: both halves must be watertight and their volumes
//!   must sum to the original volume.

use crate::plane::{Frame, Plane};
use crate::region::{ring_area, ring_contains, PlanarRegion, Polygon};
use glam::{DVec2, DVec3, Vec3};
use openslicer_core::Mesh;
use rustc_hash::{FxHashMap, FxHashSet};

#[derive(Clone, Debug)]
pub struct CutOptions {
    /// Vertices closer than this to the plane are treated as lying on it. `None` picks a size-relative default.
    pub tolerance: Option<f64>,
    /// Close the cut faces. Without caps the halves are open shells.
    pub cap: bool,
    /// Verify watertightness and volume conservation (costs one extra pass over the mesh).
    pub verify: bool,
}

impl Default for CutOptions {
    fn default() -> Self {
        Self { tolerance: None, cap: true, verify: true }
    }
}

#[derive(Clone, Debug, Default)]
pub struct CutReport {
    pub tolerance: f64,
    pub input_watertight: bool,
    pub volume_in: f64,
    pub volume_above: f64,
    pub volume_below: f64,
    /// |V_above + V_below − V_in| / |V_in| (only meaningful for watertight input).
    pub volume_error: f64,
    pub cap_loops: usize,
    /// Cut chains that could not be closed into loops (open input geometry).
    pub open_chains: usize,
    pub cap_area: f64,
    /// Non-fatal observations.
    pub notes: Vec<String>,
    /// Correctness problems. Empty means the cut verified cleanly.
    pub problems: Vec<String>,
}

impl CutReport {
    pub fn is_valid(&self) -> bool {
        self.problems.is_empty()
    }
}

pub struct CutResult {
    /// Part on the side the plane normal points to.
    pub above: Mesh,
    /// Part on the opposite side.
    pub below: Mesh,
    /// The cut section (cap polygons) in the plane's frame.
    pub section: PlanarRegion,
    pub report: CutReport,
}

/// Relative volume mismatch allowed after a cut. Planar snapping moves vertices by at most `tolerance`,
/// so the error is bounded by `cap_area * tolerance`; this limit is far looser than that and far tighter than a missing face.
const VOLUME_TOLERANCE: f64 = 1e-6;

const NONE: u32 = u32::MAX;

/// Output mesh under construction on one side of the plane.
struct Side {
    verts: Vec<Vec3>,
    tris: Vec<[u32; 3]>,
    orig_map: Vec<u32>,
    /// Plane-vertex id for each output vertex that lies on the plane, else NONE.
    pv_of: Vec<u32>,
    pv_map: Vec<u32>,
}

impl Side {
    fn new(n_orig: usize) -> Self {
        Self { verts: Vec::new(), tris: Vec::new(), orig_map: vec![NONE; n_orig], pv_of: Vec::new(), pv_map: Vec::new() }
    }

    fn orig(&mut self, mesh: &Mesh, v: u32) -> u32 {
        let slot = self.orig_map[v as usize];
        if slot != NONE {
            return slot;
        }
        let idx = self.verts.len() as u32;
        self.verts.push(mesh.vertices[v as usize]);
        self.pv_of.push(NONE);
        self.orig_map[v as usize] = idx;
        idx
    }

    fn plane_vertex(&mut self, pv: u32, pos: Vec3) -> u32 {
        if self.pv_map.len() <= pv as usize {
            self.pv_map.resize(pv as usize + 1, NONE);
        }
        let slot = self.pv_map[pv as usize];
        if slot != NONE {
            return slot;
        }
        let idx = self.verts.len() as u32;
        self.verts.push(pos);
        self.pv_of.push(pv);
        self.pv_map[pv as usize] = idx;
        idx
    }
}

/// A vertex reference during triangle splitting.
#[derive(Clone, Copy)]
enum Ref {
    /// Original vertex strictly off the plane.
    Orig(u32),
    /// Vertex on the plane (snapped original or edge crossing), by plane-vertex id.
    Plane(u32),
}

fn edge_key(a: u32, b: u32) -> u64 {
    let (lo, hi) = if a < b { (a, b) } else { (b, a) };
    ((lo as u64) << 32) | hi as u64
}

pub fn cut_plane(mesh: &Mesh, plane: &Plane, opts: &CutOptions) -> CutResult {
    let frame = plane.frame();
    let mut report = CutReport::default();
    let bounds = mesh.bounds();
    let diag = bounds.map(|b| b.size().length() as f64).unwrap_or(0.0);
    let tol = opts.tolerance.unwrap_or_else(|| (diag * 1e-6).clamp(1e-5, 1e-3));
    report.tolerance = tol;

    let in_stats = opts.verify.then(|| mesh.stats());
    if let Some(s) = &in_stats {
        report.input_watertight = s.is_watertight();
        report.volume_in = s.signed_volume;
    }

    let verts: Vec<DVec3> = mesh.vertices.iter().map(|v| v.as_dvec3()).collect();
    let dist: Vec<f64> = verts.iter().map(|&p| plane.signed_distance(p)).collect();
    let class: Vec<i8> = dist
        .iter()
        .map(|&s| {
            if s > tol {
                1
            } else if s < -tol {
                -1
            } else {
                0
            }
        })
        .collect();

    let mut above = Side::new(verts.len());
    let mut below = Side::new(verts.len());

    // Plane vertices shared by both sides.
    let mut pv_pos: Vec<DVec3> = Vec::new();
    let mut on_pv: Vec<u32> = vec![NONE; verts.len()];
    let mut cross_pv: FxHashMap<u64, u32> = FxHashMap::default();

    let mut on_vertex = |v: u32, pv_pos: &mut Vec<DVec3>| -> u32 {
        if on_pv[v as usize] == NONE {
            on_pv[v as usize] = pv_pos.len() as u32;
            pv_pos.push(plane.project(verts[v as usize]));
        }
        on_pv[v as usize]
    };
    let mut crossing = |a: u32, b: u32, pv_pos: &mut Vec<DVec3>| -> u32 {
        *cross_pv.entry(edge_key(a, b)).or_insert_with(|| {
            // Canonical endpoint order so both neighbouring triangles compute identical coordinates.
            let (lo, hi) = if a < b { (a, b) } else { (b, a) };
            let (sl, sh) = (dist[lo as usize], dist[hi as usize]);
            let t = sl / (sl - sh);
            let p = verts[lo as usize] + (verts[hi as usize] - verts[lo as usize]) * t;
            pv_pos.push(plane.project(p));
            (pv_pos.len() - 1) as u32
        })
    };

    for tri in &mesh.triangles {
        let c = [class[tri[0] as usize], class[tri[1] as usize], class[tri[2] as usize]];
        let (np, nn) = (c.iter().filter(|&&x| x > 0).count(), c.iter().filter(|&&x| x < 0).count());

        // Whole triangle on one side (zero-class vertices lie on the shared boundary).
        if nn == 0 || np == 0 {
            let side_is_above = if np > 0 {
                true
            } else if nn > 0 {
                false
            } else {
                // Coplanar triangle: it bounds the solid on the side opposite to its normal.
                let (a, b, cc) = (verts[tri[0] as usize], verts[tri[1] as usize], verts[tri[2] as usize]);
                (b - a).cross(cc - a).dot(plane.n) < 0.0
            };
            let mut idx = [0u32; 3];
            for k in 0..3 {
                let v = tri[k];
                let r = if c[k] == 0 { Ref::Plane(on_vertex(v, &mut pv_pos)) } else { Ref::Orig(v) };
                let side = if side_is_above { &mut above } else { &mut below };
                idx[k] = resolve(side, mesh, &pv_pos, r);
            }
            let side = if side_is_above { &mut above } else { &mut below };
            side.tris.push(idx);
            continue;
        }

        // Straddling triangle: clip the polygon against each half-space.
        let mut pos_poly: [Option<Ref>; 4] = [None; 4];
        let mut neg_poly: [Option<Ref>; 4] = [None; 4];
        let (mut pn, mut nnp) = (0usize, 0usize);
        for k in 0..3 {
            let (a, b) = (tri[k], tri[(k + 1) % 3]);
            let (ca, cb) = (c[k], c[(k + 1) % 3]);
            let ra = if ca == 0 { Ref::Plane(on_vertex(a, &mut pv_pos)) } else { Ref::Orig(a) };
            if ca >= 0 {
                pos_poly[pn] = Some(ra);
                pn += 1;
            }
            if ca <= 0 {
                neg_poly[nnp] = Some(ra);
                nnp += 1;
            }
            if (ca as i32) * (cb as i32) < 0 {
                let x = Ref::Plane(crossing(a, b, &mut pv_pos));
                pos_poly[pn] = Some(x);
                pn += 1;
                neg_poly[nnp] = Some(x);
                nnp += 1;
            }
        }
        for (poly, count, side) in [(&pos_poly, pn, &mut above), (&neg_poly, nnp, &mut below)] {
            let ids: Vec<u32> = poly[..count].iter().map(|r| resolve(side, mesh, &pv_pos, r.unwrap())).collect();
            for k in 1..count.saturating_sub(1) {
                side.tris.push([ids[0], ids[k], ids[k + 1]]);
            }
        }
    }

    // ---- cap -----------------------------------------------------------------
    let pv_to_2d = |pv: u32| frame.to_2d(pv_pos[pv as usize]);
    let mut section = PlanarRegion { plane: *plane, frame, polygons: Vec::new() };
    if opts.cap && !pv_pos.is_empty() {
        let cap_edges = boundary_cap_edges(&below);
        let (loops, open) = chain_loops(&cap_edges, &pv_to_2d);
        report.open_chains = open;
        report.cap_loops = loops.len();
        if open > 0 {
            report.problems.push(format!(
                "{open} cut chain(s) could not be closed: the input mesh has holes or non-manifold edges along the cut, so the cut face is incomplete"
            ));
        }
        match group_loops(&loops, &pv_to_2d) {
            Err(e) => report.problems.push(format!("cut face could not be built: {e}")),
            Ok(g) => {
                section.polygons = polygons_from(&loops, &g, &pv_to_2d);
                match triangulate_loops(&loops, &g, &section.polygons, &pv_to_2d) {
                    Ok((tris, area)) => {
                        report.cap_area = area;
                        for t in tris {
                            let p = |k: usize| pv_pos[t[k] as usize].as_vec3();
                            let (b0, b1, b2) =
                                (below.plane_vertex(t[0], p(0)), below.plane_vertex(t[1], p(1)), below.plane_vertex(t[2], p(2)));
                            below.tris.push([b0, b1, b2]);
                            let (a0, a1, a2) =
                                (above.plane_vertex(t[0], p(0)), above.plane_vertex(t[1], p(1)), above.plane_vertex(t[2], p(2)));
                            above.tris.push([a0, a2, a1]);
                        }
                    }
                    Err(e) => report.problems.push(format!("cut face could not be triangulated: {e}")),
                }
            }
        }
    }

    let result = CutResult {
        above: weld_coincident(Mesh::new(above.verts, above.tris)),
        below: weld_coincident(Mesh::new(below.verts, below.tris)),
        section,
        report,
    };
    finish(result, opts)
}

/// Merges output vertices that landed on exactly the same position (two crossing points of a sliver input
/// triangle can round to the same f32 point) and drops the zero-area triangles that result.
fn weld_coincident(mesh: Mesh) -> Mesh {
    let mut seen: FxHashMap<[u32; 3], u32> = FxHashMap::default();
    let mut remap: Vec<u32> = Vec::with_capacity(mesh.vertices.len());
    let mut keep: Vec<Vec3> = Vec::with_capacity(mesh.vertices.len());
    for v in &mesh.vertices {
        let key = [(v.x + 0.0).to_bits(), (v.y + 0.0).to_bits(), (v.z + 0.0).to_bits()];
        let id = *seen.entry(key).or_insert_with(|| {
            keep.push(*v);
            (keep.len() - 1) as u32
        });
        remap.push(id);
    }
    if keep.len() == mesh.vertices.len() {
        return mesh;
    }
    let tris = mesh
        .triangles
        .iter()
        .map(|t| [remap[t[0] as usize], remap[t[1] as usize], remap[t[2] as usize]])
        .filter(|t| t[0] != t[1] && t[1] != t[2] && t[0] != t[2])
        .collect();
    Mesh::new(keep, tris)
}

fn finish(mut r: CutResult, opts: &CutOptions) -> CutResult {
    if !opts.verify {
        return r;
    }
    let rep = &mut r.report;
    for (name, mesh) in [("upper", &r.above), ("lower", &r.below)] {
        if mesh.is_empty() {
            rep.notes.push(format!("the {name} part is empty"));
            continue;
        }
        let s = mesh.stats();
        if name == "upper" {
            rep.volume_above = s.signed_volume;
        } else {
            rep.volume_below = s.signed_volume;
        }
        if opts.cap && rep.input_watertight && !s.is_watertight() {
            rep.problems.push(format!(
                "the {name} part is not watertight after the cut ({} open edges, {} non-manifold edges, {} degenerate triangles)",
                s.boundary_edges, s.non_manifold_edges, s.degenerate_triangles
            ));
        }
    }
    if opts.cap && rep.input_watertight && rep.volume_in.abs() > 0.0 {
        rep.volume_error = (rep.volume_above + rep.volume_below - rep.volume_in).abs() / rep.volume_in.abs();
        if rep.volume_error > VOLUME_TOLERANCE {
            rep.problems.push(format!(
                "volume not conserved: the parts sum to {:.6} mm³ but the model is {:.6} mm³ (relative error {:.2e})",
                rep.volume_above + rep.volume_below,
                rep.volume_in,
                rep.volume_error
            ));
        }
    }
    if !rep.input_watertight {
        rep.notes.push("the input mesh is not watertight; the cut was performed but volume conservation cannot be verified".into());
    }
    r
}

fn resolve(side: &mut Side, mesh: &Mesh, pv_pos: &[DVec3], r: Ref) -> u32 {
    match r {
        Ref::Orig(v) => side.orig(mesh, v),
        Ref::Plane(pv) => side.plane_vertex(pv, pv_pos[pv as usize].as_vec3()),
    }
}

/// Cap half-edges (plane-vertex ids): for every boundary edge of the lower half lying on the plane, the reversed edge.
fn boundary_cap_edges(below: &Side) -> Vec<(u32, u32)> {
    let mut directed: FxHashSet<(u32, u32)> = FxHashSet::default();
    for t in &below.tris {
        for k in 0..3 {
            let (a, b) = (t[k], t[(k + 1) % 3]);
            let (pa, pb) = (below.pv_of[a as usize], below.pv_of[b as usize]);
            if pa != NONE && pb != NONE && pa != pb {
                directed.insert((pa, pb));
            }
        }
    }
    let mut out: Vec<(u32, u32)> = directed.iter().filter(|&&(a, b)| !directed.contains(&(b, a))).map(|&(a, b)| (b, a)).collect();
    out.sort_unstable(); // deterministic output
    out
}

/// Chains directed edges into loops. At a vertex with several outgoing edges the leftmost turn is taken,
/// which keeps loops simple (touching loops stay separate). Returns the loops and the number of unclosed chains.
pub(crate) fn chain_loops(edges: &[(u32, u32)], pos: &impl Fn(u32) -> DVec2) -> (Vec<Vec<u32>>, usize) {
    let mut out: FxHashMap<u32, Vec<usize>> = FxHashMap::default();
    for (i, &(a, _)) in edges.iter().enumerate() {
        out.entry(a).or_default().push(i);
    }
    let mut used = vec![false; edges.len()];
    let mut loops = Vec::new();
    let mut open = 0usize;
    for start in 0..edges.len() {
        if used[start] {
            continue;
        }
        let mut ring = vec![edges[start].0];
        let mut cur = start;
        let mut closed = false;
        loop {
            used[cur] = true;
            let (a, b) = edges[cur];
            if b == edges[start].0 {
                closed = true;
                break;
            }
            ring.push(b);
            let candidates: Vec<usize> = out.get(&b).map(|v| v.iter().copied().filter(|&e| !used[e]).collect()).unwrap_or_default();
            let next = match candidates.len() {
                0 => break,
                1 => candidates[0],
                _ => {
                    let incoming = (pos(b) - pos(a)).normalize_or_zero();
                    *candidates
                        .iter()
                        .max_by(|&&x, &&y| {
                            let ang = |e: usize| {
                                let d = (pos(edges[e].1) - pos(b)).normalize_or_zero();
                                incoming.perp_dot(d).atan2(incoming.dot(d))
                            };
                            ang(x).total_cmp(&ang(y))
                        })
                        .unwrap()
                }
            };
            cur = next;
        }
        if closed {
            loops.push(ring);
        } else {
            open += 1;
        }
    }
    (loops, open)
}

/// Loops grouped into polygons: each outer loop with the holes it encloses (indices into the loop list).
pub(crate) struct Grouping {
    pub outers: Vec<usize>,
    pub holes_of: Vec<Vec<usize>>,
}

/// Classifies loops (counter-clockwise = outline, clockwise = hole) and assigns each hole to its smallest enclosing outline.
pub(crate) fn group_loops(loops: &[Vec<u32>], pos: &impl Fn(u32) -> DVec2) -> Result<Grouping, String> {
    let mut outers: Vec<(usize, f64)> = Vec::new();
    let mut holes: Vec<usize> = Vec::new();
    for (i, l) in loops.iter().enumerate() {
        if l.len() < 3 {
            continue;
        }
        let pts: Vec<DVec2> = l.iter().map(|&v| pos(v)).collect();
        let a = ring_area(&pts);
        if a > 0.0 {
            outers.push((i, a));
        } else if a < 0.0 {
            holes.push(i);
        }
    }
    outers.sort_by(|a, b| a.1.total_cmp(&b.1));
    let mut holes_of: Vec<Vec<usize>> = vec![Vec::new(); outers.len()];
    for &h in &holes {
        let hp: Vec<DVec2> = loops[h].iter().map(|&v| pos(v)).collect();
        let samples: Vec<DVec2> = hp.iter().step_by((hp.len() / 8).max(1)).copied().collect();
        let h_area = ring_area(&hp).abs();
        let owner = outers.iter().position(|&(o, oa)| {
            let op: Vec<DVec2> = loops[o].iter().map(|&v| pos(v)).collect();
            oa > h_area && samples.iter().filter(|&&p| ring_contains(&op, p)).count() * 2 > samples.len()
        });
        match owner {
            Some(o) => holes_of[o].push(h),
            None => return Err("a hole has no enclosing outline".into()),
        }
    }
    Ok(Grouping { outers: outers.into_iter().map(|(i, _)| i).collect(), holes_of })
}

pub(crate) fn polygons_from(loops: &[Vec<u32>], g: &Grouping, pos: &impl Fn(u32) -> DVec2) -> Vec<Polygon> {
    g.outers
        .iter()
        .enumerate()
        .map(|(oi, &o)| Polygon {
            outer: loops[o].iter().map(|&v| pos(v)).collect(),
            holes: g.holes_of[oi].iter().map(|&h| loops[h].iter().map(|&v| pos(v)).collect()).collect(),
        })
        .collect()
}

/// Triangulates grouped loops into cap triangles over the existing vertex ids, using a constrained Delaunay
/// triangulation with exact predicates. Unlike ear clipping it cannot produce zero-area triangles, even when
/// many boundary points are collinear (flat walls, heightfield edges). Boundary vertices are never moved or
/// added, so the cap stays watertight against the walls.
fn triangulate_loops(
    loops: &[Vec<u32>],
    g: &Grouping,
    polys: &[Polygon],
    pos: &impl Fn(u32) -> DVec2,
) -> Result<(Vec<[u32; 3]>, f64), String> {
    use spade::{ConstrainedDelaunayTriangulation, Point2, Triangulation};
    let mut tris = Vec::new();
    let mut total = 0.0;
    for (oi, &o) in g.outers.iter().enumerate() {
        let mut rings: Vec<&Vec<u32>> = vec![&loops[o]];
        rings.extend(g.holes_of[oi].iter().map(|&h| &loops[h]));
        let mut cdt: ConstrainedDelaunayTriangulation<Point2<f64>> = ConstrainedDelaunayTriangulation::new();
        let mut handle_of: FxHashMap<u32, spade::handles::FixedVertexHandle> = FxHashMap::default();
        // First vertex id seen at each handle (vertices at identical positions share a handle; the output weld merges the rest).
        let mut id_of: Vec<u32> = Vec::new();
        for ring in &rings {
            for &v in ring.iter() {
                if handle_of.contains_key(&v) {
                    continue;
                }
                let p = pos(v);
                let h = cdt.insert(Point2::new(p.x, p.y)).map_err(|e| format!("triangulation input rejected: {e:?}"))?;
                if h.index() == id_of.len() {
                    id_of.push(v);
                }
                handle_of.insert(v, h);
            }
        }
        for ring in &rings {
            let n = ring.len();
            for i in 0..n {
                let (ha, hb) = (handle_of[&ring[i]], handle_of[&ring[(i + 1) % n]]);
                if ha == hb {
                    continue;
                }
                if !cdt.can_add_constraint(ha, hb) {
                    return Err("the cut outline crosses itself".into());
                }
                cdt.add_constraint(ha, hb);
            }
        }
        let poly = &polys[oi];
        // Classify faces purely topologically: start outside, and every constraint edge crossed toggles
        // inside/outside (even-odd). No geometric inside tests, so near-collinear boundary points cannot confuse it.
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
                if let Some(n) = e.rev().face().as_inner() {
                    let nd = d + u32::from(e.is_constraint_edge());
                    let slot = &mut depth[n.fix().index()];
                    if *slot == u32::MAX {
                        *slot = nd;
                        queue.push_back(n.fix());
                    }
                }
            }
        }
        let mut tri_area = 0.0;
        for face in cdt.inner_faces() {
            if depth[face.fix().index()] % 2 == 0 {
                continue;
            }
            let ids = face.vertices().map(|v| id_of[v.fix().index()]);
            let (a, b, c) = (pos(ids[0]), pos(ids[1]), pos(ids[2]));
            let signed = (b - a).perp_dot(c - a) * 0.5;
            tri_area += signed.abs();
            tris.push(if signed >= 0.0 { [ids[0], ids[1], ids[2]] } else { [ids[0], ids[2], ids[1]] });
        }
        let poly_area = poly.area();
        total += tri_area;
        if (tri_area - poly_area).abs() > 1e-7 * poly_area.abs().max(1e-9) {
            return Err(format!("triangulated area {tri_area:.9} differs from region area {poly_area:.9}"));
        }
    }
    Ok((tris, total))
}

/// Convenience: the cut section only (no meshes), as a planar region. Useful for previews.
pub fn section_frame(plane: &Plane) -> Frame {
    plane.frame()
}

#[cfg(test)]
mod tests {
    use super::*;
    use glam::Vec3;

    fn cube(size: f32) -> Mesh {
        let mut m = Mesh::cuboid(Vec3::splat(size));
        m.translate(Vec3::splat(size * 0.5));
        m
    }

    #[test]
    fn cut_cube_through_middle_gives_two_watertight_halves() {
        let m = cube(10.0);
        let r = cut_plane(&m, &Plane::axis(2, 5.0), &CutOptions::default());
        assert!(r.report.is_valid(), "{:?}", r.report);
        assert!(r.above.stats().is_watertight() && r.below.stats().is_watertight());
        assert!((r.above.stats().signed_volume - 500.0).abs() < 1e-3);
        assert!((r.below.stats().signed_volume - 500.0).abs() < 1e-3);
        assert!((r.report.cap_area - 100.0).abs() < 1e-6);
        assert_eq!(r.report.cap_loops, 1);
        assert!(r.above.bounds().unwrap().min.z >= 5.0 - 1e-4);
        assert!(r.below.bounds().unwrap().max.z <= 5.0 + 1e-4);
    }

    #[test]
    fn plane_through_vertices_is_handled() {
        // Cut exactly through the cube's mid-plane and through its corner vertices (diagonal-ish plane).
        let m = cube(10.0);
        let plane = Plane::from_point_normal(DVec3::new(0.0, 0.0, 0.0), DVec3::new(1.0, -1.0, 0.0)).unwrap();
        let r = cut_plane(&m, &plane, &CutOptions::default());
        assert!(r.report.is_valid(), "{:?}", r.report);
        let (a, b) = (r.above.stats().signed_volume, r.below.stats().signed_volume);
        assert!((a - 500.0).abs() < 1e-3 && (b - 500.0).abs() < 1e-3, "{a} {b}");
    }

    #[test]
    fn plane_on_a_face_gives_empty_side_not_corruption() {
        let m = cube(10.0);
        let r = cut_plane(&m, &Plane::axis(2, 10.0), &CutOptions::default());
        assert!(r.report.is_valid(), "{:?}", r.report);
        assert!(r.above.is_empty());
        assert!((r.below.stats().signed_volume - 1000.0).abs() < 1e-3);
        let r = cut_plane(&m, &Plane::axis(2, 0.0), &CutOptions::default());
        assert!(r.below.is_empty());
        assert!(r.above.stats().is_watertight());
    }

    #[test]
    fn plane_missing_the_model_returns_it_whole_on_one_side() {
        let m = cube(10.0);
        let r = cut_plane(&m, &Plane::axis(0, 50.0), &CutOptions::default());
        assert!(r.above.is_empty());
        assert_eq!(r.below.triangles.len(), 12);
        assert!(r.report.is_valid());
    }

    #[test]
    fn oblique_cut_conserves_volume() {
        let m = cube(10.0);
        let plane = Plane::from_point_normal(DVec3::new(5.0, 5.0, 5.0), DVec3::new(0.3, 0.5, 0.8)).unwrap();
        let r = cut_plane(&m, &plane, &CutOptions::default());
        assert!(r.report.is_valid(), "{:?}", r.report);
        assert!(r.report.volume_error < 1e-9);
    }

    #[test]
    fn cut_with_a_hole_through_it_makes_a_cap_with_a_hole() {
        // Tube: cut perpendicular to its axis, the cap must be an annulus (area π(R²−r²)).
        let tube = crate::tests_support::tube(20.0, 8.0, 60.0, 128);
        let r = cut_plane(&tube, &Plane::axis(2, 30.0), &CutOptions::default());
        assert!(r.report.is_valid(), "{:?}", r.report);
        assert_eq!(r.report.cap_loops, 2);
        let ring_area = std::f64::consts::PI * (20.0f64.powi(2) - 8.0f64.powi(2));
        assert!((r.report.cap_area - ring_area).abs() / ring_area < 0.01, "{} vs {}", r.report.cap_area, ring_area);
        assert_eq!(r.section.polygons.len(), 1);
        assert_eq!(r.section.polygons[0].holes.len(), 1);
    }

    #[test]
    fn open_mesh_is_reported_not_silently_accepted() {
        let mut m = cube(10.0);
        m.triangles.pop(); // remove a triangle: open mesh
        let r = cut_plane(&m, &Plane::axis(2, 5.0), &CutOptions::default());
        assert!(!r.report.input_watertight);
        assert!(!r.report.notes.is_empty());
    }

    #[test]
    fn torus_cut_two_ways_conserves_volume() {
        let torus = crate::tests_support::torus(30.0, 10.0, 96, 48);
        let v = torus.stats().signed_volume;
        let r = cut_plane(&torus, &Plane::axis(0, 0.0), &CutOptions::default());
        assert!(r.report.is_valid(), "{:?}", r.report);
        assert!((r.report.volume_above + r.report.volume_below - v).abs() / v < 1e-9);
        // Cutting at the very top of the tube ring (z = r) is a tangency: must not corrupt anything.
        let r = cut_plane(&torus, &Plane::axis(2, 5.0), &CutOptions::default());
        assert!(r.report.is_valid(), "{:?}", r.report);
    }

    fn random_plane(rng: &mut crate::tests_support::Lcg, m: &Mesh) -> Plane {
        let b = m.bounds().unwrap();
        let n = DVec3::new(rng.next_f64() - 0.5, rng.next_f64() - 0.5, rng.next_f64() - 0.5);
        let p = b.min.as_dvec3() + b.size().as_dvec3() * DVec3::new(rng.next_f64(), rng.next_f64(), rng.next_f64());
        Plane::from_point_normal(p, n).unwrap()
    }

    #[test]
    fn random_planes_through_complex_shapes_always_verify() {
        use crate::tests_support::*;
        let shapes: Vec<(&str, Mesh)> = vec![
            ("torus", torus(30.0, 10.0, 72, 36)),
            ("tube", tube(20.0, 8.0, 60.0, 64)),
            ("l_prism", l_prism()),
            ("sphere", sphere(25.0, 48, 24, false)),
            ("thin shell", sphere_shell(25.0, 23.0, 48, 24)),
        ];
        let mut rng = Lcg(12345);
        let mut cuts = 0;
        for (name, m) in &shapes {
            for _ in 0..60 {
                let plane = random_plane(&mut rng, m);
                let r = cut_plane(m, &plane, &CutOptions::default());
                assert!(r.report.is_valid(), "{name}: plane {plane:?}: {:?}", r.report);
                cuts += 1;
            }
        }
        assert_eq!(cuts, 300);
    }

    #[test]
    fn axis_planes_through_exact_vertex_heights_always_verify() {
        use crate::tests_support::*;
        // Planes exactly through rows of vertices: the worst case for degenerate classification.
        let shapes: Vec<(&str, Mesh)> = vec![
            ("torus", torus(30.0, 10.0, 72, 36)),
            ("sphere", sphere(25.0, 48, 24, false)),
            ("l_prism", l_prism()),
            ("tube", tube(20.0, 8.0, 60.0, 64)),
        ];
        for (name, m) in &shapes {
            for axis in 0..3 {
                let mut offsets: Vec<f32> = m.vertices.iter().map(|v| v[axis]).collect();
                offsets.sort_by(|a, b| a.total_cmp(b));
                offsets.dedup();
                for off in offsets.iter().step_by((offsets.len() / 12).max(1)) {
                    let r = cut_plane(m, &Plane::axis(axis, *off as f64), &CutOptions::default());
                    assert!(r.report.is_valid(), "{name} axis {axis} at {off}: {:?}", r.report);
                }
            }
        }
    }

    #[test]
    fn repeated_cuts_keep_total_volume() {
        use crate::tests_support::*;
        let m = torus(30.0, 10.0, 72, 36);
        let v = m.stats().signed_volume;
        let mut pieces = vec![m];
        for plane in [Plane::axis(0, 0.0), Plane::axis(1, 5.0), Plane::axis(2, 2.0), Plane::axis(0, -20.0)] {
            let mut next = Vec::new();
            for p in pieces {
                let r = cut_plane(&p, &plane, &CutOptions::default());
                assert!(r.report.is_valid(), "{:?}", r.report);
                for half in [r.above, r.below] {
                    if !half.is_empty() {
                        next.push(half);
                    }
                }
            }
            pieces = next;
        }
        let total: f64 = pieces.iter().map(|p| p.stats().signed_volume).sum();
        assert!((total - v).abs() / v < 1e-8, "{total} vs {v}");
        assert!(pieces.len() >= 8);
    }

    #[test]
    #[ignore = "performance check, run with --release -- --ignored"]
    fn million_triangle_cut_performance() {
        let m = crate::tests_support::torus(300.0, 80.0, 1000, 500);
        let t = std::time::Instant::now();
        let r = cut_plane(
            &m,
            &Plane::from_point_normal(DVec3::new(10.0, 0.0, 0.0), DVec3::new(0.2, 1.0, 0.3)).unwrap(),
            &CutOptions::default(),
        );
        eprintln!(
            "1M-triangle cut (with verification): {:?}, cap loops {}, valid {}",
            t.elapsed(),
            r.report.cap_loops,
            r.report.is_valid()
        );
        assert!(r.report.is_valid(), "{:?}", r.report);
        let t = std::time::Instant::now();
        let r = cut_plane(&m, &Plane::axis(2, 20.0), &CutOptions { verify: false, ..Default::default() });
        eprintln!("1M-triangle cut (no verification): {:?}, cap loops {}", t.elapsed(), r.report.cap_loops);
    }
}
