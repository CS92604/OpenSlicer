//! Mesh Booleans (union, difference, intersection) with validation.
//!
//! The kernel is `boolmesh`, a pure-Rust implementation of the Manifold
//! algorithm, which requires closed, manifold, consistently oriented inputs.
//! This wrapper checks that precondition, converts between mesh types,
//! converts kernel failures and panics into errors, and verifies each result:
//! it must be watertight and its volume must lie within the bounds the
//! operation implies. A result that fails verification is returned as an error.

use boolmesh::prelude::{compute_boolean, Manifold, OpType};
use glam::Vec3;
use openslicer_core::Mesh;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum BoolOp {
    Union,
    Difference,
    Intersection,
}

#[derive(Debug, Clone, PartialEq, thiserror::Error)]
pub enum BoolError {
    #[error("the {which} mesh cannot be used for a Boolean operation: {reason}")]
    InvalidInput { which: &'static str, reason: String },
    #[error("the Boolean kernel failed: {0}")]
    Kernel(String),
    #[error("the Boolean result failed verification: {0}")]
    Verification(String),
}

#[derive(Clone, Debug, Default)]
pub struct BoolReport {
    pub volume_a: f64,
    pub volume_b: f64,
    pub volume_result: f64,
}

fn check_input(which: &'static str, m: &Mesh) -> Result<f64, BoolError> {
    let s = m.stats();
    let bad = |reason: String| Err(BoolError::InvalidInput { which, reason });
    if s.triangle_count == 0 {
        return bad("it is empty".into());
    }
    if s.boundary_edges > 0 || s.non_manifold_edges > 0 {
        return bad(format!("it is not a closed manifold ({} open edges, {} non-manifold edges)", s.boundary_edges, s.non_manifold_edges));
    }
    if s.degenerate_triangles > 0 {
        return bad(format!("it has {} degenerate triangles", s.degenerate_triangles));
    }
    if s.signed_volume <= 0.0 {
        return bad("its normals point inward (negative volume)".into());
    }
    Ok(s.signed_volume)
}

fn to_manifold(m: &Mesh) -> Result<Manifold, String> {
    let pos: Vec<f64> = m.vertices.iter().flat_map(|v| [v.x as f64, v.y as f64, v.z as f64]).collect();
    let idx: Vec<usize> = m.triangles.iter().flat_map(|t| [t[0] as usize, t[1] as usize, t[2] as usize]).collect();
    Manifold::new(&pos, &idx)
}

fn from_manifold(m: &Manifold) -> Mesh {
    let vertices = m.ps.iter().map(|p| Vec3::new(p.x as f32, p.y as f32, p.z as f32)).collect();
    let triangles = m.hs.chunks_exact(3).map(|h| [h[0].tail as u32, h[1].tail as u32, h[2].tail as u32]).collect();
    Mesh::new(vertices, triangles)
}

/// Merges vertices that sit at the same position when they are the two ends of an edge of a zero-area triangle
/// (the kernel can emit such duplicate vertices). Triangles that collapse are dropped.
fn merge_coincident_vertices(m: &mut Mesh) -> usize {
    let mut parent: Vec<u32> = (0..m.vertices.len() as u32).collect();
    fn root(parent: &mut [u32], mut v: u32) -> u32 {
        while parent[v as usize] != v {
            parent[v as usize] = parent[parent[v as usize] as usize];
            v = parent[v as usize];
        }
        v
    }
    let mut merged = 0;
    for t in &m.triangles {
        let [a, b, c] = t.map(|i| m.vertices[i as usize].as_dvec3());
        if (b - a).cross(c - a).length() * 0.5 >= 1e-12 {
            continue;
        }
        for (u, v) in [(t[0], t[1]), (t[1], t[2]), (t[2], t[0])] {
            if u != v && (m.vertices[u as usize] - m.vertices[v as usize]).length() < 1e-6 {
                let (ru, rv) = (root(&mut parent, u), root(&mut parent, v));
                if ru != rv {
                    parent[rv as usize] = ru;
                    merged += 1;
                }
            }
        }
    }
    if merged > 0 {
        for t in &mut m.triangles {
            *t = t.map(|v| root(&mut parent, v));
        }
        m.triangles.retain(|t| t[0] != t[1] && t[1] != t[2] && t[0] != t[2]);
    }
    merged
}

/// Removes zero-area triangles whose three vertices are collinear, by flipping the long edge they share with
/// their neighbour. The covered surface is unchanged exactly, because the third vertex lies on that edge.
/// Flips that would create a duplicate edge are skipped; whatever remains is caught by the caller's checks.
fn remove_flat_triangles(m: &mut Mesh) -> usize {
    use std::collections::{HashMap, HashSet};
    let flat = |m: &Mesh, t: &[u32; 3]| {
        let [a, b, c] = t.map(|i| m.vertices[i as usize].as_dvec3());
        (b - a).cross(c - a).length() * 0.5 < 1e-12
    };
    let mut removed = 0;
    for _ in 0..8 {
        let flats: Vec<usize> = (0..m.triangles.len()).filter(|&i| flat(m, &m.triangles[i])).collect();
        if flats.is_empty() {
            break;
        }
        let mut directed: HashMap<(u32, u32), usize> = HashMap::new();
        for (i, t) in m.triangles.iter().enumerate() {
            for k in 0..3 {
                directed.insert((t[k], t[(k + 1) % 3]), i);
            }
        }
        let mut touched: HashSet<usize> = HashSet::new();
        let mut flips: Vec<(usize, usize, [u32; 3], [u32; 3])> = Vec::new();
        for f in flats {
            if touched.contains(&f) {
                continue;
            }
            let t = m.triangles[f];
            // The longest edge p -> q; b is the vertex between them.
            let len = |k: usize| (m.vertices[t[(k + 1) % 3] as usize] - m.vertices[t[k] as usize]).length();
            let k = (0..3).max_by(|&a, &b| len(a).total_cmp(&len(b))).unwrap();
            let (p, q, b) = (t[k], t[(k + 1) % 3], t[(k + 2) % 3]);
            let Some(&n) = directed.get(&(q, p)) else { continue };
            if n == f || touched.contains(&n) || flat(m, &m.triangles[n]) {
                continue;
            }
            let x = *m.triangles[n].iter().find(|&&v| v != p && v != q).unwrap();
            if directed.contains_key(&(b, x)) || directed.contains_key(&(x, b)) {
                continue;
            }
            touched.insert(f);
            touched.insert(n);
            flips.push((f, n, [b, p, x], [x, q, b]));
        }
        if flips.is_empty() {
            break;
        }
        for (f, n, t1, t2) in flips {
            m.triangles[f] = t1;
            m.triangles[n] = t2;
            removed += 1;
        }
    }
    removed
}

/// Runs a Boolean operation on two closed solids and verifies the result.
/// An empty result (for example the intersection of disjoint solids) is returned as an empty mesh.
pub fn boolean(a: &Mesh, b: &Mesh, op: BoolOp) -> Result<(Mesh, BoolReport), BoolError> {
    let va = check_input("first", a)?;
    let vb = check_input("second", b)?;
    let run = || -> Result<Mesh, BoolError> {
        let ma = to_manifold(a).map_err(|e| BoolError::InvalidInput { which: "first", reason: e })?;
        let mb = to_manifold(b).map_err(|e| BoolError::InvalidInput { which: "second", reason: e })?;
        let kind = match op {
            BoolOp::Union => OpType::Add,
            BoolOp::Difference => OpType::Subtract,
            BoolOp::Intersection => OpType::Intersect,
        };
        match compute_boolean(&ma, &mb, kind) {
            Ok(m) => {
                let mut mesh = from_manifold(&m);
                merge_coincident_vertices(&mut mesh);
                remove_flat_triangles(&mut mesh);
                Ok(mesh)
            }
            // The kernel reports an empty result as an error.
            Err(e) if e.contains("empty") => Ok(Mesh::default()),
            Err(e) => Err(BoolError::Kernel(e)),
        }
    };
    let result =
        std::panic::catch_unwind(std::panic::AssertUnwindSafe(run)).map_err(|_| BoolError::Kernel("the kernel panicked".into()))??;

    let mut report = BoolReport { volume_a: va, volume_b: vb, ..Default::default() };
    if result.is_empty() {
        let ok = match op {
            BoolOp::Intersection => true,
            // A can only vanish completely if B is at least as large.
            BoolOp::Difference => vb >= va * (1.0 - 1e-6),
            BoolOp::Union => false,
        };
        return if ok { Ok((result, report)) } else { Err(BoolError::Verification("the result is unexpectedly empty".into())) };
    }
    let s = result.stats();
    if !s.is_watertight() {
        return Err(BoolError::Verification(format!(
            "the result is not watertight ({} open edges, {} non-manifold edges, {} degenerate triangles)",
            s.boundary_edges, s.non_manifold_edges, s.degenerate_triangles
        )));
    }
    let v = s.signed_volume;
    report.volume_result = v;
    let slack = 1e-5 * (va + vb);
    let (lo, hi) = match op {
        BoolOp::Union => (va.max(vb), va + vb),
        BoolOp::Difference => ((va - vb).max(0.0), va),
        BoolOp::Intersection => (0.0, va.min(vb)),
    };
    if v < lo - slack || v > hi + slack {
        return Err(BoolError::Verification(format!("volume {v:.6} mm³ lies outside the possible range {lo:.6}..{hi:.6} mm³ for {op:?}")));
    }
    Ok((result, report))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn flat_triangles_are_flipped_away_without_changing_the_solid() {
        // A cuboid whose bottom edge 0-1 carries an extra vertex b, closed by a flat triangle.
        let mut m = Mesh::cuboid(Vec3::new(10.0, 20.0, 30.0));
        let b = m.vertices.len() as u32;
        m.vertices.push((m.vertices[0] + m.vertices[1]) * 0.5);
        m.triangles.retain(|t| *t != [0, 2, 1]);
        m.triangles.extend([[0, 2, b], [b, 2, 1], [1, 0, b]]);
        let before = m.stats();
        assert_eq!((before.boundary_edges, before.degenerate_triangles), (0, 1), "{before:?}");
        assert_eq!(remove_flat_triangles(&mut m), 1);
        let after = m.stats();
        assert!(after.is_watertight(), "{after:?}");
        assert!((after.signed_volume - before.signed_volume).abs() < 1e-9 && (after.surface_area - before.surface_area).abs() < 1e-9);
    }
    use crate::tests_support::*;

    fn cube_at(c: Vec3, size: f32) -> Mesh {
        let mut m = Mesh::cuboid(Vec3::splat(size));
        m.translate(c);
        m
    }

    fn cylinder(center: Vec3, r: f32, h: f32, n: usize) -> Mesh {
        let mut verts = Vec::new();
        for z in [0.0, h] {
            for k in 0..n {
                let a = std::f32::consts::TAU * k as f32 / n as f32;
                verts.push(center + Vec3::new(r * a.cos(), r * a.sin(), z));
            }
        }
        verts.push(center);
        verts.push(center + Vec3::new(0.0, 0.0, h));
        let (bc, tc) = (2 * n as u32, 2 * n as u32 + 1);
        let mut tris = Vec::new();
        for k in 0..n as u32 {
            let k2 = (k + 1) % n as u32;
            let nn = n as u32;
            tris.push([bc, k2, k]);
            tris.push([tc, nn + k, nn + k2]);
            tris.push([k, k2, nn + k2]);
            tris.push([k, nn + k2, nn + k]);
        }
        Mesh::new(verts, tris)
    }

    #[test]
    fn union_difference_intersection_of_overlapping_cubes() {
        let (a, b) = (cube_at(Vec3::ZERO, 10.0), cube_at(Vec3::new(5.0, 0.0, 0.0), 10.0));
        let vol = |m: &Mesh| m.stats().signed_volume;
        assert!((vol(&boolean(&a, &b, BoolOp::Union).unwrap().0) - 1500.0).abs() < 1e-3);
        assert!((vol(&boolean(&a, &b, BoolOp::Difference).unwrap().0) - 500.0).abs() < 1e-3);
        assert!((vol(&boolean(&a, &b, BoolOp::Intersection).unwrap().0) - 500.0).abs() < 1e-3);
    }

    #[test]
    fn pin_unions_onto_a_face_and_stays_watertight() {
        let block = cube_at(Vec3::ZERO, 40.0);
        let pin = cylinder(Vec3::new(0.0, 0.0, 20.0), 4.0, 10.0, 48);
        let (u, _) = boolean(&block, &pin, BoolOp::Union).unwrap();
        let expected = 64000.0 + pin.stats().signed_volume;
        assert!((u.stats().signed_volume - expected).abs() < 1e-2);
        assert!(u.stats().is_watertight());
    }

    #[test]
    fn socket_subtraction_removes_exactly_the_socket_volume() {
        let block = cube_at(Vec3::ZERO, 40.0);
        let sock = cylinder(Vec3::new(5.0, 5.0, 12.0), 4.5, 8.0, 48); // flush with the top face
        let (d, _) = boolean(&block, &sock, BoolOp::Difference).unwrap();
        assert!((d.stats().signed_volume - (64000.0 - sock.stats().signed_volume)).abs() < 1e-2);
    }

    #[test]
    fn subtracting_a_larger_solid_that_contains_the_first_is_empty() {
        let (inner, outer) = (cube_at(Vec3::ZERO, 10.0), cube_at(Vec3::ZERO, 20.0));
        let (r, _) = boolean(&inner, &outer, BoolOp::Difference).unwrap();
        assert!(r.is_empty());
        // The reverse leaves a hollow shell and is not empty.
        let (r, _) = boolean(&outer, &inner, BoolOp::Difference).unwrap();
        assert!((r.stats().signed_volume - 7000.0).abs() < 1e-2);
    }

    #[test]
    fn disjoint_intersection_is_empty_not_an_error() {
        let (a, b) = (cube_at(Vec3::ZERO, 10.0), cube_at(Vec3::new(50.0, 0.0, 0.0), 10.0));
        let (r, _) = boolean(&a, &b, BoolOp::Intersection).unwrap();
        assert!(r.is_empty());
        let (u, _) = boolean(&a, &b, BoolOp::Union).unwrap();
        assert!((u.stats().signed_volume - 2000.0).abs() < 1e-3);
    }

    #[test]
    fn invalid_inputs_are_rejected_with_a_reason() {
        let a = cube_at(Vec3::ZERO, 10.0);
        let mut open = a.clone();
        open.triangles.pop();
        assert!(matches!(boolean(&open, &a, BoolOp::Union), Err(BoolError::InvalidInput { which: "first", .. })));
        let mut inverted = a.clone();
        for t in &mut inverted.triangles {
            t.swap(1, 2);
        }
        let err = boolean(&a, &inverted, BoolOp::Union).unwrap_err();
        assert!(err.to_string().contains("inward"), "{err}");
        assert!(matches!(boolean(&a, &Mesh::default(), BoolOp::Union), Err(BoolError::InvalidInput { .. })));
    }

    #[test]
    fn works_on_thin_shells_and_curved_surfaces() {
        let shell = sphere_shell(25.0, 23.0, 48, 24);
        let cutter = cube_at(Vec3::new(25.0, 0.0, 0.0), 20.0);
        let (r, _) = boolean(&shell, &cutter, BoolOp::Difference).unwrap();
        let removed = shell.stats().signed_volume - r.stats().signed_volume;
        assert!(removed > 0.0 && removed < cutter.stats().signed_volume);
        assert!(r.stats().is_watertight());
    }
}
