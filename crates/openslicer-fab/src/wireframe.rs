//! Wireframe: the model as a cage of tubular hoops that follow its cross-section contours.
//!
//! Sections are taken at regular spacing along each chosen axis; every contour (outer and hole) becomes a
//! closed tube. Hoops from different axes cross each other, so the result is a lattice that outlines the form.
//! Overlapping hoops can be merged into single solids with Boolean unions.

use crate::{ensure_fit, FabError, FabReport, FabResult, Part};
use glam::DVec3;
use openslicer_core::control::Control;
use openslicer_core::Mesh;
use openslicer_geom::build::tube;
use openslicer_geom::section::cross_section;
use openslicer_geom::Plane;
use openslicer_split::exec::Piece;

#[derive(Clone, Debug)]
pub struct WireframeSettings {
    /// Distance between hoops along each enabled axis (mm).
    pub spacing: f64,
    pub strut_radius: f64,
    /// Which axes to take sections along (X, Y, Z).
    pub axes: [bool; 3],
    /// Contours are simplified to within this distance (mm) before sweeping; larger values give straighter struts.
    pub simplify: f64,
    pub segments: usize,
    /// Merge crossing hoops into single solids with Boolean unions (skipped when there are too many hoops).
    pub merge: bool,
    pub usable: DVec3,
}

impl WireframeSettings {
    pub fn new(usable: DVec3) -> Self {
        Self { spacing: 40.0, strut_radius: 1.5, axes: [false, false, true], simplify: 0.5, segments: 12, merge: false, usable }
    }
}

/// Douglas-Peucker simplification of a closed ring (3D points); keeps at least 3 points.
pub fn simplify_ring(ring: &[DVec3], tol: f64) -> Vec<DVec3> {
    let n = ring.len();
    if n <= 3 || tol <= 0.0 {
        return ring.to_vec();
    }
    // Anchor at point 0 and the point farthest from it, then simplify each half.
    let far = (1..n).max_by(|&a, &b| (ring[a] - ring[0]).length_squared().total_cmp(&(ring[b] - ring[0]).length_squared())).unwrap();
    let mut keep = vec![false; n];
    keep[0] = true;
    keep[far] = true;
    let mut stack = vec![(0usize, far), (far, n)];
    while let Some((a, b)) = stack.pop() {
        let (pa, pb) = (ring[a], ring[b % n]);
        let dir = pb - pa;
        let len2 = dir.length_squared();
        let mut worst = (0.0, a);
        for (i, p) in ring.iter().enumerate().take(b).skip(a + 1) {
            let d = if len2 == 0.0 { (*p - pa).length() } else { ((*p - pa).cross(dir)).length() / len2.sqrt() };
            if d > worst.0 {
                worst = (d, i);
            }
        }
        if worst.0 > tol {
            keep[worst.1] = true;
            stack.push((a, worst.1));
            stack.push((worst.1, b));
        }
    }
    let out: Vec<DVec3> = ring.iter().zip(&keep).filter(|(_, k)| **k).map(|(p, _)| *p).collect();
    if out.len() >= 3 {
        out
    } else {
        ring.iter().step_by((n / 3).max(1)).take(3).copied().collect()
    }
}

pub fn wireframe(mesh: &Mesh, s: &WireframeSettings, ctl: &Control) -> Result<FabResult, FabError> {
    if mesh.is_empty() {
        return Err(FabError::Invalid("the mesh is empty".into()));
    }
    if !(s.spacing > 0.0 && s.strut_radius > 0.0 && s.simplify >= 0.0 && s.segments >= 6) {
        return Err(FabError::Invalid("spacing and strut radius must be positive, and at least 6 sides are needed".into()));
    }
    if !s.axes.iter().any(|&a| a) {
        return Err(FabError::Invalid("choose at least one axis".into()));
    }
    if s.spacing <= 2.0 * s.strut_radius {
        return Err(FabError::Invalid("the hoop spacing must exceed the strut diameter".into()));
    }
    let stats = mesh.stats();
    if !stats.is_watertight() {
        return Err(FabError::Invalid("the mesh must be watertight to take cross-sections; repair it first".into()));
    }
    let b = mesh.bounds().unwrap();
    let mut report = FabReport { method: "Wireframe".into(), source_volume: stats.signed_volume, ..Default::default() };
    report.max_deviation = Some(s.simplify + s.strut_radius);
    let mut tubes: Vec<(String, Mesh)> = Vec::new();
    let mut skipped = 0usize;
    for axis in (0..3).filter(|&a| s.axes[a]) {
        let (lo, hi) = (b.min[axis] as f64, b.max[axis] as f64);
        let n = (((hi - lo) / s.spacing).ceil() as usize).max(1);
        for k in 0..n {
            ctl.check()?;
            let pos = lo + (k as f64 + 0.5) * (hi - lo) / n as f64;
            let region =
                cross_section(mesh, &Plane::axis(axis, pos)).map_err(|e| FabError::Invalid(format!("section at {pos:.1} mm: {e}")))?;
            let frame = region.frame;
            for poly in &region.polygons {
                for ring in std::iter::once(&poly.outer).chain(poly.holes.iter()) {
                    let pts: Vec<DVec3> = ring.iter().map(|&q| frame.to_3d(q)).collect();
                    let pts = simplify_ring(&pts, s.simplify);
                    let perimeter: f64 = (0..pts.len()).map(|i| (pts[(i + 1) % pts.len()] - pts[i]).length()).sum();
                    if pts.len() < 3 || perimeter < 12.0 * s.strut_radius {
                        skipped += 1;
                        continue;
                    }
                    match tube(&pts, s.strut_radius, s.segments, true) {
                        Ok(m) => tubes.push((format!("{} hoop {}", ["X", "Y", "Z"][axis], k + 1), m)),
                        Err(_) => skipped += 1,
                    }
                }
            }
        }
    }
    if tubes.is_empty() {
        return Err(FabError::Invalid("no hoops could be built at this spacing; reduce the spacing or the strut radius".into()));
    }
    if skipped > 0 {
        report.notes.push(format!("{skipped} contour(s) were too small for the strut radius and were skipped"));
    }
    let mut parts: Vec<Part> = Vec::new();
    if s.merge && tubes.len() <= 150 {
        ctl.report(0.6, "Merging crossing hoops");
        let mut remaining: Vec<(String, Mesh)> = tubes;
        while let Some((label, first)) = remaining.pop() {
            let mut group = first;
            let mut names = vec![label];
            loop {
                ctl.check()?;
                let touching = remaining.iter().position(|(_, m)| {
                    let (a, c) = (group.bounds().unwrap(), m.bounds().unwrap());
                    a.min.cmple(c.max).all() && a.max.cmpge(c.min).all()
                });
                let Some(i) = touching else { break };
                let (l, m) = remaining.remove(i);
                match openslicer_geom::boolean::boolean(&group, &m, openslicer_geom::boolean::BoolOp::Union) {
                    Ok((u, _)) => {
                        group = u;
                        names.push(l);
                    }
                    Err(e) => {
                        report.notes.push(format!("{l} could not be merged ({e}); it stays a separate part"));
                        if let Some(piece) = Piece::new(m) {
                            parts.push(Part { piece, label: l, outline: None });
                        }
                    }
                }
            }
            if let Some(piece) = Piece::new(group) {
                parts.push(Part {
                    piece,
                    label: if names.len() == 1 { names[0].clone() } else { format!("Lattice of {} hoops", names.len()) },
                    outline: None,
                });
            }
        }
    } else {
        if s.merge {
            report.notes.push("too many hoops to merge; they are left as separate overlapping parts".into());
        }
        for (label, m) in tubes {
            if let Some(piece) = Piece::new(m) {
                parts.push(Part { piece, label, outline: None });
            }
        }
    }
    report.notes.push(format!("{} part(s) from {} axis section(s)", parts.len(), s.axes.iter().filter(|&&a| a).count()));
    let parts = ensure_fit(parts, s.usable, &mut report, ctl)?;
    Ok(FabResult::finish(parts, report))
}

#[cfg(test)]
mod tests {
    use super::*;
    use glam::Vec3;

    fn cylinder(r: f32, h: f32, n: usize) -> Mesh {
        openslicer_geom::build::cylinder(r as f64, 0.0, h as f64, 0.0, 0.0, n)
    }

    #[test]
    fn simplify_keeps_corners_and_drops_nearly_straight_points() {
        let ring: Vec<DVec3> = vec![(0.0, 0.0), (5.0, 0.01), (10.0, 0.0), (10.0, 5.0), (10.0, 10.0), (0.0, 10.0)]
            .into_iter()
            .map(|(x, y)| DVec3::new(x, y, 0.0))
            .collect();
        let s = simplify_ring(&ring, 0.1);
        assert_eq!(s.len(), 4, "{s:?}");
        assert_eq!(simplify_ring(&ring, 0.0).len(), 6);
        assert!(simplify_ring(&ring[..3], 5.0).len() == 3);
    }

    #[test]
    fn cylinder_gets_one_hoop_per_section_and_each_is_a_valid_tube() {
        let m = cylinder(25.0, 100.0, 96);
        let mut s = WireframeSettings::new(DVec3::splat(250.0));
        s.spacing = 25.0;
        let r = wireframe(&m, &s, &Control::none()).unwrap();
        assert!(r.report.is_valid(), "{:?}", r.report);
        assert_eq!(r.parts.len(), 4);
        // A hoop of radius ~25 with a 1.5 mm strut: volume ~ pi*1.5^2 * 2*pi*25.
        let expected = std::f64::consts::PI * 1.5 * 1.5 * std::f64::consts::TAU * 25.0;
        for p in &r.parts {
            assert!((p.piece.volume - expected).abs() / expected < 0.08, "{} vs {expected}", p.piece.volume);
        }
        assert!(r.report.fill_ratio < 0.2);
    }

    #[test]
    fn crossing_hoops_from_two_axes_merge_into_one_lattice() {
        let m = cylinder(25.0, 50.0, 96);
        let mut s = WireframeSettings::new(DVec3::splat(250.0));
        s.axes = [true, false, true];
        s.spacing = 30.0;
        s.merge = true;
        let r = wireframe(&m, &s, &Control::none()).unwrap();
        assert!(r.report.is_valid(), "{:?}", r.report);
        // Z hoops and X-section hoops (which are rectangles on a cylinder's side) cross: they form one lattice.
        assert!(r.parts.len() <= 2, "{}", r.parts.len());
        assert!(r.parts.iter().any(|p| p.label.starts_with("Lattice")), "{:?}", r.parts.iter().map(|p| &p.label).collect::<Vec<_>>());
        for p in &r.parts {
            assert!(p.piece.mesh.stats().is_watertight());
        }
    }

    #[test]
    fn hollow_shapes_get_inner_and_outer_hoops() {
        let tube_mesh = {
            // A hollow cylinder built from two cylinders is not a closed manifold without caps, so use a sphere shell.
            let (ro, ri, nu, nv) = (30.0f32, 24.0f32, 64usize, 32usize);
            let sph = |r: f32, flip: bool| {
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
                if flip {
                    for t in &mut tris {
                        t.swap(1, 2);
                    }
                }
                Mesh::new(verts, tris)
            };
            openslicer_geom::build::merge(&[sph(ro, false), sph(ri, true)])
        };
        let mut s = WireframeSettings::new(DVec3::splat(250.0));
        s.spacing = 20.0;
        let r = wireframe(&tube_mesh, &s, &Control::none()).unwrap();
        assert!(r.report.is_valid(), "{:?}", r.report);
        // Three mid-planes: each cuts an annulus -> an outer and an inner hoop.
        assert!(r.parts.len() >= 4 && r.parts.len() % 2 == 0, "{}", r.parts.len());
    }

    #[test]
    fn invalid_settings_are_rejected() {
        let m = cylinder(10.0, 20.0, 32);
        let usable = DVec3::splat(250.0);
        assert!(wireframe(&m, &WireframeSettings { axes: [false; 3], ..WireframeSettings::new(usable) }, &Control::none()).is_err());
        assert!(wireframe(&m, &WireframeSettings { spacing: 2.0, ..WireframeSettings::new(usable) }, &Control::none()).is_err());
        assert!(wireframe(&m, &WireframeSettings { strut_radius: 0.0, ..WireframeSettings::new(usable) }, &Control::none()).is_err());
        let mut open = m.clone();
        open.triangles.pop();
        assert!(wireframe(&open, &WireframeSettings::new(usable), &Control::none()).is_err());
    }
}
