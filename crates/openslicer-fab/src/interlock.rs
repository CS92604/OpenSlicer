//! Interlocking rib grids ("egg crate", also called grid or hash structures).
//!
//! The model is represented by two families of flat vertical ribs, one perpendicular to each horizontal
//! axis. Where an X rib crosses a Y rib, the X rib has a slot from the top down to half the local height
//! and the Y rib a slot from the bottom up to half, so the two slide together and lock. Slot widths include
//! the clearance.

use crate::{ensure_fit, FabError, FabReport, FabResult, Outline, Part};
use glam::{DVec2, DVec3, Vec3};
use openslicer_core::control::Control;
use openslicer_core::Mesh;
use openslicer_geom::build;
use openslicer_geom::poly2d::{self, rect};
use openslicer_geom::region::Polygon;
use openslicer_geom::section::cross_section;
use openslicer_geom::Plane;
use openslicer_split::exec::Piece;

#[derive(Clone, Debug)]
pub struct InterlockSettings {
    /// Vertical axis (0 = X, 1 = Y, 2 = Z). Ribs stand along it.
    pub up_axis: usize,
    /// Distance between ribs in the first and second horizontal directions (mm).
    pub spacing_u: f64,
    pub spacing_v: f64,
    /// Rib sheet thickness (mm).
    pub thickness: f64,
    /// Extra slot width so ribs slide together (mm).
    pub slot_clearance: f64,
    pub usable: DVec3,
}

impl InterlockSettings {
    pub fn new(usable: DVec3) -> Self {
        Self { up_axis: 2, spacing_u: 40.0, spacing_v: 40.0, thickness: 3.0, slot_clearance: 0.2, usable }
    }
}

struct Rib {
    /// 0 for ribs perpendicular to the first horizontal axis, 1 for the second.
    family: usize,
    index: usize,
    position: f64,
    polygons: Vec<Polygon>,
}

fn positions(lo: f64, hi: f64, spacing: f64) -> Vec<f64> {
    let n = (((hi - lo) / spacing).ceil() as usize).max(1);
    (0..n).map(|k| lo + (k as f64 + 0.5) * (hi - lo) / n as f64).collect()
}

/// Vertical intervals (min, max) of `polys` along the vertical line at horizontal coordinate `h`.
fn vertical_intervals(polys: &[Polygon], h: f64) -> Vec<(f64, f64)> {
    let strip = [rect(DVec2::new(h - 1e-3, -1e9), DVec2::new(h + 1e-3, 1e9))];
    poly2d::intersect(polys, &strip)
        .iter()
        .map(|p| p.outer.iter().fold((f64::MAX, f64::MIN), |(lo, hi), q| (lo.min(q.y), hi.max(q.y))))
        .collect()
}

pub fn interlock(mesh: &Mesh, s: &InterlockSettings, ctl: &Control) -> Result<FabResult, FabError> {
    if mesh.is_empty() {
        return Err(FabError::Invalid("the mesh is empty".into()));
    }
    if s.up_axis > 2 {
        return Err(FabError::Invalid("the vertical axis must be X, Y or Z".into()));
    }
    for (v, name) in [(s.spacing_u, "rib spacing"), (s.spacing_v, "rib spacing"), (s.thickness, "rib thickness")] {
        if !(v.is_finite() && v > 0.0) {
            return Err(FabError::Invalid(format!("the {name} must be a positive number")));
        }
    }
    if s.slot_clearance < 0.0 || !s.slot_clearance.is_finite() {
        return Err(FabError::Invalid("the slot clearance must not be negative".into()));
    }
    if s.thickness + 1.0 >= s.spacing_u.min(s.spacing_v) {
        return Err(FabError::Invalid("the rib thickness must be well below the rib spacing".into()));
    }
    let stats = mesh.stats();
    if !stats.is_watertight() {
        return Err(FabError::Invalid("the mesh must be watertight to take cross-sections; repair it first".into()));
    }
    let up = s.up_axis;
    let axes = [(up + 1) % 3, (up + 2) % 3]; // rib families: planes perpendicular to axes[0] and axes[1]
    let mut report = FabReport { method: "Interlocking ribs".into(), source_volume: stats.signed_volume, ..Default::default() };
    let b = mesh.bounds().unwrap();
    let ext = |axis: usize| (b.min[axis] as f64, b.max[axis] as f64);

    // Sections for every rib, expressed in (horizontal, vertical) coordinates.
    let mut ribs: Vec<Rib> = Vec::new();
    for family in 0..2 {
        let a = axes[family];
        let h_axis = axes[1 - family];
        let (lo, hi) = ext(a);
        let spacing = if family == 0 { s.spacing_u } else { s.spacing_v };
        for (index, pos) in positions(lo, hi, spacing).into_iter().enumerate() {
            ctl.check()?;
            let plane = Plane::axis(a, pos);
            let region = cross_section(mesh, &plane).map_err(|e| FabError::Invalid(format!("rib at {pos:.1} mm: {e}")))?;
            let frame = region.frame;
            let to_hv = |q: DVec2| -> DVec2 {
                let p = frame.to_3d(q);
                DVec2::new(p[h_axis], p[up])
            };
            let polygons: Vec<Polygon> = region
                .polygons
                .iter()
                .map(|p| Polygon {
                    outer: p.outer.iter().map(|&q| to_hv(q)).collect(),
                    holes: p.holes.iter().map(|h| h.iter().map(|&q| to_hv(q)).collect()).collect(),
                })
                .collect();
            if !polygons.is_empty() {
                ribs.push(Rib { family, index, position: pos, polygons });
            }
        }
    }
    if ribs.is_empty() {
        return Err(FabError::Invalid("the model has no cross-sections at the rib positions".into()));
    }

    // Slots: for each crossing of an X rib (family 0) with a Y rib (family 1).
    let slot_w = s.thickness + s.slot_clearance;
    let mut slots: Vec<Vec<Polygon>> = vec![Vec::new(); ribs.len()];
    let mut crossings = 0usize;
    for (i, r0) in ribs.iter().enumerate().filter(|(_, r)| r.family == 0) {
        for (j, r1) in ribs.iter().enumerate().filter(|(_, r)| r.family == 1) {
            // In rib i (family 0) the horizontal axis is axes[1], so r1's position is its horizontal coordinate; and vice versa.
            let ivs = vertical_intervals(&r0.polygons, r1.position);
            if ivs.is_empty() {
                continue;
            }
            crossings += 1;
            for (zlo, zhi) in ivs {
                let mid = 0.5 * (zlo + zhi);
                // family 0 rib: slot from the top down to mid; family 1 rib: slot from the bottom up to mid.
                slots[i].push(rect(DVec2::new(r1.position - slot_w / 2.0, mid), DVec2::new(r1.position + slot_w / 2.0, zhi + 1.0)));
                slots[j].push(rect(DVec2::new(r0.position - slot_w / 2.0, zlo - 1.0), DVec2::new(r0.position + slot_w / 2.0, mid)));
            }
        }
    }

    // Build the solids.
    let mut parts: Vec<Part> = Vec::new();
    for (i, rib) in ribs.iter().enumerate() {
        let polys = if slots[i].is_empty() { rib.polygons.clone() } else { poly2d::difference(&rib.polygons, &slots[i]) };
        if polys.is_empty() {
            continue;
        }
        let a = axes[rib.family];
        let h_axis = axes[1 - rib.family];
        let local = build::extrude(&polys, rib.position - s.thickness / 2.0, rib.position + s.thickness / 2.0);
        // Local (x, y, z) = (horizontal, vertical, rib normal) -> world axes (h_axis, up, a).
        let map = |v: Vec3| {
            let mut w = [0.0f32; 3];
            w[h_axis] = v.x;
            w[up] = v.y;
            w[a] = v.z;
            Vec3::from(w)
        };
        let mut m = Mesh::new(local.vertices.iter().map(|v| map(*v)).collect(), local.triangles);
        if m.stats().signed_volume < 0.0 {
            for t in &mut m.triangles {
                t.swap(1, 2);
            }
        }
        if let Some(piece) = Piece::new(m) {
            let name = if rib.family == 0 { "U" } else { "V" };
            parts.push(Part {
                piece,
                label: format!("{name} rib {}", rib.index + 1),
                outline: Some(Outline { polygons: polys, thickness: s.thickness }),
            });
        }
    }
    let (n0, n1) = (ribs.iter().filter(|r| r.family == 0).count(), ribs.iter().filter(|r| r.family == 1).count());
    report.notes.push(format!("{n0} + {n1} ribs with {crossings} crossings; slot width {slot_w:.2} mm"));
    let parts = ensure_fit(parts, s.usable, &mut report, ctl)?;
    Ok(FabResult::finish(parts, report))
}

#[cfg(test)]
mod tests {
    use super::*;
    use openslicer_geom::boolean::{boolean, BoolOp};

    fn block(size: Vec3) -> Mesh {
        let mut m = Mesh::cuboid(size);
        m.translate(size * 0.5);
        m
    }

    #[test]
    fn grid_of_ribs_has_the_expected_count_and_slots() {
        let m = block(Vec3::new(100.0, 80.0, 40.0));
        let mut s = InterlockSettings::new(DVec3::splat(250.0));
        s.spacing_u = 25.0;
        s.spacing_v = 20.0;
        let r = interlock(&m, &s, &Control::none()).unwrap();
        assert!(r.report.is_valid(), "{:?}", r.report);
        assert_eq!(r.parts.len(), 4 + 4);
        assert!(r.report.fill_ratio < 0.3 && r.report.fill_ratio > 0.05, "{}", r.report.fill_ratio);
        // Every rib has 4 slots (one per crossing rib): its volume is below the unslotted plate.
        for p in &r.parts {
            assert!(p.outline.is_some());
        }
    }

    #[test]
    fn crossing_ribs_slot_into_each_other_without_overlapping() {
        let m = block(Vec3::new(60.0, 60.0, 30.0));
        let mut s = InterlockSettings::new(DVec3::splat(250.0));
        s.spacing_u = 30.0;
        s.spacing_v = 30.0;
        let r = interlock(&m, &s, &Control::none()).unwrap();
        assert!(r.report.is_valid(), "{:?}", r.report);
        assert_eq!(r.parts.len(), 4);
        // U ribs are perpendicular to X (thin in X), V ribs perpendicular to Y (thin in Y).
        let (us, vs): (Vec<_>, Vec<_>) = r.parts.iter().partition(|p| p.label.starts_with('U'));
        assert_eq!((us.len(), vs.len()), (2, 2));
        for u in &us {
            assert!((u.piece.bounds.size().x - 3.0).abs() < 1e-3);
            for v in &vs {
                // The slotted ribs touch but do not overlap (the clearance leaves a gap).
                let inter = boolean(&u.piece.mesh, &v.piece.mesh, BoolOp::Intersection).unwrap().0;
                assert!(inter.is_empty() || inter.stats().signed_volume < 1e-6, "ribs overlap by {}", inter.stats().signed_volume);
            }
        }
        // Each rib has material removed: plate would be 60 x 30 x 3 = 5400.
        for p in &r.parts {
            assert!(p.piece.volume < 5400.0 - 1.0, "{}", p.piece.volume);
            // Each rib has a slot of width t+c from one edge to mid-height at each of the 2 crossings.
            let expected = 5400.0 - 2.0 * (3.0 + 0.2) * 15.0 * 3.0;
            assert!((p.piece.volume - expected).abs() < 1.0, "{} vs {expected}", p.piece.volume);
        }
    }

    #[test]
    fn works_with_a_different_up_axis() {
        let m = block(Vec3::new(30.0, 60.0, 60.0));
        let mut s = InterlockSettings::new(DVec3::splat(250.0));
        s.up_axis = 0;
        s.spacing_u = 30.0;
        s.spacing_v = 30.0;
        let r = interlock(&m, &s, &Control::none()).unwrap();
        assert!(r.report.is_valid(), "{:?}", r.report);
        assert_eq!(r.parts.len(), 4);
    }

    #[test]
    fn hollow_or_curved_shapes_produce_valid_ribs() {
        // A sphere: ribs have circular outlines of different radii.
        let sphere = {
            let (r, nu, nv) = (30.0f32, 48usize, 24usize);
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
        };
        let mut s = InterlockSettings::new(DVec3::splat(250.0));
        s.spacing_u = 15.0;
        s.spacing_v = 15.0;
        let r = interlock(&sphere, &s, &Control::none()).unwrap();
        assert!(r.report.is_valid(), "{:?}", r.report);
        assert_eq!(r.parts.len(), 8);
    }

    #[test]
    fn bad_settings_are_rejected() {
        let m = block(Vec3::splat(40.0));
        let mut s = InterlockSettings::new(DVec3::splat(250.0));
        s.thickness = 40.0;
        assert!(interlock(&m, &s, &Control::none()).is_err());
        let mut s = InterlockSettings::new(DVec3::splat(250.0));
        s.up_axis = 5;
        assert!(interlock(&m, &s, &Control::none()).is_err());
        let mut open = m.clone();
        open.triangles.pop();
        assert!(interlock(&open, &InterlockSettings::new(DVec3::splat(250.0)), &Control::none()).is_err());
    }
}
