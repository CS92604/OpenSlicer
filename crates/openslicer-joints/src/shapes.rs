//! Connector geometry for every kind.
//!
//! Point-based connectors are built around the local origin with +Z pointing from the male piece into the
//! female piece and the interface plane at `z = 0`. Region-based connectors (lip, lap, terrace) are built
//! directly in the plane's 2D coordinates with the same convention.
//!
//! Female pockets start slightly below the plane (`MOUTH`) so they open cleanly through the cut face.

use crate::settings::{ConnectorKind, ConnectorSettings};
use glam::{DVec2, Vec3};
use openslicer_core::Mesh;
use openslicer_geom::boolean::{boolean, BoolOp};
use openslicer_geom::build::{self, merge};
use openslicer_geom::poly2d::{self, circle, rect};
use openslicer_geom::region::Polygon;

/// How far pockets start below the interface plane so they open cleanly (mm).
pub const MOUTH: f64 = 0.5;

/// Geometry of one connector, in the local frame described in the module docs.
#[derive(Clone, Debug, Default)]
pub struct Tools {
    /// Unioned into the male piece.
    pub male: Option<Mesh>,
    /// Subtracted from the female piece.
    pub female: Option<Mesh>,
    /// Subtracted from the male piece (dowel sockets).
    pub male_pocket: Option<Mesh>,
    /// Loose parts such as dowels, positioned in the local frame.
    pub loose: Vec<Mesh>,
    /// Footprint of the male feature on the interface plane.
    pub footprint: Vec<Polygon>,
    /// Footprint of the female pocket grown by the required wall; must stay inside the interface region.
    pub guard: Vec<Polygon>,
    /// Guard height above the plane: material needed behind the pocket (pocket depth + wall).
    pub guard_depth: f64,
    /// True when the pocket is open at the side (dovetail groove, claw gallery): the guard may touch the region boundary.
    pub open_ended: bool,
    /// Extra shapes that must nest: (inner, outer) pairs checked for containment (male in female).
    pub fit_pairs: Vec<(Mesh, Mesh)>,
}

/// Unions small tool meshes into one valid solid (concatenating when they are disjoint).
pub fn combine(meshes: Vec<Mesh>) -> Result<Mesh, String> {
    let mut acc: Option<Mesh> = None;
    for m in meshes {
        acc = Some(match acc {
            None => m,
            Some(a) => {
                let (ab, mb) = (a.bounds().unwrap(), m.bounds().unwrap());
                let overlaps = ab.min.cmple(mb.max + Vec3::splat(1e-3)).all() && ab.max.cmpge(mb.min - Vec3::splat(1e-3)).all();
                if overlaps {
                    boolean(&a, &m, BoolOp::Union).map_err(|e| format!("could not combine connector parts: {e}"))?.0
                } else {
                    merge(&[a, m])
                }
            }
        });
    }
    acc.ok_or_else(|| "no geometry".to_string())
}

/// Prism whose profile is given in (y, z) and which extends along x from `x0` to `x1`.
fn prism_along_x(profile: &[DVec2], x0: f64, x1: f64) -> Mesh {
    let m = build::extrude(&[Polygon { outer: profile.to_vec(), holes: vec![] }], x0, x1);
    // Local coordinates of `extrude` are (a, b, c) = (y, z, x); a cyclic permutation keeps orientation.
    Mesh::new(m.vertices.iter().map(|v| Vec3::new(v.z, v.x, v.y)).collect(), m.triangles)
}

fn square(half: f64) -> Vec<DVec2> {
    vec![DVec2::new(-half, -half), DVec2::new(half, -half), DVec2::new(half, half), DVec2::new(-half, half)]
}

fn rect_poly(cx: f64, cy: f64, hx: f64, hy: f64) -> Polygon {
    rect(DVec2::new(cx - hx, cy - hy), DVec2::new(cx + hx, cy + hy))
}

fn check_solid(name: &str, m: &Mesh) -> Result<(), String> {
    let s = m.stats();
    if s.is_watertight() && s.signed_volume > 0.0 {
        Ok(())
    } else {
        Err(format!("the generated {name} is not a valid solid ({} open edges, volume {:.4} mm³)", s.boundary_edges, s.signed_volume))
    }
}

/// Builds a point-based connector. `exit` is the distance from the connector centre to the open end of a
/// dovetail groove / claw gallery along local +X (used only by those kinds).
pub fn point_tools(s: &ConnectorSettings, exit: f64) -> Result<Tools, String> {
    s.validate()?;
    let (w, h, c, cz) = (s.size, s.height, s.clearance, s.clearance_z);
    let seg = s.segments;
    let mut t = Tools { guard_depth: h + cz + s.min_wall, ..Default::default() };
    match s.kind {
        ConnectorKind::Pin => {
            let r = w / 2.0;
            t.male = Some(build::cylinder(r, 0.0, h, 0.0, s.chamfer, seg));
            t.female = Some(build::cylinder(r + c, -MOUTH, h + cz, 0.0, 0.0, seg));
            t.footprint = vec![Polygon { outer: circle(DVec2::ZERO, r, seg), holes: vec![] }];
            t.guard = vec![Polygon { outer: circle(DVec2::ZERO, r + c + s.min_wall, seg), holes: vec![] }];
        }
        ConnectorKind::Dowel => {
            let r = w / 2.0;
            // Sockets of depth `h` in both pieces; the dowel is shorter by the depth clearance on each end.
            let half = h - cz.max(0.2);
            if half <= 0.0 {
                return Err("the dowel depth is too small for the depth clearance".into());
            }
            t.female = Some(build::cylinder(r + c, -MOUTH, h, 0.0, 0.0, seg));
            t.male_pocket = Some(build::cylinder(r + c, -h, MOUTH, 0.0, 0.0, seg));
            let dowel = build::cylinder(r, -half, half, s.chamfer, s.chamfer, seg);
            t.loose = vec![dowel];
            t.footprint = vec![Polygon { outer: circle(DVec2::ZERO, r, seg), holes: vec![] }];
            t.guard = vec![Polygon { outer: circle(DVec2::ZERO, r + c + s.min_wall, seg), holes: vec![] }];
            t.guard_depth = h + s.min_wall;
        }
        ConnectorKind::TabSlot => {
            let l = s.length_or_auto();
            t.male = Some(build::cuboid(glam::DVec3::new(-l / 2.0, -w / 2.0, 0.0), glam::DVec3::new(l / 2.0, w / 2.0, h)));
            t.female = Some(build::cuboid(
                glam::DVec3::new(-l / 2.0 - c, -w / 2.0 - c, -MOUTH),
                glam::DVec3::new(l / 2.0 + c, w / 2.0 + c, h + cz),
            ));
            t.footprint = vec![rect_poly(0.0, 0.0, l / 2.0, w / 2.0)];
            t.guard = vec![rect_poly(0.0, 0.0, l / 2.0 + c + s.min_wall, w / 2.0 + c + s.min_wall)];
        }
        ConnectorKind::Pyramid => {
            let (half, top) = (w / 2.0, w / 2.0 * (1.0 - s.taper));
            t.male = Some(build::loft(&square(half), &square(top), 0.0, h));
            let k = (half - top) / h; // inward slope per mm of height
            if top * 2.0 < s.min_feature {
                return Err(format!(
                    "the pyramid tip would be {:.2} mm wide, below the minimum feature size of {:.2} mm; reduce the taper or increase the size",
                    top * 2.0,
                    s.min_feature
                ));
            }
            let (b, tp) = (half + c + k * MOUTH, half + c - k * (h + cz));
            t.female = Some(build::loft(&square(b), &square(tp), -MOUTH, h + cz));
            t.footprint = vec![Polygon { outer: square(half), holes: vec![] }];
            t.guard = vec![Polygon { outer: square(half + c + s.min_wall), holes: vec![] }];
        }
        ConnectorKind::Puzzle => {
            let rh = w / 2.0;
            let neck_w = (rh * 0.7).max(s.min_feature);
            let shape = poly2d::union(
                &[Polygon { outer: circle(DVec2::ZERO, rh, seg), holes: vec![] }],
                &[rect(DVec2::new(-rh * 1.75, -neck_w / 2.0), DVec2::new(0.0, neck_w / 2.0))],
            );
            if shape.len() != 1 {
                return Err("the puzzle knob outline is invalid".into());
            }
            let grown = poly2d::offset(&shape, c, 0.2);
            t.male = Some(build::extrude(&shape, 0.0, h));
            t.female = Some(build::extrude(&grown, -MOUTH, h + cz));
            t.footprint = shape.clone();
            t.guard = poly2d::offset(&shape, c + s.min_wall, 0.2);
        }
        ConnectorKind::Dovetail => {
            let l = s.length_or_auto();
            let k = s.flank_angle.to_radians().tan();
            let wt = w / 2.0;
            let wb = wt - k * h; // half-width at the base (narrower than the top: the undercut)
            if wb < s.min_feature / 2.0 {
                return Err(format!(
                    "the dovetail neck would be {:.2} mm wide; increase the width, lower the height or reduce the flank angle",
                    wb * 2.0
                ));
            }
            let prof = |z0: f64, z1: f64, grow: f64| -> Vec<DVec2> {
                // half-width at z is wb + k*z (+ clearance measured perpendicular to the flank)
                let g = grow / s.flank_angle.to_radians().cos();
                vec![
                    DVec2::new(-(wb + k * z0 + g), z0),
                    DVec2::new(wb + k * z0 + g, z0),
                    DVec2::new(wt + k * (z1 - h) + g, z1),
                    DVec2::new(-(wt + k * (z1 - h) + g), z1),
                ]
            };
            t.male = Some(prism_along_x(&prof(0.0, h, 0.0), -l / 2.0, l / 2.0));
            let x_end = exit.max(l / 2.0 + c + s.min_wall) + 1.0;
            t.female = Some(prism_along_x(&prof(-MOUTH, h + cz, c), -l / 2.0 - c, x_end));
            t.footprint = vec![rect_poly(0.0, 0.0, l / 2.0, wt)];
            let gw = wt + k * cz + c / s.flank_angle.to_radians().cos() + s.min_wall;
            t.guard = vec![rect(DVec2::new(-l / 2.0 - c - s.min_wall, -gw), DVec2::new(l / 2.0 + c + s.min_wall, gw))];
            t.open_ended = true;
        }
        ConnectorKind::Claw => {
            let ts = (w / 2.0).max(s.min_feature);
            let hk = (h / 3.0).clamp(s.min_feature, 3.0);
            if hk >= h {
                return Err("the claw is too short for its hook".into());
            }
            let ov = s.overhang;
            let b = glam::DVec3::new;
            let stem = build::cuboid(b(-ts / 2.0, -w / 2.0, 0.0), b(ts / 2.0, w / 2.0, h));
            let hook = build::cuboid(b(ts / 2.0, -w / 2.0, h - hk), b(ts / 2.0 + ov, w / 2.0, h));
            let male = combine(vec![stem, hook])?;
            let sl = s.slide;
            // Shaft swept by the stem, plus a gallery for the hook, both with clearance.
            let shaft = build::cuboid(b(-sl - ts / 2.0 - c, -w / 2.0 - c, -MOUTH), b(ts / 2.0 + c, w / 2.0 + c, h + cz));
            let gallery = build::cuboid(b(-sl + ts / 2.0 - c, -w / 2.0 - c, h - hk - c), b(ts / 2.0 + ov + c, w / 2.0 + c, h + cz));
            let female = combine(vec![shaft, gallery])?;
            // The claw is inserted `slide` mm towards -X and then slid into place.
            let inserted =
                Mesh::new(male.vertices.iter().map(|v| *v + Vec3::new(-(sl as f32), 0.0, 0.0)).collect(), male.triangles.clone());
            t.fit_pairs = vec![(male.clone(), female.clone()), (inserted, female.clone())];
            t.male = Some(male);
            t.female = Some(female);
            let (x0, x1) = (-sl - ts / 2.0 - c - s.min_wall, ts / 2.0 + ov + c + s.min_wall);
            let gw = w / 2.0 + c + s.min_wall;
            t.footprint = vec![rect(DVec2::new(-ts / 2.0, -w / 2.0), DVec2::new(ts / 2.0 + ov, w / 2.0))];
            t.guard = vec![rect(DVec2::new(x0, -gw), DVec2::new(x1, gw))];
            let _ = exit;
        }
        ConnectorKind::Lip | ConnectorKind::LapJoint | ConnectorKind::Terrace => {
            return Err(format!("{} spans the whole interface; use region_tools", s.kind.label()));
        }
        ConnectorKind::Comb => {
            return Err("finger joints are cut from the pieces themselves; see the comb module".into());
        }
    }
    if t.fit_pairs.is_empty() {
        if let (Some(m), Some(f)) = (&t.male, &t.female) {
            t.fit_pairs.push((m.clone(), f.clone()));
        }
    }
    for (name, m) in [("male part", &t.male), ("socket", &t.female), ("pocket", &t.male_pocket)] {
        if let Some(m) = m {
            check_solid(name, m)?;
        }
    }
    for l in &t.loose {
        check_solid("dowel", l)?;
    }
    Ok(t)
}

/// Builds a region-spanning connector (lip, lap, terrace). `angle` is the direction (radians) of the lap's
/// dividing normal / the terrace's rising direction.
pub fn region_tools(s: &ConnectorSettings, region: &[Polygon], angle: f64, inradius: f64) -> Result<Tools, String> {
    s.validate()?;
    let (h, c, cz) = (s.height, s.clearance, s.clearance_z);
    let mut t = Tools { guard_depth: h + cz + s.min_wall, ..Default::default() };
    match s.kind {
        ConnectorKind::Lip => {
            let wall = 2.0 * inradius;
            let width = if s.lip_width > 0.0 { s.lip_width } else { (wall / 3.0).clamp(s.min_feature, 3.0) };
            let inset = if s.lip_inset > 0.0 { s.lip_inset } else { ((wall - width) / 2.0).min((2.0 * width).max(3.0)) };
            if inset < c + s.min_wall - 1e-9 {
                return Err(format!(
                    "the wall is {:.1} mm thick: a {:.1} mm lip needs at least {:.1} mm (lip + 2 × clearance + 2 × minimum wall). Reduce the minimum wall or the lip width, or use another connector",
                    wall,
                    width,
                    width + 2.0 * (c + s.min_wall)
                ));
            }
            let outer = poly2d::offset(region, -inset, 0.1);
            let inner = poly2d::offset(region, -(inset + width), 0.1);
            let band = if inner.is_empty() { outer } else { poly2d::difference(&outer, &inner) };
            if band.is_empty() || poly2d::area(&band) < 1e-3 {
                return Err("the interface is too narrow for a lip".into());
            }
            let groove = poly2d::offset(&band, c, 0.2);
            t.male = Some(build::extrude(&band, 0.0, h));
            t.female = Some(build::extrude(&groove, -MOUTH, h + cz));
            t.footprint = band.clone();
            t.guard = poly2d::offset(&band, c + s.min_wall, 0.2);
        }
        ConnectorKind::LapJoint => {
            let half = half_plane(region, angle, 0.0);
            let half_region = poly2d::intersect(region, &half);
            if half_region.is_empty() {
                return Err("the interface cannot be split for a lap joint".into());
            }
            let pocket = poly2d::offset(&half_region, c, 0.2);
            t.male = Some(build::extrude(&half_region, 0.0, h));
            t.female = Some(build::extrude(&pocket, -MOUTH, h + cz));
            t.footprint = half_region;
            t.open_ended = true;
        }
        ConnectorKind::Terrace => {
            let (lo, hi) = extent_along(region, angle);
            let n = s.steps;
            let step_w = (hi - lo) / n as f64;
            if step_w < s.min_feature * 2.0 {
                return Err(format!("each terrace step would be {:.1} mm wide; use fewer steps or a larger interface", step_w));
            }
            let (mut males, mut females) = (Vec::new(), Vec::new());
            for k in 1..n {
                let strip = strip_polygon(
                    region,
                    angle,
                    lo + step_w * k as f64,
                    lo + step_w * (k as f64 + 1.0) + if k + 1 == n { 1.0 } else { 0.0 },
                );
                let part = poly2d::intersect(region, &strip);
                if part.is_empty() {
                    continue;
                }
                let height = h * k as f64;
                males.push(build::extrude(&part, 0.0, height));
                // The pocket strip grows by the clearance along the rising direction so the risers clear each other.
                let wide = strip_polygon(
                    region,
                    angle,
                    lo + step_w * k as f64 - c,
                    lo + step_w * (k as f64 + 1.0) + if k + 1 == n { 1.0 } else { 0.0 },
                );
                let pocket = poly2d::intersect(&poly2d::offset(region, c, 0.2), &wide);
                females.push(build::extrude(&pocket, -MOUTH, height + cz));
                t.footprint.extend(part);
            }
            if males.is_empty() {
                return Err("the terrace has no raised steps on this interface".into());
            }
            t.guard_depth = h * (n - 1) as f64 + cz + s.min_wall;
            t.male = Some(combine(males)?);
            t.female = Some(combine(females)?);
            t.open_ended = true;
        }
        _ => return Err(format!("{} is placed at points; use point_tools", s.kind.label())),
    }
    if let (Some(m), Some(f)) = (&t.male, &t.female) {
        t.fit_pairs.push((m.clone(), f.clone()));
    }
    for (name, m) in [("male part", &t.male), ("pocket", &t.female)] {
        if let Some(m) = m {
            check_solid(name, m)?;
        }
    }
    Ok(t)
}

/// Large rectangle covering the side `x' > offset` of the line through the origin whose normal has direction `angle`.
fn half_plane(region: &[Polygon], angle: f64, offset: f64) -> Vec<Polygon> {
    let (lo, hi) = bounds(region);
    let big = (hi - lo).length() * 2.0 + 10.0;
    let centre = (lo + hi) * 0.5;
    let (d, p) = (DVec2::new(angle.cos(), angle.sin()), DVec2::new(-angle.sin(), angle.cos()));
    let base = centre + d * offset;
    vec![Polygon { outer: vec![base - p * big, base + d * big - p * big, base + d * big + p * big, base + p * big], holes: vec![] }]
}

pub fn strip_polygon(region: &[Polygon], angle: f64, a: f64, b: f64) -> Vec<Polygon> {
    let (lo, hi) = bounds(region);
    let big = (hi - lo).length() * 2.0 + 10.0;
    let (d, p) = (DVec2::new(angle.cos(), angle.sin()), DVec2::new(-angle.sin(), angle.cos()));
    let (pa, pb) = (d * a, d * b);
    vec![Polygon { outer: vec![pa - p * big, pb - p * big, pb + p * big, pa + p * big], holes: vec![] }]
}

fn bounds(region: &[Polygon]) -> (DVec2, DVec2) {
    let (mut lo, mut hi) = (DVec2::splat(f64::MAX), DVec2::splat(f64::MIN));
    for p in region.iter().flat_map(|p| p.outer.iter()) {
        lo = lo.min(*p);
        hi = hi.max(*p);
    }
    (lo, hi)
}

/// (min, max) of the region's coordinate along direction `angle`.
pub fn extent_along(region: &[Polygon], angle: f64) -> (f64, f64) {
    let d = DVec2::new(angle.cos(), angle.sin());
    region.iter().flat_map(|p| p.outer.iter()).fold((f64::MAX, f64::MIN), |(lo, hi), p| (lo.min(p.dot(d)), hi.max(p.dot(d))))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::settings::ConnectorKind::*;

    fn fits(inner: &Mesh, outer: &Mesh) -> bool {
        match boolean(inner, outer, BoolOp::Difference) {
            Ok((m, _)) => m.is_empty() || m.stats().signed_volume < 1e-6 * inner.stats().signed_volume,
            Err(_) => false,
        }
    }

    #[test]
    fn every_point_connector_builds_valid_solids_and_the_male_fits_the_female() {
        for kind in [Pin, Dowel, Dovetail, TabSlot, Puzzle, Pyramid, Claw] {
            let s = ConnectorSettings::new(kind);
            let t = point_tools(&s, 30.0).unwrap_or_else(|e| panic!("{kind:?}: {e}"));
            assert!(!t.fit_pairs.is_empty() || kind == Dowel, "{kind:?}");
            for (inner, outer) in &t.fit_pairs {
                assert!(fits(inner, outer), "{kind:?}: the male part does not fit its pocket");
            }
            assert!(!t.footprint.is_empty() && !t.guard.is_empty(), "{kind:?}");
            // Clearance is real: the pocket is strictly larger than the male part.
            if let (Some(m), Some(f)) = (&t.male, &t.female) {
                assert!(f.stats().signed_volume > m.stats().signed_volume, "{kind:?}");
            }
        }
    }

    #[test]
    fn dowel_is_shorter_than_its_two_sockets_and_fits_both() {
        let s = ConnectorSettings::new(Dowel);
        let t = point_tools(&s, 0.0).unwrap();
        let dowel = &t.loose[0];
        let (a, b) = (t.male_pocket.as_ref().unwrap(), t.female.as_ref().unwrap());
        let db = dowel.bounds().unwrap();
        assert!((db.size().z as f64) < 2.0 * s.height, "{db:?}");
        // Dowel half-lengths are on both sides of the plane and inside the sockets.
        assert!(fits(dowel, &combine(vec![a.clone(), b.clone()]).unwrap()));
    }

    #[test]
    fn pin_clearance_is_the_requested_radial_gap() {
        let mut s = ConnectorSettings::new(Pin);
        s.clearance = 0.3;
        let t = point_tools(&s, 0.0).unwrap();
        let (m, f) = (t.male.unwrap().bounds().unwrap(), t.female.unwrap().bounds().unwrap());
        assert!(((f.size().x - m.size().x) / 2.0 - 0.3).abs() < 1e-3);
    }

    #[test]
    fn dovetail_has_an_undercut_and_an_open_groove() {
        let s = ConnectorSettings::new(Dovetail);
        let t = point_tools(&s, 25.0).unwrap();
        let male = t.male.unwrap();
        // Wider at the top than at the base: compare extents at the two ends in z.
        let width_at = |zmax: bool| {
            let zs: Vec<f32> = male.vertices.iter().map(|v| v.z).collect();
            let target = if zmax { zs.iter().cloned().fold(f32::MIN, f32::max) } else { zs.iter().cloned().fold(f32::MAX, f32::min) };
            let ys: Vec<f32> = male.vertices.iter().filter(|v| (v.z - target).abs() < 1e-5).map(|v| v.y).collect();
            ys.iter().cloned().fold(f32::MIN, f32::max) - ys.iter().cloned().fold(f32::MAX, f32::min)
        };
        assert!(width_at(true) > width_at(false) + 1.0, "top {} base {}", width_at(true), width_at(false));
        assert!(t.open_ended);
        let groove = t.female.unwrap().bounds().unwrap();
        assert!(groove.max.x >= 25.0, "the groove runs out to the boundary: {groove:?}");
    }

    #[test]
    fn impossible_dovetail_and_pyramid_are_rejected_with_advice() {
        let mut s = ConnectorSettings::new(Dovetail);
        s.size = 5.0;
        s.height = 10.0;
        let e = point_tools(&s, 20.0).unwrap_err();
        assert!(e.contains("neck"), "{e}");
        let mut s = ConnectorSettings::new(Pyramid);
        s.size = 6.0;
        s.taper = 0.85; // tip 0.9 mm wide, below the 1.5 mm minimum feature
        assert!(point_tools(&s, 0.0).unwrap_err().contains("pyramid tip"));
    }

    #[test]
    fn claw_fits_in_both_the_inserted_and_the_locked_position() {
        let s = ConnectorSettings::new(Claw);
        let t = point_tools(&s, 0.0).unwrap();
        assert_eq!(t.fit_pairs.len(), 2);
        for (inner, outer) in &t.fit_pairs {
            assert!(fits(inner, outer));
        }
    }

    #[test]
    fn lip_follows_a_ring_shaped_interface_and_the_groove_fits_it() {
        // A 5 mm wide ring (thin wall): a 1.5 mm lip is centred in it.
        let ring = poly2d::difference(
            &[Polygon { outer: circle(DVec2::ZERO, 50.0, 128), holes: vec![] }],
            &[Polygon { outer: circle(DVec2::ZERO, 44.0, 128), holes: vec![] }],
        );
        let mut s = ConnectorSettings::new(Lip);
        s.min_wall = 0.9;
        s.lip_width = 1.5;
        let t = region_tools(&s, &ring, 0.0, 3.0).unwrap();
        let male = t.male.as_ref().unwrap();
        // The lip is a ring of radius ~47 (middle of the wall).
        let radii: Vec<f32> = male.vertices.iter().map(|v| (v.x * v.x + v.y * v.y).sqrt()).collect();
        let (lo, hi) = (radii.iter().cloned().fold(f32::MAX, f32::min), radii.iter().cloned().fold(f32::MIN, f32::max));
        assert!(lo > 45.5 && hi < 48.5, "lip radii {lo}..{hi}");
        assert!(fits(male, t.female.as_ref().unwrap()));
    }

    #[test]
    fn lip_on_a_wall_that_is_too_thin_explains_why() {
        let thin = vec![rect(DVec2::new(0.0, 0.0), DVec2::new(100.0, 2.5))];
        let s = ConnectorSettings::new(Lip);
        let e = region_tools(&s, &thin, 0.0, 1.25).unwrap_err();
        assert!(e.contains("wall is 2.5 mm"), "{e}");
    }

    #[test]
    fn lap_joint_covers_half_the_interface() {
        let sq = vec![rect(DVec2::new(0.0, 0.0), DVec2::new(40.0, 40.0))];
        let s = ConnectorSettings::new(LapJoint);
        let t = region_tools(&s, &sq, 0.0, 20.0).unwrap();
        let area = poly2d::area(&t.footprint);
        assert!((area - 800.0).abs() < 1.0, "{area}");
        assert!(fits(t.male.as_ref().unwrap(), t.female.as_ref().unwrap()));
    }

    #[test]
    fn terrace_has_steps_of_increasing_height() {
        let sq = vec![rect(DVec2::new(0.0, 0.0), DVec2::new(60.0, 30.0))];
        let s = ConnectorSettings::new(Terrace); // 3 steps, 2 mm per step
        let t = region_tools(&s, &sq, 0.0, 15.0).unwrap();
        let male = t.male.unwrap();
        let tallest = male.bounds().unwrap().max.z;
        assert!((tallest - 4.0).abs() < 1e-4, "two raised steps of 2 and 4 mm: {tallest}");
        assert!(fits(&male, t.female.as_ref().unwrap()));
        let mut narrow = ConnectorSettings::new(Terrace);
        narrow.steps = 8;
        assert!(region_tools(&narrow, &[rect(DVec2::ZERO, DVec2::new(20.0, 10.0))], 0.0, 5.0).is_err());
    }
}
