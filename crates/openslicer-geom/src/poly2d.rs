//! 2D polygon Booleans and offsetting (planar regions with holes).
//!
//! Backed by `i_overlay`, which snaps to an integer grid internally. Use these
//! for derived shapes (connector footprints, offsets, stacking contours). Do not
//! use them to produce cap geometry that must share exact vertices with a mesh.

use crate::region::{ring_area, Polygon};
use glam::DVec2;
use i_overlay::core::fill_rule::FillRule;
use i_overlay::core::overlay_rule::OverlayRule;
use i_overlay::float::single::SingleFloatOverlay;
use i_overlay::mesh::float::outline::offset::OutlineOffset;
use i_overlay::mesh::float::style::{LineJoin, OutlineStyle};

type Contours = Vec<Vec<[f64; 2]>>;

fn ring_to_pts(r: &[DVec2]) -> Vec<[f64; 2]> {
    r.iter().map(|p| [p.x, p.y]).collect()
}

/// All rings of the polygons as a flat contour list (outers CCW, holes CW), for non-zero fill.
fn to_contours(polys: &[Polygon]) -> Contours {
    let mut out = Vec::new();
    for p in polys {
        let mut outer = p.outer.clone();
        if ring_area(&outer) < 0.0 {
            outer.reverse();
        }
        out.push(ring_to_pts(&outer));
        for h in &p.holes {
            let mut h = h.clone();
            if ring_area(&h) > 0.0 {
                h.reverse();
            }
            out.push(ring_to_pts(&h));
        }
    }
    out
}

fn from_shapes(shapes: Vec<Contours>) -> Vec<Polygon> {
    let to_ring = |c: &Vec<[f64; 2]>| -> Vec<DVec2> { c.iter().map(|p| DVec2::new(p[0], p[1])).collect() };
    shapes
        .into_iter()
        .filter(|s| !s.is_empty() && s[0].len() >= 3)
        .map(|s| {
            let mut outer = to_ring(&s[0]);
            if ring_area(&outer) < 0.0 {
                outer.reverse();
            }
            let holes = s[1..]
                .iter()
                .filter(|h| h.len() >= 3)
                .map(|h| {
                    let mut r = to_ring(h);
                    if ring_area(&r) > 0.0 {
                        r.reverse();
                    }
                    r
                })
                .collect();
            Polygon { outer, holes }
        })
        .collect()
}

fn overlay(a: &[Polygon], b: &[Polygon], rule: OverlayRule) -> Vec<Polygon> {
    let (sa, sb) = (to_contours(a), to_contours(b));
    from_shapes(sa.overlay(&sb, rule, FillRule::NonZero))
}

pub fn intersect(a: &[Polygon], b: &[Polygon]) -> Vec<Polygon> {
    overlay(a, b, OverlayRule::Intersect)
}

pub fn union(a: &[Polygon], b: &[Polygon]) -> Vec<Polygon> {
    overlay(a, b, OverlayRule::Union)
}

pub fn difference(a: &[Polygon], b: &[Polygon]) -> Vec<Polygon> {
    overlay(a, b, OverlayRule::Difference)
}

/// Union of all polygons in `polys` (merging any that overlap or touch).
pub fn union_all(polys: &[Polygon]) -> Vec<Polygon> {
    let subject = to_contours(polys);
    let empty: Contours = Vec::new();
    from_shapes(subject.overlay(&empty, OverlayRule::Union, FillRule::NonZero))
}

pub fn area(polys: &[Polygon]) -> f64 {
    polys.iter().map(Polygon::area).sum()
}

/// Offsets polygons by `delta` (positive grows, negative shrinks) with rounded corners.
/// `arc_step` is the maximum angular step in radians used to approximate rounded corners
/// (the library clamps it to 0.03..0.79; 0.1 is about 6 degrees per segment).
pub fn offset(polys: &[Polygon], delta: f64, arc_step: f64) -> Vec<Polygon> {
    if polys.is_empty() {
        return Vec::new();
    }
    if delta == 0.0 {
        return polys.to_vec();
    }
    let mut out = Vec::new();
    for p in polys {
        let shape = to_contours(std::slice::from_ref(p));
        let style = OutlineStyle::new(delta).line_join(LineJoin::Round(arc_step));
        out.extend(from_shapes(shape.outline(&style)));
    }
    out
}

/// Approximates a circle with `segments` vertices (counter-clockwise).
pub fn circle(center: DVec2, radius: f64, segments: usize) -> Vec<DVec2> {
    (0..segments)
        .map(|k| {
            let a = std::f64::consts::TAU * k as f64 / segments as f64;
            center + DVec2::new(a.cos(), a.sin()) * radius
        })
        .collect()
}

pub fn rect(min: DVec2, max: DVec2) -> Polygon {
    Polygon { outer: vec![min, DVec2::new(max.x, min.y), max, DVec2::new(min.x, max.y)], holes: vec![] }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn square(x: f64, y: f64, s: f64) -> Polygon {
        rect(DVec2::new(x, y), DVec2::new(x + s, y + s))
    }

    #[test]
    fn boolean_areas() {
        let (a, b) = ([square(0.0, 0.0, 10.0)], [square(5.0, 5.0, 10.0)]);
        assert!((area(&intersect(&a, &b)) - 25.0).abs() < 1e-6);
        assert!((area(&union(&a, &b)) - 175.0).abs() < 1e-6);
        assert!((area(&difference(&a, &b)) - 75.0).abs() < 1e-6);
    }

    #[test]
    fn union_all_merges_overlapping_and_touching_polygons() {
        let tiles = [square(0.0, 0.0, 10.0), square(10.0, 0.0, 10.0), square(5.0, 5.0, 10.0)];
        let u = union_all(&tiles);
        assert_eq!(u.len(), 1);
        assert!((area(&u) - (100.0 + 100.0 + 100.0 - 25.0 - 25.0)).abs() < 1e-6);
        assert!(union_all(&[]).is_empty());
    }

    #[test]
    fn difference_creates_a_hole_with_clockwise_orientation() {
        let r = difference(&[square(0.0, 0.0, 10.0)], &[square(3.0, 3.0, 4.0)]);
        assert_eq!(r.len(), 1);
        assert_eq!(r[0].holes.len(), 1);
        assert!(ring_area(&r[0].outer) > 0.0 && ring_area(&r[0].holes[0]) < 0.0);
        assert!((area(&r) - 84.0).abs() < 1e-6);
    }

    #[test]
    fn offset_grows_and_shrinks_with_the_expected_sign() {
        let sq = [square(0.0, 0.0, 10.0)];
        let grown = area(&offset(&sq, 1.0, 0.05));
        // 10x10 grown by 1 with round corners: 100 + 4*10*1 + pi*1^2.
        assert!((grown - (100.0 + 40.0 + std::f64::consts::PI)).abs() < 0.2, "{grown}");
        let shrunk = area(&offset(&sq, -1.0, 0.05));
        assert!((shrunk - 64.0).abs() < 0.2, "{shrunk}");
        assert!(offset(&sq, -6.0, 0.05).is_empty(), "shrinking past the inradius must vanish");
    }

    #[test]
    fn shrinking_a_ring_keeps_its_hole_and_thins_both_walls() {
        let ring = difference(&[square(0.0, 0.0, 20.0)], &[square(5.0, 5.0, 10.0)]); // wall thickness 5
        let thin = offset(&ring, -1.0, 0.05);
        assert!((area(&thin) - (18.0 * 18.0 - 12.0 * 12.0)).abs() < 1.0, "{}", area(&thin));
    }
}
