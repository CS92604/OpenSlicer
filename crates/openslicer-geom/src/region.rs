//! Planar regions: polygons with holes expressed in a plane's 2D frame.

use crate::plane::{Frame, Plane};
use glam::DVec2;

/// A simple polygon with holes. The outer ring is counter-clockwise, holes clockwise.
#[derive(Clone, Debug, PartialEq)]
pub struct Polygon {
    pub outer: Vec<DVec2>,
    pub holes: Vec<Vec<DVec2>>,
}

/// A set of polygons lying in a plane.
#[derive(Clone, Debug)]
pub struct PlanarRegion {
    pub plane: Plane,
    pub frame: Frame,
    pub polygons: Vec<Polygon>,
}

/// Signed area of a ring (positive when counter-clockwise).
pub fn ring_area(ring: &[DVec2]) -> f64 {
    let n = ring.len();
    (0..n).map(|i| ring[i].perp_dot(ring[(i + 1) % n])).sum::<f64>() * 0.5
}

/// Point-in-ring test (even-odd, ray casting).
pub fn ring_contains(ring: &[DVec2], p: DVec2) -> bool {
    let mut inside = false;
    let n = ring.len();
    let mut j = n - 1;
    for i in 0..n {
        let (a, b) = (ring[i], ring[j]);
        if (a.y > p.y) != (b.y > p.y) && p.x < (b.x - a.x) * (p.y - a.y) / (b.y - a.y) + a.x {
            inside = !inside;
        }
        j = i;
    }
    inside
}

impl Polygon {
    pub fn area(&self) -> f64 {
        ring_area(&self.outer).abs() - self.holes.iter().map(|h| ring_area(h).abs()).sum::<f64>()
    }
}

impl PlanarRegion {
    pub fn area(&self) -> f64 {
        self.polygons.iter().map(Polygon::area).sum()
    }

    pub fn is_empty(&self) -> bool {
        self.polygons.is_empty()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn square(x: f64, y: f64, s: f64) -> Vec<DVec2> {
        vec![DVec2::new(x, y), DVec2::new(x + s, y), DVec2::new(x + s, y + s), DVec2::new(x, y + s)]
    }

    #[test]
    fn area_and_containment() {
        let sq = square(0.0, 0.0, 10.0);
        assert_eq!(ring_area(&sq), 100.0);
        let mut cw = sq.clone();
        cw.reverse();
        assert_eq!(ring_area(&cw), -100.0);
        assert!(ring_contains(&sq, DVec2::new(5.0, 5.0)));
        assert!(!ring_contains(&sq, DVec2::new(15.0, 5.0)));
        let p = Polygon { outer: sq, holes: vec![cw.iter().map(|v| *v * 0.5 + DVec2::splat(2.5)).collect()] };
        assert!((p.area() - 75.0).abs() < 1e-9);
    }
}
