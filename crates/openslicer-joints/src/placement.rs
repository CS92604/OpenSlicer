//! Connector placement on an interface region.

use glam::DVec2;
use openslicer_geom::poly2d;
use openslicer_geom::region::{ring_contains, Polygon};

/// Distance from `p` to the nearest boundary segment of the polygons, with the boundary point.
pub fn nearest_boundary(polys: &[Polygon], p: DVec2) -> Option<(f64, DVec2, DVec2)> {
    let mut best: Option<(f64, DVec2, DVec2)> = None;
    for poly in polys {
        for ring in std::iter::once(&poly.outer).chain(poly.holes.iter()) {
            let n = ring.len();
            for i in 0..n {
                let (a, b) = (ring[i], ring[(i + 1) % n]);
                let ab = b - a;
                let len2 = ab.length_squared();
                if len2 == 0.0 {
                    continue;
                }
                let t = ((p - a).dot(ab) / len2).clamp(0.0, 1.0);
                let q = a + ab * t;
                let d = (p - q).length();
                if best.is_none_or(|(bd, _, _)| d < bd) {
                    best = Some((d, q, ab / len2.sqrt()));
                }
            }
        }
    }
    best
}

pub fn contains(polys: &[Polygon], p: DVec2) -> bool {
    polys.iter().any(|poly| ring_contains(&poly.outer, p) && !poly.holes.iter().any(|h| ring_contains(h, p)))
}

/// Largest distance from any interior point to the boundary: half the local wall thickness for thin regions.
pub fn inradius(polys: &[Polygon]) -> f64 {
    candidates(polys, 0.0, 1.0, 3000).iter().filter_map(|&p| nearest_boundary(polys, p).map(|b| b.0)).fold(0.0, f64::max)
}

/// Grid of candidate points inside `polys` shrunk by `inset`. The grid pitch is `pitch`, coarsened to keep at most `max` points.
pub fn candidates(polys: &[Polygon], inset: f64, pitch: f64, max: usize) -> Vec<DVec2> {
    let valid = if inset > 0.0 { poly2d::offset(polys, -inset, 0.1) } else { polys.to_vec() };
    if valid.is_empty() {
        return Vec::new();
    }
    let (mut lo, mut hi) = (DVec2::splat(f64::MAX), DVec2::splat(f64::MIN));
    for poly in &valid {
        for p in &poly.outer {
            lo = lo.min(*p);
            hi = hi.max(*p);
        }
    }
    let mut pitch = pitch.max(1e-3);
    let count = |pitch: f64| ((hi.x - lo.x) / pitch + 1.0) * ((hi.y - lo.y) / pitch + 1.0);
    while count(pitch) > max as f64 {
        pitch *= 1.25;
    }
    let mut out = Vec::new();
    let (nx, ny) = (((hi.x - lo.x) / pitch).floor() as usize + 1, ((hi.y - lo.y) / pitch).floor() as usize + 1);
    // Centre the grid in the bounding box.
    let off = DVec2::new((hi.x - lo.x) - (nx - 1) as f64 * pitch, (hi.y - lo.y) - (ny - 1) as f64 * pitch) * 0.5;
    for i in 0..nx {
        for j in 0..ny {
            let p = lo + off + DVec2::new(i as f64, j as f64) * pitch;
            if contains(&valid, p) {
                out.push(p);
            }
        }
    }
    // Thin bands (shell walls) can slip between grid points, so also sample along the boundary of the valid
    // region: its edges are valid centres too (a closed set), and for a thin band they run along the centreline.
    let mut budget = max;
    for poly in &valid {
        for ring in std::iter::once(&poly.outer).chain(poly.holes.iter()) {
            let n = ring.len();
            for i in 0..n {
                let (a, b) = (ring[i], ring[(i + 1) % n]);
                let steps = (((b - a).length() / pitch).ceil() as usize).max(1);
                for k in 0..steps {
                    if budget == 0 {
                        break;
                    }
                    out.push(a + (b - a) * (k as f64 / steps as f64));
                    budget -= 1;
                }
            }
        }
    }
    // Tiny regions can fall between everything: always offer a centroid-like fallback point.
    if out.is_empty() {
        for poly in &valid {
            let c = poly.outer.iter().fold(DVec2::ZERO, |a, b| a + *b) / poly.outer.len() as f64;
            if contains(&valid, c) {
                out.push(c);
            }
        }
    }
    out
}

/// Chooses well-spread points among `cands` (farthest-point sampling). Always returns at least `min_count`
/// points when enough candidates exist, and adds more until every candidate is within `spacing` of one, up to `max_count`.
/// The result is ordered so that any prefix is itself well spread, which lets callers fall back to later points.
pub fn spread(polys: &[Polygon], cands: &[DVec2], spacing: f64, min_count: usize, max_count: usize) -> Vec<DVec2> {
    if cands.is_empty() {
        return Vec::new();
    }
    // Start at the candidate deepest inside the region.
    let first = cands
        .iter()
        .copied()
        .max_by(|a, b| nearest_boundary(polys, *a).map_or(0.0, |x| x.0).total_cmp(&nearest_boundary(polys, *b).map_or(0.0, |x| x.0)))
        .unwrap();
    let mut chosen = vec![first];
    let mut mind: Vec<f64> = cands.iter().map(|c| (*c - first).length()).collect();
    while chosen.len() < max_count {
        let (idx, d) = mind.iter().enumerate().fold((0, -1.0), |best, (i, &d)| if d > best.1 { (i, d) } else { best });
        if d < 1e-9 || (d < spacing && chosen.len() >= min_count) {
            break;
        }
        let p = cands[idx];
        chosen.push(p);
        for (i, c) in cands.iter().enumerate() {
            mind[i] = mind[i].min((*c - p).length());
        }
    }
    chosen
}

/// Incremental farthest-point selection over a fixed candidate set. Candidates can be accepted or rejected
/// (for example when a 3D check fails) and the next pick is always the live candidate farthest from all accepted ones.
pub struct FarthestPoint {
    cands: Vec<DVec2>,
    depth: Vec<f64>,
    /// Preference for candidates deeper inside the region (edge candidates are valid but marginal).
    weight: Vec<f64>,
    mind: Vec<f64>,
    alive: Vec<bool>,
    accepted: Vec<DVec2>,
}

impl FarthestPoint {
    pub fn new(polys: &[Polygon], cands: Vec<DVec2>) -> Self {
        let depth: Vec<f64> = cands.iter().map(|&c| nearest_boundary(polys, c).map_or(0.0, |b| b.0)).collect();
        let n = cands.len();
        let deepest = depth.iter().cloned().fold(0.0, f64::max).max(1e-9);
        let weight = depth.iter().map(|d| 0.35 + 0.65 * (d / deepest)).collect();
        Self { cands, mind: vec![f64::INFINITY; n], depth, weight, alive: vec![true; n], accepted: Vec::new() }
    }

    /// Index of the next candidate to try, or `None` when the coverage rule is satisfied
    /// (every live candidate is within `spacing` of an accepted point and `min_count` is met) or `max_count` is reached.
    pub fn next(&self, spacing: f64, min_count: usize, max_count: usize) -> Option<usize> {
        if self.accepted.len() >= max_count {
            return None;
        }
        let score = |i: usize| if self.accepted.is_empty() { self.depth[i] } else { self.mind[i] * self.weight[i] };
        let best = (0..self.cands.len()).filter(|&i| self.alive[i]).max_by(|&a, &b| score(a).total_cmp(&score(b)))?;
        if !self.accepted.is_empty() && self.accepted.len() >= min_count && self.mind[best] < spacing {
            return None;
        }
        Some(best)
    }

    pub fn point(&self, i: usize) -> DVec2 {
        self.cands[i]
    }

    pub fn accept(&mut self, i: usize) {
        let p = self.cands[i];
        self.alive[i] = false;
        self.accepted.push(p);
        for (j, c) in self.cands.iter().enumerate() {
            self.mind[j] = self.mind[j].min((*c - p).length());
        }
    }

    pub fn reject(&mut self, i: usize) {
        self.alive[i] = false;
    }

    pub fn accepted(&self) -> &[DVec2] {
        &self.accepted
    }
}

/// Rotation (radians) that points the local +X axis along the boundary tangent nearest to `p`.
pub fn tangent_angle(polys: &[Polygon], p: DVec2) -> f64 {
    nearest_boundary(polys, p).map_or(0.0, |(_, _, t)| t.y.atan2(t.x))
}

/// Rotation (radians) that points the local +X axis from `p` towards the nearest boundary point.
pub fn towards_boundary_angle(polys: &[Polygon], p: DVec2) -> f64 {
    nearest_boundary(polys, p).map_or(0.0, |(_, q, _)| {
        let d = q - p;
        d.y.atan2(d.x)
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use openslicer_geom::poly2d::{circle, difference, rect};

    fn square(s: f64) -> Vec<Polygon> {
        vec![rect(DVec2::ZERO, DVec2::splat(s))]
    }

    #[test]
    fn nearest_boundary_and_inradius_of_a_square() {
        let sq = square(100.0);
        let (d, q, t) = nearest_boundary(&sq, DVec2::new(30.0, 50.0)).unwrap();
        assert!((d - 30.0).abs() < 1e-9 && (q - DVec2::new(0.0, 50.0)).length() < 1e-9);
        assert!((t.length() - 1.0).abs() < 1e-9);
        assert!((inradius(&sq) - 50.0).abs() < 2.0);
    }

    #[test]
    fn inradius_of_a_thin_ring_is_half_the_wall() {
        let ring = difference(
            &[Polygon { outer: circle(DVec2::ZERO, 100.0, 180), holes: vec![] }],
            &[Polygon { outer: circle(DVec2::ZERO, 96.0, 180), holes: vec![] }],
        );
        let r = inradius(&ring);
        assert!((r - 2.0).abs() < 0.5, "wall 4 mm -> inradius ~2, got {r}");
    }

    #[test]
    fn candidates_stay_inside_the_inset_region() {
        let sq = square(100.0);
        let c = candidates(&sq, 10.0, 5.0, 4000);
        assert!(!c.is_empty());
        assert!(c.iter().all(|p| p.x >= 9.9 && p.x <= 90.1 && p.y >= 9.9 && p.y <= 90.1));
        assert!(candidates(&sq, 60.0, 5.0, 4000).is_empty(), "inset beyond the inradius leaves nothing");
    }

    #[test]
    fn spread_returns_well_separated_points_and_respects_limits() {
        let sq = square(100.0);
        let c = candidates(&sq, 8.0, 4.0, 4000);
        let pts = spread(&sq, &c, 40.0, 1, 50);
        assert!(pts.len() >= 4 && pts.len() <= 16, "{}", pts.len());
        for (i, a) in pts.iter().enumerate() {
            for b in &pts[i + 1..] {
                assert!((*a - *b).length() >= 40.0 - 1e-6 - 4.0, "points too close: {a:?} {b:?}");
            }
        }
        assert_eq!(spread(&sq, &c, 1000.0, 3, 50).len(), 3, "min_count is honoured even when spacing is satisfied");
        assert_eq!(spread(&sq, &c, 1.0, 1, 5).len(), 5, "max_count caps the result");
        assert!(spread(&sq, &[], 10.0, 1, 5).is_empty());
    }

    #[test]
    fn farthest_point_skips_rejected_candidates_and_stops_at_coverage() {
        let sq = square(100.0);
        let c = candidates(&sq, 8.0, 4.0, 4000);
        let mut fp = FarthestPoint::new(&sq, c);
        let mut tried = 0;
        while let Some(i) = fp.next(40.0, 1, 50) {
            tried += 1;
            // Reject every other pick, as a failing 3D check would.
            if tried % 2 == 0 {
                fp.reject(i);
            } else {
                fp.accept(i);
            }
            assert!(tried < 200, "must terminate");
        }
        assert!(fp.accepted().len() >= 4);
        let acc = fp.accepted().to_vec();
        for (i, a) in acc.iter().enumerate() {
            for b in &acc[i + 1..] {
                assert!((*a - *b).length() > 1.0);
            }
        }
        // max_count is a hard cap
        let mut fp = FarthestPoint::new(&sq, candidates(&sq, 8.0, 4.0, 4000));
        while let Some(i) = fp.next(1.0, 1, 3) {
            fp.accept(i);
        }
        assert_eq!(fp.accepted().len(), 3);
    }

    #[test]
    fn thin_strip_gets_points_along_its_centreline() {
        let strip = vec![rect(DVec2::new(0.0, 0.0), DVec2::new(200.0, 5.0))];
        let c = candidates(&strip, 1.5, 2.0, 4000);
        assert!(!c.is_empty());
        assert!(c.iter().all(|p| (p.y - 2.5).abs() <= 1.0 + 1e-9));
        let pts = spread(&strip, &c, 50.0, 1, 20);
        assert!(pts.len() >= 3);
        // The coverage rule: every candidate is within the spacing of some chosen point.
        assert!(c.iter().all(|q| pts.iter().any(|p| (*p - *q).length() <= 50.0)));
    }

    #[test]
    fn orientation_helpers_follow_the_boundary() {
        let sq = square(100.0);
        // Near the bottom edge (y = 0) the tangent is along X.
        let a = tangent_angle(&sq, DVec2::new(50.0, 5.0));
        assert!(a.cos().abs() > 0.999);
        // Direction towards the nearest edge from a point close to the left edge is -X.
        let b = towards_boundary_angle(&sq, DVec2::new(5.0, 50.0));
        assert!((b.cos() + 1.0).abs() < 1e-6);
    }
}
