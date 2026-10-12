//! Bounding-volume hierarchy over triangles, and exact triangle/triangle intersection.

use glam::DVec3;
use openslicer_core::Mesh;

#[derive(Clone, Copy, Debug)]
struct Bounds {
    min: DVec3,
    max: DVec3,
}

impl Bounds {
    fn empty() -> Self {
        Self { min: DVec3::splat(f64::MAX), max: DVec3::splat(f64::MIN) }
    }
    fn grow(&mut self, p: DVec3) {
        self.min = self.min.min(p);
        self.max = self.max.max(p);
    }
    fn union(&self, o: &Bounds) -> Bounds {
        Bounds { min: self.min.min(o.min), max: self.max.max(o.max) }
    }
    fn overlaps(&self, o: &Bounds, pad: f64) -> bool {
        self.min.x <= o.max.x + pad
            && self.max.x >= o.min.x - pad
            && self.min.y <= o.max.y + pad
            && self.max.y >= o.min.y - pad
            && self.min.z <= o.max.z + pad
            && self.max.z >= o.min.z - pad
    }
}

enum Node {
    Leaf { bounds: Bounds, first: usize, count: usize },
    Inner { bounds: Bounds, left: usize, right: usize },
}

impl Node {
    fn bounds(&self) -> &Bounds {
        match self {
            Node::Leaf { bounds, .. } | Node::Inner { bounds, .. } => bounds,
        }
    }
}

pub struct Bvh {
    nodes: Vec<Node>,
    /// Triangle indices, grouped by leaf.
    order: Vec<u32>,
    tri_bounds: Vec<Bounds>,
}

const LEAF_SIZE: usize = 4;

impl Bvh {
    pub fn build(mesh: &Mesh) -> Self {
        let tri_bounds: Vec<Bounds> = (0..mesh.triangles.len())
            .map(|i| {
                let mut b = Bounds::empty();
                for p in mesh.triangle(i) {
                    b.grow(p.as_dvec3());
                }
                b
            })
            .collect();
        let centroids: Vec<DVec3> = tri_bounds.iter().map(|b| (b.min + b.max) * 0.5).collect();
        let mut order: Vec<u32> = (0..mesh.triangles.len() as u32).collect();
        let mut nodes = Vec::new();
        if !order.is_empty() {
            let n = order.len();
            build_node(&mut nodes, &mut order, 0, n, &tri_bounds, &centroids);
        }
        Self { nodes, order, tri_bounds }
    }

    /// All pairs `(i, j)`, `i < j`, of triangles whose bounding boxes overlap (within `pad`).
    pub fn overlapping_pairs(&self, mut visit: impl FnMut(u32, u32)) {
        if self.nodes.is_empty() {
            return;
        }
        let mut stack = vec![(0usize, 0usize)];
        while let Some((a, b)) = stack.pop() {
            let (na, nb) = (&self.nodes[a], &self.nodes[b]);
            if !na.bounds().overlaps(nb.bounds(), 0.0) {
                continue;
            }
            match (na, nb) {
                (Node::Leaf { first: fa, count: ca, .. }, Node::Leaf { first: fb, count: cb, .. }) => {
                    for ia in *fa..fa + ca {
                        let start = if a == b { ia + 1 } else { *fb };
                        let end = fb + cb;
                        for ib in start..end {
                            let (x, y) = (self.order[ia], self.order[ib]);
                            if x != y && self.tri_bounds[x as usize].overlaps(&self.tri_bounds[y as usize], 0.0) {
                                visit(x.min(y), x.max(y));
                            }
                        }
                    }
                }
                (Node::Inner { left, right, .. }, _) if a == b => {
                    stack.push((*left, *left));
                    stack.push((*right, *right));
                    stack.push((*left, *right));
                }
                (Node::Inner { left, right, bounds: ba }, _) if matches!(nb, Node::Leaf { .. }) || volume(ba) >= volume(nb.bounds()) => {
                    stack.push((*left, b));
                    stack.push((*right, b));
                }
                (_, Node::Inner { left, right, .. }) => {
                    stack.push((a, *left));
                    stack.push((a, *right));
                }
                _ => {}
            }
        }
    }
}

fn volume(b: &Bounds) -> f64 {
    let d = b.max - b.min;
    d.x * d.y * d.z
}

fn build_node(nodes: &mut Vec<Node>, order: &mut [u32], lo: usize, hi: usize, tb: &[Bounds], cen: &[DVec3]) -> usize {
    let mut bounds = Bounds::empty();
    for &i in &order[lo..hi] {
        bounds = bounds.union(&tb[i as usize]);
    }
    let id = nodes.len();
    if hi - lo <= LEAF_SIZE {
        nodes.push(Node::Leaf { bounds, first: lo, count: hi - lo });
        return id;
    }
    // Split at the median along the widest axis of the centroid bounds.
    let mut cb = Bounds::empty();
    for &i in &order[lo..hi] {
        cb.grow(cen[i as usize]);
    }
    let ext = cb.max - cb.min;
    let axis = if ext.x >= ext.y && ext.x >= ext.z {
        0
    } else if ext.y >= ext.z {
        1
    } else {
        2
    };
    let mid = (lo + hi) / 2;
    order[lo..hi].select_nth_unstable_by(mid - lo, |&a, &b| cen[a as usize][axis].total_cmp(&cen[b as usize][axis]));
    nodes.push(Node::Inner { bounds, left: 0, right: 0 });
    let left = build_node(nodes, order, lo, mid, tb, cen);
    let right = build_node(nodes, order, mid, hi, tb, cen);
    nodes[id] = Node::Inner { bounds, left, right };
    id
}

/// Distance from point `p` to the triangle `t` (Ericson, Real-Time Collision Detection).
pub fn point_triangle_distance(p: DVec3, t: [DVec3; 3]) -> f64 {
    let (a, b, c) = (t[0], t[1], t[2]);
    let (ab, ac, ap) = (b - a, c - a, p - a);
    let (d1, d2) = (ab.dot(ap), ac.dot(ap));
    if d1 <= 0.0 && d2 <= 0.0 {
        return (p - a).length();
    }
    let bp = p - b;
    let (d3, d4) = (ab.dot(bp), ac.dot(bp));
    if d3 >= 0.0 && d4 <= d3 {
        return (p - b).length();
    }
    let vc = d1 * d4 - d3 * d2;
    if vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0 {
        let v = d1 / (d1 - d3);
        return (p - (a + ab * v)).length();
    }
    let cp = p - c;
    let (d5, d6) = (ab.dot(cp), ac.dot(cp));
    if d6 >= 0.0 && d5 <= d6 {
        return (p - c).length();
    }
    let vb = d5 * d2 - d1 * d6;
    if vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0 {
        let w = d2 / (d2 - d6);
        return (p - (a + ac * w)).length();
    }
    let va = d3 * d6 - d5 * d4;
    if va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0 {
        let w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        return (p - (b + (c - b) * w)).length();
    }
    let denom = 1.0 / (va + vb + vc);
    let (v, w) = (vb * denom, vc * denom);
    (p - (a + ab * v + ac * w)).length()
}

impl Bvh {
    /// Distance from `p` to the nearest triangle of `mesh` (which must be the mesh the BVH was built from),
    /// or `limit` if nothing is closer. The limit prunes the search, which makes near-surface queries cheap.
    pub fn distance_to(&self, mesh: &Mesh, p: DVec3, limit: f64) -> f64 {
        if self.nodes.is_empty() {
            return limit;
        }
        let box_dist = |b: &Bounds| -> f64 {
            let d = (b.min - p).max(p - b.max).max(DVec3::ZERO);
            d.length()
        };
        let mut best = limit;
        let mut stack = vec![0usize];
        while let Some(i) = stack.pop() {
            let node = &self.nodes[i];
            if box_dist(node.bounds()) >= best {
                continue;
            }
            match node {
                Node::Leaf { first, count, .. } => {
                    for k in *first..first + count {
                        let tri = self.order[k] as usize;
                        let d = point_triangle_distance(p, mesh.triangle(tri).map(|v| v.as_dvec3()));
                        if d < best {
                            best = d;
                        }
                    }
                }
                Node::Inner { left, right, .. } => {
                    // Visit the nearer child last so it is popped first.
                    let (dl, dr) = (box_dist(self.nodes[*left].bounds()), box_dist(self.nodes[*right].bounds()));
                    if dl < dr {
                        stack.push(*right);
                        stack.push(*left);
                    } else {
                        stack.push(*left);
                        stack.push(*right);
                    }
                }
            }
        }
        best
    }
}

/// Whether two triangles intersect (touching counts: triangles that share an edge overlap along it, so callers
/// that look for self-intersections must skip pairs that share a vertex). Uses the Möller interval test.
pub fn triangles_intersect(a: [DVec3; 3], b: [DVec3; 3]) -> bool {
    const EPS: f64 = 1e-9;
    let n2 = (b[1] - b[0]).cross(b[2] - b[0]);
    let d2 = -n2.dot(b[0]);
    let da = [n2.dot(a[0]) + d2, n2.dot(a[1]) + d2, n2.dot(a[2]) + d2];
    if da.iter().all(|&d| d > EPS) || da.iter().all(|&d| d < -EPS) {
        return false;
    }
    let n1 = (a[1] - a[0]).cross(a[2] - a[0]);
    let d1 = -n1.dot(a[0]);
    let db = [n1.dot(b[0]) + d1, n1.dot(b[1]) + d1, n1.dot(b[2]) + d1];
    if db.iter().all(|&d| d > EPS) || db.iter().all(|&d| d < -EPS) {
        return false;
    }
    let dir = n1.cross(n2);
    if dir.length_squared() < 1e-24 {
        // Coplanar triangles: treat as non-intersecting unless their interiors overlap, which is rare for
        // valid meshes; the shared-vertex filter in the caller handles the common adjacent case.
        return coplanar_overlap(a, b, n1);
    }
    let axis = if dir.x.abs() >= dir.y.abs() && dir.x.abs() >= dir.z.abs() {
        0
    } else if dir.y.abs() >= dir.z.abs() {
        1
    } else {
        2
    };
    let interval = |t: [DVec3; 3], d: [f64; 3]| -> Option<(f64, f64)> {
        let p = [t[0][axis], t[1][axis], t[2][axis]];
        let mut ts = Vec::with_capacity(2);
        for (i, j) in [(0, 1), (1, 2), (2, 0)] {
            let (di, dj) = (d[i], d[j]);
            if (di > EPS && dj < -EPS) || (di < -EPS && dj > EPS) {
                ts.push(p[i] + (p[j] - p[i]) * di / (di - dj));
            } else if di.abs() <= EPS {
                ts.push(p[i]);
            }
        }
        if ts.len() < 2 {
            return None;
        }
        let (lo, hi) = ts.iter().fold((f64::MAX, f64::MIN), |(l, h), &v| (l.min(v), h.max(v)));
        Some((lo, hi))
    };
    match (interval(a, da), interval(b, db)) {
        (Some((a0, a1)), Some((b0, b1))) => a0.max(b0) < a1.min(b1) - EPS,
        _ => false,
    }
}

fn coplanar_overlap(a: [DVec3; 3], b: [DVec3; 3], n: DVec3) -> bool {
    // Project to the dominant plane and test for proper edge crossings.
    let drop = if n.x.abs() >= n.y.abs() && n.x.abs() >= n.z.abs() {
        0
    } else if n.y.abs() >= n.z.abs() {
        1
    } else {
        2
    };
    let p = |v: DVec3| -> (f64, f64) {
        match drop {
            0 => (v.y, v.z),
            1 => (v.x, v.z),
            _ => (v.x, v.y),
        }
    };
    let cross = |o: (f64, f64), a: (f64, f64), b: (f64, f64)| (a.0 - o.0) * (b.1 - o.1) - (a.1 - o.1) * (b.0 - o.0);
    let (pa, pb) = (a.map(p), b.map(p));
    for i in 0..3 {
        for j in 0..3 {
            let (a0, a1, b0, b1) = (pa[i], pa[(i + 1) % 3], pb[j], pb[(j + 1) % 3]);
            let (d1, d2, d3, d4) = (cross(a0, a1, b0), cross(a0, a1, b1), cross(b0, b1, a0), cross(b0, b1, a1));
            if d1 * d2 < -1e-18 && d3 * d4 < -1e-18 {
                return true;
            }
        }
    }
    false
}

#[cfg(test)]
mod tests {
    use super::*;
    use glam::Vec3;

    fn t(a: [f64; 3], b: [f64; 3], c: [f64; 3]) -> [DVec3; 3] {
        [DVec3::from(a), DVec3::from(b), DVec3::from(c)]
    }

    #[test]
    fn triangle_intersection_cases() {
        let flat = t([0.0, 0.0, 0.0], [10.0, 0.0, 0.0], [0.0, 10.0, 0.0]);
        // Piercing triangle.
        assert!(triangles_intersect(flat, t([2.0, 2.0, -5.0], [3.0, 2.0, 5.0], [2.0, 3.0, 5.0])));
        // Far above.
        assert!(!triangles_intersect(flat, t([2.0, 2.0, 1.0], [3.0, 2.0, 5.0], [2.0, 3.0, 5.0])));
        // Sharing an edge (adjacent faces of a fold) touch along it; the self-intersection search skips such neighbours.
        assert!(triangles_intersect(flat, t([0.0, 0.0, 0.0], [10.0, 0.0, 0.0], [0.0, 0.0, 10.0])));
        // Touching only at a corner point is not an overlap.
        assert!(!triangles_intersect(flat, t([10.0, 0.0, 0.0], [20.0, 0.0, 5.0], [20.0, 5.0, 5.0])));
        // Coplanar overlapping.
        assert!(triangles_intersect(flat, t([2.0, 2.0, 0.0], [12.0, 2.0, 0.0], [2.0, 12.0, 0.0])));
        // Coplanar disjoint.
        assert!(!triangles_intersect(flat, t([20.0, 20.0, 0.0], [30.0, 20.0, 0.0], [20.0, 30.0, 0.0])));
    }

    #[test]
    fn point_triangle_distance_regions() {
        let tri = t([0.0, 0.0, 0.0], [10.0, 0.0, 0.0], [0.0, 10.0, 0.0]);
        assert!((point_triangle_distance(DVec3::new(2.0, 2.0, 3.0), tri) - 3.0).abs() < 1e-12, "above the face");
        assert!((point_triangle_distance(DVec3::new(-3.0, -4.0, 0.0), tri) - 5.0).abs() < 1e-12, "beyond a vertex");
        assert!((point_triangle_distance(DVec3::new(5.0, -2.0, 0.0), tri) - 2.0).abs() < 1e-12, "beyond an edge");
        assert!((point_triangle_distance(DVec3::new(8.0, 8.0, 0.0), tri) - 6.0 / 2f64.sqrt()).abs() < 1e-9, "beyond the hypotenuse edge");
        assert!(point_triangle_distance(DVec3::new(1.0, 1.0, 0.0), tri).abs() < 1e-12, "on the face");
    }

    #[test]
    fn bvh_distance_matches_brute_force() {
        let mesh = {
            let mut m = Mesh::cuboid(Vec3::splat(10.0));
            m.translate(Vec3::new(3.0, 4.0, 5.0));
            m
        };
        let bvh = Bvh::build(&mesh);
        for p in [DVec3::new(20.0, 4.0, 5.0), DVec3::new(3.0, 4.0, 5.0), DVec3::new(-9.0, -9.0, -9.0), DVec3::new(3.0, 14.0, 5.5)] {
            let brute = (0..mesh.triangles.len())
                .map(|i| point_triangle_distance(p, mesh.triangle(i).map(|v| v.as_dvec3())))
                .fold(f64::MAX, f64::min);
            assert!((bvh.distance_to(&mesh, p, f64::MAX) - brute).abs() < 1e-9, "{p:?}");
        }
        // The limit caps the answer.
        assert_eq!(bvh.distance_to(&mesh, DVec3::new(100.0, 0.0, 0.0), 5.0), 5.0);
    }

    #[test]
    fn bvh_finds_all_overlapping_pairs_like_brute_force() {
        let mesh = {
            let mut m = Mesh::cuboid(Vec3::splat(10.0));
            let mut b = Mesh::cuboid(Vec3::splat(10.0));
            b.translate(Vec3::new(4.0, 3.0, 2.0));
            let base = m.vertices.len() as u32;
            m.vertices.extend(b.vertices);
            m.triangles.extend(b.triangles.iter().map(|t| [t[0] + base, t[1] + base, t[2] + base]));
            m
        };
        let bvh = Bvh::build(&mesh);
        let mut from_bvh = Vec::new();
        bvh.overlapping_pairs(|a, b| from_bvh.push((a, b)));
        from_bvh.sort();
        from_bvh.dedup();
        let bounds = |i: usize| {
            let v = mesh.triangle(i);
            (v[0].min(v[1]).min(v[2]), v[0].max(v[1]).max(v[2]))
        };
        let mut brute = Vec::new();
        for i in 0..mesh.triangles.len() {
            for j in i + 1..mesh.triangles.len() {
                let ((a0, a1), (b0, b1)) = (bounds(i), bounds(j));
                if a0.cmple(b1).all() && a1.cmpge(b0).all() {
                    brute.push((i as u32, j as u32));
                }
            }
        }
        assert_eq!(from_bvh, brute);
    }
}
