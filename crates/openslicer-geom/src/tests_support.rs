//! Procedural test meshes (closed, outward-facing) used across the geometry tests.

use glam::Vec3;
use openslicer_core::Mesh;
use std::f32::consts::PI;

/// Torus around Z with major radius `big` and minor radius `small`.
pub fn torus(big: f32, small: f32, nu: usize, nv: usize) -> Mesh {
    let mut verts = Vec::with_capacity(nu * nv);
    for i in 0..nu {
        for j in 0..nv {
            let (u, v) = (2.0 * PI * i as f32 / nu as f32, 2.0 * PI * j as f32 / nv as f32);
            let r = big + small * v.cos();
            verts.push(Vec3::new(r * u.cos(), r * u.sin(), small * v.sin()));
        }
    }
    let mut tris = Vec::new();
    for i in 0..nu {
        for j in 0..nv {
            let (i1, j1) = ((i + 1) % nu, (j + 1) % nv);
            let (a, b, c, d) = ((i * nv + j) as u32, (i1 * nv + j) as u32, (i1 * nv + j1) as u32, (i * nv + j1) as u32);
            tris.push([a, b, c]);
            tris.push([a, c, d]);
        }
    }
    Mesh::new(verts, tris)
}

/// Hollow tube (outer radius `ro`, inner radius `ri`) along Z from 0 to `height`.
pub fn tube(ro: f32, ri: f32, height: f32, n: usize) -> Mesh {
    let mut verts = Vec::new();
    // rings: outer bottom, outer top, inner bottom, inner top
    for (r, z) in [(ro, 0.0), (ro, height), (ri, 0.0), (ri, height)] {
        for k in 0..n {
            let a = 2.0 * PI * k as f32 / n as f32;
            verts.push(Vec3::new(r * a.cos(), r * a.sin(), z));
        }
    }
    let ring = |r: usize, k: usize| (r * n + k % n) as u32;
    let mut tris = Vec::new();
    for k in 0..n {
        // outer wall (faces outward)
        tris.push([ring(0, k), ring(0, k + 1), ring(1, k + 1)]);
        tris.push([ring(0, k), ring(1, k + 1), ring(1, k)]);
        // inner wall (faces inward, towards the axis)
        tris.push([ring(2, k), ring(3, k + 1), ring(2, k + 1)]);
        tris.push([ring(2, k), ring(3, k), ring(3, k + 1)]);
        // top annulus (faces +z)
        tris.push([ring(1, k), ring(1, k + 1), ring(3, k + 1)]);
        tris.push([ring(1, k), ring(3, k + 1), ring(3, k)]);
        // bottom annulus (faces -z)
        tris.push([ring(0, k), ring(2, k + 1), ring(0, k + 1)]);
        tris.push([ring(0, k), ring(2, k), ring(2, k + 1)]);
    }
    Mesh::new(verts, tris)
}

/// UV sphere (poles are single vertices) of radius `r`; `flip` reverses the winding (inner surface of a shell).
pub fn sphere(r: f32, nu: usize, nv: usize, flip: bool) -> Mesh {
    let mut verts = vec![Vec3::new(0.0, 0.0, -r), Vec3::new(0.0, 0.0, r)];
    for j in 1..nv {
        let th = PI * j as f32 / nv as f32;
        for i in 0..nu {
            let ph = 2.0 * PI * i as f32 / nu as f32;
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
}

/// Thin spherical shell: outer sphere plus an inner sphere with inverted winding.
pub fn sphere_shell(ro: f32, ri: f32, nu: usize, nv: usize) -> Mesh {
    let mut m = sphere(ro, nu, nv, false);
    let inner = sphere(ri, nu, nv, true);
    let base = m.vertices.len() as u32;
    m.vertices.extend(inner.vertices);
    m.triangles.extend(inner.triangles.iter().map(|t| [t[0] + base, t[1] + base, t[2] + base]));
    m
}

/// Prism of height `h` over a polygon with holes (outer CCW, holes CW), triangulated with earcut.
pub fn extrude(outer: &[[f32; 2]], holes: &[Vec<[f32; 2]>], h: f32) -> Mesh {
    let mut pts: Vec<[f32; 2]> = outer.to_vec();
    let mut starts = Vec::new();
    for hole in holes {
        starts.push(pts.len());
        pts.extend(hole.iter().copied());
    }
    let flat: Vec<f32> = pts.iter().flat_map(|p| [p[0], p[1]]).collect();
    let cap = earcutr::earcut(&flat, &starts, 2).unwrap();
    let n = pts.len();
    let mut verts: Vec<Vec3> = pts.iter().map(|p| Vec3::new(p[0], p[1], 0.0)).collect();
    verts.extend(pts.iter().map(|p| Vec3::new(p[0], p[1], h)));
    let mut tris = Vec::new();
    for t in cap.chunks_exact(3) {
        let (a, b, c) = (t[0], t[1], t[2]);
        let area = (pts[b][0] - pts[a][0]) * (pts[c][1] - pts[a][1]) - (pts[b][1] - pts[a][1]) * (pts[c][0] - pts[a][0]);
        let (b, c) = if area > 0.0 { (b, c) } else { (c, b) };
        tris.push([(a + n) as u32, (b + n) as u32, (c + n) as u32]); // top, faces +z
        tris.push([a as u32, c as u32, b as u32]); // bottom, faces -z
    }
    let mut rings: Vec<(usize, usize)> = vec![(0, outer.len())];
    let mut offset = outer.len();
    for hole in holes {
        rings.push((offset, hole.len()));
        offset += hole.len();
    }
    for (start, len) in rings {
        for k in 0..len {
            // The outer ring is CCW and holes are CW, so one ordering faces away from the solid on both.
            let (a, b) = (start + k, start + (k + 1) % len);
            tris.push([a as u32, b as u32, (b + n) as u32]);
            tris.push([a as u32, (b + n) as u32, (a + n) as u32]);
        }
    }
    Mesh::new(verts, tris)
}

/// L-shaped (concave) prism with a square hole through it.
pub fn l_prism() -> Mesh {
    let outer = [[0.0, 0.0], [60.0, 0.0], [60.0, 20.0], [20.0, 20.0], [20.0, 60.0], [0.0, 60.0]];
    let hole = vec![[5.0, 5.0], [5.0, 12.0], [12.0, 12.0], [12.0, 5.0]];
    extrude(&outer, &[hole], 30.0)
}

/// Deterministic pseudo-random numbers for randomized tests.
pub struct Lcg(pub u64);

impl Lcg {
    pub fn next_f64(&mut self) -> f64 {
        self.0 = self.0.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
        ((self.0 >> 11) as f64) / ((1u64 << 53) as f64)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn procedural_meshes_are_closed_and_outward() {
        for m in
            [torus(30.0, 10.0, 64, 32), tube(20.0, 8.0, 60.0, 64), sphere(10.0, 32, 16, false), sphere_shell(20.0, 18.0, 48, 24), l_prism()]
        {
            let s = m.stats();
            assert!(s.is_watertight(), "{s:?}");
            assert!(s.signed_volume > 0.0, "{s:?}");
        }
    }
}
