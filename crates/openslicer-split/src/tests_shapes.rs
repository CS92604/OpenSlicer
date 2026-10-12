//! Procedural shapes for tests (closed, outward-facing meshes).

use glam::Vec3;
use openslicer_core::Mesh;
use std::f32::consts::PI;

pub fn sphere_flip(r: f32, nu: usize, nv: usize, flip: bool) -> Mesh {
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

pub fn sphere(r: f32, nu: usize, nv: usize) -> Mesh {
    sphere_flip(r, nu, nv, false)
}

pub fn sphere_shell(ro: f32, ri: f32, nu: usize, nv: usize) -> Mesh {
    let mut m = sphere(ro, nu, nv);
    let inner = sphere_flip(ri, nu, nv, true);
    let base = m.vertices.len() as u32;
    m.vertices.extend(inner.vertices);
    m.triangles.extend(inner.triangles.iter().map(|t| [t[0] + base, t[1] + base, t[2] + base]));
    m
}

/// Torus around Z.
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
