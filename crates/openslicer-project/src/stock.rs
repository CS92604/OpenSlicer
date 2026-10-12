//! Built-in sample models, generated procedurally so they ship inside the executable with no data files.
//!
//! They are deliberately larger than a typical print bed and cover three cases that matter for splitting:
//! a thin curved shell (dome), a solid tapered body (wing) and a large thin compound-curved panel
//! (body panel, the hardtop/bonnet case).

use glam::Vec3;
use openslicer_core::Mesh;
use std::f32::consts::{PI, TAU};

pub struct StockModel {
    pub id: &'static str,
    pub name: &'static str,
    pub description: &'static str,
}

pub const STOCK: [StockModel; 3] = [
    StockModel {
        id: "dome",
        name: "Dome shell",
        description: "A 400 mm hemispherical helmet shell with a 4 mm wall, open at the bottom. A thin, doubly curved, hollow part: the case that needs seam-following joints.",
    },
    StockModel {
        id: "wing",
        name: "Tapered wing",
        description: "A solid 900 mm wing (NACA 0018 section) with taper and sweep. A long solid part that needs several cuts across its span.",
    },
    StockModel {
        id: "panel",
        name: "Body panel",
        description: "A 1100 × 600 mm compound-curved panel, 3 mm thick. A thin automotive-style skin like a bonnet or hardtop section.",
    },
];

pub fn build(id: &str) -> Option<Mesh> {
    match id {
        "dome" => Some(dome(200.0, 4.0)),
        "wing" => Some(wing(900.0, 300.0, 150.0, 120.0, 0.18)),
        "panel" => Some(panel(1100.0, 600.0, 3.0)),
        _ => None,
    }
}

/// Makes the surface face outwards (positive enclosed volume).
fn finish(tris: Vec<[Vec3; 3]>) -> Mesh {
    let mut mesh = Mesh::from_triangle_soup(tris, 0.0);
    if mesh.stats().signed_volume < 0.0 {
        for t in &mut mesh.triangles {
            t.swap(1, 2);
        }
    }
    mesh
}

/// Hemispherical shell of outer radius `r` and wall `t`, open at z = 0.
fn dome(r: f32, t: f32) -> Mesh {
    const N: usize = 48; // rings from the pole to the equator
    const M: usize = 96; // segments around
    let point = |radius: f32, i: usize, j: usize| -> Vec3 {
        let theta = (i as f32 / N as f32) * (PI / 2.0);
        let phi = ((j % M) as f32 / M as f32) * TAU;
        let z = if i == N { 0.0 } else { radius * theta.cos() };
        Vec3::new(radius * theta.sin() * phi.cos(), radius * theta.sin() * phi.sin(), z)
    };
    let mut tris = Vec::new();
    for i in 0..N {
        for j in 0..M {
            for (radius, flip) in [(r, false), (r - t, true)] {
                let (a, b, c, d) = (point(radius, i, j), point(radius, i, j + 1), point(radius, i + 1, j + 1), point(radius, i + 1, j));
                let mut quad = Vec::new();
                if i > 0 {
                    quad.push([a, d, b]);
                }
                quad.push([b, d, c]);
                for mut tri in quad {
                    if flip {
                        tri.swap(1, 2);
                    }
                    tris.push(tri);
                }
            }
        }
    }
    for j in 0..M {
        let (o0, o1, i0, i1) = (point(r, N, j), point(r, N, j + 1), point(r - t, N, j), point(r - t, N, j + 1));
        tris.push([o0, i0, i1]);
        tris.push([o0, i1, o1]);
    }
    finish(tris)
}

/// NACA 00xx half-thickness at chord fraction `x`, as a fraction of the chord.
fn naca_half_thickness(x: f32, thickness: f32) -> f32 {
    5.0 * thickness * (0.2969 * x.sqrt() - 0.1260 * x - 0.3516 * x * x + 0.2843 * x * x * x - 0.1036 * x * x * x * x)
}

/// Solid tapered, swept wing. The chord runs along X, the span along Y and the thickness along Z.
fn wing(span: f32, root_chord: f32, tip_chord: f32, tip_sweep: f32, thickness: f32) -> Mesh {
    const K: usize = 48;
    // Closed section outline: upper surface leading edge → trailing edge, then lower surface back.
    let xs: Vec<f32> = (0..=K).map(|k| 0.5 * (1.0 - (PI * k as f32 / K as f32).cos())).collect();
    let mut outline: Vec<(f32, f32)> = xs.iter().map(|&x| (x, naca_half_thickness(x, thickness))).collect();
    outline.extend(xs[1..K].iter().rev().map(|&x| (x, -naca_half_thickness(x, thickness))));
    let n = outline.len();
    let section = |y_frac: f32, k: usize| -> Vec3 {
        let chord = root_chord + (tip_chord - root_chord) * y_frac;
        let (u, v) = outline[k % n];
        Vec3::new(tip_sweep * y_frac + u * chord, span * y_frac, v * chord)
    };
    let centre = |y_frac: f32| -> Vec3 {
        let chord = root_chord + (tip_chord - root_chord) * y_frac;
        Vec3::new(tip_sweep * y_frac + 0.4 * chord, span * y_frac, 0.0)
    };
    let mut tris = Vec::new();
    for k in 0..n {
        let (a, b, c, d) = (section(0.0, k), section(0.0, k + 1), section(1.0, k + 1), section(1.0, k));
        tris.push([a, b, c]);
        tris.push([a, c, d]);
        // Caps: a fan from a point inside the section.
        tris.push([centre(0.0), section(0.0, k + 1), section(0.0, k)]);
        tris.push([centre(1.0), section(1.0, k), section(1.0, k + 1)]);
    }
    finish(tris)
}

/// Height of the panel's outer skin at (x, y), and its gradient.
fn panel_height(x: f32, y: f32, length: f32, width: f32) -> (f32, f32, f32) {
    let (u, v) = (2.0 * x / length, 2.0 * y / width);
    let z = 90.0 * (1.0 - v * v) + 40.0 * (1.0 - u * u) + 25.0 * (1.0 - u * u) * (1.0 - v * v);
    let dzdx = (-80.0 * u - 50.0 * u * (1.0 - v * v)) * (2.0 / length);
    let dzdy = (-180.0 * v - 50.0 * v * (1.0 - u * u)) * (2.0 / width);
    (z, dzdx, dzdy)
}

/// Compound-curved skin of uniform thickness `t`, measured along the surface normal.
fn panel(length: f32, width: f32, t: f32) -> Mesh {
    const NX: usize = 60;
    const NY: usize = 40;
    let outer = |i: usize, j: usize| -> Vec3 {
        let (x, y) = ((i as f32 / NX as f32 - 0.5) * length, (j as f32 / NY as f32 - 0.5) * width);
        Vec3::new(x, y, panel_height(x, y, length, width).0)
    };
    let inner = |i: usize, j: usize| -> Vec3 {
        let p = outer(i, j);
        let (_, gx, gy) = panel_height(p.x, p.y, length, width);
        p - Vec3::new(-gx, -gy, 1.0).normalize() * t
    };
    let mut tris = Vec::new();
    for i in 0..NX {
        for j in 0..NY {
            let ids = [(i, j), (i + 1, j), (i + 1, j + 1), (i, j + 1)];
            let o: Vec<Vec3> = ids.iter().map(|&(a, b)| outer(a, b)).collect();
            let n: Vec<Vec3> = ids.iter().map(|&(a, b)| inner(a, b)).collect();
            tris.push([o[0], o[1], o[2]]);
            tris.push([o[0], o[2], o[3]]);
            tris.push([n[0], n[2], n[1]]);
            tris.push([n[0], n[3], n[2]]);
        }
    }
    // Perimeter, counter-clockwise seen from above.
    let mut rim: Vec<(usize, usize)> = (0..NX).map(|i| (i, 0)).collect();
    rim.extend((0..NY).map(|j| (NX, j)));
    rim.extend((1..=NX).rev().map(|i| (i, NY)));
    rim.extend((1..=NY).rev().map(|j| (0, j)));
    for k in 0..rim.len() {
        let (p, q) = (rim[k], rim[(k + 1) % rim.len()]);
        let (tp, tq, bp, bq) = (outer(p.0, p.1), outer(q.0, q.1), inner(p.0, p.1), inner(q.0, q.1));
        tris.push([tp, bp, bq]);
        tris.push([tp, bq, tq]);
    }
    finish(tris)
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::HashMap;

    /// Every directed edge must appear exactly once and be matched by its reverse: a consistently oriented closed surface.
    fn assert_consistently_oriented(m: &Mesh, what: &str) {
        let mut edges: HashMap<(u32, u32), u32> = HashMap::new();
        for t in &m.triangles {
            for k in 0..3 {
                *edges.entry((t[k], t[(k + 1) % 3])).or_default() += 1;
            }
        }
        for (&(a, b), &count) in &edges {
            assert_eq!(count, 1, "{what}: directed edge {a}->{b} used {count} times");
            assert_eq!(edges.get(&(b, a)), Some(&1), "{what}: edge {a}->{b} has no opposite");
        }
    }

    fn stock(id: &str) -> Mesh {
        build(id).unwrap()
    }

    #[test]
    fn every_model_is_closed_oriented_and_big() {
        for s in &STOCK {
            let m = stock(s.id);
            let st = m.stats();
            assert!(st.is_watertight(), "{}: {st:?}", s.id);
            assert!(!st.is_inverted() && st.signed_volume > 0.0, "{}", s.id);
            assert_consistently_oriented(&m, s.id);
            assert!(st.bounds.unwrap().size().max_element() > 300.0, "{} should exceed a print bed", s.id);
        }
        assert!(build("nope").is_none());
    }

    #[test]
    fn dome_volume_matches_the_analytic_shell() {
        let st = stock("dome").stats();
        let exact = 2.0 / 3.0 * PI as f64 * (200f64.powi(3) - 196f64.powi(3));
        // The tessellation sits inside the true sphere, so allow a small deficit.
        assert!((st.signed_volume / exact - 1.0).abs() < 0.02, "{} vs {exact}", st.signed_volume);
        let b = st.bounds.unwrap();
        assert!((b.size().x - 400.0).abs() < 1.0 && (b.size().z - 200.0).abs() < 0.01, "{b:?}");
    }

    #[test]
    fn wing_volume_matches_the_section_area_and_caps_are_not_folded() {
        let (span, c0, c1, thickness) = (900.0f64, 300.0f64, 150.0f64, 0.18f32);
        let m = wing(span as f32, c0 as f32, c1 as f32, 120.0, thickness);
        // Section area of the unit-chord outline by the shoelace formula.
        const K: usize = 48;
        let xs: Vec<f32> = (0..=K).map(|k| 0.5 * (1.0 - (PI * k as f32 / K as f32).cos())).collect();
        let mut o: Vec<(f64, f64)> = xs.iter().map(|&x| (x as f64, naca_half_thickness(x, thickness) as f64)).collect();
        o.extend(xs[1..K].iter().rev().map(|&x| (x as f64, -(naca_half_thickness(x, thickness) as f64))));
        let area: f64 = (0..o.len()).map(|k| o[k].0 * o[(k + 1) % o.len()].1 - o[(k + 1) % o.len()].0 * o[k].1).sum::<f64>().abs() / 2.0;
        let exact = area * span * (c0 * c0 + c0 * c1 + c1 * c1) / 3.0;
        let v = m.stats().signed_volume;
        assert!((v / exact - 1.0).abs() < 1e-3, "{v} vs {exact}");
        // Every cap triangle on the root face must have the same orientation (the fan is not folded).
        let signs: Vec<bool> = m
            .triangles
            .iter()
            .filter(|t| t.iter().all(|&i| m.vertices[i as usize].y == 0.0))
            .map(|t| {
                let [a, b, c] = t.map(|i| m.vertices[i as usize]);
                (b - a).cross(c - a).y < 0.0
            })
            .collect();
        assert!(signs.len() > 50 && signs.iter().all(|&s| s == signs[0]), "{} cap triangles", signs.len());
        let size = m.bounds().unwrap().size();
        assert!((size.y - 900.0).abs() < 1e-3 && size.z > 50.0 && size.z < 60.0, "{size:?}");
    }

    #[test]
    fn panel_keeps_its_thickness_everywhere() {
        let (l, w, t) = (1100.0, 600.0, 3.0);
        let m = panel(l, w, t);
        // Volume ≈ area × thickness (surface area measured on the outer skin).
        let st = m.stats();
        let skin = st.surface_area / 2.0;
        assert!((st.signed_volume / (skin * t as f64) - 1.0).abs() < 0.03, "{} vs {}", st.signed_volume, skin * t as f64);
        let size = st.bounds.unwrap().size();
        assert!((size.x - l).abs() < 1.0 && (size.y - w).abs() < 1.0, "{size:?}");
    }
}
