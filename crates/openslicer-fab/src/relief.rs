//! Depth relief: the view of a model from one direction, as a heightfield slab (with optional bas-relief
//! compression of the depth range).

use crate::{ensure_fit, FabError, FabReport, FabResult, Part};
use glam::{DMat3, DVec3};
use openslicer_core::control::Control;
use openslicer_core::Mesh;
use openslicer_geom::build::heightfield;
use openslicer_split::exec::Piece;

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Compression {
    /// Heights keep their proportions.
    Linear,
    /// Logarithmic compression: `strength` > 0 raises the foreground less and keeps detail in shallow areas (bas-relief).
    Log { strength: f64 },
}

#[derive(Clone, Debug)]
pub struct ReliefSettings {
    /// Direction the viewer looks along. The relief rises towards the viewer.
    pub view_direction: DVec3,
    /// Sample spacing in mm.
    pub pixel: f64,
    /// Thickness of the flat backing slab (mm).
    pub base_thickness: f64,
    /// Height of the relief above the slab for the nearest surface (mm).
    pub relief_height: f64,
    pub compression: Compression,
    pub usable: DVec3,
    /// Refuse grids with more samples than this.
    pub max_samples: usize,
}

impl ReliefSettings {
    pub fn new(usable: DVec3) -> Self {
        Self {
            view_direction: -DVec3::Z,
            pixel: 0.5,
            base_thickness: 2.0,
            relief_height: 6.0,
            compression: Compression::Linear,
            usable,
            max_samples: 4_000_000,
        }
    }
}

/// Rotation that takes `from` to `to` (both need not be normalised).
fn rotation_between(from: DVec3, to: DVec3) -> DMat3 {
    let (f, t) = (from.normalize(), to.normalize());
    let c = f.dot(t);
    if c > 1.0 - 1e-12 {
        return DMat3::IDENTITY;
    }
    if c < -1.0 + 1e-12 {
        let axis = if f.x.abs() < 0.9 { DVec3::X } else { DVec3::Y };
        let a = f.cross(axis).normalize();
        return DMat3::from_axis_angle(a, std::f64::consts::PI);
    }
    let axis = f.cross(t).normalize();
    DMat3::from_axis_angle(axis, c.acos())
}

/// Rasterises the nearest-surface height of `tris` (in a frame where +Z points at the viewer) into a grid.
fn zbuffer(verts: &[DVec3], tris: &[[u32; 3]], min: DVec3, nx: usize, ny: usize, d: f64, ctl: &Control) -> Result<Vec<f64>, FabError> {
    let mut z = vec![f64::NEG_INFINITY; nx * ny];
    for (ti, t) in tris.iter().enumerate() {
        if ti % 65536 == 0 {
            ctl.check()?;
        }
        let (a, b, c) = (verts[t[0] as usize] - min, verts[t[1] as usize] - min, verts[t[2] as usize] - min);
        let det = (b.x - a.x) * (c.y - a.y) - (c.x - a.x) * (b.y - a.y);
        if det.abs() < 1e-18 {
            continue;
        }
        let (x0, x1) = (a.x.min(b.x).min(c.x), a.x.max(b.x).max(c.x));
        let (y0, y1) = (a.y.min(b.y).min(c.y), a.y.max(b.y).max(c.y));
        let (i0, i1) = (((x0 / d).ceil().max(0.0)) as usize, (((x1 / d).floor()) as usize).min(nx - 1));
        let (j0, j1) = (((y0 / d).ceil().max(0.0)) as usize, (((y1 / d).floor()) as usize).min(ny - 1));
        for j in j0..=j1.min(ny - 1) {
            let y = j as f64 * d;
            for i in i0..=i1 {
                let x = i as f64 * d;
                let u = ((x - a.x) * (c.y - a.y) - (c.x - a.x) * (y - a.y)) / det;
                let v = ((b.x - a.x) * (y - a.y) - (x - a.x) * (b.y - a.y)) / det;
                const EPS: f64 = -1e-9;
                if u >= EPS && v >= EPS && u + v <= 1.0 - EPS {
                    let h = a.z + u * (b.z - a.z) + v * (c.z - a.z);
                    let slot = &mut z[j * nx + i];
                    if h > *slot {
                        *slot = h;
                    }
                }
            }
        }
    }
    Ok(z)
}

pub fn relief_from_mesh(mesh: &Mesh, s: &ReliefSettings, ctl: &Control) -> Result<FabResult, FabError> {
    if mesh.is_empty() {
        return Err(FabError::Invalid("the mesh is empty".into()));
    }
    if !(s.pixel > 0.0 && s.base_thickness > 0.0 && s.relief_height > 0.0 && s.view_direction.length() > 0.0) {
        return Err(FabError::Invalid("pixel size, thicknesses and view direction must be positive / non-zero".into()));
    }
    if let Compression::Log { strength } = s.compression {
        if !(strength > 0.0 && strength.is_finite()) {
            return Err(FabError::Invalid("the compression strength must be positive".into()));
        }
    }
    let r = rotation_between(-s.view_direction, DVec3::Z);
    // Snap to a 1e-9 mm grid: exact rotations (such as 180 degrees) leave ~1e-15 noise that would make
    // coverage along the model's boundary inconsistent between faces.
    let snap = |x: f64| (x * 1e9).round() / 1e9;
    let verts: Vec<DVec3> =
        mesh.vertices.iter().map(|v| (r * v.as_dvec3()).to_array()).map(|a| DVec3::new(snap(a[0]), snap(a[1]), snap(a[2]))).collect();
    let (min, max) = verts.iter().fold((DVec3::splat(f64::MAX), DVec3::splat(f64::MIN)), |(l, h), p| (l.min(*p), h.max(*p)));
    let (w, h) = (max.x - min.x, max.y - min.y);
    let (nx, ny) = ((w / s.pixel).ceil() as usize + 1, (h / s.pixel).ceil() as usize + 1);
    if nx < 2 || ny < 2 {
        return Err(FabError::Invalid("the view is too small for this pixel size".into()));
    }
    if nx * ny > s.max_samples {
        return Err(FabError::Invalid(format!("{nx} × {ny} samples is too many; increase the pixel size")));
    }
    ctl.report(0.1, "Rendering depth");
    let z = zbuffer(&verts, &mesh.triangles, min, nx, ny, s.pixel, ctl)?;
    // Normalise over the *visible* surface (what the viewer sees), not over hidden back geometry.
    let (zmin, zmax) = z.iter().filter(|v| v.is_finite()).fold((f64::MAX, f64::MIN), |(l, h), &v| (l.min(v), h.max(v)));
    let covered = z.iter().filter(|v| v.is_finite()).count();
    if covered == 0 {
        return Err(FabError::Invalid("the model does not cover any sample; use a smaller pixel size".into()));
    }
    let depth = (zmax - zmin).max(1e-9);
    let heights: Vec<f32> = z
        .iter()
        .map(|&zv| {
            let t = if zv.is_finite() { ((zv - zmin) / depth).clamp(0.0, 1.0) } else { 0.0 };
            let t = match s.compression {
                Compression::Linear => t,
                Compression::Log { strength } => (1.0 + strength * t).ln() / (1.0 + strength).ln(),
            };
            (s.base_thickness + t * s.relief_height) as f32
        })
        .collect();
    let m = heightfield(&heights, nx, ny, s.pixel, s.pixel, 0.0).map_err(FabError::Invalid)?;
    let mut report = FabReport { method: "Relief".into(), source_volume: mesh.stats().signed_volume, ..Default::default() };
    report.notes.push(format!("{nx} × {ny} samples, {:.0}% of the view covered by the model", 100.0 * covered as f64 / (nx * ny) as f64));
    let parts: Vec<Part> = Piece::new(m).into_iter().map(|piece| Part { piece, label: "Relief".into(), outline: None }).collect();
    let parts = ensure_fit(parts, s.usable, &mut report, ctl)?;
    // The relief is a different object from the source: the volume ratio is not meaningful.
    let mut res = FabResult::finish(parts, report);
    res.report.fill_ratio = 0.0;
    Ok(res)
}

#[cfg(test)]
mod tests {
    use super::*;
    use glam::Vec3;

    fn sphere(r: f32) -> Mesh {
        let (nu, nv) = (64usize, 32usize);
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
    }

    #[test]
    fn rotation_between_maps_vectors_correctly() {
        for from in [DVec3::Z, DVec3::X, DVec3::new(1.0, 2.0, 3.0), -DVec3::Z] {
            let r = rotation_between(from, DVec3::Z);
            assert!((r * from.normalize() - DVec3::Z).length() < 1e-9, "{from:?}");
            assert!((r.determinant() - 1.0).abs() < 1e-9);
        }
    }

    #[test]
    fn dome_relief_peaks_at_the_centre_and_is_flat_outside() {
        let mut s = ReliefSettings::new(DVec3::splat(250.0));
        s.pixel = 1.0;
        s.base_thickness = 2.0;
        s.relief_height = 10.0;
        let r = relief_from_mesh(&sphere(20.0), &s, &Control::none()).unwrap();
        assert!(r.report.is_valid(), "{:?}", r.report);
        assert_eq!(r.parts.len(), 1);
        let b = r.parts[0].piece.bounds;
        // The nearest point of the sphere (the top) is at full relief height.
        assert!((b.max.z - 12.0).abs() < 0.05, "{}", b.max.z);
        // Footprint is the sphere's diameter.
        assert!((b.size().x - 40.0).abs() < 1.5 && (b.size().y - 40.0).abs() < 1.5);
        // Volume between a flat slab and a full-height slab.
        let v = r.parts[0].piece.volume;
        assert!(v > 40.0 * 40.0 * 2.0 && v < 40.0 * 40.0 * 12.0, "{v}");
    }

    #[test]
    fn view_direction_selects_the_side_that_is_rendered() {
        // A stepped part: the left half (x 0..10) is 10 mm tall and the right half (x 10..20) is 20 mm tall.
        let mut low = Mesh::cuboid(Vec3::new(10.0, 20.0, 10.0));
        low.translate(Vec3::new(5.0, 10.0, 5.0));
        let mut high = Mesh::cuboid(Vec3::new(10.0, 20.0, 20.0));
        high.translate(Vec3::new(15.0, 10.0, 10.0));
        let both = openslicer_geom::build::merge(&[low, high]);
        let mut s = ReliefSettings::new(DVec3::splat(250.0));
        s.pixel = 1.0;
        s.base_thickness = 2.0;
        s.relief_height = 6.0;
        // From above the right half is nearer: it rises the full 6 mm over the left half.
        let top = relief_from_mesh(&both, &s, &Control::none()).unwrap();
        assert!(top.report.is_valid(), "{:?}", top.report);
        let zs = |m: &Mesh, x: f32| m.vertices.iter().filter(|v| (v.x - x).abs() < 0.01).map(|v| v.z).fold(0.0f32, f32::max);
        let m = &top.parts[0].piece.mesh;
        assert!((zs(m, 5.0) - 2.0).abs() < 1e-3, "left half stays on the base: {}", zs(m, 5.0));
        assert!((zs(m, 15.0) - 8.0).abs() < 1e-3, "right half rises: {}", zs(m, 15.0));
        // From below both bottoms are flat at z = 0, so there is no relief at all.
        s.view_direction = DVec3::Z;
        let bottom = relief_from_mesh(&both, &s, &Control::none()).unwrap();
        let mb = &bottom.parts[0].piece.mesh;
        assert!((zs(mb, 5.0) - 2.0).abs() < 1e-3 && (zs(mb, 15.0) - 2.0).abs() < 1e-3);
        // A flat face is a plain slab.
        assert!(bottom.parts[0].piece.volume < top.parts[0].piece.volume);
    }

    #[test]
    fn log_compression_lifts_shallow_detail() {
        let mut s = ReliefSettings::new(DVec3::splat(250.0));
        s.pixel = 1.0;
        let lin = relief_from_mesh(&sphere(20.0), &s, &Control::none()).unwrap();
        s.compression = Compression::Log { strength: 8.0 };
        let log = relief_from_mesh(&sphere(20.0), &s, &Control::none()).unwrap();
        // Same peak height, but compression raises the mid-range, so there is more material overall.
        assert!((lin.parts[0].piece.bounds.max.z - log.parts[0].piece.bounds.max.z).abs() < 0.05);
        assert!(log.parts[0].piece.volume > lin.parts[0].piece.volume);
    }

    #[test]
    fn bad_inputs_are_rejected() {
        let m = sphere(10.0);
        let mut s = ReliefSettings::new(DVec3::splat(250.0));
        s.pixel = 0.0;
        assert!(relief_from_mesh(&m, &s, &Control::none()).is_err());
        let mut s = ReliefSettings::new(DVec3::splat(250.0));
        s.pixel = 0.01;
        s.max_samples = 1000;
        assert!(relief_from_mesh(&m, &s, &Control::none()).unwrap_err().to_string().contains("too many"));
        assert!(relief_from_mesh(&Mesh::default(), &ReliefSettings::new(DVec3::splat(250.0)), &Control::none()).is_err());
        let mut s = ReliefSettings::new(DVec3::splat(250.0));
        s.compression = Compression::Log { strength: -1.0 };
        assert!(relief_from_mesh(&m, &s, &Control::none()).is_err());
    }
}
