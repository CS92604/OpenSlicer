//! Stacked layers: the model as a pile of flat sheets, each the cross-section at the middle of its layer.

use crate::{ensure_fit, FabError, FabReport, FabResult, Outline, Part};
use glam::DVec3;
use openslicer_core::control::Control;
use openslicer_core::Mesh;
use openslicer_geom::build::{self, LocalFrame};
use openslicer_geom::poly2d;
use openslicer_geom::region::Polygon;
use openslicer_geom::section::cross_section;
use openslicer_geom::Plane;
use openslicer_joints::placement::{candidates, FarthestPoint};
use openslicer_split::exec::Piece;

#[derive(Clone, Debug)]
pub struct StackSettings {
    /// Stacking direction (layers are perpendicular to it).
    pub axis: DVec3,
    /// Target sheet thickness in mm; the actual thickness is the model's extent divided into equal layers.
    pub layer_thickness: f64,
    /// Number of alignment holes through every layer (0 for none), with their diameter. The holes sit in the
    /// region every layer shares; the dowel rods that go through them are returned as extra parts.
    pub registration_holes: usize,
    pub hole_diameter: f64,
    pub hole_clearance: f64,
    /// Usable build volume; oversized layers are tiled.
    pub usable: DVec3,
}

impl StackSettings {
    pub fn new(usable: DVec3) -> Self {
        Self { axis: DVec3::Z, layer_thickness: 3.0, registration_holes: 0, hole_diameter: 4.0, hole_clearance: 0.2, usable }
    }
}

pub fn stack(mesh: &Mesh, s: &StackSettings, ctl: &Control) -> Result<FabResult, FabError> {
    if mesh.is_empty() {
        return Err(FabError::Invalid("the mesh is empty".into()));
    }
    if !(s.layer_thickness.is_finite() && s.layer_thickness > 0.0) {
        return Err(FabError::Invalid("the layer thickness must be a positive number".into()));
    }
    let n = s.axis.try_normalize().ok_or_else(|| FabError::Invalid("the stacking axis must not be zero".into()))?;
    let stats = mesh.stats();
    let mut report = FabReport { method: "Stacked layers".into(), source_volume: stats.signed_volume, ..Default::default() };
    if !stats.is_watertight() {
        return Err(FabError::Invalid("the mesh must be watertight to take cross-sections; repair it first".into()));
    }
    let (lo, hi) = mesh.vertices.iter().fold((f64::MAX, f64::MIN), |(l, h), v| {
        let d = v.as_dvec3().dot(n);
        (l.min(d), h.max(d))
    });
    let count = (((hi - lo) / s.layer_thickness).ceil() as usize).max(1);
    if count > 5000 {
        return Err(FabError::Invalid(format!("{count} layers would be needed; increase the layer thickness")));
    }
    let t = (hi - lo) / count as f64;
    report.max_deviation = Some(t / 2.0);

    // Cross-section at the middle of each layer.
    let mut layers: Vec<(f64, Vec<Polygon>)> = Vec::new();
    let mut frame = None;
    for k in 0..count {
        ctl.check()?;
        ctl.report(k as f32 / count as f32, &format!("Sectioning layer {} of {count}", k + 1));
        let mid = lo + (k as f64 + 0.5) * t;
        let plane = Plane { n, d: mid };
        match cross_section(mesh, &plane) {
            Ok(r) => {
                frame.get_or_insert(plane.frame());
                if !r.polygons.is_empty() {
                    layers.push((lo + k as f64 * t, r.polygons));
                }
            }
            Err(e) => return Err(FabError::Invalid(format!("layer {}: {e}", k + 1))),
        }
    }
    if layers.is_empty() {
        return Err(FabError::Invalid("the model has no cross-sections".into()));
    }

    // Alignment holes in the region common to all layers.
    let mut rods: Vec<Part> = Vec::new();
    if s.registration_holes > 0 {
        let mut common = layers[0].1.clone();
        for (_, p) in &layers[1..] {
            common = poly2d::intersect(&common, p);
        }
        let r = s.hole_diameter / 2.0 + s.hole_clearance;
        let inset = r + 1.5;
        let cands = candidates(&common, inset, (s.hole_diameter).max(2.0), 1500);
        let mut fp = FarthestPoint::new(&common, cands);
        let mut holes: Vec<glam::DVec2> = Vec::new();
        while let Some(i) = fp.next(f64::INFINITY, s.registration_holes, s.registration_holes) {
            holes.push(fp.point(i));
            fp.accept(i);
        }
        if holes.len() < s.registration_holes {
            report.notes.push(format!(
                "only {} of {} alignment holes fit in the region every layer shares",
                holes.len(),
                s.registration_holes
            ));
        }
        if !holes.is_empty() {
            let discs: Vec<Polygon> = holes.iter().map(|&c| Polygon { outer: poly2d::circle(c, r, 32), holes: vec![] }).collect();
            for (_, polys) in &mut layers {
                *polys = poly2d::difference(polys, &discs);
            }
            let f = frame.unwrap();
            for (i, c) in holes.iter().enumerate() {
                let rod = build::cylinder(s.hole_diameter / 2.0, 0.0, hi - lo, 0.4, 0.4, 32);
                let placed = LocalFrame::on_plane(&f, *c, 0.0).place(&rod);
                // The rod starts at the first layer's bottom face: shift along the axis to `lo`.
                let shift = (n * (lo - f.origin.dot(n))).as_vec3();
                let mesh = Mesh::new(placed.vertices.iter().map(|v| *v + shift).collect(), placed.triangles.clone());
                if let Some(mut piece) = Piece::new(mesh) {
                    piece.name = format!("Rod {}", i + 1);
                    rods.push(Part { piece, label: format!("Rod {}", i + 1), outline: None });
                }
            }
        }
    }

    let f = frame.unwrap();
    let mut parts: Vec<Part> = Vec::new();
    for (i, (z0, polys)) in layers.iter().enumerate() {
        if polys.is_empty() {
            report.notes.push(format!("layer {} became empty after drilling the alignment holes", i + 1));
            continue;
        }
        // Extrude in the plane frame (z along n), then move so the layer spans [z0, z0 + t] along the axis.
        let local = build::extrude(polys, z0 - f.origin.dot(n), z0 + t - f.origin.dot(n));
        let mesh = LocalFrame::on_plane(&f, glam::DVec2::ZERO, 0.0).place(&local);
        if let Some(piece) = Piece::new(mesh) {
            parts.push(Part { piece, label: format!("Layer {}", i + 1), outline: Some(Outline { polygons: polys.clone(), thickness: t }) });
        }
    }
    let parts = ensure_fit(parts, s.usable, &mut report, ctl)?;
    let mut all = parts;
    all.extend(rods);
    report.notes.push(format!("{count} layers of {t:.2} mm"));
    Ok(FabResult::finish(all, report))
}

#[cfg(test)]
mod tests {
    use super::*;
    use glam::Vec3;

    fn block(size: Vec3) -> Mesh {
        let mut m = Mesh::cuboid(size);
        m.translate(size * 0.5);
        m
    }

    #[test]
    fn a_block_stacks_into_exact_layers_with_no_volume_error() {
        let m = block(Vec3::new(60.0, 40.0, 30.0));
        let mut s = StackSettings::new(DVec3::splat(250.0));
        s.layer_thickness = 3.0;
        let r = stack(&m, &s, &Control::none()).unwrap();
        assert!(r.report.is_valid(), "{:?}", r.report);
        assert_eq!(r.parts.len(), 10);
        assert!((r.report.fill_ratio - 1.0).abs() < 1e-6);
        assert!(r.parts.iter().all(|p| p.outline.is_some()));
        // Layers are stacked contiguously.
        let mut zs: Vec<(f32, f32)> = r.parts.iter().map(|p| (p.piece.bounds.min.z, p.piece.bounds.max.z)).collect();
        zs.sort_by(|a, b| a.0.total_cmp(&b.0));
        for w in zs.windows(2) {
            assert!((w[0].1 - w[1].0).abs() < 1e-4);
        }
    }

    #[test]
    fn a_sphere_stack_approximates_the_volume_within_the_layer_error() {
        let m = openslicer_split_test_sphere(40.0);
        let mut s = StackSettings::new(DVec3::splat(250.0));
        s.layer_thickness = 2.0;
        let r = stack(&m, &s, &Control::none()).unwrap();
        assert!(r.report.is_valid(), "{:?}", r.report);
        assert!(r.parts.len() >= 39);
        assert!((r.report.fill_ratio - 1.0).abs() < 0.02, "{}", r.report.fill_ratio);
        // 80 mm tall in 40 layers of 2 mm: the stated deviation bound is half a layer.
        assert!((r.report.max_deviation.unwrap() - 1.0).abs() < 1e-9);
    }

    #[test]
    fn registration_holes_and_rods_are_generated_and_fit() {
        let m = block(Vec3::new(80.0, 80.0, 12.0));
        let mut s = StackSettings::new(DVec3::splat(250.0));
        s.layer_thickness = 3.0;
        s.registration_holes = 2;
        let r = stack(&m, &s, &Control::none()).unwrap();
        assert!(r.report.is_valid(), "{:?}", r.report);
        let layers: Vec<_> = r.parts.iter().filter(|p| p.outline.is_some()).collect();
        let rods: Vec<_> = r.parts.iter().filter(|p| p.outline.is_none()).collect();
        assert_eq!(layers.len(), 4);
        assert_eq!(rods.len(), 2);
        // Each layer has two holes, so its volume is the plate minus two clearance-sized cylinders.
        let plate = 80.0 * 80.0 * 3.0;
        for l in &layers {
            assert!(l.piece.volume < plate - 2.0 * std::f64::consts::PI * 2.2 * 2.2 * 3.0 * 0.95);
            assert_eq!(l.outline.as_ref().unwrap().polygons[0].holes.len(), 2);
        }
        // Rods are slightly smaller than the holes.
        assert!(rods[0].piece.bounds.size().x < 2.0 * 2.2);
    }

    #[test]
    fn oversized_layers_are_tiled_to_fit() {
        let m = block(Vec3::new(400.0, 100.0, 10.0));
        let mut s = StackSettings::new(DVec3::new(220.0, 220.0, 250.0));
        s.layer_thickness = 5.0;
        let r = stack(&m, &s, &Control::none()).unwrap();
        assert!(r.report.is_valid(), "{:?}", r.report);
        assert!(r.parts.len() >= 4, "two layers, each tiled in two: {}", r.parts.len());
        assert!(r.report.notes.iter().any(|n| n.contains("tiled")));
        assert!(r.parts.iter().all(|p| openslicer_split::fit::best_orientation(&p.piece.mesh, s.usable, &[]).is_some()));
        assert!((r.report.fill_ratio - 1.0).abs() < 1e-6);
    }

    #[test]
    fn open_meshes_and_bad_settings_are_refused_clearly() {
        let mut m = block(Vec3::splat(10.0));
        m.triangles.pop();
        let s = StackSettings::new(DVec3::splat(250.0));
        assert!(stack(&m, &s, &Control::none()).unwrap_err().to_string().contains("watertight"));
        let mut bad = StackSettings::new(DVec3::splat(250.0));
        bad.layer_thickness = 0.0;
        assert!(stack(&block(Vec3::splat(10.0)), &bad, &Control::none()).is_err());
        let mut z = StackSettings::new(DVec3::splat(250.0));
        z.axis = DVec3::ZERO;
        assert!(stack(&block(Vec3::splat(10.0)), &z, &Control::none()).is_err());
    }

    fn openslicer_split_test_sphere(r: f32) -> Mesh {
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
}
