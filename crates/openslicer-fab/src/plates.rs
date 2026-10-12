//! Plates: the surface approximated by flat panels.
//!
//! Triangles are grouped into near-planar regions by region growing (neighbouring triangles whose normals
//! and positions stay within tolerance of the region's plane). Each region becomes a flat plate of the chosen
//! thickness lying just inside the surface. Plate outlines are kept so they can be cut from sheet stock.

use crate::{ensure_fit, FabError, FabReport, FabResult, Outline, Part};
use glam::{DVec2, DVec3};
use openslicer_core::control::Control;
use openslicer_core::Mesh;
use openslicer_geom::build::{self, LocalFrame};
use openslicer_geom::poly2d;
use openslicer_geom::region::Polygon;
use openslicer_geom::Plane;
use openslicer_split::exec::Piece;
use rustc_hash::FxHashMap;

#[derive(Clone, Debug)]
pub struct PlateSettings {
    /// Sheet thickness (mm).
    pub thickness: f64,
    /// Neighbouring triangles join a plate while their normal stays within this angle of the plate normal (degrees).
    pub max_normal_angle_deg: f64,
    /// ... and while all their vertices stay within this distance of the plate plane (mm).
    pub max_deviation: f64,
    /// Plates smaller than this area (mm²) are dropped (and counted as uncovered surface).
    pub min_area: f64,
    pub usable: DVec3,
}

impl PlateSettings {
    pub fn new(usable: DVec3) -> Self {
        Self { thickness: 3.0, max_normal_angle_deg: 12.0, max_deviation: 1.5, min_area: 100.0, usable }
    }
}

struct Tri {
    normal: DVec3,
    area: f64,
    verts: [DVec3; 3],
}

pub fn plates(mesh: &Mesh, s: &PlateSettings, ctl: &Control) -> Result<FabResult, FabError> {
    if mesh.is_empty() {
        return Err(FabError::Invalid("the mesh is empty".into()));
    }
    if !(s.thickness > 0.0 && s.max_deviation > 0.0 && s.min_area >= 0.0 && (1.0..=60.0).contains(&s.max_normal_angle_deg)) {
        return Err(FabError::Invalid("thickness and deviation must be positive, and the normal angle between 1 and 60 degrees".into()));
    }
    let stats = mesh.stats();
    if stats.signed_volume <= 0.0 {
        return Err(FabError::Invalid("the mesh has inward-facing normals; repair it first so plates sit inside the surface".into()));
    }
    let tris: Vec<Tri> = (0..mesh.triangles.len())
        .map(|i| {
            let v = mesh.triangle(i).map(|p| p.as_dvec3());
            let c = (v[1] - v[0]).cross(v[2] - v[0]);
            Tri { normal: c.normalize_or_zero(), area: c.length() * 0.5, verts: v }
        })
        .collect();
    // Edge adjacency.
    let mut edge_tris: FxHashMap<(u32, u32), Vec<u32>> = FxHashMap::default();
    for (i, t) in mesh.triangles.iter().enumerate() {
        for k in 0..3 {
            let (a, b) = (t[k], t[(k + 1) % 3]);
            edge_tris.entry((a.min(b), a.max(b))).or_default().push(i as u32);
        }
    }
    let neighbours = |i: usize| -> Vec<usize> {
        let t = mesh.triangles[i];
        let mut out = Vec::new();
        for k in 0..3 {
            let (a, b) = (t[k], t[(k + 1) % 3]);
            if let Some(list) = edge_tris.get(&(a.min(b), a.max(b))) {
                out.extend(list.iter().map(|&x| x as usize).filter(|&x| x != i));
            }
        }
        out
    };

    let cos_max = s.max_normal_angle_deg.to_radians().cos();
    let mut order: Vec<usize> = (0..tris.len()).filter(|&i| tris[i].area > 1e-12).collect();
    order.sort_by(|&a, &b| tris[b].area.total_cmp(&tris[a].area));
    let mut region_of = vec![usize::MAX; tris.len()];
    let mut regions: Vec<Vec<usize>> = Vec::new();
    for (k, &seed) in order.iter().enumerate() {
        if k % 4096 == 0 {
            ctl.check()?;
        }
        if region_of[seed] != usize::MAX {
            continue;
        }
        let id = regions.len();
        let origin = (tris[seed].verts[0] + tris[seed].verts[1] + tris[seed].verts[2]) / 3.0;
        let plane = Plane { n: tris[seed].normal, d: tris[seed].normal.dot(origin) };
        let mut members = vec![seed];
        region_of[seed] = id;
        let mut stack = vec![seed];
        while let Some(t) = stack.pop() {
            for n in neighbours(t) {
                if region_of[n] != usize::MAX || tris[n].area <= 1e-12 {
                    continue;
                }
                let ok = tris[n].normal.dot(plane.n) >= cos_max
                    && tris[n].verts.iter().all(|&v| plane.signed_distance(v).abs() <= s.max_deviation);
                if ok {
                    region_of[n] = id;
                    members.push(n);
                    stack.push(n);
                }
            }
        }
        regions.push(members);
    }

    let total_area: f64 = tris.iter().map(|t| t.area).sum();
    let mut parts: Vec<Part> = Vec::new();
    let mut covered = 0.0;
    let mut worst_dev = 0.0f64;
    for members in &regions {
        let area: f64 = members.iter().map(|&i| tris[i].area).sum();
        if area < s.min_area {
            continue;
        }
        // Best-fit plane: area-weighted normal through the area-weighted centroid.
        let nsum: DVec3 = members.iter().map(|&i| tris[i].normal * tris[i].area).sum();
        let centroid: DVec3 =
            members.iter().map(|&i| (tris[i].verts[0] + tris[i].verts[1] + tris[i].verts[2]) / 3.0 * tris[i].area).sum::<DVec3>() / area;
        let Some(n) = nsum.try_normalize() else { continue };
        let plane = Plane { n, d: n.dot(centroid) };
        let frame = plane.frame();
        let projected: Vec<Polygon> = members
            .iter()
            .map(|&i| Polygon { outer: tris[i].verts.iter().map(|&v| frame.to_2d(v)).collect::<Vec<DVec2>>(), holes: vec![] })
            .collect();
        let outline = poly2d::union_all(&projected);
        if outline.is_empty() {
            continue;
        }
        for &i in members {
            for v in tris[i].verts {
                worst_dev = worst_dev.max(plane.signed_distance(v).abs());
            }
        }
        // The plate lies just inside the surface: from -thickness to 0 along the outward normal.
        let local = build::extrude(&outline, -s.thickness, 0.0);
        let mesh = LocalFrame::on_plane(&frame, DVec2::ZERO, 0.0).place(&local);
        if let Some(piece) = Piece::new(mesh) {
            covered += area;
            parts.push(Part {
                piece,
                label: format!("Plate ({area:.0} mm²)"),
                outline: Some(Outline { polygons: outline, thickness: s.thickness }),
            });
        }
    }
    if parts.is_empty() {
        return Err(FabError::Invalid("no plate is large enough; lower the minimum plate area or raise the angle tolerance".into()));
    }
    let mut report = FabReport { method: "Plates".into(), source_volume: stats.signed_volume, ..Default::default() };
    report.max_deviation = Some(worst_dev.max(s.max_deviation));
    report.notes.push(format!("{} plates cover {:.1}% of the surface area", parts.len(), 100.0 * covered / total_area.max(1e-12)));
    if covered < 0.8 * total_area {
        report.notes.push("less than 80% of the surface is covered; lower the minimum plate area or raise the tolerances".into());
    }
    let parts = ensure_fit(parts, s.usable, &mut report, ctl)?;
    Ok(FabResult::finish(parts, report))
}

#[cfg(test)]
mod tests {
    use super::*;
    use glam::Vec3;

    fn sphere(r: f32) -> Mesh {
        let (nu, nv) = (48usize, 24usize);
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
    fn a_cube_becomes_six_plates_of_the_right_size() {
        let mut m = Mesh::cuboid(Vec3::splat(60.0));
        m.translate(Vec3::splat(30.0));
        let r = plates(&m, &PlateSettings::new(DVec3::splat(250.0)), &Control::none()).unwrap();
        assert!(r.report.is_valid(), "{:?}", r.report);
        assert_eq!(r.parts.len(), 6);
        for p in &r.parts {
            assert!((p.piece.volume - 60.0 * 60.0 * 3.0).abs() < 1e-3, "{}", p.piece.volume);
            let o = p.outline.as_ref().unwrap();
            assert!((poly2d::area(&o.polygons) - 3600.0).abs() < 1e-3);
            assert_eq!(o.thickness, 3.0);
        }
        // Plates lie inside the cube: every vertex is within the original bounds.
        for p in &r.parts {
            assert!(p.piece.bounds.min.min_element() >= -1e-3 && p.piece.bounds.max.max_element() <= 60.0 + 1e-3);
        }
    }

    #[test]
    fn a_sphere_is_covered_by_many_flat_plates_inside_the_surface() {
        let m = sphere(50.0);
        let r = plates(&m, &PlateSettings { min_area: 0.0, ..PlateSettings::new(DVec3::splat(250.0)) }, &Control::none()).unwrap();
        assert!(r.report.is_valid(), "{:?}", r.report);
        assert!(r.parts.len() > 20, "{}", r.parts.len());
        assert!(r.report.notes[0].contains("cover"));
        let total_plate_area: f64 = r.parts.iter().map(|p| poly2d::area(&p.outline.as_ref().unwrap().polygons)).sum();
        let sphere_area = m.stats().surface_area;
        assert!(total_plate_area > 0.85 * sphere_area && total_plate_area < 1.2 * sphere_area, "{total_plate_area} vs {sphere_area}");
        // The plates stay within the sphere's radius (they sit inside the surface), allowing for plate edges
        // that lie on the faceted surface.
        for p in &r.parts {
            let farthest = p.piece.mesh.vertices.iter().map(|v| v.length()).fold(0.0f32, f32::max);
            assert!(farthest < 51.0, "a plate pokes out of the surface: {farthest}");
        }
    }

    #[test]
    fn tolerances_control_plate_size() {
        let m = sphere(50.0);
        let tight = plates(
            &m,
            &PlateSettings { min_area: 0.0, max_normal_angle_deg: 6.0, max_deviation: 0.5, ..PlateSettings::new(DVec3::splat(250.0)) },
            &Control::none(),
        )
        .unwrap();
        let loose = plates(
            &m,
            &PlateSettings { min_area: 0.0, max_normal_angle_deg: 25.0, max_deviation: 4.0, ..PlateSettings::new(DVec3::splat(250.0)) },
            &Control::none(),
        )
        .unwrap();
        assert!(tight.parts.len() > loose.parts.len(), "{} vs {}", tight.parts.len(), loose.parts.len());
    }

    #[test]
    fn inverted_meshes_and_bad_settings_are_rejected() {
        let mut m = Mesh::cuboid(Vec3::splat(10.0));
        for t in &mut m.triangles {
            t.swap(1, 2);
        }
        assert!(plates(&m, &PlateSettings::new(DVec3::splat(250.0)), &Control::none()).unwrap_err().to_string().contains("inward"));
        let ok = Mesh::cuboid(Vec3::splat(10.0));
        assert!(plates(&ok, &PlateSettings { thickness: 0.0, ..PlateSettings::new(DVec3::splat(250.0)) }, &Control::none()).is_err());
        assert!(plates(&ok, &PlateSettings { min_area: 1.0e9, ..PlateSettings::new(DVec3::splat(250.0)) }, &Control::none()).is_err());
    }
}
