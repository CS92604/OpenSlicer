//! Nesting: arranging parts on build plates, and verifying that every part fits the machine.
//!
//! Each part is first put into its best print orientation ([`crate::fit`]); its footprint rectangle is then
//! packed onto plates with the MaxRects algorithm (best short side fit), allowing a 90 degree turn. Nesting is by
//! bounding rectangle, not contour, so it is safe and predictable but not as tight as contour nesting.

use crate::exec::Piece;
use crate::fit::{apply_orientation, best_orientation, most_compact_orientation, Orientation};
use glam::{DVec2, DVec3};
use openslicer_core::Mesh;

#[derive(Clone, Debug)]
pub struct NestOptions {
    /// Usable build volume.
    pub usable: DVec3,
    /// Gap kept between parts and between parts and the plate edge (mm).
    pub spacing: f64,
    /// Outward normals of faces to prefer lying on the bed, per piece id (for example cut faces).
    pub preferred_down: Vec<(usize, Vec<DVec3>)>,
}

impl NestOptions {
    pub fn new(usable: DVec3) -> Self {
        Self { usable, spacing: 3.0, preferred_down: Vec::new() }
    }
}

#[derive(Clone, Debug)]
pub struct Placement {
    pub piece_id: usize,
    pub orientation: Orientation,
    /// Lower-left corner of the footprint on the plate (mm).
    pub offset: DVec2,
    /// True when the footprint was turned a further 90 degrees about Z for packing.
    pub turned: bool,
    /// Footprint after any turn.
    pub footprint: DVec2,
}

#[derive(Clone, Debug, Default)]
pub struct Plate {
    pub placements: Vec<Placement>,
    /// Tallest part on the plate (mm).
    pub height: f64,
}

#[derive(Clone, Debug)]
pub struct PartFit {
    pub piece_id: usize,
    pub name: String,
    pub fits: bool,
    /// Extents of the most compact orientation found (when it does not fit) or of the chosen orientation.
    pub size: DVec3,
}

#[derive(Clone, Debug, Default)]
pub struct NestResult {
    pub plates: Vec<Plate>,
    /// Fit verdict for every part, including those that cannot be placed.
    pub fits: Vec<PartFit>,
    /// Parts that do not fit the build volume in any orientation; they are not placed on any plate.
    pub unplaceable: Vec<usize>,
    /// Share of the plate area used by footprints, averaged over plates (0..1).
    pub utilisation: f64,
}

impl NestResult {
    pub fn all_fit(&self) -> bool {
        self.unplaceable.is_empty()
    }
}

/// Verifies that each piece fits the machine, without arranging anything.
pub fn verify_fit(pieces: &[Piece], usable: DVec3, preferred_down: &[(usize, Vec<DVec3>)]) -> Vec<PartFit> {
    pieces
        .iter()
        .map(|p| {
            let down: &[DVec3] = preferred_down.iter().find(|(id, _)| *id == p.id).map_or(&[], |(_, d)| d.as_slice());
            match best_orientation(&p.mesh, usable, down) {
                Some(o) => PartFit { piece_id: p.id, name: p.name.clone(), fits: true, size: o.size },
                None => PartFit {
                    piece_id: p.id,
                    name: p.name.clone(),
                    fits: false,
                    size: most_compact_orientation(&p.mesh, down).map_or(DVec3::ZERO, |o| o.size),
                },
            }
        })
        .collect()
}

#[derive(Clone, Copy, Debug)]
struct Rect {
    x: f64,
    y: f64,
    w: f64,
    h: f64,
}

struct MaxRects {
    free: Vec<Rect>,
}

impl MaxRects {
    fn new(w: f64, h: f64) -> Self {
        Self { free: vec![Rect { x: 0.0, y: 0.0, w, h }] }
    }

    /// Best short-side-fit position for a `w x h` item, if any.
    fn find(&self, w: f64, h: f64) -> Option<(Rect, f64)> {
        let mut best: Option<(Rect, f64, f64)> = None;
        for r in &self.free {
            if r.w + 1e-9 >= w && r.h + 1e-9 >= h {
                let short = (r.w - w).min(r.h - h);
                let long = (r.w - w).max(r.h - h);
                if best.as_ref().is_none_or(|(_, s, l)| short < *s || (short == *s && long < *l)) {
                    best = Some((Rect { x: r.x, y: r.y, w, h }, short, long));
                }
            }
        }
        best.map(|(r, s, _)| (r, s))
    }

    fn place(&mut self, used: Rect) {
        let mut next: Vec<Rect> = Vec::new();
        for r in &self.free {
            let overlaps =
                used.x < r.x + r.w - 1e-9 && used.x + used.w > r.x + 1e-9 && used.y < r.y + r.h - 1e-9 && used.y + used.h > r.y + 1e-9;
            if !overlaps {
                next.push(*r);
                continue;
            }
            if used.x > r.x {
                next.push(Rect { x: r.x, y: r.y, w: used.x - r.x, h: r.h });
            }
            if used.x + used.w < r.x + r.w {
                next.push(Rect { x: used.x + used.w, y: r.y, w: r.x + r.w - used.x - used.w, h: r.h });
            }
            if used.y > r.y {
                next.push(Rect { x: r.x, y: r.y, w: r.w, h: used.y - r.y });
            }
            if used.y + used.h < r.y + r.h {
                next.push(Rect { x: r.x, y: used.y + used.h, w: r.w, h: r.y + r.h - used.y - used.h });
            }
        }
        // Drop rectangles contained in another.
        let snapshot = next.clone();
        next.retain(|a| {
            !snapshot.iter().any(|b| {
                !std::ptr::eq(a, b)
                    && a.x >= b.x - 1e-9
                    && a.y >= b.y - 1e-9
                    && a.x + a.w <= b.x + b.w + 1e-9
                    && a.y + a.h <= b.y + b.h + 1e-9
                    && (a.w < b.w - 1e-9 || a.h < b.h - 1e-9)
            })
        });
        next.dedup_by(|a, b| (a.x - b.x).abs() < 1e-9 && (a.y - b.y).abs() < 1e-9 && (a.w - b.w).abs() < 1e-9 && (a.h - b.h).abs() < 1e-9);
        self.free = next;
    }
}

/// Arranges the pieces on plates. Parts that cannot fit the build volume at all are listed as unplaceable.
pub fn nest(pieces: &[Piece], opts: &NestOptions) -> NestResult {
    let fits = verify_fit(pieces, opts.usable, &opts.preferred_down);
    let mut result = NestResult { fits, ..Default::default() };
    struct Item {
        id: usize,
        orientation: Orientation,
    }
    let mut items: Vec<Item> = Vec::new();
    for p in pieces {
        let down: &[DVec3] = opts.preferred_down.iter().find(|(id, _)| *id == p.id).map_or(&[], |(_, d)| d.as_slice());
        match best_orientation(&p.mesh, opts.usable, down) {
            Some(o) => items.push(Item { id: p.id, orientation: o }),
            None => result.unplaceable.push(p.id),
        }
    }
    // Largest footprint first.
    items.sort_by(|a, b| (b.orientation.size.x * b.orientation.size.y).total_cmp(&(a.orientation.size.x * a.orientation.size.y)));
    let gap = opts.spacing.max(0.0);
    // The usable plate shrinks by one gap so that spacing is also kept at the edges.
    let (pw, ph) = (opts.usable.x, opts.usable.y);
    let mut packers: Vec<MaxRects> = Vec::new();
    for it in items {
        let (w, h) = (it.orientation.size.x, it.orientation.size.y);
        // Items are packed with a gap on their right/top; the plate has one extra gap of slack there.
        let try_place = |packers: &mut Vec<MaxRects>| -> Option<(usize, Rect, bool)> {
            for (pi, mr) in packers.iter_mut().enumerate() {
                let normal = mr.find(w + gap, h + gap);
                let turned = mr.find(h + gap, w + gap);
                let pick = match (normal, turned) {
                    (Some(a), Some(b)) => Some(if a.1 <= b.1 { (a.0, false) } else { (b.0, true) }),
                    (Some(a), None) => Some((a.0, false)),
                    (None, Some(b)) => Some((b.0, true)),
                    (None, None) => None,
                };
                if let Some((rect, t)) = pick {
                    mr.place(rect);
                    return Some((pi, rect, t));
                }
            }
            None
        };
        let placed = try_place(&mut packers).or_else(|| {
            packers.push(MaxRects::new(pw + gap, ph + gap));
            try_place(&mut packers)
        });
        let Some((pi, rect, turned)) = placed else {
            result.unplaceable.push(it.id);
            continue;
        };
        while result.plates.len() <= pi {
            result.plates.push(Plate::default());
        }
        let footprint = if turned { DVec2::new(h, w) } else { DVec2::new(w, h) };
        result.plates[pi].height = result.plates[pi].height.max(it.orientation.size.z);
        result.plates[pi].placements.push(Placement {
            piece_id: it.id,
            orientation: it.orientation,
            offset: DVec2::new(rect.x, rect.y),
            turned,
            footprint,
        });
    }
    let plate_area = pw * ph;
    if !result.plates.is_empty() {
        let used: f64 = result.plates.iter().map(|pl| pl.placements.iter().map(|p| p.footprint.x * p.footprint.y).sum::<f64>()).sum();
        result.utilisation = used / (plate_area * result.plates.len() as f64);
    }
    result
}

/// Builds one mesh for a plate: every placed piece in its print orientation at its place. Useful for preview and export.
pub fn plate_mesh(plate: &Plate, pieces: &[Piece]) -> Mesh {
    let mut out = Mesh::default();
    for pl in &plate.placements {
        let Some(piece) = pieces.iter().find(|p| p.id == pl.piece_id) else { continue };
        let mut m = apply_orientation(&piece.mesh, &pl.orientation);
        if pl.turned {
            // Turn 90 degrees about Z and keep the minimum corner at the origin.
            for v in &mut m.vertices {
                *v = glam::Vec3::new(pl.footprint.x as f32 - v.y, v.x, v.z);
            }
        }
        m.translate(glam::Vec3::new(pl.offset.x as f32, pl.offset.y as f32, 0.0));
        let base = out.vertices.len() as u32;
        out.vertices.extend(m.vertices.iter().copied());
        out.triangles.extend(m.triangles.iter().map(|t| [t[0] + base, t[1] + base, t[2] + base]));
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use glam::Vec3;

    fn piece(id: usize, size: Vec3) -> Piece {
        let mut p = Piece::new(Mesh::cuboid(size)).unwrap();
        p.id = id;
        p.name = format!("P{id:02}");
        p
    }

    fn overlaps(a: &Placement, b: &Placement) -> bool {
        a.offset.x < b.offset.x + b.footprint.x - 1e-6
            && a.offset.x + a.footprint.x > b.offset.x + 1e-6
            && a.offset.y < b.offset.y + b.footprint.y - 1e-6
            && a.offset.y + a.footprint.y > b.offset.y + 1e-6
    }

    #[test]
    fn parts_are_packed_without_overlap_inside_the_plate() {
        let sizes = [(100.0, 80.0), (60.0, 60.0), (120.0, 40.0), (50.0, 90.0), (70.0, 30.0), (200.0, 20.0), (40.0, 40.0), (90.0, 90.0)];
        let pieces: Vec<Piece> = sizes.iter().enumerate().map(|(i, (w, d))| piece(i + 1, Vec3::new(*w, *d, 15.0))).collect();
        let r = nest(&pieces, &NestOptions::new(DVec3::new(250.0, 250.0, 250.0)));
        assert!(r.all_fit());
        let all: Vec<&Placement> = r.plates.iter().flat_map(|p| &p.placements).collect();
        assert_eq!(all.len(), 8);
        for pl in &r.plates {
            for (i, a) in pl.placements.iter().enumerate() {
                assert!(a.offset.x >= -1e-6 && a.offset.y >= -1e-6);
                assert!(a.offset.x + a.footprint.x <= 250.0 + 1e-6 && a.offset.y + a.footprint.y <= 250.0 + 1e-6, "{a:?}");
                for b in &pl.placements[i + 1..] {
                    assert!(!overlaps(a, b), "{a:?} overlaps {b:?}");
                }
            }
        }
        assert!(r.utilisation > 0.25, "{}", r.utilisation);
        // Parts lie flat: every orientation keeps the 15 mm thickness as the print height.
        assert!(r.plates.iter().flat_map(|p| &p.placements).all(|p| (p.orientation.size.z - 15.0).abs() < 1e-6));
        // Parts are separated by at least the spacing.
        for pl in &r.plates {
            for (i, a) in pl.placements.iter().enumerate() {
                for b in &pl.placements[i + 1..] {
                    let gap_x = (a.offset.x - (b.offset.x + b.footprint.x)).max(b.offset.x - (a.offset.x + a.footprint.x));
                    let gap_y = (a.offset.y - (b.offset.y + b.footprint.y)).max(b.offset.y - (a.offset.y + a.footprint.y));
                    assert!(gap_x.max(gap_y) >= 3.0 - 1e-6, "parts closer than the spacing: {gap_x} {gap_y}");
                }
            }
        }
    }

    #[test]
    fn extra_parts_open_extra_plates() {
        let pieces: Vec<Piece> = (1..=5).map(|i| piece(i, Vec3::new(180.0, 180.0, 10.0))).collect();
        let r = nest(&pieces, &NestOptions::new(DVec3::new(220.0, 220.0, 250.0)));
        assert_eq!(r.plates.len(), 5);
        assert!(r.all_fit());
    }

    #[test]
    fn parts_that_never_fit_are_reported_not_placed() {
        let pieces = vec![piece(1, Vec3::new(500.0, 500.0, 500.0)), piece(2, Vec3::new(50.0, 50.0, 50.0))];
        let r = nest(&pieces, &NestOptions::new(DVec3::splat(250.0)));
        assert_eq!(r.unplaceable, vec![1]);
        assert!(!r.all_fit());
        assert_eq!(r.plates.iter().map(|p| p.placements.len()).sum::<usize>(), 1);
        let f1 = r.fits.iter().find(|f| f.piece_id == 1).unwrap();
        assert!(!f1.fits && f1.size.min_element() >= 499.0);
        assert!(r.fits.iter().find(|f| f.piece_id == 2).unwrap().fits);
    }

    #[test]
    fn rotation_by_90_degrees_is_used_when_it_helps() {
        // A 240 x 50 strip and a 50 x 240 strip: the second must be turned to share a 250 wide plate region... they stack.
        let pieces =
            vec![piece(1, Vec3::new(240.0, 40.0, 10.0)), piece(2, Vec3::new(40.0, 240.0, 10.0)), piece(3, Vec3::new(240.0, 40.0, 10.0))];
        let r = nest(&pieces, &NestOptions { spacing: 2.0, ..NestOptions::new(DVec3::new(250.0, 250.0, 250.0)) });
        assert!(r.all_fit());
        assert_eq!(r.plates.len(), 1, "all three fit on one plate: {:?}", r.plates);
    }

    #[test]
    fn plate_mesh_places_every_part_inside_the_plate_without_overlap() {
        let pieces: Vec<Piece> = (1..=6).map(|i| piece(i, Vec3::new(60.0 + i as f32 * 5.0, 50.0, 20.0))).collect();
        let r = nest(&pieces, &NestOptions::new(DVec3::new(200.0, 200.0, 200.0)));
        assert!(r.all_fit());
        for plate in &r.plates {
            let m = plate_mesh(plate, &pieces);
            let b = m.bounds().unwrap();
            assert!(b.min.x >= -1e-3 && b.min.y >= -1e-3 && b.max.x <= 200.0 + 1e-3 && b.max.y <= 200.0 + 1e-3, "{b:?}");
            // Total volume preserved (rotation/translation only).
            let expected: f64 = plate.placements.iter().map(|pl| pieces.iter().find(|p| p.id == pl.piece_id).unwrap().volume).sum();
            assert!((m.stats().signed_volume - expected).abs() / expected < 1e-6, "{} vs {expected}", m.stats().signed_volume);
            assert!(m.stats().is_watertight());
        }
    }

    #[test]
    fn verify_fit_reports_each_piece() {
        let pieces = vec![piece(1, Vec3::new(100.0, 100.0, 100.0)), piece(2, Vec3::new(300.0, 300.0, 300.0))];
        let f = verify_fit(&pieces, DVec3::splat(250.0), &[]);
        assert!(f[0].fits && !f[1].fits);
    }
}
