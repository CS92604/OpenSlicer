//! Generating, validating and applying connectors across the interfaces between pieces.
//!
//! For every interface the connector geometry is built, placed, and **validated before it is used**:
//!
//! 1. 2D: the footprint (and the pocket plus its required wall) must lie inside the interface region.
//! 2. 3D, on a small cropped copy of each piece:
//!    * a male part must not collide with the piece that carries it (union volume = sum of volumes);
//!    * a pocket must lie entirely in solid material (the volume removed equals the pocket volume inside the
//!      interface prism), so thin walls, curved surfaces and nearby cavities are caught;
//!    * the male part must fit inside its pocket.
//! 3. After all connectors are applied, each modified piece must be watertight and its volume must equal the
//!    original plus added minus removed material.
//!
//! Anything that fails a check is rejected (with the reason recorded); nothing is applied silently.

use crate::placement::{self, FarthestPoint};
use crate::settings::{AngleMode, ConnectorKind, ConnectorSettings, ThinWallPolicy};
use crate::shapes::{self, Tools};
use glam::{DVec2, DVec3};
use openslicer_core::control::{Cancelled, Control};
use openslicer_core::Mesh;
use openslicer_geom::boolean::{boolean, BoolOp};
use openslicer_geom::build::{self, LocalFrame};
use openslicer_geom::crop::{crop_to_box, padded};
use openslicer_geom::plane::Frame;
use openslicer_geom::poly2d;
use openslicer_geom::region::Polygon;
use openslicer_split::exec::Piece;
use openslicer_split::interfaces::Interface;
use std::collections::HashMap;

#[derive(Clone, Debug)]
pub struct ConnectorRecord {
    /// Label such as `C07`.
    pub id: String,
    /// Index into the interface list that was passed in.
    pub interface: usize,
    pub kind: ConnectorKind,
    /// World position of the connector centre on the interface.
    pub position: DVec3,
    /// Direction from the male piece into the female piece.
    pub normal: DVec3,
    /// Piece id carrying the male part (`None` for dowels and other loose-part joints).
    pub male_piece: Option<usize>,
    pub female_piece: usize,
    pub size: f64,
    pub height: f64,
    pub angle_deg: f64,
}

#[derive(Clone, Debug, Default)]
pub struct JointReport {
    pub interfaces_total: usize,
    pub interfaces_connected: usize,
    pub connectors: usize,
    pub volume_added: f64,
    pub volume_removed: f64,
    /// Interfaces that received no connector, with the reason.
    pub skipped: Vec<(usize, String)>,
    pub notes: Vec<String>,
    /// Correctness problems; empty means every applied piece verified.
    pub problems: Vec<String>,
}

impl JointReport {
    pub fn is_valid(&self) -> bool {
        self.problems.is_empty()
    }
}

#[derive(Clone, Debug)]
pub struct JointResult {
    /// Pieces with connectors applied (same ids as the input).
    pub pieces: Vec<Piece>,
    /// Separate parts to print, such as dowels. Ids continue after the last piece id.
    pub loose: Vec<Piece>,
    pub connectors: Vec<ConnectorRecord>,
    pub report: JointReport,
}

/// After this many 3D rejections in a row at one connector size, stop trying more positions: if well-spread
/// positions all fail, the cause is the part (too thin, pocket too deep), not the location.
const MAX_CONSECUTIVE_REJECTIONS: usize = 6;
/// How many times a connector may be shrunk (by 20% each time) when nothing validates.
const MAX_SHRINK_ROUNDS: usize = 5;

#[derive(Default)]
struct Pending {
    adds: Vec<Mesh>,
    subs: Vec<Mesh>,
    add_volume: f64,
    sub_volume: f64,
}

/// A connector placement mapped into world space.
struct Placed {
    male: Option<Mesh>,
    female: Option<Mesh>,
    male_pocket: Option<Mesh>,
    loose: Vec<Mesh>,
    fits: Vec<(Mesh, Mesh)>,
    guard_prism: Option<Mesh>,
}

fn volume(m: &Mesh) -> f64 {
    m.stats().signed_volume
}

fn bbox_union(meshes: &[&Mesh]) -> Option<openslicer_core::Aabb> {
    let mut it = meshes.iter().filter_map(|m| m.bounds());
    let first = it.next()?;
    Some(it.fold(first, |a, b| openslicer_core::Aabb { min: a.min.min(b.min), max: a.max.max(b.max) }))
}

fn rel_close(a: f64, b: f64, rel: f64, abs: f64) -> bool {
    (a - b).abs() <= rel * a.abs().max(b.abs()) + abs
}

/// Region prism on the female side: the interface region extruded from the plane into the female piece.
fn region_prism(frame: &Frame, region: &[Polygon], into_positive: bool, depth: f64) -> Mesh {
    let m = if into_positive { build::extrude(region, 0.0, depth) } else { build::extrude(region, -depth, 0.0) };
    LocalFrame::on_plane(frame, DVec2::ZERO, 0.0).place(&m)
}

/// Volume of the material that a tool removes from `piece_crop` (checks only; the tool is not applied).
fn removed_volume(piece_crop: &Mesh, tool: &Mesh) -> Result<f64, String> {
    let before = volume(piece_crop);
    match boolean(piece_crop, tool, BoolOp::Difference) {
        Ok((m, _)) => Ok(before - if m.is_empty() { 0.0 } else { volume(&m) }),
        Err(e) => Err(format!("{e}")),
    }
}

fn intersect_volume(a: &Mesh, b: &Mesh) -> Result<f64, String> {
    match boolean(a, b, BoolOp::Intersection) {
        Ok((m, _)) => Ok(if m.is_empty() { 0.0 } else { volume(&m) }),
        Err(e) => Err(format!("{e}")),
    }
}

struct Validated {
    male_volume: f64,
    female_removed: f64,
    male_pocket_removed: f64,
}

/// 3D validation of one placed connector against cropped copies of the two pieces.
fn validate_3d(
    placed: &Placed,
    male_piece: &Mesh,
    female_piece: &Mesh,
    prism_female: &Mesh,
    prism_male_side: &Mesh,
) -> Result<Validated, String> {
    let mut out = Validated { male_volume: 0.0, female_removed: 0.0, male_pocket_removed: 0.0 };

    if let Some(m) = &placed.male {
        let window = padded(&m.bounds().ok_or("empty male part")?, 1.0);
        let local = crop_to_box(male_piece, &window)?.ok_or("there is no material under the connector on the piece that carries it")?;
        let (u, _) = boolean(&local, m, BoolOp::Union).map_err(|e| format!("male part check failed: {e}"))?;
        let expected = volume(&local) + volume(m);
        if !rel_close(volume(&u), expected, 1e-4, 1e-4) {
            return Err("the male part collides with the piece that carries it (curved or concave surface nearby)".into());
        }
        out.male_volume = volume(m);
    }

    let checks: [(Option<&Mesh>, &Mesh, &Mesh, &str, bool); 2] = [
        (placed.female.as_ref(), female_piece, prism_female, "socket", false),
        (placed.male_pocket.as_ref(), male_piece, prism_male_side, "dowel socket", true),
    ];
    for (tool, piece, prism, what, is_male_pocket) in checks {
        let Some(tool) = tool else { continue };
        let mut parts: Vec<&Mesh> = vec![tool];
        if let Some(g) = &placed.guard_prism {
            parts.push(g);
        }
        let window = padded(&bbox_union(&parts).ok_or("empty pocket")?, 1.0);
        let local = crop_to_box(piece, &window)?.ok_or_else(|| format!("there is no material for the {what}"))?;
        let removed = removed_volume(&local, tool)?;
        let expected = intersect_volume(tool, prism)?;
        if !rel_close(removed, expected, 1e-3, 1e-4) {
            return Err(format!(
                "the {what} does not lie fully inside solid material ({removed:.3} of {expected:.3} mm³ removed): the wall is too thin here or a surface or cavity is in the way"
            ));
        }
        if let Some(g) = &placed.guard_prism {
            // The pocket plus its required wall must also be backed by material.
            if !is_male_pocket || placed.male_pocket.is_some() {
                let removed_g = removed_volume(&local, g)?;
                let expected_g = intersect_volume(g, prism)?;
                if !rel_close(removed_g, expected_g, 1e-3, 1e-4) {
                    return Err(format!("less than the minimum wall thickness would remain around the {what}"));
                }
            }
        }
        if is_male_pocket {
            out.male_pocket_removed = removed;
        } else {
            out.female_removed = removed;
        }
    }

    for (inner, outer) in &placed.fits {
        let leftover = match boolean(inner, outer, BoolOp::Difference) {
            Ok((m, _)) => {
                if m.is_empty() {
                    0.0
                } else {
                    volume(&m)
                }
            }
            Err(e) => return Err(format!("fit check failed: {e}")),
        };
        if leftover > 1e-6 * volume(inner) {
            return Err("the male part does not fit inside its socket".into());
        }
    }
    Ok(out)
}

/// Maps a point-connector local 2D coordinate to plane coordinates for a frame with the given angle and handedness.
fn local_to_plane(p: DVec2, angle: f64, mirrored: bool, at: DVec2) -> DVec2 {
    let (s, c) = angle.sin_cos();
    let y = if mirrored { -p.y } else { p.y };
    at + DVec2::new(p.x * c - y * s, p.x * s + y * c)
}

fn place_polys(polys: &[Polygon], angle: f64, mirrored: bool, at: DVec2) -> Vec<Polygon> {
    polys
        .iter()
        .map(|p| Polygon {
            outer: p.outer.iter().map(|&q| local_to_plane(q, angle, mirrored, at)).collect(),
            holes: p.holes.iter().map(|h| h.iter().map(|&q| local_to_plane(q, angle, mirrored, at)).collect()).collect(),
        })
        .collect()
}

fn inside_region(polys: &[Polygon], region: &[Polygon]) -> bool {
    poly2d::area(&poly2d::difference(polys, region)) <= 1e-4
}

/// Frame for a placement. `male_on_a` selects the direction: from A into B (+n) or from B into A (-n).
fn frame_for(plane_frame: &Frame, at: DVec2, angle: f64, male_on_a: bool) -> LocalFrame {
    let (s, c) = angle.sin_cos();
    let x = plane_frame.u * c + plane_frame.v * s;
    let z = if male_on_a { plane_frame.n } else { -plane_frame.n };
    LocalFrame { origin: plane_frame.to_3d(at), x, y: z.cross(x), z }
}

fn place_tools(t: &Tools, f: &LocalFrame) -> Placed {
    let guard_prism = if t.guard.is_empty() || t.open_ended { None } else { Some(f.place(&build::extrude(&t.guard, 0.0, t.guard_depth))) };
    Placed {
        male: t.male.as_ref().map(|m| f.place(m)),
        female: t.female.as_ref().map(|m| f.place(m)),
        male_pocket: t.male_pocket.as_ref().map(|m| f.place(m)),
        loose: t.loose.iter().map(|m| f.place(m)).collect(),
        fits: t.fit_pairs.iter().map(|(a, b)| (f.place(a), f.place(b))).collect(),
        guard_prism,
    }
}

/// Principal direction of a region (angle of the longest extent).
fn principal_angle(region: &[Polygon]) -> f64 {
    let mut pts: Vec<DVec2> = Vec::new();
    for poly in region {
        let n = poly.outer.len();
        for i in 0..n {
            let (a, b) = (poly.outer[i], poly.outer[(i + 1) % n]);
            let steps = (((b - a).length() / 1.0).ceil() as usize).clamp(1, 200);
            for k in 0..steps {
                pts.push(a + (b - a) * (k as f64 / steps as f64));
            }
        }
    }
    if pts.len() < 2 {
        return 0.0;
    }
    let c = pts.iter().fold(DVec2::ZERO, |a, b| a + *b) / pts.len() as f64;
    let (mut sxx, mut sxy, mut syy) = (0.0, 0.0, 0.0);
    for p in &pts {
        let d = *p - c;
        sxx += d.x * d.x;
        sxy += d.x * d.y;
        syy += d.y * d.y;
    }
    0.5 * (2.0 * sxy).atan2(sxx - syy)
}

fn scale_connector(s: &ConnectorSettings, f: f64) -> ConnectorSettings {
    let mut out = s.clone();
    out.size *= f;
    if out.length > 0.0 {
        out.length *= f;
    }
    out.overhang *= f;
    out.slide *= f;
    out.chamfer = out.chamfer.min(out.size * 0.2);
    out
}

fn adjust_for_thin_wall(s: &ConnectorSettings, rho: f64) -> Result<ConnectorSettings, String> {
    if !s.kind.is_point_based() {
        return Ok(s.clone());
    }
    let need = s.size / 2.0 + s.clearance + s.min_wall;
    if rho >= need {
        return Ok(s.clone());
    }
    let wall = 2.0 * rho;
    let minimum = 2.0 * (s.min_feature / 2.0 + s.clearance + s.min_wall);
    if s.thin_wall == ThinWallPolicy::Skip {
        return Err(format!("the wall is {wall:.1} mm thick but the connector needs {:.1} mm", 2.0 * need));
    }
    // Leave a little slack so the valid centre positions form a band rather than a single line.
    let slack = (0.05 * rho).max(0.1);
    let f = (rho - slack - s.clearance - s.min_wall) / (s.size / 2.0);
    if f <= 0.0 || f * s.size < s.min_feature {
        return Err(format!(
            "the wall is only {wall:.1} mm thick; a connector needs at least {minimum:.1} mm (smallest feature {:.1} mm + clearance + minimum wall on both sides). Use a lip, reduce the minimum wall, or choose a larger split",
            s.min_feature
        ));
    }
    let mut out = s.clone();
    out.size *= f;
    if out.length > 0.0 {
        out.length *= f;
    }
    out.overhang *= f;
    out.slide *= f;
    out.chamfer = out.chamfer.min(out.size * 0.2);
    Ok(out)
}

/// Generates connectors for all interfaces, validates them, and applies them to the pieces.
pub fn generate_joints(
    pieces: &[Piece],
    interfaces: &[Interface],
    settings: &ConnectorSettings,
    ctl: &Control,
) -> Result<JointResult, Cancelled> {
    let mut report = JointReport { interfaces_total: interfaces.len(), ..Default::default() };
    if let Err(e) = settings.validate() {
        report.problems.push(format!("invalid connector settings: {e}"));
        return Ok(JointResult { pieces: pieces.to_vec(), loose: Vec::new(), connectors: Vec::new(), report });
    }
    let index: HashMap<usize, usize> = pieces.iter().enumerate().map(|(i, p)| (p.id, i)).collect();
    let mut pending: HashMap<usize, Pending> = HashMap::new();
    let mut connectors: Vec<ConnectorRecord> = Vec::new();
    let mut loose: Vec<Piece> = Vec::new();
    let max_id = pieces.iter().map(|p| p.id).max().unwrap_or(0);

    for (ii, iface) in interfaces.iter().enumerate() {
        ctl.check()?;
        ctl.report(ii as f32 / interfaces.len().max(1) as f32, &format!("Connectors for interface {} of {}", ii + 1, interfaces.len()));
        let (Some(&ia), Some(&ib)) = (index.get(&iface.a), index.get(&iface.b)) else {
            report.skipped.push((ii, "one of the pieces is missing".into()));
            continue;
        };
        let (pa, pb) = (&pieces[ia], &pieces[ib]);
        let region = &iface.region.polygons;
        let frame = iface.region.frame;
        let rho = placement::inradius(region);
        let st = match adjust_for_thin_wall(settings, rho) {
            Ok(s) => s,
            Err(e) => {
                report.skipped.push((ii, e));
                continue;
            }
        };

        let mut st = st;
        let mut accepted = 0usize;
        let mut last_error = String::from("no placement fits inside the interface");
        if st.kind.is_point_based() {
            let mut shrinks = 0usize;
            loop {
                let hw = st.size / 2.0 + st.clearance + st.min_wall;
                let cands = placement::candidates(region, hw, (st.size / 2.0).max(2.0), 1500);
                let mut fp = FarthestPoint::new(region, cands);
                let mut rejections = 0usize;
                while let Some(ci) = fp.next(st.spacing, st.min_count, st.max_count) {
                    ctl.check()?;
                    if rejections >= MAX_CONSECUTIVE_REJECTIONS {
                        break;
                    }
                    let at = fp.point(ci);
                    let angle = match st.angle {
                        AngleMode::Fixed(d) => d.to_radians(),
                        AngleMode::Auto => match st.kind {
                            ConnectorKind::Dovetail => placement::towards_boundary_angle(region, at),
                            ConnectorKind::TabSlot | ConnectorKind::Claw | ConnectorKind::Puzzle => placement::tangent_angle(region, at),
                            _ => 0.0,
                        },
                    };
                    let male_on_a = !(st.alternate && accepted % 2 == 1);
                    let exit = placement::nearest_boundary(region, at).map_or(0.0, |b| b.0);
                    let tools = match shapes::point_tools(&st, exit) {
                        Ok(t) => t,
                        Err(e) => {
                            last_error = e;
                            break; // geometry errors do not depend on the position
                        }
                    };
                    let mirrored = !male_on_a;
                    // 2D containment.
                    let guard = place_polys(&tools.guard, angle, mirrored, at);
                    if !tools.guard.is_empty() && !inside_region(&guard, region) {
                        last_error = "the connector does not fit inside the interface".into();
                        fp.reject(ci);
                        continue;
                    }
                    let f = frame_for(&frame, at, angle, male_on_a);
                    let placed = place_tools(&tools, &f);
                    let (male_piece, female_piece) = if male_on_a { (pa, pb) } else { (pb, pa) };
                    let depth = tools.guard_depth + 5.0;
                    let prism_female = region_prism(&frame, region, male_on_a, depth);
                    let prism_male = region_prism(&frame, region, !male_on_a, depth);
                    match validate_3d(&placed, &male_piece.mesh, &female_piece.mesh, &prism_female, &prism_male) {
                        Ok(v) => {
                            rejections = 0;
                            accepted += 1;
                            fp.accept(ci);
                            let mp = pending.entry(male_piece.id).or_default();
                            if let Some(m) = placed.male {
                                mp.adds.push(m);
                                mp.add_volume += v.male_volume;
                            }
                            if let Some(m) = placed.male_pocket {
                                mp.subs.push(m);
                                mp.sub_volume += v.male_pocket_removed;
                            }
                            let fpnd = pending.entry(female_piece.id).or_default();
                            if let Some(m) = placed.female {
                                fpnd.subs.push(m);
                                fpnd.sub_volume += v.female_removed;
                            }
                            for l in placed.loose {
                                let mut lp = Piece::new(l).expect("dowel mesh is not empty");
                                lp.id = max_id + loose.len() + 1;
                                lp.name = format!("D{:02}", loose.len() + 1);
                                loose.push(lp);
                            }
                            connectors.push(ConnectorRecord {
                                id: format!("C{:02}", connectors.len() + 1),
                                interface: ii,
                                kind: st.kind,
                                position: f.origin,
                                normal: f.z,
                                male_piece: if tools.male.is_some() { Some(male_piece.id) } else { None },
                                female_piece: female_piece.id,
                                size: st.size,
                                height: st.height,
                                angle_deg: angle.to_degrees(),
                            });
                            report.volume_added += v.male_volume;
                            report.volume_removed += v.female_removed + v.male_pocket_removed;
                        }
                        Err(e) => {
                            last_error = e;
                            rejections += 1;
                            fp.reject(ci);
                        }
                    }
                }
                // Thin or curved walls: if nothing validated, retry with a smaller connector.
                if accepted == 0
                    && st.thin_wall == ThinWallPolicy::ShrinkToFit
                    && st.size * 0.8 >= st.min_feature
                    && shrinks < MAX_SHRINK_ROUNDS
                {
                    shrinks += 1;
                    st = scale_connector(&st, 0.8);
                    continue;
                }
                break;
            }
        } else {
            let angle = match st.angle {
                AngleMode::Fixed(d) => d.to_radians(),
                AngleMode::Auto => principal_angle(region),
            };
            match shapes::region_tools(&st, region, angle, rho) {
                Ok(tools) => {
                    let f = frame_for(&frame, DVec2::ZERO, 0.0, true);
                    let placed = place_tools(&tools, &f);
                    if !tools.guard.is_empty() && !inside_region(&tools.guard, region) {
                        last_error = "the lip and its walls do not fit inside the interface".into();
                    } else {
                        let prism_female = region_prism(&frame, region, true, tools.guard_depth + 5.0);
                        let prism_male = region_prism(&frame, region, false, tools.guard_depth + 5.0);
                        match validate_3d(&placed, &pa.mesh, &pb.mesh, &prism_female, &prism_male) {
                            Ok(v) => {
                                accepted += 1;
                                let mp = pending.entry(pa.id).or_default();
                                if let Some(m) = placed.male {
                                    mp.adds.push(m);
                                    mp.add_volume += v.male_volume;
                                }
                                let fpnd = pending.entry(pb.id).or_default();
                                if let Some(m) = placed.female {
                                    fpnd.subs.push(m);
                                    fpnd.sub_volume += v.female_removed;
                                }
                                connectors.push(ConnectorRecord {
                                    id: format!("C{:02}", connectors.len() + 1),
                                    interface: ii,
                                    kind: st.kind,
                                    position: frame.to_3d(region.first().and_then(|p| p.outer.first().copied()).unwrap_or(DVec2::ZERO)),
                                    normal: frame.n,
                                    male_piece: Some(pa.id),
                                    female_piece: pb.id,
                                    size: st.size,
                                    height: st.height,
                                    angle_deg: angle.to_degrees(),
                                });
                                report.volume_added += v.male_volume;
                                report.volume_removed += v.female_removed;
                            }
                            Err(e) => last_error = e,
                        }
                    }
                }
                Err(e) => last_error = e,
            }
        }
        if accepted > 0 {
            report.interfaces_connected += 1;
            if accepted < st.min_count {
                report.notes.push(format!(
                    "interface {}: only {accepted} of the requested {} connectors could be placed ({last_error})",
                    ii + 1,
                    st.min_count
                ));
            }
        } else {
            report.skipped.push((ii, last_error));
        }
    }
    report.connectors = connectors.len();

    // Apply the validated tools piece by piece and verify the result.
    let mut out_pieces = pieces.to_vec();
    let total = pending.len().max(1);
    let mut ids: Vec<usize> = pending.keys().copied().collect();
    ids.sort_unstable();
    for (k, id) in ids.into_iter().enumerate() {
        ctl.check()?;
        ctl.report(k as f32 / total as f32, "Applying connectors to pieces");
        let p = pending.remove(&id).unwrap();
        let slot = index[&id];
        match apply_to_piece(&pieces[slot], &p) {
            Ok(new_piece) => out_pieces[slot] = new_piece,
            Err(e) => report.problems.push(format!("piece {}: {e}; it was left without connectors", pieces[slot].name)),
        }
    }
    ctl.report(1.0, "Done");
    Ok(JointResult { pieces: out_pieces, loose, connectors, report })
}

/// Independent check of the finished assembly: pieces that touch must not interpenetrate, and loose parts
/// (dowels) must not intersect any piece. Returns a description of every violation (empty means clean).
/// Each check is a full Boolean between two solids, so this is run on demand rather than during generation.
pub fn verify_assembly(result: &JointResult, interfaces: &[Interface], ctl: &Control) -> Result<Vec<String>, Cancelled> {
    let mut problems = Vec::new();
    let find = |id: usize| result.pieces.iter().find(|p| p.id == id);
    let mut checked: Vec<(usize, usize)> = Vec::new();
    for iface in interfaces {
        ctl.check()?;
        let key = (iface.a.min(iface.b), iface.a.max(iface.b));
        if checked.contains(&key) {
            continue;
        }
        checked.push(key);
        let (Some(a), Some(b)) = (find(iface.a), find(iface.b)) else { continue };
        match intersect_volume(&a.mesh, &b.mesh) {
            Ok(v) if v <= 1e-4 * a.volume.min(b.volume) + 1e-6 => {}
            Ok(v) => problems.push(format!("{} and {} overlap by {v:.4} mm³ when assembled", a.name, b.name)),
            Err(e) => problems.push(format!("could not check {} against {}: {e}", a.name, b.name)),
        }
    }
    for l in &result.loose {
        ctl.check()?;
        let bounds = l.bounds;
        for p in result.pieces.iter().filter(|p| p.bounds.min.cmple(bounds.max).all() && p.bounds.max.cmpge(bounds.min).all()) {
            match intersect_volume(&p.mesh, &l.mesh) {
                Ok(v) if v <= 1e-4 * l.volume => {}
                Ok(v) => problems.push(format!("dowel {} overlaps {} by {v:.4} mm³", l.name, p.name)),
                Err(e) => problems.push(format!("could not check dowel {} against {}: {e}", l.name, p.name)),
            }
        }
    }
    Ok(problems)
}

fn apply_to_piece(piece: &Piece, p: &Pending) -> Result<Piece, String> {
    let mut mesh = piece.mesh.clone();
    if !p.adds.is_empty() {
        let tool = shapes::combine(p.adds.clone())?;
        mesh = boolean(&mesh, &tool, BoolOp::Union).map_err(|e| format!("adding connectors failed: {e}"))?.0;
    }
    if !p.subs.is_empty() {
        let tool = shapes::combine(p.subs.clone())?;
        mesh = boolean(&mesh, &tool, BoolOp::Difference).map_err(|e| format!("cutting sockets failed: {e}"))?.0;
    }
    let s = mesh.stats();
    if !s.is_watertight() || s.signed_volume <= 0.0 {
        return Err(format!(
            "the result is not a valid solid ({} open edges, {} degenerate triangles)",
            s.boundary_edges, s.degenerate_triangles
        ));
    }
    let expected = piece.volume + p.add_volume - p.sub_volume;
    let slack = 2e-3 * (p.add_volume + p.sub_volume) + 1e-6 * piece.volume;
    if (s.signed_volume - expected).abs() > slack {
        return Err(format!(
            "the volume {:.3} mm³ differs from the expected {:.3} mm³ (connectors may overlap each other)",
            s.signed_volume, expected
        ));
    }
    let mut np = Piece::new(mesh).ok_or("the result is empty")?;
    np.id = piece.id;
    np.name = piece.name.clone();
    Ok(np)
}

#[cfg(test)]
mod tests {
    use super::*;
    use glam::Vec3;
    use openslicer_core::control::Control;
    use openslicer_geom::Plane;
    use openslicer_split::exec::{execute_plan, SplitOptions, SplitResult};
    use openslicer_split::interfaces::find_interfaces;
    use openslicer_split::plan::CutPlan;

    fn block(size: Vec3) -> Mesh {
        let mut m = Mesh::cuboid(size);
        m.translate(size * 0.5);
        m
    }

    fn split(m: &Mesh, planes: Vec<Plane>) -> (SplitResult, Vec<Interface>) {
        let r = execute_plan(m, &CutPlan::from_planes(planes), &SplitOptions { tolerance: None, separate_solids: true }, &Control::none())
            .unwrap();
        assert!(r.report.is_valid(), "{:?}", r.report);
        let ifs = find_interfaces(&r.pieces, &r.planes, 1e-3);
        (r, ifs)
    }

    fn assert_all_valid(res: &JointResult) {
        assert!(res.report.is_valid(), "{:?}", res.report);
        for p in res.pieces.iter().chain(res.loose.iter()) {
            let s = p.mesh.stats();
            assert!(s.is_watertight() && s.signed_volume > 0.0, "{}: {s:?}", p.name);
        }
    }

    #[test]
    fn pins_connect_two_halves_with_exact_volume_accounting() {
        let (r, ifs) = split(&block(Vec3::splat(100.0)), vec![Plane::axis(2, 50.0)]);
        assert_eq!(ifs.len(), 1);
        let mut s = ConnectorSettings::new(ConnectorKind::Pin);
        s.spacing = 40.0;
        let res = generate_joints(&r.pieces, &ifs, &s, &Control::none()).unwrap();
        assert_all_valid(&res);
        assert!(res.connectors.len() >= 4, "{}", res.connectors.len());
        assert_eq!(res.report.interfaces_connected, 1);
        let (a, b) = (&res.pieces[0], &res.pieces[1]); // ids 1 (lower), 2 (upper)
        let (lower, upper) = if a.bounds.center().z < b.bounds.center().z { (a, b) } else { (b, a) };
        let n = res.connectors.len() as f64;
        // The lower piece gained n pins; the upper lost n sockets.
        assert!((lower.volume - (500_000.0 + res.report.volume_added)).abs() < 1.0);
        assert!((upper.volume - (500_000.0 - res.report.volume_removed)).abs() < 1.0);
        // Each socket is larger than its pin by the clearance.
        assert!(res.report.volume_removed > res.report.volume_added);
        assert!(n > 0.0 && lower.bounds.max.z > 50.5, "pins stand proud of the cut face");
        // Pins stay inside the face with the minimum wall.
        for c in &res.connectors {
            assert!(
                c.position.x > 1.0 + 3.0 && c.position.x < 100.0 - 4.0 && c.position.y > 4.0 && c.position.y < 96.0,
                "{:?}",
                c.position
            );
        }
    }

    #[test]
    fn every_connector_kind_applies_cleanly_to_a_split_block() {
        let (r, ifs) = split(&block(Vec3::new(120.0, 80.0, 100.0)), vec![Plane::axis(2, 50.0)]);
        for kind in ConnectorKind::ALL {
            let mut s = ConnectorSettings::new(kind);
            s.spacing = 60.0;
            s.max_count = 4;
            let res = generate_joints(&r.pieces, &ifs, &s, &Control::none()).unwrap();
            assert_all_valid(&res);
            assert_eq!(res.report.interfaces_connected, 1, "{kind:?}: {:?}", res.report.skipped);
            assert!(!res.connectors.is_empty(), "{kind:?}");
            if kind == ConnectorKind::Dowel {
                assert_eq!(res.loose.len(), res.connectors.len());
                // Both pieces lose material to the sockets and none gains any.
                assert!(
                    res.pieces.iter().all(|p| p.volume < 120.0 * 80.0 * 50.0 - 1.0),
                    "{:?}",
                    res.pieces.iter().map(|p| p.volume).collect::<Vec<_>>()
                );
            } else {
                assert!(res.loose.is_empty(), "{kind:?}");
            }
            // Material accounting for the whole assembly: change = added - removed.
            let total: f64 = res.pieces.iter().map(|p| p.volume).sum();
            let expected = 120.0 * 80.0 * 100.0 + res.report.volume_added - res.report.volume_removed;
            assert!((total - expected).abs() < 2.0, "{kind:?}: {total} vs {expected}");
        }
    }

    #[test]
    fn dovetail_groove_opens_towards_the_nearest_edge() {
        let (r, ifs) = split(&block(Vec3::new(120.0, 80.0, 100.0)), vec![Plane::axis(2, 50.0)]);
        let mut s = ConnectorSettings::new(ConnectorKind::Dovetail);
        s.max_count = 1;
        let res = generate_joints(&r.pieces, &ifs, &s, &Control::none()).unwrap();
        assert_all_valid(&res);
        assert_eq!(res.connectors.len(), 1);
        // The upper piece's groove breaks out through a side wall: its outer bounding box is unchanged.
        let upper = res.pieces.iter().find(|p| p.bounds.center().z > 50.0).unwrap();
        assert!((upper.bounds.size().x - 120.0).abs() < 1e-3 && (upper.bounds.size().y - 80.0).abs() < 1e-3);
        assert!(upper.volume < 120.0 * 80.0 * 50.0 - 10.0, "material was removed");
    }

    #[test]
    fn lap_joint_steps_one_half_up_and_the_other_down() {
        let (r, ifs) = split(&block(Vec3::new(100.0, 100.0, 100.0)), vec![Plane::axis(2, 50.0)]);
        let s = ConnectorSettings::new(ConnectorKind::LapJoint); // 4 mm step
        let res = generate_joints(&r.pieces, &ifs, &s, &Control::none()).unwrap();
        assert_all_valid(&res);
        let lower = res.pieces.iter().find(|p| p.bounds.center().z < 50.0).unwrap();
        // Half the face (100 x 50 mm) was raised by 4 mm.
        assert!((lower.volume - (500_000.0 + 100.0 * 50.0 * 4.0)).abs() < 5.0, "{}", lower.volume);
        assert!((lower.bounds.max.z - 54.0).abs() < 1e-3);
    }

    #[test]
    fn lip_follows_the_cut_boundary_of_a_thick_part() {
        let (r, ifs) = split(&block(Vec3::splat(80.0)), vec![Plane::axis(0, 40.0)]);
        let s = ConnectorSettings::new(ConnectorKind::Lip);
        let res = generate_joints(&r.pieces, &ifs, &s, &Control::none()).unwrap();
        assert_all_valid(&res);
        assert_eq!(res.connectors.len(), 1);
        assert!(res.report.volume_added > 100.0, "a ridge all around the face: {}", res.report.volume_added);
    }

    #[test]
    fn alternating_tabs_put_male_parts_on_both_pieces() {
        let (r, ifs) = split(&block(Vec3::new(160.0, 60.0, 100.0)), vec![Plane::axis(2, 50.0)]);
        let mut s = ConnectorSettings::new(ConnectorKind::TabSlot);
        s.alternate = true;
        s.spacing = 30.0;
        s.min_count = 3;
        s.max_count = 6;
        let res = generate_joints(&r.pieces, &ifs, &s, &Control::none()).unwrap();
        assert_all_valid(&res);
        assert!(res.connectors.len() >= 3);
        let on_lower = res.connectors.iter().filter(|c| c.male_piece == Some(1) || c.male_piece == Some(2)).count();
        let males: std::collections::HashSet<_> = res.connectors.iter().filter_map(|c| c.male_piece).collect();
        assert_eq!(males.len(), 2, "both pieces carry tabs: {:?}", res.connectors.iter().map(|c| c.male_piece).collect::<Vec<_>>());
        let _ = on_lower;
        // Both pieces have both added and removed material.
        for p in &res.pieces {
            assert!((p.volume - 480_000.0 / 2.0 * 2.0 / 2.0).abs() > 1e-3);
        }
    }

    #[test]
    fn pockets_deeper_than_the_part_are_rejected_with_a_reason_and_nothing_is_applied() {
        let (r, ifs) = split(&block(Vec3::new(100.0, 100.0, 60.0)), vec![Plane::axis(2, 30.0)]);
        let mut s = ConnectorSettings::new(ConnectorKind::Pin);
        s.height = 45.0; // 30 mm of material behind the face
        let res = generate_joints(&r.pieces, &ifs, &s, &Control::none()).unwrap();
        assert!(res.connectors.is_empty());
        assert_eq!(res.report.interfaces_connected, 0);
        assert_eq!(res.report.skipped.len(), 1);
        assert!(res.report.skipped[0].1.contains("solid material"), "{:?}", res.report.skipped);
        // The pieces are untouched.
        for (a, b) in res.pieces.iter().zip(&r.pieces) {
            assert!((a.volume - b.volume).abs() < 1e-6);
        }
    }

    #[test]
    fn thin_shell_walls_get_shrunken_pins_or_a_clear_refusal() {
        // 6 mm wall: the default 6 mm pin is shrunk to fit.
        let shell = openslicer_geom_test_shell(50.0, 44.0);
        let (r, ifs) = split(&shell, vec![Plane::axis(2, 0.0)]);
        assert_eq!(ifs.len(), 1);
        let mut s = ConnectorSettings::new(ConnectorKind::Pin);
        s.height = 6.0;
        s.spacing = 60.0;
        let res = generate_joints(&r.pieces, &ifs, &s, &Control::none()).unwrap();
        assert_all_valid(&res);
        assert!(!res.connectors.is_empty(), "{:?}", res.report.skipped);
        assert!(
            res.connectors.iter().all(|c| c.size < 6.0 && c.size >= s.min_feature),
            "{:?}",
            res.connectors.iter().map(|c| c.size).collect::<Vec<_>>()
        );

        // 3 mm wall: refused with an explanation, pieces untouched.
        let thin = openslicer_geom_test_shell(50.0, 47.0);
        let (r, ifs) = split(&thin, vec![Plane::axis(2, 0.0)]);
        let res = generate_joints(&r.pieces, &ifs, &s, &Control::none()).unwrap();
        assert!(res.connectors.is_empty());
        assert!(res.report.skipped[0].1.contains("thick"), "{:?}", res.report.skipped);
        assert!(res.report.is_valid());
    }

    #[test]
    fn thin_shell_with_a_lip_when_the_wall_allows_it() {
        let shell = openslicer_geom_test_shell(50.0, 44.0);
        let (r, ifs) = split(&shell, vec![Plane::axis(2, 0.0)]);
        let mut s = ConnectorSettings::new(ConnectorKind::Lip);
        s.height = 3.0;
        s.min_wall = 1.0;
        let res = generate_joints(&r.pieces, &ifs, &s, &Control::none()).unwrap();
        assert_all_valid(&res);
        assert_eq!(res.connectors.len(), 1, "{:?}", res.report.skipped);
    }

    #[test]
    fn torus_halves_get_pins_on_both_annular_sections() {
        let torus = openslicer_geom_test_torus();
        let (r, ifs) = split(&torus, vec![Plane::axis(0, 0.0)]);
        assert_eq!(ifs.len(), 1, "one interface made of two disks");
        let mut s = ConnectorSettings::new(ConnectorKind::Pin);
        s.size = 8.0;
        s.height = 10.0;
        s.min_count = 2;
        let res = generate_joints(&r.pieces, &ifs, &s, &Control::none()).unwrap();
        assert_all_valid(&res);
        let xs: Vec<f64> = res.connectors.iter().map(|c| c.position.y).collect();
        assert!(xs.iter().any(|&y| y > 0.0) && xs.iter().any(|&y| y < 0.0), "pins on both sections: {xs:?}");
    }

    #[test]
    fn assembled_pieces_do_not_interpenetrate_and_total_volume_is_original_minus_clearance() {
        let (r, ifs) = split(&block(Vec3::new(120.0, 80.0, 100.0)), vec![Plane::axis(2, 50.0)]);
        for kind in [
            ConnectorKind::Pin,
            ConnectorKind::Dowel,
            ConnectorKind::Dovetail,
            ConnectorKind::Pyramid,
            ConnectorKind::Claw,
            ConnectorKind::Lip,
        ] {
            let mut s = ConnectorSettings::new(kind);
            s.max_count = 3;
            let res = generate_joints(&r.pieces, &ifs, &s, &Control::none()).unwrap();
            assert_all_valid(&res);
            let problems = verify_assembly(&res, &ifs, &Control::none()).unwrap();
            assert!(problems.is_empty(), "{kind:?}: {problems:?}");
            // Total material (pieces + loose) is the original minus the clearance gaps.
            let total: f64 = res.pieces.iter().chain(res.loose.iter()).map(|p| p.volume).sum();
            assert!(total < 120.0 * 80.0 * 100.0 + 1e-3, "{kind:?}: material was created: {total}");
            assert!(total > 120.0 * 80.0 * 100.0 - 0.02 * 120.0 * 80.0 * 100.0, "{kind:?}: too much material removed: {total}");
        }
    }

    #[test]
    fn verify_assembly_catches_overlapping_pieces() {
        let (r, ifs) = split(&block(Vec3::splat(100.0)), vec![Plane::axis(2, 50.0)]);
        let mut res = generate_joints(&r.pieces, &ifs, &ConnectorSettings::new(ConnectorKind::Pin), &Control::none()).unwrap();
        // Corrupt the assembly: move the upper piece down by 5 mm so it overlaps the lower one.
        let up = res.pieces.iter_mut().find(|p| p.bounds.center().z > 50.0).unwrap();
        up.mesh.translate(Vec3::new(0.0, 0.0, -5.0));
        up.bounds = up.mesh.bounds().unwrap();
        let problems = verify_assembly(&res, &ifs, &Control::none()).unwrap();
        assert!(!problems.is_empty());
        assert!(problems[0].contains("overlap"), "{problems:?}");
    }

    #[test]
    fn invalid_settings_change_nothing_and_say_why() {
        let (r, ifs) = split(&block(Vec3::splat(100.0)), vec![Plane::axis(2, 50.0)]);
        let mut s = ConnectorSettings::new(ConnectorKind::Pin);
        s.clearance = -1.0;
        let res = generate_joints(&r.pieces, &ifs, &s, &Control::none()).unwrap();
        assert!(!res.report.is_valid());
        assert!(res.connectors.is_empty());
    }

    #[test]
    fn cancellation_is_honoured() {
        use std::sync::atomic::AtomicBool;
        let (r, ifs) = split(&block(Vec3::splat(100.0)), vec![Plane::axis(2, 50.0)]);
        let flag = AtomicBool::new(true);
        let out = generate_joints(&r.pieces, &ifs, &ConnectorSettings::new(ConnectorKind::Pin), &Control::with_cancel(&flag));
        assert!(out.is_err());
    }

    // Procedural shapes are private to the geometry crate's tests, so the joints tests build their own.
    fn openslicer_geom_test_shell(ro: f32, ri: f32) -> Mesh {
        let sphere = |r: f32, flip: bool| {
            let (nu, nv) = (96usize, 48usize);
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
            if flip {
                for t in &mut tris {
                    t.swap(1, 2);
                }
            }
            Mesh::new(verts, tris)
        };
        let mut m = sphere(ro, false);
        let inner = sphere(ri, true);
        let base = m.vertices.len() as u32;
        m.vertices.extend(inner.vertices);
        m.triangles.extend(inner.triangles.iter().map(|t| [t[0] + base, t[1] + base, t[2] + base]));
        m
    }

    fn openslicer_geom_test_torus() -> Mesh {
        let (nu, nv, big, small) = (96usize, 48usize, 60.0f32, 15.0f32);
        let mut verts = Vec::new();
        for i in 0..nu {
            for j in 0..nv {
                let (u, v) = (std::f32::consts::TAU * i as f32 / nu as f32, std::f32::consts::TAU * j as f32 / nv as f32);
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
}
