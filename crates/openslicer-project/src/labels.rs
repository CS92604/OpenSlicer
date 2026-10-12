//! Part labels: the part number and its assembly step, engraved into or raised from the part.
//!
//! The label (for example `P07-3`: part 7, assembled third) is built from seven-segment style strokes, so it
//! needs no font. It is placed on a **flat patch** of the part's own surface: the search samples the surface,
//! prefers hidden places (the cavity side of a shell, a mating cut face when engraving), and accepts a patch only
//! when rays across the whole text area hit the surface within a small tolerance. Connectors, finger steps and
//! sockets are therefore avoided automatically, because they are not flat.
//!
//! The text is then added (raised) or subtracted (engraved) with a Boolean, and the result is verified to be a
//! closed solid whose volume changed by exactly the expected amount. If anything fails, the part is left as it
//! was and the reason is recorded: a label is never forced onto a part.

use crate::assembly::AssemblyStep;
use glam::{DVec2, DVec3};
use openslicer_core::control::{Cancelled, Control};
use openslicer_core::Mesh;
use openslicer_geom::boolean::{boolean, BoolOp};
use openslicer_geom::build::{self, LocalFrame};
use openslicer_geom::poly2d;
use openslicer_geom::region::Polygon;
use openslicer_split::exec::Piece;
use openslicer_split::interfaces::Interface;
use serde::{Deserialize, Serialize};

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub enum LabelMode {
    #[default]
    Off,
    /// Engraved into the part (hidden cut faces and shell interiors preferred).
    Inset,
    /// Raised from the part (never on a mating cut face).
    Outset,
}

impl LabelMode {
    pub fn label(self) -> &'static str {
        match self {
            LabelMode::Off => "Off",
            LabelMode::Inset => "Engraved (inset)",
            LabelMode::Outset => "Raised (outset)",
        }
    }
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct LabelOptions {
    pub mode: LabelMode,
    /// Character height in mm. Smaller sizes are tried when the larger one does not fit a flat area.
    pub height: f64,
    /// Relief depth in mm.
    pub depth: f64,
}

impl Default for LabelOptions {
    fn default() -> Self {
        Self { mode: LabelMode::Off, height: 8.0, depth: 0.6 }
    }
}

impl LabelOptions {
    pub fn validate(&self) -> Result<(), String> {
        if !(3.0..=40.0).contains(&self.height) {
            return Err("the label height must be between 3 and 40 mm".into());
        }
        if !(0.2..=3.0).contains(&self.depth) {
            return Err("the label depth must be between 0.2 and 3 mm".into());
        }
        Ok(())
    }
}

#[derive(Clone, Debug)]
pub struct LabelRecord {
    pub piece_id: usize,
    pub text: String,
    pub placed: bool,
    /// Where it went, or why it was not placed.
    pub note: String,
}

/// `P07` + step 3 → `P07-3`.
pub fn label_text(part_name: &str, step: usize) -> String {
    format!("{part_name}-{step}")
}

/// Segments a (top), b (top right), c (bottom right), d (bottom), e (bottom left), f (top left), g (middle).
fn segments(c: char) -> &'static str {
    match c.to_ascii_uppercase() {
        '0' => "abcdef",
        '1' => "bc",
        '2' => "abdeg",
        '3' => "abcdg",
        '4' => "bcfg",
        '5' => "acdfg",
        '6' => "acdefg",
        '7' => "abc",
        '8' => "abcdefg",
        '9' => "abcdfg",
        'P' => "abefg",
        'D' => "bcdeg",
        'L' => "def",
        'S' => "acdfg",
        '-' => "g",
        _ => "",
    }
}

/// Glyph outlines for `text`, centred on the origin, plus the overall width.
pub fn glyphs(text: &str, height: f64) -> (Vec<Polygon>, f64) {
    let (h, s) = (height, (0.15 * height).max(1.2));
    let (cw, gap) = (0.55 * h, 0.3 * h);
    let n = text.chars().count().max(1);
    let total = n as f64 * cw + (n - 1) as f64 * gap;
    let mut out: Vec<Polygon> = Vec::new();
    for (i, ch) in text.chars().enumerate() {
        let x0 = i as f64 * (cw + gap) - total / 2.0;
        let r = |x: f64, y: f64, w: f64, hh: f64| poly2d::rect(DVec2::new(x0 + x, y - h / 2.0), DVec2::new(x0 + x + w, y - h / 2.0 + hh));
        let mut strokes = Vec::new();
        for seg in segments(ch).chars() {
            strokes.push(match seg {
                'a' => r(0.0, h - s, cw, s),
                'b' => r(cw - s, h / 2.0, s, h / 2.0),
                'c' => r(cw - s, 0.0, s, h / 2.0),
                'd' => r(0.0, 0.0, cw, s),
                'e' => r(0.0, 0.0, s, h / 2.0),
                'f' => r(0.0, h / 2.0, s, h / 2.0),
                _ => r(0.0, h / 2.0 - s / 2.0, cw, s),
            });
        }
        out.extend(poly2d::union_all(&strokes));
    }
    (out, total)
}

struct Patch {
    centre: DVec3,
    normal: DVec3,
    x: DVec3,
    y: DVec3,
    /// Thickness of material behind the patch, in mm.
    thickness: f64,
    on_interface: bool,
    hidden: bool,
}

fn tangent_frame(n: DVec3) -> (DVec3, DVec3) {
    let up = if n.z.abs() > 0.9 { DVec3::Y } else { DVec3::Z };
    let y = (up - n * up.dot(n)).normalize();
    let x = y.cross(n).normalize();
    (x, y)
}

/// Looks for a flat patch of `w` × `h` mm. Returns the best one, or the reason for failure.
fn find_patch(
    piece: &Piece,
    planes: &[openslicer_geom::plane::Plane],
    mode: LabelMode,
    w: f64,
    h: f64,
    depth: f64,
) -> Result<Patch, String> {
    let mesh = &piece.mesh;
    let centre_of_part = piece.bounds.center().as_dvec3();
    let step = (mesh.triangles.len() / 400).max(1);
    let tol = 0.3;
    struct Cand {
        c: DVec3,
        n: DVec3,
        score: f64,
        on_interface: bool,
        inner: bool,
    }
    let mut cands: Vec<Cand> = Vec::new();
    for (ti, _) in mesh.triangles.iter().enumerate().step_by(step) {
        let [a, b, c] = mesh.triangle(ti).map(|v| v.as_dvec3());
        let cross = (b - a).cross(c - a);
        if cross.length() < 1e-9 {
            continue;
        }
        let n = cross.normalize();
        let cen = (a + b + c) / 3.0;
        let on_interface = planes.iter().any(|p| p.signed_distance(cen).abs() < 0.01 && p.n.dot(n).abs() > 0.99);
        if mode == LabelMode::Outset && on_interface {
            continue;
        }
        let to_centre = (centre_of_part - cen).normalize_or_zero();
        let inner = n.dot(to_centre) > 0.3;
        let score = if inner { 4.0 } else { 0.0 }
            + if mode == LabelMode::Inset && on_interface { 3.0 } else { 0.0 }
            + if n.z < -0.7 { 1.0 } else { 0.0 }
            + 0.001 * (cross.length().min(1000.0));
        cands.push(Cand { c: cen, n, score, on_interface, inner });
    }
    if cands.is_empty() {
        return Err("no suitable surface".into());
    }
    cands.sort_by(|a, b| b.score.total_cmp(&a.score));
    let margin = 1.0;
    let (hx, hy) = (w / 2.0 + margin, h / 2.0 + margin);
    for cand in cands.into_iter().take(80) {
        let (x, y) = tangent_frame(cand.n);
        let mut flat = true;
        'grid: for iu in -2..=2 {
            for iv in -1..=1 {
                let p = cand.c + x * (hx * iu as f64 / 2.0) + y * (hy * iv as f64) + cand.n * 2.0;
                match mesh.ray_intersect(p.as_vec3(), (-cand.n).as_vec3()) {
                    Some(t) if (t as f64 - 2.0).abs() <= tol => {}
                    _ => {
                        flat = false;
                        break 'grid;
                    }
                }
            }
        }
        if !flat {
            continue;
        }
        // Material behind the patch: from just inside the surface, along the inward normal, to where it leaves.
        let inside = cand.c - cand.n * 0.05;
        let thickness = mesh.ray_intersect(inside.as_vec3(), (-cand.n).as_vec3()).map_or(f64::INFINITY, |t| t as f64 + 0.05);
        let need = if mode == LabelMode::Inset { depth + 1.2 } else { 1.2 };
        if thickness < need {
            continue;
        }
        return Ok(Patch { centre: cand.c, normal: cand.n, x, y, thickness, on_interface: cand.on_interface, hidden: cand.inner });
    }
    Err(format!("no flat area of {w:.0} × {h:.0} mm with enough material behind it was found"))
}

fn vol(m: &Mesh) -> f64 {
    if m.is_empty() {
        0.0
    } else {
        m.stats().signed_volume
    }
}

fn place_label(piece: &Piece, planes: &[openslicer_geom::plane::Plane], text: &str, o: &LabelOptions) -> Result<(Mesh, String), String> {
    let mut last = String::new();
    for scale in [1.0, 0.75, 0.6, 0.5] {
        let height = (o.height * scale).max(3.0);
        let (polys, width) = glyphs(text, height);
        match find_patch(piece, planes, o.mode, width, height, o.depth) {
            Ok(p) => {
                let frame = LocalFrame { origin: p.centre, x: p.x, y: p.y, z: p.normal };
                let (z0, z1) = if o.mode == LabelMode::Inset { (-o.depth, 0.6) } else { (-0.5, o.depth) };
                let text_solid = frame.place(&build::extrude(&polys, z0, z1));
                let inside =
                    boolean(&piece.mesh, &text_solid, BoolOp::Intersection).map_err(|e| format!("could not measure the label: {e}"))?.0;
                let (op, expected) = if o.mode == LabelMode::Inset {
                    (BoolOp::Difference, piece.volume - vol(&inside))
                } else {
                    (BoolOp::Union, piece.volume + vol(&text_solid) - vol(&inside))
                };
                let result = boolean(&piece.mesh, &text_solid, op).map_err(|e| format!("could not apply the label: {e}"))?.0;
                let s = result.stats();
                if !s.is_watertight() || s.signed_volume <= 0.0 {
                    last = format!("the labelled part was not a closed solid ({} open edges)", s.boundary_edges);
                    continue;
                }
                if (s.signed_volume - expected).abs() > 1e-5 * piece.volume + 1e-2 {
                    last = format!("the labelled part had volume {:.2} mm³ instead of {:.2}", s.signed_volume, expected);
                    continue;
                }
                let place = if p.on_interface {
                    "on a mating face"
                } else if p.hidden {
                    "on the inside"
                } else {
                    "on the surface"
                };
                return Ok((result, format!("{place}, {height:.0} mm high, {:.1} mm of material behind", p.thickness.min(99.0))));
            }
            Err(e) => last = e,
        }
    }
    Err(last)
}

/// Adds a label to every piece. Pieces that cannot take one are left unchanged and explained in the result.
pub fn apply_labels(
    pieces: &mut [Piece],
    interfaces: &[Interface],
    steps: &[AssemblyStep],
    opts: &LabelOptions,
    ctl: &Control,
) -> Result<Vec<LabelRecord>, Cancelled> {
    let mut records = Vec::new();
    if opts.mode == LabelMode::Off {
        return Ok(records);
    }
    let total = pieces.len();
    for (i, piece) in pieces.iter_mut().enumerate() {
        ctl.check()?;
        ctl.report(i as f32 / total.max(1) as f32, &format!("Labelling {}", piece.name));
        let step = steps.iter().find(|s| s.part_id == piece.id).map_or(i + 1, |s| s.number);
        let text = label_text(&piece.name, step);
        let planes: Vec<_> = interfaces.iter().filter(|f| f.a == piece.id || f.b == piece.id).map(|f| f.plane).collect();
        match place_label(piece, &planes, &text, opts) {
            Ok((mesh, note)) => {
                if let Some(mut np) = Piece::new(mesh) {
                    np.id = piece.id;
                    np.name = piece.name.clone();
                    *piece = np;
                    records.push(LabelRecord { piece_id: piece.id, text, placed: true, note });
                }
            }
            Err(why) => records.push(LabelRecord { piece_id: piece.id, text, placed: false, note: why }),
        }
    }
    Ok(records)
}

#[cfg(test)]
mod tests {
    use super::*;
    use glam::Vec3;

    fn block_piece() -> Piece {
        let mut m = Mesh::cuboid(Vec3::new(120.0, 80.0, 40.0));
        m.translate(Vec3::new(60.0, 40.0, 20.0));
        let mut p = Piece::new(m).unwrap();
        p.id = 1;
        p.name = "P01".into();
        p
    }

    #[test]
    fn text_is_built_from_strokes_and_centred() {
        let (polys, width) = glyphs("P07-3", 8.0);
        assert!(polys.len() >= 5, "{}", polys.len());
        let (lo, hi) = polys.iter().flat_map(|p| p.outer.iter()).fold((f64::MAX, f64::MIN), |(l, h), p| (l.min(p.x), h.max(p.x)));
        assert!((lo + width / 2.0).abs() < 1e-6 && (hi - width / 2.0).abs() < 1e-6, "{lo} {hi} {width}");
        assert_eq!(glyphs("?", 8.0).0.len(), 0);
        assert_eq!(label_text("P07", 3), "P07-3");
    }

    #[test]
    fn engraving_removes_exactly_the_text_volume() {
        let p = block_piece();
        let o = LabelOptions { mode: LabelMode::Inset, ..Default::default() };
        let (m, note) = place_label(&p, &[], "P01-1", &o).unwrap();
        let removed = p.volume - m.stats().signed_volume;
        assert!(removed > 20.0 && removed < 400.0, "removed {removed} mm³ ({note})");
        assert!(m.stats().is_watertight());
    }

    #[test]
    fn raised_text_adds_material_and_stays_attached() {
        let p = block_piece();
        let o = LabelOptions { mode: LabelMode::Outset, ..Default::default() };
        let (m, _) = place_label(&p, &[], "P01-1", &o).unwrap();
        let added = m.stats().signed_volume - p.volume;
        assert!(added > 20.0 && added < 400.0, "added {added}");
        assert!(m.stats().is_watertight());
        // One connected solid: the text is fused to the part, not floating.
        assert_eq!(openslicer_geom::components::split_components(&m).len(), 1);
    }

    #[test]
    fn a_wall_too_thin_for_the_relief_is_refused_with_a_reason() {
        let mut m = Mesh::cuboid(Vec3::new(120.0, 80.0, 1.5));
        m.translate(Vec3::new(60.0, 40.0, 0.75));
        let mut p = Piece::new(m).unwrap();
        p.name = "P01".into();
        let o = LabelOptions { mode: LabelMode::Inset, ..Default::default() };
        let err = place_label(&p, &[], "P01-1", &o).unwrap_err();
        assert!(err.contains("material behind") || err.contains("flat area"), "{err}");
    }

    #[test]
    fn options_are_validated() {
        assert!(LabelOptions::default().validate().is_ok());
        assert!(LabelOptions { height: 1.0, ..Default::default() }.validate().is_err());
        assert!(LabelOptions { depth: 9.0, ..Default::default() }.validate().is_err());
    }
}
