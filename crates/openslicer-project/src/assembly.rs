//! Assembly planning: build order, exploded views, and printable instructions with diagrams.
//!
//! The order is derived from which pieces touch: it starts from the lowest piece and always adds the piece
//! with the most contact area with what is already assembled, so every step attaches to the existing assembly
//! (a disconnected piece starts a new sub-assembly). Diagrams are rendered from the real meshes as SVG.

use glam::{DVec3, Vec3};
use openslicer_core::Mesh;
use openslicer_joints::apply::ConnectorRecord;
use openslicer_joints::settings::ConnectorKind;
use openslicer_split::exec::Piece;
use openslicer_split::interfaces::Interface;
use rustc_hash::FxHashMap;
use std::fmt::Write;

#[derive(Clone, Debug)]
pub struct AssemblyStep {
    pub number: usize,
    pub part_id: usize,
    pub part_name: String,
    /// Pieces this one is joined to, with the contact area (mm²).
    pub attach_to: Vec<(usize, String, f64)>,
    /// Connector labels used in this step.
    pub connectors: Vec<String>,
    /// Loose parts (dowels) used in this step.
    pub loose: Vec<String>,
    /// True when the piece touches nothing assembled so far (start of a sub-assembly).
    pub starts_subassembly: bool,
}

pub fn plan_assembly(pieces: &[Piece], interfaces: &[Interface], connectors: &[ConnectorRecord], loose: &[Piece]) -> Vec<AssemblyStep> {
    let mut contact: FxHashMap<(usize, usize), f64> = FxHashMap::default();
    for i in interfaces {
        *contact.entry((i.a.min(i.b), i.a.max(i.b))).or_insert(0.0) += i.area;
    }
    let area_between = |a: usize, b: usize| contact.get(&(a.min(b), a.max(b))).copied().unwrap_or(0.0);
    let name_of = |id: usize| pieces.iter().find(|p| p.id == id).map_or_else(|| id.to_string(), |p| p.name.clone());
    // Dowel connectors map to loose parts in creation order.
    let dowel_labels: Vec<(String, usize)> = connectors
        .iter()
        .filter(|c| c.kind == ConnectorKind::Dowel)
        .enumerate()
        .map(|(k, c)| (loose.get(k).map_or_else(|| format!("D{:02}", k + 1), |l| l.name.clone()), c.interface))
        .collect();

    let mut remaining: Vec<usize> = pieces.iter().map(|p| p.id).collect();
    let z = |id: usize| pieces.iter().find(|p| p.id == id).map_or(0.0, |p| p.bounds.center().z as f64);
    let mut assembled: Vec<usize> = Vec::new();
    let mut steps = Vec::new();
    while !remaining.is_empty() {
        let scored: Vec<(usize, f64)> =
            remaining.iter().map(|&r| (r, assembled.iter().map(|&a| area_between(r, a)).sum::<f64>())).collect();
        let best = scored
            .iter()
            .filter(|(_, s)| *s > 0.0)
            .max_by(|a, b| a.1.total_cmp(&b.1).then(z(b.0).total_cmp(&z(a.0))).then(b.0.cmp(&a.0)))
            .copied();
        let (next, starts) = match best {
            Some((id, _)) => (id, false),
            None => (*remaining.iter().min_by(|&&a, &&b| z(a).total_cmp(&z(b)).then(a.cmp(&b))).unwrap(), true),
        };
        remaining.retain(|&r| r != next);
        let attach_to: Vec<(usize, String, f64)> =
            assembled.iter().filter(|&&a| area_between(next, a) > 0.0).map(|&a| (a, name_of(a), area_between(next, a))).collect();
        let partners: Vec<usize> = attach_to.iter().map(|(a, _, _)| *a).collect();
        let iface_involved = |ii: usize| {
            let i = &interfaces[ii];
            (i.a == next && partners.contains(&i.b)) || (i.b == next && partners.contains(&i.a))
        };
        let conns: Vec<String> = connectors.iter().filter(|c| iface_involved(c.interface)).map(|c| c.id.clone()).collect();
        let loose_used: Vec<String> = dowel_labels.iter().filter(|(_, ii)| iface_involved(*ii)).map(|(n, _)| n.clone()).collect();
        steps.push(AssemblyStep {
            number: steps.len() + 1,
            part_id: next,
            part_name: name_of(next),
            attach_to,
            connectors: conns,
            loose: loose_used,
            starts_subassembly: starts,
        });
        assembled.push(next);
    }
    steps
}

/// Exploded-view offsets: each piece moves away from the assembly's centre by `factor` times its distance from it.
pub fn explode_offsets(pieces: &[Piece], factor: f64) -> FxHashMap<usize, DVec3> {
    let total: f64 = pieces.iter().map(|p| p.volume.max(1e-9)).sum();
    let centre: DVec3 = pieces.iter().map(|p| p.bounds.center().as_dvec3() * p.volume.max(1e-9)).sum::<DVec3>() / total.max(1e-9);
    pieces
        .iter()
        .map(|p| {
            let d = p.bounds.center().as_dvec3() - centre;
            (p.id, if d.length() < 1e-6 { DVec3::Z * factor } else { d * factor })
        })
        .collect()
}

// ---- SVG rendering --------------------------------------------------------------------------------------------

/// Reduces a mesh to roughly `target` triangles by merging vertices that fall in the same grid cell.
/// Only for diagrams; never used for export.
pub fn decimate_for_display(mesh: &Mesh, target: usize) -> Mesh {
    if mesh.triangles.len() <= target {
        return mesh.clone();
    }
    let b = mesh.bounds().unwrap();
    let size = b.size();
    let mut cell = (size.x * size.y * size.z).max(1e-9).cbrt() / (target as f32 / 6.0).max(2.0).cbrt();
    cell = cell.max(size.max_element() / 400.0).max(1e-6);
    let key = |v: Vec3| [((v.x - b.min.x) / cell) as i32, ((v.y - b.min.y) / cell) as i32, ((v.z - b.min.z) / cell) as i32];
    let mut cells: FxHashMap<[i32; 3], (u32, Vec3, u32)> = FxHashMap::default();
    let mut next = 0u32;
    let remap: Vec<u32> = mesh
        .vertices
        .iter()
        .map(|&v| {
            let e = cells.entry(key(v)).or_insert_with(|| {
                next += 1;
                (next - 1, Vec3::ZERO, 0)
            });
            e.1 += v;
            e.2 += 1;
            e.0
        })
        .collect();
    let mut verts = vec![Vec3::ZERO; next as usize];
    for (id, sum, n) in cells.values() {
        verts[*id as usize] = *sum / *n as f32;
    }
    let tris = mesh
        .triangles
        .iter()
        .map(|t| [remap[t[0] as usize], remap[t[1] as usize], remap[t[2] as usize]])
        .filter(|t| t[0] != t[1] && t[1] != t[2] && t[0] != t[2])
        .collect();
    Mesh::new(verts, tris)
}

#[derive(Clone, Debug)]
pub struct RenderItem<'a> {
    pub mesh: &'a Mesh,
    pub offset: DVec3,
    /// Base colour (r, g, b) in 0..=255.
    pub color: [u8; 3],
    pub label: Option<String>,
}

#[derive(Clone, Copy, Debug)]
pub struct ViewSettings {
    pub width: f64,
    pub height: f64,
    /// Rotation about Z and elevation, in degrees.
    pub yaw_deg: f64,
    pub elevation_deg: f64,
    pub max_triangles_per_item: usize,
}

impl Default for ViewSettings {
    fn default() -> Self {
        Self { width: 640.0, height: 480.0, yaw_deg: -35.0, elevation_deg: 30.0, max_triangles_per_item: 2500 }
    }
}

fn esc(s: &str) -> String {
    s.replace('&', "&amp;").replace('<', "&lt;").replace('>', "&gt;").replace('"', "&quot;")
}

/// Renders items as a flat-shaded orthographic SVG (painter's algorithm, back faces culled).
pub fn render_svg(items: &[RenderItem], v: &ViewSettings) -> String {
    let (yaw, el) = (v.yaw_deg.to_radians(), v.elevation_deg.to_radians());
    // Camera basis (Z up): right, up and the direction pointing from the scene towards the viewer.
    let to_viewer = DVec3::new(yaw.sin() * el.cos(), -yaw.cos() * el.cos(), el.sin());
    let right = DVec3::Z.cross(to_viewer).normalize();
    let up = to_viewer.cross(right);
    let light = (to_viewer * 0.6 + right * -0.4 + up * 0.7).normalize();

    struct Tri {
        depth: f64,
        pts: [(f64, f64); 3],
        shade: f64,
        color: [u8; 3],
    }
    let mut tris: Vec<Tri> = Vec::new();
    let mut labels: Vec<(f64, f64, String)> = Vec::new();
    for item in items {
        let m = decimate_for_display(item.mesh, v.max_triangles_per_item);
        let world: Vec<DVec3> = m.vertices.iter().map(|p| p.as_dvec3() + item.offset).collect();
        let (mut lo, mut hi) = (DVec3::splat(f64::MAX), DVec3::splat(f64::MIN));
        for t in &m.triangles {
            let [a, b, c] = t.map(|i| world[i as usize]);
            let n = (b - a).cross(c - a);
            let len = n.length();
            if len < 1e-12 {
                continue;
            }
            let n = n / len;
            lo = lo.min(a).min(b).min(c);
            hi = hi.max(a).max(b).max(c);
            if n.dot(to_viewer) <= 0.0 {
                continue; // back face
            }
            let proj = |p: DVec3| (p.dot(right), -p.dot(up));
            tris.push(Tri {
                depth: ((a + b + c) / 3.0).dot(to_viewer),
                pts: [proj(a), proj(b), proj(c)],
                shade: 0.35 + 0.65 * n.dot(light).max(0.0),
                color: item.color,
            });
        }
        if let Some(l) = &item.label {
            let c = (lo + hi) * 0.5;
            labels.push((c.dot(right), -c.dot(up), l.clone()));
        }
    }
    if tris.is_empty() {
        return format!("<svg xmlns=\"http://www.w3.org/2000/svg\" viewBox=\"0 0 {} {}\"/>", v.width, v.height);
    }
    tris.sort_by(|a, b| a.depth.total_cmp(&b.depth));
    let (mut minx, mut miny, mut maxx, mut maxy) = (f64::MAX, f64::MAX, f64::MIN, f64::MIN);
    for t in &tris {
        for p in t.pts {
            minx = minx.min(p.0);
            maxx = maxx.max(p.0);
            miny = miny.min(p.1);
            maxy = maxy.max(p.1);
        }
    }
    let margin = 24.0;
    let scale = ((v.width - 2.0 * margin) / (maxx - minx).max(1e-9)).min((v.height - 2.0 * margin) / (maxy - miny).max(1e-9));
    let (ox, oy) = (
        margin + ((v.width - 2.0 * margin) - (maxx - minx) * scale) / 2.0,
        margin + ((v.height - 2.0 * margin) - (maxy - miny) * scale) / 2.0,
    );
    let map = |p: (f64, f64)| (ox + (p.0 - minx) * scale, oy + (p.1 - miny) * scale);
    let mut svg = String::new();
    let _ = write!(svg, "<svg xmlns=\"http://www.w3.org/2000/svg\" viewBox=\"0 0 {} {}\" width=\"{}\" height=\"{}\" font-family=\"Segoe UI, Arial, sans-serif\">", v.width, v.height, v.width, v.height);
    svg.push_str("<rect width=\"100%\" height=\"100%\" fill=\"#ffffff\"/>");
    for t in &tris {
        let c = t.color.map(|c| (c as f64 * t.shade).round().clamp(0.0, 255.0) as u8);
        let (a, b, d) = (map(t.pts[0]), map(t.pts[1]), map(t.pts[2]));
        let _ = write!(svg, "<polygon points=\"{:.1},{:.1} {:.1},{:.1} {:.1},{:.1}\" fill=\"#{:02x}{:02x}{:02x}\" stroke=\"#{:02x}{:02x}{:02x}\" stroke-width=\"0.4\"/>", a.0, a.1, b.0, b.1, d.0, d.1, c[0], c[1], c[2], c[0], c[1], c[2]);
    }
    for (x, y, text) in &labels {
        let (px, py) = map((*x, *y));
        let _ = write!(svg, "<text x=\"{px:.1}\" y=\"{py:.1}\" font-size=\"14\" font-weight=\"600\" text-anchor=\"middle\" fill=\"#111\" stroke=\"#fff\" stroke-width=\"3\" paint-order=\"stroke\">{}</text>", esc(text));
    }
    svg.push_str("</svg>");
    svg
}

// ---- Instructions ----------------------------------------------------------------------------------------------

pub struct DocInput<'a> {
    pub title: &'a str,
    pub pieces: &'a [Piece],
    pub loose: &'a [Piece],
    pub connectors: &'a [ConnectorRecord],
    pub interfaces: &'a [Interface],
    pub steps: &'a [AssemblyStep],
    pub printer_name: &'a str,
    pub notes: &'a [String],
}

const GREY: [u8; 3] = [168, 174, 184];
const HIGHLIGHT: [u8; 3] = [0, 120, 215];
const PALETTE: [[u8; 3]; 8] =
    [[0, 120, 215], [232, 134, 12], [16, 137, 62], [196, 43, 28], [136, 23, 152], [0, 153, 188], [153, 118, 0], [90, 98, 110]];

/// A self-contained HTML document: overview, parts list, connector table and one diagram per step.
pub fn instructions_html(d: &DocInput) -> String {
    let mut h = String::new();
    let _ = write!(
        h,
        "<!doctype html><html lang=\"en\"><head><meta charset=\"utf-8\"><title>{t}</title><style>\
         body{{font-family:'Segoe UI',Arial,sans-serif;margin:2rem auto;max-width:960px;color:#1b1b1b;line-height:1.45}}\
         h1{{font-size:1.9rem;margin-bottom:.2rem}}h2{{border-bottom:2px solid #0078d4;padding-bottom:.2rem;margin-top:2.2rem}}\
         table{{border-collapse:collapse;width:100%;font-size:.92rem}}th,td{{border:1px solid #ccc;padding:.35rem .6rem;text-align:left}}th{{background:#f0f4f8}}\
         .step{{page-break-inside:avoid;border:1px solid #d0d7de;border-radius:8px;padding:1rem;margin:1.2rem 0}}.step svg{{width:100%;height:auto;border:1px solid #eee}}\
         .muted{{color:#555}}.warn{{background:#fff4ce;border-left:4px solid #f7c948;padding:.5rem 1rem;margin:.5rem 0}}code{{background:#f3f3f3;padding:0 .25rem}}\
         @media print{{body{{margin:0}}h2{{page-break-after:avoid}}}}</style></head><body>",
        t = esc(d.title)
    );
    let _ = write!(
        h,
        "<h1>{}</h1><p class=\"muted\">Assembly instructions · {} parts · printer profile: {}</p>",
        esc(d.title),
        d.pieces.len() + d.loose.len(),
        esc(d.printer_name)
    );
    for n in d.notes {
        let _ = write!(h, "<div class=\"warn\">{}</div>", esc(n));
    }
    // Overview diagram.
    let offsets = explode_offsets(d.pieces, 0.35);
    let overview: Vec<RenderItem> = d
        .pieces
        .iter()
        .enumerate()
        .map(|(i, p)| RenderItem { mesh: &p.mesh, offset: offsets[&p.id], color: PALETTE[i % PALETTE.len()], label: Some(p.name.clone()) })
        .collect();
    h.push_str("<h2>Overview (exploded)</h2>");
    h.push_str(&render_svg(&overview, &ViewSettings::default()));
    // Parts list.
    h.push_str("<h2>Parts list</h2><table><tr><th>Part</th><th>Size (mm)</th><th>Volume (cm³)</th><th>Triangles</th></tr>");
    for p in d.pieces.iter().chain(d.loose.iter()) {
        let s = p.bounds.size();
        let _ = write!(
            h,
            "<tr><td><b>{}</b></td><td>{:.1} × {:.1} × {:.1}</td><td>{:.2}</td><td>{}</td></tr>",
            esc(&p.name),
            s.x,
            s.y,
            s.z,
            p.volume / 1000.0,
            p.mesh.triangles.len()
        );
    }
    h.push_str("</table>");
    // Connectors.
    if !d.connectors.is_empty() {
        h.push_str(
            "<h2>Connectors</h2><table><tr><th>ID</th><th>Type</th><th>Between</th><th>Position (mm)</th><th>Size × height</th></tr>",
        );
        let name = |id: usize| d.pieces.iter().find(|p| p.id == id).map_or_else(|| id.to_string(), |p| p.name.clone());
        for c in d.connectors {
            let male = c.male_piece.map_or_else(|| "loose part".to_string(), name);
            let _ = write!(
                h,
                "<tr><td>{}</td><td>{}</td><td>{} → {}</td><td>{:.1}, {:.1}, {:.1}</td><td>{:.1} × {:.1}</td></tr>",
                esc(&c.id),
                c.kind.label(),
                esc(&male),
                esc(&name(c.female_piece)),
                c.position.x,
                c.position.y,
                c.position.z,
                c.size,
                c.height
            );
        }
        h.push_str("</table>");
    }
    // Steps.
    h.push_str("<h2>Assembly steps</h2>");
    let mut done: Vec<usize> = Vec::new();
    for step in d.steps {
        let _ = write!(h, "<div class=\"step\"><h3>Step {}: add {}</h3>", step.number, esc(&step.part_name));
        if step.starts_subassembly {
            let _ = write!(
                h,
                "<p>Place <b>{}</b> as the {} of the assembly.</p>",
                esc(&step.part_name),
                if step.number == 1 { "base" } else { "start of a separate sub-assembly" }
            );
        } else {
            let partners: Vec<String> =
                step.attach_to.iter().map(|(_, n, a)| format!("<b>{}</b> ({:.0} mm² contact)", esc(n), a)).collect();
            let _ = write!(h, "<p>Join <b>{}</b> to {}.</p>", esc(&step.part_name), partners.join(", "));
        }
        if !step.connectors.is_empty() {
            let _ = write!(h, "<p>Connectors: {}.</p>", step.connectors.join(", "));
        }
        if !step.loose.is_empty() {
            let _ = write!(h, "<p>Insert dowel(s) {} before joining the parts.</p>", step.loose.join(", "));
        }
        let mut items: Vec<RenderItem> = Vec::new();
        for p in d.pieces {
            if done.contains(&p.id) {
                items.push(RenderItem { mesh: &p.mesh, offset: DVec3::ZERO, color: GREY, label: None });
            }
        }
        if let Some(p) = d.pieces.iter().find(|p| p.id == step.part_id) {
            // Show the new part lifted away from the assembly along its outward direction.
            let off = explode_offsets(d.pieces, 0.0);
            let _ = off;
            let dir = explode_offsets(d.pieces, 0.25).get(&p.id).copied().unwrap_or(DVec3::Z * 10.0);
            items.push(RenderItem { mesh: &p.mesh, offset: dir, color: HIGHLIGHT, label: Some(p.name.clone()) });
        }
        h.push_str(&render_svg(&items, &ViewSettings { width: 560.0, height: 380.0, max_triangles_per_item: 1500, ..Default::default() }));
        h.push_str("</div>");
        done.push(step.part_id);
    }
    h.push_str("<p class=\"muted\">Generated by OpenSlicer. Print one test joint first and adjust the clearance for your printer and material.</p></body></html>");
    h
}

#[cfg(test)]
mod tests {
    use super::*;
    use openslicer_core::control::Control;
    use openslicer_geom::Plane;
    use openslicer_joints::apply::generate_joints;
    use openslicer_joints::settings::ConnectorSettings;
    use openslicer_split::exec::{execute_plan, SplitOptions};
    use openslicer_split::interfaces::find_interfaces;
    use openslicer_split::plan::CutPlan;

    fn strip() -> (Vec<Piece>, Vec<Interface>) {
        let mut m = Mesh::cuboid(Vec3::new(300.0, 60.0, 30.0));
        m.translate(Vec3::new(150.0, 30.0, 15.0));
        let r = execute_plan(
            &m,
            &CutPlan::from_planes([Plane::axis(0, 100.0), Plane::axis(0, 200.0)]),
            &SplitOptions { tolerance: None, separate_solids: true },
            &Control::none(),
        )
        .unwrap();
        let ifs = find_interfaces(&r.pieces, &r.planes, 1e-3);
        (r.pieces, ifs)
    }

    #[test]
    fn steps_follow_contact_and_every_step_attaches_to_the_assembly() {
        let (pieces, ifs) = strip();
        let steps = plan_assembly(&pieces, &ifs, &[], &[]);
        assert_eq!(steps.len(), 3);
        assert!(steps[0].starts_subassembly && steps[0].attach_to.is_empty());
        for s in &steps[1..] {
            assert!(!s.starts_subassembly && !s.attach_to.is_empty(), "{s:?}");
        }
        let ids: Vec<usize> = steps.iter().map(|s| s.part_id).collect();
        let mut sorted = ids.clone();
        sorted.sort();
        sorted.dedup();
        assert_eq!(sorted.len(), 3, "every part appears exactly once");
        // The middle piece touches both neighbours: whichever order, it is never first-then-isolated.
        let last = &steps[2];
        assert!(last.attach_to.iter().all(|(id, _, a)| ids[..2].contains(id) && *a > 0.0));
    }

    #[test]
    fn disconnected_parts_start_new_subassemblies_and_connectors_are_listed() {
        let (pieces, ifs) = strip();
        let res =
            generate_joints(&pieces, &ifs, &ConnectorSettings::new(openslicer_joints::settings::ConnectorKind::Pin), &Control::none())
                .unwrap();
        let steps = plan_assembly(&res.pieces, &ifs, &res.connectors, &res.loose);
        let listed: usize = steps.iter().map(|s| s.connectors.len()).sum();
        assert_eq!(listed, res.connectors.len(), "every connector appears in exactly one step");
        // With no interfaces at all, each piece is its own sub-assembly.
        let isolated = plan_assembly(&pieces, &[], &[], &[]);
        assert!(isolated.iter().all(|s| s.starts_subassembly));
    }

    #[test]
    fn dowel_steps_name_the_loose_parts() {
        let (pieces, ifs) = strip();
        let res =
            generate_joints(&pieces, &ifs, &ConnectorSettings::new(openslicer_joints::settings::ConnectorKind::Dowel), &Control::none())
                .unwrap();
        assert!(!res.loose.is_empty());
        let steps = plan_assembly(&res.pieces, &ifs, &res.connectors, &res.loose);
        let named: Vec<&String> = steps.iter().flat_map(|s| &s.loose).collect();
        assert_eq!(named.len(), res.loose.len());
        assert!(res.loose.iter().all(|l| named.contains(&&l.name)));
    }

    #[test]
    fn exploded_offsets_push_parts_apart_from_the_centre() {
        let (pieces, _) = strip();
        let off = explode_offsets(&pieces, 0.5);
        let mut by_x: Vec<(f32, DVec3)> = pieces.iter().map(|p| (p.bounds.center().x, off[&p.id])).collect();
        by_x.sort_by(|a, b| a.0.total_cmp(&b.0));
        assert!(by_x[0].1.x < 0.0 && by_x[2].1.x > 0.0 && by_x[1].1.x.abs() < 1.0);
    }

    #[test]
    fn decimation_reduces_triangles_and_keeps_the_shape() {
        let torus = {
            let (nu, nv, big, small) = (120usize, 60usize, 60.0f32, 15.0f32);
            let mut verts = Vec::new();
            for i in 0..nu {
                for j in 0..nv {
                    let (u, v) = (std::f32::consts::TAU * i as f32 / nu as f32, std::f32::consts::TAU * j as f32 / nv as f32);
                    verts.push(Vec3::new((big + small * v.cos()) * u.cos(), (big + small * v.cos()) * u.sin(), small * v.sin()));
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
        };
        let d = decimate_for_display(&torus, 1500);
        assert!(d.triangles.len() < torus.triangles.len() / 2 && d.triangles.len() > 200, "{}", d.triangles.len());
        let (b0, b1) = (torus.bounds().unwrap().size(), d.bounds().unwrap().size());
        assert!((b0 - b1).length() < 0.2 * b0.length());
    }

    #[test]
    fn svg_is_valid_xml_with_depth_sorted_polygons_and_labels() {
        let (pieces, _) = strip();
        let items: Vec<RenderItem> = pieces
            .iter()
            .map(|p| RenderItem { mesh: &p.mesh, offset: DVec3::ZERO, color: [0, 120, 215], label: Some(p.name.clone()) })
            .collect();
        let svg = render_svg(&items, &ViewSettings::default());
        let doc = roxmltree::Document::parse(&svg).expect("valid XML");
        let polys = doc.descendants().filter(|n| n.has_tag_name("polygon")).count();
        // 3 boxes seen from a corner show 3 faces each (12 triangles minimum); cut faces carry extra vertices from
        // the cap triangulation, and back faces are culled so the count is well below the total.
        let total: usize = pieces.iter().map(|p| p.mesh.triangles.len()).sum();
        assert!(polys >= 18 && polys < total, "{polys} of {total}");
        let labels: Vec<_> = doc.descendants().filter(|n| n.has_tag_name("text")).filter_map(|n| n.text()).collect();
        assert_eq!(labels.len(), 3);
        assert!(render_svg(&[], &ViewSettings::default()).contains("<svg"));
        // Names with markup characters are escaped.
        let tricky = vec![RenderItem { mesh: &pieces[0].mesh, offset: DVec3::ZERO, color: GREY, label: Some("<b>&".into()) }];
        assert!(roxmltree::Document::parse(&render_svg(&tricky, &ViewSettings::default())).is_ok());
    }

    #[test]
    fn instructions_document_contains_every_step_part_and_connector() {
        let (pieces, ifs) = strip();
        let res =
            generate_joints(&pieces, &ifs, &ConnectorSettings::new(openslicer_joints::settings::ConnectorKind::Pin), &Control::none())
                .unwrap();
        let steps = plan_assembly(&res.pieces, &ifs, &res.connectors, &res.loose);
        let html = instructions_html(&DocInput {
            title: "Test <model>",
            pieces: &res.pieces,
            loose: &res.loose,
            connectors: &res.connectors,
            interfaces: &ifs,
            steps: &steps,
            printer_name: "Ender-3",
            notes: &["check the clearance".to_string()],
        });
        assert!(html.starts_with("<!doctype html>") && html.contains("Test &lt;model&gt;"));
        for s in &steps {
            assert!(html.contains(&format!("Step {}:", s.number)));
        }
        for p in &res.pieces {
            assert!(html.contains(&p.name));
        }
        for c in &res.connectors {
            assert!(html.contains(&c.id));
        }
        assert_eq!(html.matches("<svg").count(), steps.len() + 1, "one diagram per step plus the overview");
        assert!(html.contains("check the clearance"));
    }
}
