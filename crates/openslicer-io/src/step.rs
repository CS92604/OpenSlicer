//! STEP (ISO 10303-21) reading and writing.
//!
//! **Reading** handles two kinds of files:
//! * *Faceted* B-reps (`FACETED_BREP` with `POLY_LOOP` faces), which is what mesh-to-STEP converters and
//!   OpenSlicer itself write. These are read exactly, without any tessellation.
//! * *Exact* B-reps (planes, cylinders, cones, spheres, tori, B-spline and NURBS surfaces, as far as the
//!   `truck` STEP reader supports them), which are tessellated with a chord tolerance of 0.1 % of the part's
//!   size. The resulting mesh is an approximation of the curved surfaces.
//!
//! Lengths are converted to millimetres when the file declares metres, centimetres, inches or feet. Assembly
//! placements (`ITEM_DEFINED_TRANSFORMATION`) are not applied, and colours, names and PMI are ignored.
//!
//! **Writing** always produces a faceted B-rep: every triangle becomes a face. It opens in CAD packages that
//! read faceted solids, but it is a mesh in a STEP wrapper, not a parametric model.

use crate::IoError;
use glam::Vec3;
use openslicer_core::Mesh;
use std::collections::HashMap;

fn bad(msg: impl Into<String>) -> IoError {
    IoError::InvalidStep(msg.into())
}

/// Splits the DATA section into statements, honouring quoted strings and `/* */` comments.
fn statements(text: &str) -> Vec<String> {
    let mut out = Vec::new();
    let mut cur = String::new();
    let (mut in_str, mut in_comment) = (false, false);
    let mut chars = text.chars().peekable();
    while let Some(c) = chars.next() {
        if in_comment {
            if c == '*' && chars.peek() == Some(&'/') {
                chars.next();
                in_comment = false;
            }
            continue;
        }
        if in_str {
            cur.push(c);
            if c == '\'' {
                if chars.peek() == Some(&'\'') {
                    cur.push(chars.next().unwrap());
                } else {
                    in_str = false;
                }
            }
            continue;
        }
        match c {
            '/' if chars.peek() == Some(&'*') => {
                chars.next();
                in_comment = true;
            }
            '\'' => {
                in_str = true;
                cur.push(c);
            }
            ';' => {
                let s = cur.trim();
                if !s.is_empty() {
                    out.push(s.to_string());
                }
                cur.clear();
            }
            c if c.is_whitespace() => {}
            c => cur.push(c),
        }
    }
    out
}

/// Millimetres per file unit, from the length unit the file declares (1 when it declares none we know).
fn length_scale(stmts: &[String]) -> f64 {
    for s in stmts {
        if !s.contains("LENGTH_UNIT") {
            continue;
        }
        if s.contains("CONVERSION_BASED_UNIT") {
            let up = s.to_ascii_uppercase();
            if up.contains("'INCH'") {
                return 25.4;
            }
            if up.contains("'FOOT'") {
                return 304.8;
            }
        }
        if let Some(p) = s.find("SI_UNIT(") {
            let args = &s[p + 8..];
            if args.starts_with(".MILLI.,.METRE.") {
                return 1.0;
            } else if args.starts_with(".CENTI.,.METRE.") {
                return 10.0;
            } else if args.starts_with("$,.METRE.") {
                return 1000.0;
            } else if args.starts_with(".MICRO.,.METRE.") {
                return 0.001;
            }
        }
    }
    1.0
}

fn refs(s: &str) -> Vec<u64> {
    s.split('#').skip(1).filter_map(|t| t.split(|c: char| !c.is_ascii_digit()).next()?.parse().ok()).collect()
}

/// Reads a faceted B-rep; `None` when the file has no `FACETED_BREP`/`POLY_LOOP` faces.
fn read_faceted(stmts: &[String], scale: f64) -> Result<Option<Vec<[Vec3; 3]>>, IoError> {
    let mut points: HashMap<u64, Vec3> = HashMap::new();
    let mut loops: HashMap<u64, Vec<u64>> = HashMap::new();
    let mut bounds: HashMap<u64, (u64, bool)> = HashMap::new();
    let mut faces: HashMap<u64, Vec<u64>> = HashMap::new();
    let mut shells: Vec<Vec<u64>> = Vec::new();
    let mut breps: Vec<u64> = Vec::new();
    let mut shell_map: HashMap<u64, Vec<u64>> = HashMap::new();
    for s in stmts {
        let Some(rest) = s.strip_prefix('#') else { continue };
        let Some((id, body)) = rest.split_once('=') else { continue };
        let Ok(id) = id.parse::<u64>() else { continue };
        let Some((name, args)) = body.split_once('(') else { continue };
        match name {
            "CARTESIAN_POINT" => {
                let inner = args.rsplit_once("(").map(|x| x.1).unwrap_or(args);
                let nums: Vec<f64> = inner.split(')').next().unwrap_or("").split(',').filter_map(|t| t.parse().ok()).collect();
                if nums.len() == 3 {
                    points.insert(id, Vec3::new((nums[0] * scale) as f32, (nums[1] * scale) as f32, (nums[2] * scale) as f32));
                }
            }
            "POLY_LOOP" => {
                loops.insert(id, refs(args));
            }
            "FACE_BOUND" | "FACE_OUTER_BOUND" => {
                let r = refs(args);
                if let Some(&l) = r.first() {
                    bounds.insert(id, (l, !args.trim_end_matches(')').ends_with(".F.")));
                }
            }
            "FACE" => {
                faces.insert(id, refs(args));
            }
            "CLOSED_SHELL" | "OPEN_SHELL" => {
                shell_map.insert(id, refs(args));
            }
            "FACETED_BREP" => {
                if let Some(&sh) = refs(args).first() {
                    breps.push(sh);
                }
            }
            _ => {}
        }
    }
    if loops.is_empty() || breps.is_empty() {
        return Ok(None);
    }
    for b in breps {
        shells.push(shell_map.get(&b).cloned().ok_or_else(|| bad("a FACETED_BREP refers to a shell that is missing"))?);
    }
    let mut tris = Vec::new();
    for face_ids in shells.iter().flatten() {
        let face = faces.get(face_ids).ok_or_else(|| bad("a shell refers to a face that is missing"))?;
        if face.len() != 1 {
            return Err(bad("a faceted face has holes or no boundary, which is not supported"));
        }
        let &(loop_id, forward) = bounds.get(&face[0]).ok_or_else(|| bad("a face refers to a boundary that is missing"))?;
        let mut poly: Vec<Vec3> = loops
            .get(&loop_id)
            .ok_or_else(|| bad("a face refers to a loop that is missing"))?
            .iter()
            .map(|p| points.get(p).copied().ok_or_else(|| bad("a loop refers to a point that is missing")))
            .collect::<Result<_, _>>()?;
        if !forward {
            poly.reverse();
        }
        triangulate_polygon(&poly, &mut tris);
    }
    Ok(Some(tris))
}

/// Fan for triangles and quads, ear clipping in the polygon's plane for larger faces.
fn triangulate_polygon(poly: &[Vec3], out: &mut Vec<[Vec3; 3]>) {
    let n = poly.len();
    if n < 3 {
        return;
    }
    if n <= 4 {
        for k in 1..n - 1 {
            out.push([poly[0], poly[k], poly[k + 1]]);
        }
        return;
    }
    // Newell normal, then drop the dominant axis.
    let mut nrm = Vec3::ZERO;
    for i in 0..n {
        let (a, b) = (poly[i], poly[(i + 1) % n]);
        nrm += Vec3::new((a.y - b.y) * (a.z + b.z), (a.z - b.z) * (a.x + b.x), (a.x - b.x) * (a.y + b.y));
    }
    let drop = nrm.abs().max_position();
    let flat: Vec<f64> = poly
        .iter()
        .flat_map(|p| match drop {
            0 => [p.y as f64, p.z as f64],
            1 => [p.x as f64, p.z as f64],
            _ => [p.x as f64, p.y as f64],
        })
        .collect();
    if let Ok(idx) = earcutr::earcut(&flat, &[], 2) {
        for t in idx.chunks_exact(3) {
            let (a, b, c) = (poly[t[0]], poly[t[1]], poly[t[2]]);
            // Earcut returns the winding of the projected polygon; restore the 3D winding.
            let tri = if (b - a).cross(c - a).dot(nrm) >= 0.0 { [a, b, c] } else { [a, c, b] };
            out.push(tri);
        }
    }
}

/// Tessellates exact B-rep geometry with the `truck` reader.
fn read_exact(text: &str, scale: f64) -> Result<Vec<[Vec3; 3]>, IoError> {
    use truck_meshalgo::prelude::*;
    use truck_stepio::r#in::Table;
    let attempt = std::panic::catch_unwind(|| -> Result<Vec<[Vec3; 3]>, String> {
        let table = Table::from_step(text).ok_or("the file could not be parsed as STEP")?;
        if table.shell.is_empty() {
            return Err("the file contains no solid or shell geometry that OpenSlicer can read".into());
        }
        let mut tris = Vec::new();
        for shell in table.shell.values() {
            let cshell = table.to_compressed_shell(shell).map_err(|e| format!("unsupported geometry: {e}"))?;
            let rough = cshell.robust_triangulation(0.01).to_polygon();
            let diameter = rough.bounding_box().diameter();
            if !diameter.is_finite() || diameter <= 0.0 {
                return Err("a shell has no size".into());
            }
            let poly = cshell.robust_triangulation(diameter * 0.001).to_polygon();
            let positions = poly.positions();
            for f in poly.faces().triangle_iter() {
                let p = |i: usize| {
                    let q = positions[f[i].pos];
                    Vec3::new(q.x as f32, q.y as f32, q.z as f32)
                };
                tris.push([p(0), p(1), p(2)]);
            }
        }
        Ok(tris)
    });
    let mut tris = match attempt {
        Ok(Ok(t)) => t,
        Ok(Err(msg)) => return Err(bad(msg)),
        Err(_) => return Err(bad("the STEP reader failed on this file (probably an unsupported surface type)")),
    };
    if scale != 1.0 {
        for t in &mut tris {
            for v in t.iter_mut() {
                *v *= scale as f32;
            }
        }
    }
    Ok(tris)
}

pub fn read(data: &[u8]) -> Result<Mesh, IoError> {
    let text = String::from_utf8_lossy(data);
    if !text.trim_start().starts_with("ISO-10303-21") {
        return Err(bad("not a STEP file (missing ISO-10303-21 header)"));
    }
    let body = text.split_once("DATA;").map(|x| x.1).unwrap_or(&text);
    let stmts = statements(body);
    let scale = length_scale(&stmts);
    let tris = match read_faceted(&stmts, scale)? {
        Some(t) => t,
        None => read_exact(&text, scale)?,
    };
    if tris.is_empty() {
        return Err(IoError::Empty);
    }
    Ok(crate::weld(tris))
}

fn num(v: f32) -> String {
    let s = format!("{v}");
    if s.contains('.') || s.contains('e') || s.contains('E') {
        s
    } else {
        format!("{s}.")
    }
}

/// Writes a faceted B-rep (one face per triangle) in millimetres.
pub fn write(mesh: &Mesh, name: &str) -> String {
    use std::fmt::Write;
    let safe: String = name.chars().map(|c| if c.is_ascii_alphanumeric() || c == '_' || c == '-' || c == ' ' { c } else { '_' }).collect();
    let mut o = String::with_capacity(mesh.triangles.len() * 160 + mesh.vertices.len() * 64 + 2048);
    o.push_str("ISO-10303-21;\nHEADER;\nFILE_DESCRIPTION(('OpenSlicer faceted B-rep'),'2;1');\n");
    let _ = writeln!(o, "FILE_NAME('{safe}.step','',('OpenSlicer'),(''),'OpenSlicer','OpenSlicer','');");
    o.push_str("FILE_SCHEMA(('CONFIG_CONTROL_DESIGN'));\nENDSEC;\nDATA;\n");
    // Fixed header entities #1..#18, points from #20, then loops, bounds, faces.
    let _ = write!(
        o,
        "#1=APPLICATION_CONTEXT('configuration controlled 3D designs of mechanical parts and assemblies');\n\
         #2=APPLICATION_PROTOCOL_DEFINITION('international standard','config_control_design',1994,#1);\n\
         #3=MECHANICAL_CONTEXT('none',#1,'mechanical');\n\
         #4=PRODUCT('{safe}','{safe}','',(#3));\n\
         #5=PRODUCT_DEFINITION_FORMATION_WITH_SPECIFIED_SOURCE('','',#4,.NOT_KNOWN.);\n\
         #6=PRODUCT_DEFINITION_CONTEXT('part definition',#1,'design');\n\
         #7=PRODUCT_DEFINITION('design','',#5,#6);\n\
         #8=PRODUCT_DEFINITION_SHAPE('','',#7);\n\
         #9=(LENGTH_UNIT()NAMED_UNIT(*)SI_UNIT(.MILLI.,.METRE.));\n\
         #10=(NAMED_UNIT(*)PLANE_ANGLE_UNIT()SI_UNIT($,.RADIAN.));\n\
         #11=(NAMED_UNIT(*)SI_UNIT($,.STERADIAN.)SOLID_ANGLE_UNIT());\n\
         #12=UNCERTAINTY_MEASURE_WITH_UNIT(LENGTH_MEASURE(1.E-06),#9,'distance_accuracy_value','');\n\
         #13=(GEOMETRIC_REPRESENTATION_CONTEXT(3)GLOBAL_UNCERTAINTY_ASSIGNED_CONTEXT((#12))GLOBAL_UNIT_ASSIGNED_CONTEXT((#9,#10,#11))REPRESENTATION_CONTEXT('ID1','3D'));\n\
         #14=CARTESIAN_POINT('',(0.,0.,0.));\n\
         #15=DIRECTION('',(0.,0.,1.));\n\
         #16=DIRECTION('',(1.,0.,0.));\n\
         #17=AXIS2_PLACEMENT_3D('',#14,#15,#16);\n"
    );
    let first_point = 20u64;
    for (i, v) in mesh.vertices.iter().enumerate() {
        let _ = writeln!(o, "#{}=CARTESIAN_POINT('',({},{},{}));", first_point + i as u64, num(v.x), num(v.y), num(v.z));
    }
    let first_face = first_point + mesh.vertices.len() as u64;
    // Per triangle: loop, bound, face = three entities.
    let mut face_ids = Vec::with_capacity(mesh.triangles.len());
    for (k, t) in mesh.triangles.iter().enumerate() {
        let base = first_face + 3 * k as u64;
        let _ = writeln!(
            o,
            "#{base}=POLY_LOOP('',(#{},#{},#{}));\n#{}=FACE_OUTER_BOUND('',#{base},.T.);\n#{}=FACE('',(#{}));",
            first_point + t[0] as u64,
            first_point + t[1] as u64,
            first_point + t[2] as u64,
            base + 1,
            base + 2,
            base + 1
        );
        face_ids.push(base + 2);
    }
    let shell = first_face + 3 * mesh.triangles.len() as u64;
    let list: Vec<String> = face_ids.iter().map(|f| format!("#{f}")).collect();
    let _ = write!(
        o,
        "#{shell}=CLOSED_SHELL('',({}));\n#{}=FACETED_BREP('{safe}',#{shell});\n#{}=FACETED_BREP_SHAPE_REPRESENTATION('',(#17,#{}),#13);\n#{}=SHAPE_DEFINITION_REPRESENTATION(#8,#{});\nENDSEC;\nEND-ISO-10303-21;\n",
        list.join(","),
        shell + 1,
        shell + 2,
        shell + 1,
        shell + 3,
        shell + 2
    );
    o
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn faceted_round_trip_is_exact() {
        let mut m = Mesh::cuboid(Vec3::new(10.0, 20.0, 30.0));
        m.translate(Vec3::new(1.5, -2.25, 3.0));
        let back = read(write(&m, "box").as_bytes()).unwrap();
        let s = back.stats();
        assert!(s.is_watertight(), "{s:?}");
        assert!((s.signed_volume - 6000.0).abs() < 1e-3, "{}", s.signed_volume);
        assert_eq!(back.bounds().unwrap().size(), m.bounds().unwrap().size());
    }

    #[test]
    fn units_are_converted_to_millimetres() {
        let m = Mesh::cuboid(Vec3::new(1.0, 2.0, 3.0));
        let text = write(&m, "inch").replace("SI_UNIT(.MILLI.,.METRE.)", "CONVERSION_BASED_UNIT('INCH',#99)");
        let back = read(text.as_bytes()).unwrap();
        assert!((back.stats().signed_volume - 6.0 * 25.4f64.powi(3)).abs() < 1.0);
    }

    #[test]
    fn polygon_faces_with_many_vertices_are_triangulated() {
        // A pentagonal prism written as 2 pentagons and 5 quads.
        let ring = |z: f32| -> Vec<Vec3> {
            (0..5)
                .map(|k| {
                    let a = k as f32 / 5.0 * std::f32::consts::TAU;
                    Vec3::new(10.0 * a.cos(), 10.0 * a.sin(), z)
                })
                .collect()
        };
        let (b, t) = (ring(0.0), ring(5.0));
        let mut tris = Vec::new();
        let mut bottom = b.clone();
        bottom.reverse();
        triangulate_polygon(&bottom, &mut tris);
        triangulate_polygon(&t, &mut tris);
        for k in 0..5 {
            let j = (k + 1) % 5;
            triangulate_polygon(&[b[k], b[j], t[j], t[k]], &mut tris);
        }
        let m = crate::weld(tris);
        assert!(m.stats().is_watertight());
        let area = 0.5 * 5.0 * 100.0 * (std::f32::consts::TAU / 5.0).sin();
        assert!((m.stats().signed_volume - area as f64 * 5.0).abs() < 0.05, "{}", m.stats().signed_volume);
    }

    #[test]
    fn exact_curved_solid_is_tessellated_into_a_closed_mesh() {
        use truck_modeling::*;
        let v = builder::vertex(Point3::new(10.0, 0.0, 0.0));
        let w = builder::rsweep(&v, Point3::origin(), Vector3::unit_z(), Rad(7.0));
        let f = builder::try_attach_plane(&[w]).unwrap();
        let solid: Solid = builder::tsweep(&f, Vector3::new(0.0, 0.0, 30.0));
        let compressed = solid.compress();
        let text = truck_stepio::out::CompleteStepDisplay::new(
            truck_stepio::out::StepModel::from(&compressed),
            truck_stepio::out::StepHeaderDescriptor::default(),
        )
        .to_string();
        let m = read(text.as_bytes()).unwrap();
        let s = m.stats();
        assert!(s.is_watertight(), "{s:?}");
        let exact = std::f64::consts::PI * 100.0 * 30.0;
        assert!((s.signed_volume / exact - 1.0).abs() < 0.01, "{} vs {exact}", s.signed_volume);
    }

    #[test]
    fn garbage_is_rejected_with_a_reason() {
        assert!(matches!(read(b"solid x\nendsolid"), Err(IoError::InvalidStep(m)) if m.contains("not a STEP")));
        assert!(matches!(read(b"ISO-10303-21;\nHEADER;\nENDSEC;\nDATA;\nENDSEC;\nEND-ISO-10303-21;\n"), Err(IoError::InvalidStep(_))));
    }
}
