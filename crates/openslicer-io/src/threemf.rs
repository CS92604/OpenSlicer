//! 3MF reading and writing: a zip archive holding `3D/3dmodel.model`.
//!
//! Reading supports mesh objects, component references, build-item transforms and the model's length unit
//! (everything is converted to millimetres). Writing produces one mesh object per input and one build item each.

use crate::IoError;
use glam::{Mat4, Vec3};
use openslicer_core::Mesh;
use std::io::{Cursor, Read, Write};

const MODEL_PATH: &str = "3D/3dmodel.model";

fn unit_to_mm(unit: &str) -> Result<f32, IoError> {
    Ok(match unit {
        "micron" => 0.001,
        "millimeter" | "" => 1.0,
        "centimeter" => 10.0,
        "inch" => 25.4,
        "foot" => 304.8,
        "meter" => 1000.0,
        other => return Err(IoError::Invalid3mf(format!("unknown unit '{other}'"))),
    })
}

fn parse_transform(s: &str) -> Result<Mat4, IoError> {
    let v: Vec<f32> = s
        .split_whitespace()
        .map(|x| x.parse::<f32>())
        .collect::<Result<_, _>>()
        .map_err(|_| IoError::Invalid3mf("bad transform".into()))?;
    if v.len() != 12 {
        return Err(IoError::Invalid3mf("a transform needs 12 numbers".into()));
    }
    // 3MF stores a 4x3 matrix acting on row vectors: p' = p * M.
    Ok(Mat4::from_cols_array(&[v[0], v[1], v[2], 0.0, v[3], v[4], v[5], 0.0, v[6], v[7], v[8], 0.0, v[9], v[10], v[11], 1.0]))
}

struct Object {
    id: String,
    name: String,
    mesh: Option<Mesh>,
    components: Vec<(String, Mat4)>,
}

/// Reads all build items of a 3MF file as separate named meshes (units converted to mm, transforms applied).
pub fn read_objects(data: &[u8]) -> Result<Vec<(String, Mesh)>, IoError> {
    let mut zip = zip::ZipArchive::new(Cursor::new(data)).map_err(|e| IoError::Invalid3mf(format!("not a valid 3MF archive: {e}")))?;
    let mut xml = String::new();
    {
        let name = (0..zip.len())
            .filter_map(|i| zip.by_index(i).ok().map(|f| f.name().to_string()))
            .find(|n| n.eq_ignore_ascii_case(MODEL_PATH))
            .ok_or_else(|| IoError::Invalid3mf(format!("{MODEL_PATH} is missing")))?;
        zip.by_name(&name)
            .map_err(|e| IoError::Invalid3mf(e.to_string()))?
            .read_to_string(&mut xml)
            .map_err(|e| IoError::Invalid3mf(e.to_string()))?;
    }
    let doc = roxmltree::Document::parse(&xml).map_err(|e| IoError::Invalid3mf(format!("the model XML is invalid: {e}")))?;
    let root = doc.root_element();
    let scale = unit_to_mm(root.attribute("unit").unwrap_or("millimeter"))?;
    let resources =
        root.children().find(|n| n.has_tag_name("resources")).ok_or_else(|| IoError::Invalid3mf("no resources element".into()))?;
    let mut objects: Vec<Object> = Vec::new();
    for obj in resources.children().filter(|n| n.has_tag_name("object")) {
        let id = obj.attribute("id").ok_or_else(|| IoError::Invalid3mf("an object has no id".into()))?.to_string();
        let name = obj.attribute("name").map(str::to_string).unwrap_or_else(|| format!("object {id}"));
        let mut o = Object { id, name, mesh: None, components: Vec::new() };
        if let Some(mesh) = obj.children().find(|n| n.has_tag_name("mesh")) {
            let verts_el =
                mesh.children().find(|n| n.has_tag_name("vertices")).ok_or_else(|| IoError::Invalid3mf("a mesh has no vertices".into()))?;
            let mut verts = Vec::new();
            for v in verts_el.children().filter(|n| n.has_tag_name("vertex")) {
                let g = |a: &str| -> Result<f32, IoError> {
                    v.attribute(a)
                        .and_then(|s| s.parse::<f32>().ok())
                        .filter(|x| x.is_finite())
                        .ok_or_else(|| IoError::Invalid3mf(format!("a vertex has an invalid {a}")))
                };
                verts.push(Vec3::new(g("x")?, g("y")?, g("z")?) * scale);
            }
            let tris_el = mesh
                .children()
                .find(|n| n.has_tag_name("triangles"))
                .ok_or_else(|| IoError::Invalid3mf("a mesh has no triangles".into()))?;
            let mut tris = Vec::new();
            for t in tris_el.children().filter(|n| n.has_tag_name("triangle")) {
                let g = |a: &str| -> Result<u32, IoError> {
                    t.attribute(a)
                        .and_then(|s| s.parse::<u32>().ok())
                        .filter(|&i| (i as usize) < verts.len())
                        .ok_or_else(|| IoError::Invalid3mf(format!("a triangle has an invalid index {a}")))
                };
                tris.push([g("v1")?, g("v2")?, g("v3")?]);
            }
            o.mesh = Some(Mesh::new(verts, tris));
        }
        if let Some(comps) = obj.children().find(|n| n.has_tag_name("components")) {
            for c in comps.children().filter(|n| n.has_tag_name("component")) {
                let oid = c.attribute("objectid").ok_or_else(|| IoError::Invalid3mf("a component has no objectid".into()))?.to_string();
                let m = c.attribute("transform").map(parse_transform).transpose()?.unwrap_or(Mat4::IDENTITY);
                o.components.push((oid, m));
            }
        }
        objects.push(o);
    }
    let find = |id: &str| objects.iter().find(|o| o.id == id);
    // Flatten an object (with its components) into a mesh, with `m` applied.
    fn flatten(objs: &[Object], id: &str, m: Mat4, depth: usize, scale: f32) -> Result<Mesh, IoError> {
        if depth > 16 {
            return Err(IoError::Invalid3mf("components are nested too deeply".into()));
        }
        let o =
            objs.iter().find(|o| o.id == id).ok_or_else(|| IoError::Invalid3mf(format!("object {id} is referenced but not defined")))?;
        let mut parts: Vec<Mesh> = Vec::new();
        if let Some(mesh) = &o.mesh {
            parts.push(apply(mesh, m, scale));
        }
        for (cid, cm) in &o.components {
            parts.push(flatten(objs, cid, m * *cm, depth + 1, scale)?);
        }
        let mut out = Mesh::default();
        for p in parts {
            let base = out.vertices.len() as u32;
            out.vertices.extend(p.vertices);
            out.triangles.extend(p.triangles.iter().map(|t| [t[0] + base, t[1] + base, t[2] + base]));
        }
        Ok(out)
    }
    fn apply(mesh: &Mesh, m: Mat4, scale: f32) -> Mesh {
        if m == Mat4::IDENTITY {
            return mesh.clone();
        }
        // Translation in the transform is in file units, vertices are already scaled to mm.
        let mut mm = m;
        mm.w_axis = (m.w_axis.truncate() * scale).extend(1.0);
        let verts = mesh.vertices.iter().map(|v| mm.transform_point3(*v)).collect();
        let mut tris = mesh.triangles.clone();
        if mm.determinant() < 0.0 {
            for t in &mut tris {
                t.swap(1, 2);
            }
        }
        Mesh::new(verts, tris)
    }
    let build = root.children().find(|n| n.has_tag_name("build"));
    let mut out = Vec::new();
    match build {
        Some(b) if b.children().any(|n| n.has_tag_name("item")) => {
            for item in b.children().filter(|n| n.has_tag_name("item")) {
                let id = item.attribute("objectid").ok_or_else(|| IoError::Invalid3mf("a build item has no objectid".into()))?;
                let m = item.attribute("transform").map(parse_transform).transpose()?.unwrap_or(Mat4::IDENTITY);
                let mesh = flatten(&objects, id, m, 0, scale)?;
                if !mesh.is_empty() {
                    out.push((find(id).map(|o| o.name.clone()).unwrap_or_default(), mesh));
                }
            }
        }
        _ => {
            for o in &objects {
                let mesh = flatten(&objects, &o.id, Mat4::IDENTITY, 0, scale)?;
                if !mesh.is_empty() {
                    out.push((o.name.clone(), mesh));
                }
            }
        }
    }
    if out.is_empty() {
        return Err(IoError::Empty);
    }
    Ok(out)
}

/// Reads a 3MF file as one mesh (all build items merged).
pub fn read(data: &[u8]) -> Result<Mesh, IoError> {
    let parts = read_objects(data)?;
    let mut out = Mesh::default();
    for (_, p) in parts {
        let base = out.vertices.len() as u32;
        out.vertices.extend(p.vertices);
        out.triangles.extend(p.triangles.iter().map(|t| [t[0] + base, t[1] + base, t[2] + base]));
    }
    Ok(out)
}

fn xml_escape(s: &str) -> String {
    s.replace('&', "&amp;").replace('<', "&lt;").replace('>', "&gt;").replace('"', "&quot;")
}

/// Writes named meshes as a 3MF file (millimetres), one object and build item each.
pub fn write(objects: &[(&str, &Mesh)]) -> Result<Vec<u8>, IoError> {
    let mut model = String::from("<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n<model unit=\"millimeter\" xml:lang=\"en-US\" xmlns=\"http://schemas.microsoft.com/3dmanufacturing/core/2015/02\">\n<metadata name=\"Application\">OpenSlicer</metadata>\n<resources>\n");
    for (i, (name, mesh)) in objects.iter().enumerate() {
        model.push_str(&format!("<object id=\"{}\" name=\"{}\" type=\"model\"><mesh><vertices>\n", i + 1, xml_escape(name)));
        for v in &mesh.vertices {
            model.push_str(&format!("<vertex x=\"{}\" y=\"{}\" z=\"{}\"/>\n", v.x, v.y, v.z));
        }
        model.push_str("</vertices><triangles>\n");
        for t in &mesh.triangles {
            model.push_str(&format!("<triangle v1=\"{}\" v2=\"{}\" v3=\"{}\"/>\n", t[0], t[1], t[2]));
        }
        model.push_str("</triangles></mesh></object>\n");
    }
    model.push_str("</resources>\n<build>\n");
    for i in 0..objects.len() {
        model.push_str(&format!("<item objectid=\"{}\"/>\n", i + 1));
    }
    model.push_str("</build>\n</model>\n");

    let mut buf = Cursor::new(Vec::new());
    {
        let mut zip = zip::ZipWriter::new(&mut buf);
        let opts = zip::write::SimpleFileOptions::default().compression_method(zip::CompressionMethod::Deflated);
        let put = |zip: &mut zip::ZipWriter<&mut Cursor<Vec<u8>>>, name: &str, data: &[u8]| -> Result<(), IoError> {
            zip.start_file(name, opts).map_err(|e| IoError::Invalid3mf(e.to_string()))?;
            zip.write_all(data).map_err(IoError::Io)
        };
        put(&mut zip, "[Content_Types].xml", b"<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n<Types xmlns=\"http://schemas.openxmlformats.org/package/2006/content-types\"><Default Extension=\"rels\" ContentType=\"application/vnd.openxmlformats-package.relationships+xml\"/><Default Extension=\"model\" ContentType=\"application/vnd.ms-package.3dmanufacturing-3dmodel+xml\"/></Types>")?;
        put(&mut zip, "_rels/.rels", b"<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n<Relationships xmlns=\"http://schemas.openxmlformats.org/package/2006/relationships\"><Relationship Target=\"/3D/3dmodel.model\" Id=\"rel0\" Type=\"http://schemas.microsoft.com/3dmanufacturing/2013/01/3dmodel\"/></Relationships>")?;
        put(&mut zip, MODEL_PATH, model.as_bytes())?;
        zip.finish().map_err(|e| IoError::Invalid3mf(e.to_string()))?;
    }
    Ok(buf.into_inner())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn cube(s: f32) -> Mesh {
        Mesh::cuboid(Vec3::splat(s))
    }

    #[test]
    fn round_trip_keeps_objects_names_and_geometry() {
        let (a, b) = (cube(10.0), cube(20.0));
        let bytes = write(&[("small & <cube>", &a), ("big", &b)]).unwrap();
        let objs = read_objects(&bytes).unwrap();
        assert_eq!(objs.len(), 2);
        assert_eq!(objs[0].0, "small & <cube>");
        assert!((objs[0].1.stats().signed_volume - 1000.0).abs() < 1e-3);
        assert!((objs[1].1.stats().signed_volume - 8000.0).abs() < 1e-3);
        assert!(objs.iter().all(|(_, m)| m.stats().is_watertight()));
        assert_eq!(read(&bytes).unwrap().triangles.len(), 24);
    }

    fn model_xml(unit: &str, extra_build: &str) -> Vec<u8> {
        let xml = format!(
            "<?xml version=\"1.0\"?><model unit=\"{unit}\" xmlns=\"http://schemas.microsoft.com/3dmanufacturing/core/2015/02\"><resources>\
             <object id=\"1\" name=\"tri\"><mesh><vertices><vertex x=\"0\" y=\"0\" z=\"0\"/><vertex x=\"1\" y=\"0\" z=\"0\"/><vertex x=\"0\" y=\"1\" z=\"0\"/><vertex x=\"0\" y=\"0\" z=\"1\"/></vertices>\
             <triangles><triangle v1=\"0\" v2=\"2\" v3=\"1\"/><triangle v1=\"0\" v2=\"1\" v3=\"3\"/><triangle v1=\"1\" v2=\"2\" v3=\"3\"/><triangle v1=\"2\" v2=\"0\" v3=\"3\"/></triangles></mesh></object>\
             <object id=\"2\"><components><component objectid=\"1\" transform=\"1 0 0 0 1 0 0 0 1 10 0 0\"/></components></object></resources>\
             <build>{extra_build}</build></model>"
        );
        let mut buf = Cursor::new(Vec::new());
        {
            let mut zip = zip::ZipWriter::new(&mut buf);
            zip.start_file(MODEL_PATH, zip::write::SimpleFileOptions::default()).unwrap();
            zip.write_all(xml.as_bytes()).unwrap();
            zip.finish().unwrap();
        }
        buf.into_inner()
    }

    #[test]
    fn units_components_and_build_transforms_are_applied() {
        // A unit tetrahedron in inches, referenced through a component shifted by 10 units, plus a build transform.
        let bytes = model_xml("inch", "<item objectid=\"2\" transform=\"1 0 0 0 1 0 0 0 1 0 5 0\"/>");
        let objs = read_objects(&bytes).unwrap();
        assert_eq!(objs.len(), 1);
        let m = &objs[0].1;
        let b = m.bounds().unwrap();
        assert!((b.min.x - 254.0).abs() < 1e-3, "component shift of 10 inches: {}", b.min.x);
        assert!((b.min.y - 127.0).abs() < 1e-3, "build shift of 5 inches: {}", b.min.y);
        assert!((b.size().x - 25.4).abs() < 1e-3);
        assert!((m.stats().signed_volume - 25.4f64.powi(3) / 6.0).abs() < 1.0, "{}", m.stats().signed_volume);
    }

    #[test]
    fn mirrored_build_transforms_keep_outward_winding() {
        let bytes = model_xml("millimeter", "<item objectid=\"1\" transform=\"-1 0 0 0 1 0 0 0 1 0 0 0\"/>");
        let m = &read_objects(&bytes).unwrap()[0].1;
        assert!(m.stats().signed_volume > 0.0);
    }

    #[test]
    fn broken_archives_are_rejected_clearly() {
        assert!(matches!(read(b"PK nothing"), Err(IoError::Invalid3mf(_))));
        assert!(matches!(read_objects(&model_xml("lightyear", "")), Err(IoError::Invalid3mf(m)) if m.contains("unit")));
        // dangling component reference
        let bad = {
            let xml = "<model xmlns=\"x\"><resources><object id=\"1\"><components><component objectid=\"9\"/></components></object></resources><build><item objectid=\"1\"/></build></model>";
            let mut buf = Cursor::new(Vec::new());
            {
                let mut zip = zip::ZipWriter::new(&mut buf);
                zip.start_file(MODEL_PATH, zip::write::SimpleFileOptions::default()).unwrap();
                zip.write_all(xml.as_bytes()).unwrap();
                zip.finish().unwrap();
            }
            buf.into_inner()
        };
        assert!(matches!(read_objects(&bad), Err(IoError::Invalid3mf(m)) if m.contains("not defined")));
        // out-of-range triangle index
        let mut cursor = Cursor::new(Vec::new());
        {
            let mut zip = zip::ZipWriter::new(&mut cursor);
            zip.start_file(MODEL_PATH, zip::write::SimpleFileOptions::default()).unwrap();
            zip.write_all(b"<model xmlns=\"x\"><resources><object id=\"1\"><mesh><vertices><vertex x=\"0\" y=\"0\" z=\"0\"/></vertices><triangles><triangle v1=\"0\" v2=\"1\" v3=\"2\"/></triangles></mesh></object></resources><build><item objectid=\"1\"/></build></model>").unwrap();
            zip.finish().unwrap();
        }
        assert!(matches!(read(&cursor.into_inner()), Err(IoError::Invalid3mf(m)) if m.contains("invalid index")));
    }
}
