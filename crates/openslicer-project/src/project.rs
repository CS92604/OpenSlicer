//! Native project files (`.osproj`): a zip archive holding `project.json` and one binary STL per model.
//! Results are not stored; they are regenerated deterministically from the models and settings.

use crate::profile::PrinterProfile;
use crate::settings::SplitSettings;
use openslicer_core::Mesh;
use openslicer_io::{stl, write_atomic};
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::io::{Read, Write};
use std::path::Path;

pub const PROJECT_VERSION: u32 = 1;
pub const EXTENSION: &str = "osproj";

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct ModelEntry {
    pub name: String,
    /// Path of the STL inside the archive.
    pub file: String,
    pub sha256: String,
    pub triangles: usize,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct ProjectFile {
    pub version: u32,
    pub name: String,
    pub printer: PrinterProfile,
    pub split: SplitSettings,
    pub models: Vec<ModelEntry>,
    pub notes: String,
}

impl Default for ProjectFile {
    fn default() -> Self {
        Self {
            version: PROJECT_VERSION,
            name: "Untitled".into(),
            printer: PrinterProfile::default(),
            split: SplitSettings::default(),
            models: Vec::new(),
            notes: String::new(),
        }
    }
}

#[derive(Debug, thiserror::Error)]
pub enum ProjectError {
    #[error("could not read or write the project: {0}")]
    Io(#[from] std::io::Error),
    #[error("the project file is damaged: {0}")]
    Damaged(String),
    #[error("this project was saved by a newer version of OpenSlicer (format {0})")]
    TooNew(u32),
    #[error(transparent)]
    File(#[from] openslicer_io::IoError),
}

fn zip_err(e: zip::result::ZipError) -> ProjectError {
    ProjectError::Damaged(e.to_string())
}

/// A project together with its meshes, in the order of `project.models`.
pub struct Project {
    pub file: ProjectFile,
    pub meshes: Vec<Mesh>,
}

fn safe_name(i: usize, name: &str) -> String {
    let stem: String = name.chars().map(|c| if c.is_ascii_alphanumeric() || matches!(c, '-' | '_') { c } else { '_' }).collect();
    format!("models/{:02}_{}.stl", i + 1, stem)
}

/// Saves atomically: an existing project is replaced only once the new file has been written completely.
pub fn save(path: &Path, project: &mut Project) -> Result<(), ProjectError> {
    if project.file.models.len() != project.meshes.len() {
        // Rebuild the entries from the meshes so names and counts always match.
        let names: Vec<String> = project.file.models.iter().map(|m| m.name.clone()).collect();
        project.file.models = (0..project.meshes.len())
            .map(|i| ModelEntry {
                name: names.get(i).cloned().unwrap_or_else(|| format!("Model {}", i + 1)),
                file: String::new(),
                sha256: String::new(),
                triangles: 0,
            })
            .collect();
    }
    let mut buf = std::io::Cursor::new(Vec::new());
    {
        let mut zip = zip::ZipWriter::new(&mut buf);
        let opts = zip::write::SimpleFileOptions::default().compression_method(zip::CompressionMethod::Deflated);
        let mut entries = Vec::new();
        for (i, (entry, mesh)) in project.file.models.iter().zip(&project.meshes).enumerate() {
            let bytes = stl::write_binary(mesh);
            let file = safe_name(i, &entry.name);
            zip.start_file(&file, opts).map_err(zip_err)?;
            zip.write_all(&bytes)?;
            entries.push(ModelEntry {
                name: entry.name.clone(),
                file,
                sha256: format!("{:x}", Sha256::digest(&bytes)),
                triangles: mesh.triangles.len(),
            });
        }
        project.file.models = entries;
        project.file.version = PROJECT_VERSION;
        zip.start_file("project.json", opts).map_err(zip_err)?;
        zip.write_all(&serde_json::to_vec_pretty(&project.file).map_err(|e| ProjectError::Damaged(e.to_string()))?)?;
        zip.finish().map_err(zip_err)?;
    }
    let bytes = buf.into_inner();
    // Read it back before replacing anything.
    load_bytes(&bytes)?;
    write_atomic(path, &bytes)?;
    Ok(())
}

pub fn load(path: &Path) -> Result<Project, ProjectError> {
    load_bytes(&std::fs::read(path)?)
}

pub fn load_bytes(bytes: &[u8]) -> Result<Project, ProjectError> {
    let mut zip = zip::ZipArchive::new(std::io::Cursor::new(bytes)).map_err(zip_err)?;
    let mut json = Vec::new();
    zip.by_name("project.json").map_err(|_| ProjectError::Damaged("project.json is missing".into()))?.read_to_end(&mut json)?;
    let probe: serde_json::Value = serde_json::from_slice(&json).map_err(|e| ProjectError::Damaged(format!("project.json: {e}")))?;
    let version = probe.get("version").and_then(|v| v.as_u64()).unwrap_or(0) as u32;
    if version > PROJECT_VERSION {
        return Err(ProjectError::TooNew(version));
    }
    let file: ProjectFile = serde_json::from_value(probe).map_err(|e| ProjectError::Damaged(format!("project.json: {e}")))?;
    let mut meshes = Vec::new();
    for m in &file.models {
        let mut data = Vec::new();
        zip.by_name(&m.file).map_err(|_| ProjectError::Damaged(format!("{} is missing", m.file)))?.read_to_end(&mut data)?;
        if format!("{:x}", Sha256::digest(&data)) != m.sha256 {
            return Err(ProjectError::Damaged(format!("{} does not match its recorded checksum", m.file)));
        }
        let mesh = stl::read(&data)?;
        if mesh.triangles.len() != m.triangles {
            return Err(ProjectError::Damaged(format!("{} holds {} triangles, expected {}", m.file, mesh.triangles.len(), m.triangles)));
        }
        meshes.push(mesh);
    }
    Ok(Project { file, meshes })
}

/// Autosave helper: the file name used next to a project for recovery data.
pub fn autosave_path(path: &Path) -> std::path::PathBuf {
    let mut p = path.as_os_str().to_owned();
    p.push(".autosave");
    p.into()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::settings::Layout;
    use glam::Vec3;
    use openslicer_joints::settings::{ConnectorKind, ConnectorSettings};

    fn tmp(tag: &str) -> std::path::PathBuf {
        std::env::temp_dir().join(format!("openslicer-proj-{tag}-{}.{EXTENSION}", std::process::id()))
    }

    fn sample() -> Project {
        let mut a = Mesh::cuboid(Vec3::new(300.0, 80.0, 40.0));
        a.translate(Vec3::new(150.0, 40.0, 20.0));
        let b = Mesh::cuboid(Vec3::new(10.0, 10.0, 10.0));
        let mut file = ProjectFile { name: "Hardtop".into(), notes: "first try".into(), ..Default::default() };
        file.printer.margin = 5.0;
        file.split.layout = Layout::Grid { divisions: [2, 1, 1], yaw_deg: 10.0 };
        file.split.connectors = Some(ConnectorSettings::new(ConnectorKind::Dovetail));
        file.models = vec![
            ModelEntry { name: "Shell".into(), file: String::new(), sha256: String::new(), triangles: 0 },
            ModelEntry { name: "weird/name:1".into(), file: String::new(), sha256: String::new(), triangles: 0 },
        ];
        Project { file, meshes: vec![a, b] }
    }

    #[test]
    fn save_and_load_round_trip_everything() {
        let path = tmp("rt");
        let mut p = sample();
        save(&path, &mut p).unwrap();
        let back = load(&path).unwrap();
        assert_eq!(back.file, p.file);
        assert_eq!(back.meshes.len(), 2);
        for (a, b) in p.meshes.iter().zip(&back.meshes) {
            assert_eq!(a.triangles.len(), b.triangles.len());
            assert!((a.stats().signed_volume - b.stats().signed_volume).abs() < 1e-3);
        }
        assert_eq!(back.file.models[1].file, "models/02_weird_name_1.stl");
        assert_eq!(back.file.printer.margin, 5.0);
        let _ = std::fs::remove_file(&path);
    }

    #[test]
    fn corruption_and_missing_parts_are_reported() {
        let path = tmp("bad");
        let mut p = sample();
        save(&path, &mut p).unwrap();
        let bytes = std::fs::read(&path).unwrap();
        // Truncated archive.
        assert!(matches!(load_bytes(&bytes[..bytes.len() / 2]), Err(ProjectError::Damaged(_))));
        // Not a zip at all.
        assert!(matches!(load_bytes(b"hello"), Err(ProjectError::Damaged(_))));
        // A model altered inside an otherwise valid archive fails its checksum.
        let mut zin = zip::ZipArchive::new(std::io::Cursor::new(&bytes)).unwrap();
        let mut out = std::io::Cursor::new(Vec::new());
        {
            let mut zw = zip::ZipWriter::new(&mut out);
            let opts = zip::write::SimpleFileOptions::default();
            for i in 0..zin.len() {
                let mut f = zin.by_index(i).unwrap();
                let mut data = Vec::new();
                f.read_to_end(&mut data).unwrap();
                if f.name().starts_with("models/01") {
                    let n = data.len();
                    data[n - 3] ^= 1;
                }
                zw.start_file(f.name(), opts).unwrap();
                zw.write_all(&data).unwrap();
            }
            zw.finish().unwrap();
        }
        let e = load_bytes(&out.into_inner()).err().unwrap();
        assert!(e.to_string().contains("checksum"), "{e}");
        let _ = std::fs::remove_file(&path);
    }

    #[test]
    fn newer_versions_are_refused_and_older_defaults_fill_in() {
        let mut buf = std::io::Cursor::new(Vec::new());
        {
            let mut zw = zip::ZipWriter::new(&mut buf);
            zw.start_file("project.json", zip::write::SimpleFileOptions::default()).unwrap();
            zw.write_all(br#"{"version": 99}"#).unwrap();
            zw.finish().unwrap();
        }
        assert!(matches!(load_bytes(&buf.into_inner()), Err(ProjectError::TooNew(99))));
        let mut buf = std::io::Cursor::new(Vec::new());
        {
            let mut zw = zip::ZipWriter::new(&mut buf);
            zw.start_file("project.json", zip::write::SimpleFileOptions::default()).unwrap();
            zw.write_all(br#"{"version": 1, "name": "Minimal"}"#).unwrap();
            zw.finish().unwrap();
        }
        let p = load_bytes(&buf.into_inner()).unwrap();
        assert_eq!(p.file.name, "Minimal");
        assert_eq!(p.file.printer, PrinterProfile::default());
        assert!(p.meshes.is_empty());
    }

    #[test]
    fn a_failed_save_keeps_the_existing_project() {
        let path = tmp("keep");
        let mut p = sample();
        save(&path, &mut p).unwrap();
        let before = std::fs::read(&path).unwrap();
        // Saving into a missing directory fails and must not touch the original.
        let bad = path.parent().unwrap().join("no-such-dir").join("x.osproj");
        assert!(save(&bad, &mut p).is_err());
        assert_eq!(std::fs::read(&path).unwrap(), before);
        assert!(!bad.exists());
        let _ = std::fs::remove_file(&path);
    }

    #[test]
    fn project_drives_the_pipeline_after_reload() {
        let path = tmp("pipe");
        let mut p = sample();
        save(&path, &mut p).unwrap();
        let back = load(&path).unwrap();
        p.file.split.connectors = None;
        let out =
            crate::pipeline::run_split(&back.meshes[0], &back.file.printer, &back.file.split, &openslicer_core::control::Control::none())
                .unwrap();
        assert!(out.pieces.len() >= 2 && out.is_valid(), "{:?}", out.problems);
        let _ = std::fs::remove_file(&path);
        assert!(autosave_path(&path).to_string_lossy().ends_with(".osproj.autosave"));
    }
}
