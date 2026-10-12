//! Saving and restoring the scene as an `.osproj` project, including the background autosave.

use crate::model::{Model, Transform};
use crate::settings::PrinterProfile;
use glam::Vec3;
use openslicer_core::Mesh;
use openslicer_project::project::{self, ModelEntry, ModelTransform, Project, ProjectFile};
use openslicer_project::settings::SplitSettings;
use std::path::{Path, PathBuf};
use std::sync::Arc;
use std::thread::JoinHandle;

/// Where the session autosave lives (next to eframe's own preferences file).
pub fn autosave_path() -> Option<PathBuf> {
    eframe::storage_dir("OpenSlicer").map(|d| d.join("autosave.osproj"))
}

/// Everything a project needs, captured cheaply so the file can be written on another thread.
pub struct Snapshot {
    pub name: String,
    pub printer: PrinterProfile,
    pub split: SplitSettings,
    pub models: Vec<(String, Arc<Mesh>, Transform)>,
}

impl Snapshot {
    pub fn capture<'a>(name: &str, printer: &PrinterProfile, split: SplitSettings, models: impl Iterator<Item = &'a Model>) -> Self {
        Self {
            name: name.to_owned(),
            printer: printer.clone(),
            split,
            models: models.map(|m| (m.name.clone(), m.mesh.clone(), m.transform)).collect(),
        }
    }

    /// A cheap fingerprint: changes whenever a model is added, removed, replaced or moved.
    pub fn fingerprint(&self) -> Vec<u64> {
        let mut out = vec![self.models.len() as u64];
        for (name, mesh, t) in &self.models {
            out.push(Arc::as_ptr(mesh) as usize as u64);
            out.push(name.len() as u64);
            for v in [t.position, t.rotation_deg, t.scale] {
                out.extend([v.x.to_bits() as u64, v.y.to_bits() as u64, v.z.to_bits() as u64]);
            }
        }
        out.extend(self.printer.build_volume.map(|v| v.to_bits() as u64));
        out.push(self.printer.margin.to_bits() as u64);
        out
    }

    pub fn write(&self, path: &Path) -> Result<(), project::ProjectError> {
        let mut file = ProjectFile { name: self.name.clone(), split: self.split.clone(), ..Default::default() };
        file.printer.name = self.printer.name.clone();
        file.printer.build_volume = self.printer.build_volume.map(f64::from);
        file.printer.margin = f64::from(self.printer.margin);
        file.printer.joint_clearance = self.split.connectors.as_ref().map_or(file.printer.joint_clearance, |c| c.clearance);
        file.models = self
            .models
            .iter()
            .map(|(name, _, t)| ModelEntry {
                name: name.clone(),
                file: String::new(),
                sha256: String::new(),
                triangles: 0,
                transform: Some(ModelTransform {
                    position: t.position.to_array(),
                    rotation_deg: t.rotation_deg.to_array(),
                    scale: t.scale.to_array(),
                }),
            })
            .collect();
        let mut proj = Project { file, meshes: self.models.iter().map(|(_, m, _)| (**m).clone()).collect() };
        if let Some(dir) = path.parent() {
            std::fs::create_dir_all(dir)?;
        }
        project::save(path, &mut proj)
    }
}

/// Printer and settings restored from a project, plus the models to put back in the scene.
pub struct Restored {
    pub printer: PrinterProfile,
    pub split: SplitSettings,
    pub models: Vec<(String, Arc<Mesh>, Option<Transform>)>,
}

pub fn read(path: &Path) -> Result<Restored, project::ProjectError> {
    let proj = project::load(path)?;
    let f = proj.file;
    Ok(Restored {
        printer: PrinterProfile {
            name: f.printer.name,
            build_volume: f.printer.build_volume.map(|v| v as f32),
            margin: f.printer.margin as f32,
        },
        split: f.split,
        models: f
            .models
            .into_iter()
            .zip(proj.meshes)
            .map(|(e, mesh)| {
                let t = e.transform.map(|t| Transform {
                    position: Vec3::from(t.position),
                    rotation_deg: Vec3::from(t.rotation_deg),
                    scale: Vec3::from(t.scale),
                });
                (e.name, Arc::new(mesh), t)
            })
            .collect(),
    })
}

/// Writes autosaves on a background thread so large scenes never stall the interface.
#[derive(Default)]
pub struct Autosaver {
    last: Vec<u64>,
    running: Option<JoinHandle<Result<(), String>>>,
    pub last_error: Option<String>,
}

impl Autosaver {
    /// Starts a write if the scene changed since the last one and none is in flight.
    /// An empty scene removes the autosave, so a deliberately cleared session is not "restored".
    pub fn tick(&mut self, snap: Snapshot, path: Option<PathBuf>) {
        self.reap();
        let Some(path) = path else { return };
        let fp = snap.fingerprint();
        if fp == self.last || self.running.is_some() {
            return;
        }
        self.last = fp;
        if snap.models.is_empty() {
            let _ = std::fs::remove_file(&path);
            return;
        }
        self.running = Some(std::thread::spawn(move || snap.write(&path).map_err(|e| e.to_string())));
    }

    fn reap(&mut self) {
        if self.running.as_ref().is_some_and(|h| h.is_finished()) {
            match self.running.take().unwrap().join() {
                Ok(Ok(())) => self.last_error = None,
                Ok(Err(e)) => {
                    self.last_error = Some(e);
                    self.last.clear(); // try again at the next tick
                }
                Err(_) => {
                    self.last_error = Some("the autosave thread crashed".into());
                    self.last.clear();
                }
            }
        }
    }

    /// Waits for a write in progress (used on exit so the file is complete).
    pub fn finish(&mut self) {
        if let Some(h) = self.running.take() {
            if let Ok(Err(e)) = h.join() {
                self.last_error = Some(e);
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use openslicer_core::MeshStats;

    fn model(id: u64) -> Model {
        let mesh = Arc::new(Mesh::cuboid(Vec3::new(40.0, 30.0, 20.0)));
        let stats: Arc<MeshStats> = Arc::new(mesh.stats());
        Model::new(id, format!("Cube {id}"), mesh, stats, None).unwrap()
    }

    fn tmp(tag: &str) -> PathBuf {
        std::env::temp_dir().join(format!("openslicer-session-{tag}-{}.osproj", std::process::id()))
    }

    #[test]
    fn scene_survives_a_save_and_restore_with_transforms_and_settings() {
        let mut a = model(1);
        a.transform.position = Vec3::new(100.0, -50.0, 10.0);
        a.transform.rotation_deg = Vec3::new(0.0, 0.0, 33.0);
        a.transform.scale = Vec3::new(2.0, 2.0, 2.0);
        let b = model(2);
        let mut printer = PrinterProfile { name: "Test printer".into(), build_volume: [300.0, 200.0, 100.0], margin: 4.0 };
        printer.margin = 4.0;
        let split = SplitSettings { nest: true, ..Default::default() };
        let snap = Snapshot::capture("Session", &printer, split.clone(), [&a, &b].into_iter());
        let path = tmp("rt");
        snap.write(&path).unwrap();
        let back = read(&path).unwrap();
        assert_eq!(back.printer, printer);
        assert_eq!(back.split, split);
        assert_eq!(back.models.len(), 2);
        assert_eq!(back.models[0].0, "Cube 1");
        assert_eq!(back.models[0].2, Some(a.transform));
        assert_eq!(back.models[0].1.triangles.len(), 12);
        let _ = std::fs::remove_file(&path);
    }

    #[test]
    fn fingerprint_changes_only_when_the_scene_changes() {
        let printer = PrinterProfile::default();
        let mut a = model(1);
        let f = |a: &Model| Snapshot::capture("s", &printer, SplitSettings::default(), std::iter::once(a)).fingerprint();
        let before = f(&a);
        assert_eq!(before, f(&a));
        a.transform.position.x += 1.0;
        assert_ne!(before, f(&a));
    }

    #[test]
    fn autosaver_writes_in_the_background_and_removes_the_file_for_an_empty_scene() {
        let printer = PrinterProfile::default();
        let a = model(1);
        let path = tmp("auto");
        let mut saver = Autosaver::default();
        let snap = || Snapshot::capture("s", &printer, SplitSettings::default(), std::iter::once(&a));
        saver.tick(snap(), Some(path.clone()));
        saver.finish();
        assert!(read(&path).is_ok(), "{:?}", saver.last_error);
        // Unchanged scene: nothing new is started.
        saver.tick(snap(), Some(path.clone()));
        assert!(saver.running.is_none());
        // Cleared scene: the autosave goes away.
        saver.tick(Snapshot::capture("s", &printer, SplitSettings::default(), std::iter::empty()), Some(path.clone()));
        assert!(!path.exists());
    }
}
