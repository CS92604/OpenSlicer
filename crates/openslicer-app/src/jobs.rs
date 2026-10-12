//! Background jobs. Heavy work never runs on the UI thread.
//!
//! Cancellation is cooperative: the flag is checked between phases (read,
//! parse, analyse, write, verify). A phase that is already running finishes,
//! but its result is discarded and no files are left behind.

use eframe::egui;
use openslicer_core::{Mesh, MeshStats};
use openslicer_io::stl;
use std::io::Write;
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc::{channel, Receiver, TryRecvError};
use std::sync::Arc;

pub enum JobEvent {
    Progress(String),
    Loaded { path: PathBuf, mesh: Arc<Mesh>, stats: Arc<MeshStats> },
    Exported { path: PathBuf, summary: String },
    Cancelled,
    Failed { title: String, detail: String },
}

pub struct Job {
    pub label: String,
    pub progress: String,
    cancel: Arc<AtomicBool>,
    rx: Receiver<JobEvent>,
}

impl Job {
    pub fn cancel(&self) {
        self.cancel.store(true, Ordering::Relaxed);
    }

    /// Drains pending events. Returns the terminal event once, if the job finished.
    pub fn poll(&mut self) -> Option<JobEvent> {
        loop {
            match self.rx.try_recv() {
                Ok(JobEvent::Progress(p)) => self.progress = p,
                Ok(done) => return Some(done),
                Err(TryRecvError::Empty) => return None,
                Err(TryRecvError::Disconnected) => {
                    return Some(JobEvent::Failed {
                        title: "Background task stopped unexpectedly".into(),
                        detail: "The worker thread ended without reporting a result.".into(),
                    })
                }
            }
        }
    }
}

fn spawn(ctx: &egui::Context, label: String, work: impl FnOnce(&dyn Fn(String), &AtomicBool) -> JobEvent + Send + 'static) -> Job {
    let (tx, rx) = channel();
    let cancel = Arc::new(AtomicBool::new(false));
    let (flag, ctx2) = (cancel.clone(), ctx.clone());
    std::thread::Builder::new()
        .name("openslicer-job".into())
        .spawn(move || {
            let progress_tx = tx.clone();
            let progress_ctx = ctx2.clone();
            let progress = move |msg: String| {
                let _ = progress_tx.send(JobEvent::Progress(msg));
                progress_ctx.request_repaint();
            };
            // A panic in geometry code must surface as an error, not a silent hang.
            let result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| work(&progress, &flag)));
            let event = result.unwrap_or_else(|_| JobEvent::Failed {
                title: "Internal error".into(),
                detail: "The operation crashed unexpectedly. The original file and your scene were not modified.".into(),
            });
            let _ = tx.send(event);
            ctx2.request_repaint();
        })
        .expect("failed to spawn worker thread");
    Job { label, progress: String::new(), cancel, rx }
}

pub fn spawn_load(ctx: &egui::Context, path: PathBuf) -> Job {
    let name = path.file_name().map(|n| n.to_string_lossy().into_owned()).unwrap_or_default();
    spawn(ctx, format!("Opening {name}"), move |progress, cancel| {
        progress("Reading mesh…".into());
        let mesh = match openslicer_io::load_mesh(&path) {
            Ok(m) => m,
            Err(e) => return JobEvent::Failed { title: format!("Could not open {name}"), detail: e.to_string() },
        };
        if cancel.load(Ordering::Relaxed) {
            return JobEvent::Cancelled;
        }
        progress(format!("Analysing {} triangles…", mesh.triangles.len()));
        let stats = mesh.stats();
        if cancel.load(Ordering::Relaxed) {
            return JobEvent::Cancelled;
        }
        JobEvent::Loaded { path, mesh: Arc::new(mesh), stats: Arc::new(stats) }
    })
}

/// Writes `mesh` as binary STL via a temporary file and atomic rename, then
/// re-reads the written file and checks it matches what was meant to be written.
pub fn spawn_export(ctx: &egui::Context, mesh: Mesh, path: PathBuf) -> Job {
    let name = path.file_name().map(|n| n.to_string_lossy().into_owned()).unwrap_or_default();
    spawn(ctx, format!("Exporting {name}"), move |progress, cancel| {
        progress("Encoding STL…".into());
        let bytes = stl::write_binary(&mesh);
        if cancel.load(Ordering::Relaxed) {
            return JobEvent::Cancelled;
        }
        progress("Writing file…".into());
        let tmp = tmp_path(&path);
        if let Err(e) = write_synced(&tmp, &bytes) {
            let _ = std::fs::remove_file(&tmp);
            return JobEvent::Failed { title: format!("Could not write {name}"), detail: e.to_string() };
        }
        if cancel.load(Ordering::Relaxed) {
            let _ = std::fs::remove_file(&tmp);
            return JobEvent::Cancelled;
        }
        progress("Verifying exported file…".into());
        let summary = match verify_export(&mesh, &tmp) {
            Ok(s) => s,
            Err(detail) => {
                let _ = std::fs::remove_file(&tmp);
                return JobEvent::Failed { title: format!("Export of {name} failed verification"), detail };
            }
        };
        if let Err(e) = std::fs::rename(&tmp, &path) {
            let _ = std::fs::remove_file(&tmp);
            return JobEvent::Failed { title: format!("Could not save {name}"), detail: e.to_string() };
        }
        JobEvent::Exported { path, summary }
    })
}

fn tmp_path(path: &Path) -> PathBuf {
    let mut s = path.as_os_str().to_owned();
    s.push(".openslicer-tmp");
    PathBuf::from(s)
}

fn write_synced(path: &Path, bytes: &[u8]) -> std::io::Result<()> {
    let mut f = std::fs::File::create(path)?;
    f.write_all(bytes)?;
    f.sync_all()
}

/// Re-imports the written file and compares it with the source mesh.
pub fn verify_export(source: &Mesh, written: &Path) -> Result<String, String> {
    let back = openslicer_io::load_mesh_as(written, "stl").map_err(|e| format!("the written file cannot be re-imported: {e}"))?;
    let (a, b) = (source.stats(), back.stats());
    if a.triangle_count != b.triangle_count {
        return Err(format!("triangle count changed: wrote {}, re-imported {}", a.triangle_count, b.triangle_count));
    }
    let tol = 1e-6 * a.signed_volume.abs().max(1.0);
    if (a.signed_volume - b.signed_volume).abs() > tol {
        return Err(format!("volume changed: {:.3} mm³ → {:.3} mm³", a.signed_volume, b.signed_volume));
    }
    if a.bounds != b.bounds {
        return Err("bounding box changed after round trip".into());
    }
    Ok(format!("{} triangles, verified by re-import", b.triangle_count))
}

#[cfg(test)]
mod tests {
    use super::*;
    use glam::Vec3;

    #[test]
    fn verify_accepts_faithful_export_and_rejects_corruption() {
        let dir = std::env::temp_dir().join(format!("openslicer-test-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let mesh = Mesh::cuboid(Vec3::new(10.0, 20.0, 30.0));
        let good = dir.join("good.stl");
        std::fs::write(&good, stl::write_binary(&mesh)).unwrap();
        assert!(verify_export(&mesh, &good).is_ok());

        let mut bytes = stl::write_binary(&mesh);
        bytes.truncate(bytes.len() - 50);
        bytes[80..84].copy_from_slice(&11u32.to_le_bytes());
        let bad = dir.join("bad.stl");
        std::fs::write(&bad, bytes).unwrap();
        assert!(verify_export(&mesh, &bad).is_err());
        std::fs::remove_dir_all(&dir).ok();
    }

    fn wait(job: &mut Job) -> JobEvent {
        for _ in 0..2000 {
            if let Some(ev) = job.poll() {
                return ev;
            }
            std::thread::sleep(std::time::Duration::from_millis(5));
        }
        panic!("job did not finish");
    }

    fn temp_dir(tag: &str) -> PathBuf {
        let dir = std::env::temp_dir().join(format!("openslicer-{tag}-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        dir
    }

    #[test]
    fn export_job_writes_a_verified_file_and_leaves_no_temp_file() {
        let ctx = egui::Context::default();
        let dir = temp_dir("export");
        let path = dir.join("part.stl");
        let mesh = Mesh::cuboid(Vec3::new(10.0, 20.0, 30.0));
        let mut job = spawn_export(&ctx, mesh.clone(), path.clone());
        assert!(matches!(wait(&mut job), JobEvent::Exported { .. }));
        assert!(!tmp_path(&path).exists());
        let back = openslicer_io::load_mesh(&path).unwrap();
        assert!((back.stats().signed_volume - 6000.0).abs() < 1e-3);
        assert!(back.stats().is_watertight());
        std::fs::remove_dir_all(&dir).ok();
    }

    #[test]
    fn export_to_missing_directory_fails_cleanly() {
        let ctx = egui::Context::default();
        let dir = temp_dir("missing");
        let path = dir.join("no-such-dir").join("part.stl");
        let mut job = spawn_export(&ctx, Mesh::cuboid(Vec3::ONE), path.clone());
        assert!(matches!(wait(&mut job), JobEvent::Failed { .. }));
        assert!(!path.exists());
        std::fs::remove_dir_all(&dir).ok();
    }

    #[test]
    fn export_does_not_replace_existing_file_when_it_fails() {
        // A directory at the destination makes the final rename fail; the temp file must be cleaned up.
        let ctx = egui::Context::default();
        let dir = temp_dir("rename");
        let path = dir.join("part.stl");
        std::fs::create_dir(&path).unwrap();
        let mut job = spawn_export(&ctx, Mesh::cuboid(Vec3::ONE), path.clone());
        assert!(matches!(wait(&mut job), JobEvent::Failed { .. }));
        assert!(path.is_dir());
        assert!(!tmp_path(&path).exists());
        std::fs::remove_dir_all(&dir).ok();
    }

    #[test]
    fn loading_a_corrupt_file_reports_a_clear_error() {
        let ctx = egui::Context::default();
        let dir = temp_dir("corrupt");
        let path = dir.join("bad.stl");
        std::fs::write(&path, b"this is not an stl file at all").unwrap();
        let mut job = spawn_load(&ctx, path);
        match wait(&mut job) {
            JobEvent::Failed { title, detail } => {
                assert!(title.contains("bad.stl"));
                assert!(detail.contains("STL"));
            }
            _ => panic!("expected failure"),
        }
        std::fs::remove_dir_all(&dir).ok();
    }

    #[test]
    fn loading_a_real_file_returns_mesh_and_stats() {
        let ctx = egui::Context::default();
        let dir = temp_dir("load");
        let path = dir.join("cube.stl");
        std::fs::write(&path, stl::write_binary(&Mesh::cuboid(Vec3::splat(10.0)))).unwrap();
        let mut job = spawn_load(&ctx, path);
        match wait(&mut job) {
            JobEvent::Loaded { mesh, stats, .. } => {
                assert_eq!(mesh.triangles.len(), 12);
                assert!(stats.is_watertight());
            }
            _ => panic!("expected load"),
        }
        std::fs::remove_dir_all(&dir).ok();
    }
}
