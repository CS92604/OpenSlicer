#![cfg_attr(all(not(debug_assertions), target_os = "windows"), windows_subsystem = "windows")]

mod app;
mod camera;
mod cutplanes;
mod gl_view;
mod icons;
mod jobs;
mod model;
mod session;
mod settings;
mod split_ui;
mod theme;
mod widgets;

use eframe::egui;
use std::path::PathBuf;

fn app_icon() -> egui::IconData {
    let png = include_bytes!("../../../assets/icon-64.png");
    match image::load_from_memory_with_format(png, image::ImageFormat::Png) {
        Ok(img) => {
            let rgba = img.to_rgba8();
            egui::IconData { width: rgba.width(), height: rgba.height(), rgba: rgba.into_raw() }
        }
        Err(_) => egui::IconData::default(),
    }
}

fn native_options(multisampling: u16) -> eframe::NativeOptions {
    eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default()
            .with_title("OpenSlicer")
            .with_inner_size([1440.0, 900.0])
            .with_min_inner_size([960.0, 600.0])
            .with_icon(app_icon()),
        depth_buffer: 24,
        multisampling,
        ..Default::default()
    }
}

fn run(files: Vec<PathBuf>, multisampling: u16) -> eframe::Result {
    eframe::run_native("OpenSlicer", native_options(multisampling), Box::new(|cc| Ok(Box::new(app::OpenSlicerApp::new(cc, files)))))
}

fn main() -> eframe::Result {
    let files: Vec<PathBuf> = std::env::args_os().skip(1).map(PathBuf::from).collect();
    // Prefer 4× MSAA; some drivers/remote desktops cannot provide it, so fall back to none.
    match run(files.clone(), 4) {
        Err(eframe::Error::NoGlutinConfigs(..)) | Err(eframe::Error::OpenGL(..)) => {
            eprintln!("4× multisampling unavailable; retrying without anti-aliasing");
            run(files, 0)
        }
        other => other,
    }
}
