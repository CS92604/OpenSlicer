#![cfg_attr(all(not(debug_assertions), target_os = "windows"), windows_subsystem = "windows")]

mod app;
mod camera;
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
    // Procedural 64×64 icon: accent hexagon on a dark rounded square.
    const N: usize = 64;
    let mut rgba = vec![0u8; N * N * 4];
    for y in 0..N {
        for x in 0..N {
            let (fx, fy) = (x as f32 - 31.5, y as f32 - 31.5);
            let corner = (fx.abs() - 22.0).max(0.0).hypot((fy.abs() - 22.0).max(0.0)) - 9.0;
            let mut px = [0u8; 4];
            if corner <= 0.0 {
                px = [0x20, 0x20, 0x20, 0xFF];
                // Hexagon (pointy-top) with a lighter top face.
                let (ax, ay) = (fx.abs(), fy);
                if ay.abs() <= 22.0 && ax <= 19.0 && ax * 0.5 + ay.abs() <= 22.0 {
                    px = if ay < 0.0 && ay + ax * 0.577 < 0.0 { [0x4C, 0xC2, 0xFF, 0xFF] } else { [0x00, 0x67, 0xC0, 0xFF] };
                }
            }
            rgba[(y * N + x) * 4..][..4].copy_from_slice(&px);
        }
    }
    egui::IconData { rgba, width: N as u32, height: N as u32 }
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
