//! Image-derived geometry: lithophanes, relief from a photo, and halftone dot plates.
//!
//! Images are decoded locally (PNG and JPEG), converted to brightness and resampled to the requested
//! physical size. The geometry is a heightfield slab, so it is always a closed, printable solid.

use crate::{ensure_fit, FabError, FabReport, FabResult, Part};
use glam::DVec3;
use openslicer_core::control::Control;
use openslicer_geom::build::heightfield;
use openslicer_split::exec::Piece;

/// Brightness image in 0..=1, row 0 at the top.
pub struct Gray {
    pub width: usize,
    pub height: usize,
    pub data: Vec<f32>,
}

/// Decodes PNG/JPEG bytes to brightness. Large images are not a problem; see [`Gray::resample`].
pub fn decode(bytes: &[u8]) -> Result<Gray, FabError> {
    let img = image::load_from_memory(bytes).map_err(|e| FabError::Invalid(format!("the image could not be read: {e}")))?;
    let l = img.to_luma8();
    let (w, h) = (l.width() as usize, l.height() as usize);
    if w < 2 || h < 2 {
        return Err(FabError::Invalid("the image is too small".into()));
    }
    Ok(Gray { width: w, height: h, data: l.pixels().map(|p| p.0[0] as f32 / 255.0).collect() })
}

impl Gray {
    /// Area-averaged resample to `nx × ny` samples (box filter), so down-scaling does not alias.
    pub fn resample(&self, nx: usize, ny: usize) -> Gray {
        let mut data = vec![0.0f32; nx * ny];
        for j in 0..ny {
            let (y0, y1) = (j * self.height / ny, (((j + 1) * self.height).div_ceil(ny)).min(self.height).max(j * self.height / ny + 1));
            for i in 0..nx {
                let (x0, x1) = (i * self.width / nx, (((i + 1) * self.width).div_ceil(nx)).min(self.width).max(i * self.width / nx + 1));
                let mut sum = 0.0;
                for y in y0..y1 {
                    for x in x0..x1 {
                        sum += self.data[y * self.width + x];
                    }
                }
                data[j * nx + i] = sum / ((y1 - y0) * (x1 - x0)) as f32;
            }
        }
        Gray { width: nx, height: ny, data }
    }

    /// Box blur with the given radius in samples (0 = none).
    pub fn blur(&self, radius: usize) -> Gray {
        if radius == 0 {
            return Gray { width: self.width, height: self.height, data: self.data.clone() };
        }
        let pass = |src: &[f32], horizontal: bool| -> Vec<f32> {
            let (w, h) = (self.width, self.height);
            let mut out = vec![0.0; w * h];
            for y in 0..h {
                for x in 0..w {
                    let (mut sum, mut n) = (0.0, 0);
                    for d in -(radius as isize)..=radius as isize {
                        let (xx, yy) = if horizontal { (x as isize + d, y as isize) } else { (x as isize, y as isize + d) };
                        if xx >= 0 && yy >= 0 && (xx as usize) < w && (yy as usize) < h {
                            sum += src[yy as usize * w + xx as usize];
                            n += 1;
                        }
                    }
                    out[y * w + x] = sum / n as f32;
                }
            }
            out
        };
        Gray { width: self.width, height: self.height, data: pass(&pass(&self.data, true), false) }
    }
}

fn grid_for(img: &Gray, width_mm: f64, pixel_mm: f64, max_samples: usize) -> Result<(usize, usize, f64), FabError> {
    if !(width_mm > 0.0 && pixel_mm > 0.0) {
        return Err(FabError::Invalid("width and sample spacing must be positive".into()));
    }
    let nx = ((width_mm / pixel_mm).round() as usize).max(2);
    let height_mm = width_mm * img.height as f64 / img.width as f64;
    let ny = ((height_mm / pixel_mm).round() as usize).max(2);
    if nx * ny > max_samples {
        return Err(FabError::Invalid(format!("{nx} × {ny} samples is too many; increase the sample spacing")));
    }
    Ok((nx, ny, height_mm))
}

/// Builds a slab from per-sample thickness, flipping rows so the image's top is at the top (+Y).
fn slab(thickness: &[f32], nx: usize, ny: usize, dx: f64, dy: f64) -> Result<openslicer_core::Mesh, FabError> {
    let mut flipped = vec![0.0f32; nx * ny];
    for j in 0..ny {
        flipped[(ny - 1 - j) * nx..(ny - j) * nx].copy_from_slice(&thickness[j * nx..(j + 1) * nx]);
    }
    heightfield(&flipped, nx, ny, dx, dy, 0.0).map_err(FabError::Invalid)
}

fn finish(
    mesh: openslicer_core::Mesh,
    label: &str,
    method: &str,
    usable: DVec3,
    notes: Vec<String>,
    ctl: &Control,
) -> Result<FabResult, FabError> {
    let mut report = FabReport { method: method.into(), notes, ..Default::default() };
    let parts: Vec<Part> = Piece::new(mesh).into_iter().map(|piece| Part { piece, label: label.into(), outline: None }).collect();
    let parts = ensure_fit(parts, usable, &mut report, ctl)?;
    Ok(FabResult::finish(parts, report))
}

#[derive(Clone, Debug)]
pub struct LithophaneSettings {
    pub width_mm: f64,
    pub min_thickness: f64,
    pub max_thickness: f64,
    /// Sample spacing in mm (0.2 to 0.4 mm suits most printers).
    pub pixel_mm: f64,
    /// Bright areas thin by default so more light passes; set to invert.
    pub invert: bool,
    pub blur_radius: usize,
    pub usable: DVec3,
    pub max_samples: usize,
}

impl LithophaneSettings {
    pub fn new(usable: DVec3) -> Self {
        Self {
            width_mm: 100.0,
            min_thickness: 0.8,
            max_thickness: 3.2,
            pixel_mm: 0.3,
            invert: false,
            blur_radius: 0,
            usable,
            max_samples: 4_000_000,
        }
    }
}

pub fn lithophane(bytes: &[u8], s: &LithophaneSettings, ctl: &Control) -> Result<FabResult, FabError> {
    if !(s.min_thickness > 0.0 && s.max_thickness > s.min_thickness) {
        return Err(FabError::Invalid("the maximum thickness must exceed the minimum thickness, which must be positive".into()));
    }
    let img = decode(bytes)?;
    let (nx, ny, height_mm) = grid_for(&img, s.width_mm, s.pixel_mm, s.max_samples)?;
    let g = img.resample(nx, ny).blur(s.blur_radius);
    let t: Vec<f32> = g
        .data
        .iter()
        .map(|&l| {
            let dark = if s.invert { l } else { 1.0 - l };
            (s.min_thickness + dark as f64 * (s.max_thickness - s.min_thickness)) as f32
        })
        .collect();
    let (dx, dy) = (s.width_mm / (nx - 1) as f64, height_mm / (ny - 1) as f64);
    let m = slab(&t, nx, ny, dx, dy)?;
    finish(m, "Lithophane", "Lithophane", s.usable, vec![format!("{nx} × {ny} samples, {:.1} × {:.1} mm", s.width_mm, height_mm)], ctl)
}

#[derive(Clone, Debug)]
pub struct ImageReliefSettings {
    pub width_mm: f64,
    pub base_thickness: f64,
    pub relief_height: f64,
    pub pixel_mm: f64,
    /// Bright areas rise by default; set to make dark areas rise.
    pub invert: bool,
    pub blur_radius: usize,
    pub usable: DVec3,
    pub max_samples: usize,
}

impl ImageReliefSettings {
    pub fn new(usable: DVec3) -> Self {
        Self {
            width_mm: 100.0,
            base_thickness: 1.5,
            relief_height: 5.0,
            pixel_mm: 0.4,
            invert: false,
            blur_radius: 1,
            usable,
            max_samples: 4_000_000,
        }
    }
}

pub fn relief_from_image(bytes: &[u8], s: &ImageReliefSettings, ctl: &Control) -> Result<FabResult, FabError> {
    if !(s.base_thickness > 0.0 && s.relief_height > 0.0) {
        return Err(FabError::Invalid("the base thickness and relief height must be positive".into()));
    }
    let img = decode(bytes)?;
    let (nx, ny, height_mm) = grid_for(&img, s.width_mm, s.pixel_mm, s.max_samples)?;
    let g = img.resample(nx, ny).blur(s.blur_radius);
    let t: Vec<f32> =
        g.data.iter().map(|&l| (s.base_thickness + (if s.invert { 1.0 - l } else { l }) as f64 * s.relief_height) as f32).collect();
    let m = slab(&t, nx, ny, s.width_mm / (nx - 1) as f64, height_mm / (ny - 1) as f64)?;
    finish(m, "Relief", "Relief from image", s.usable, vec![format!("{nx} × {ny} samples")], ctl)
}

#[derive(Clone, Debug)]
pub struct HalftoneSettings {
    pub width_mm: f64,
    /// Distance between dot centres (mm).
    pub pitch_mm: f64,
    /// Largest dot diameter as a fraction of the pitch (1.0 lets the darkest dots touch).
    pub max_dot_fraction: f64,
    pub base_thickness: f64,
    pub dot_height: f64,
    /// Samples per dot pitch used to draw each dot (higher = rounder dots, bigger mesh).
    pub samples_per_dot: usize,
    pub invert: bool,
    pub usable: DVec3,
    pub max_samples: usize,
}

impl HalftoneSettings {
    pub fn new(usable: DVec3) -> Self {
        Self {
            width_mm: 100.0,
            pitch_mm: 2.0,
            max_dot_fraction: 0.95,
            base_thickness: 1.0,
            dot_height: 1.0,
            samples_per_dot: 10,
            invert: false,
            usable,
            max_samples: 4_000_000,
        }
    }
}

/// Dot-pattern plate: each cell gets a raised dot whose area is proportional to the image's darkness.
pub fn halftone(bytes: &[u8], s: &HalftoneSettings, ctl: &Control) -> Result<FabResult, FabError> {
    if !(s.pitch_mm > 0.0
        && s.base_thickness > 0.0
        && s.dot_height > 0.0
        && (0.1..=1.0).contains(&s.max_dot_fraction)
        && s.samples_per_dot >= 4)
    {
        return Err(FabError::Invalid(
            "pitch, thicknesses must be positive, the dot fraction between 0.1 and 1, and at least 4 samples per dot".into(),
        ));
    }
    let img = decode(bytes)?;
    let cells_x = ((s.width_mm / s.pitch_mm).round() as usize).max(1);
    let height_mm = s.width_mm * img.height as f64 / img.width as f64;
    let cells_y = ((height_mm / s.pitch_mm).round() as usize).max(1);
    let (nx, ny) = (cells_x * s.samples_per_dot + 1, cells_y * s.samples_per_dot + 1);
    if nx * ny > s.max_samples {
        return Err(FabError::Invalid(format!("{nx} × {ny} samples is too many; use a larger pitch or fewer samples per dot")));
    }
    let tone = img.resample(cells_x, cells_y);
    let step = s.pitch_mm / s.samples_per_dot as f64;
    let mut t = vec![s.base_thickness as f32; nx * ny];
    for cj in 0..cells_y {
        for ci in 0..cells_x {
            let l = tone.data[cj * cells_x + ci] as f64;
            let darkness = if s.invert { l } else { 1.0 - l };
            let radius = 0.5 * s.pitch_mm * s.max_dot_fraction * darkness.sqrt();
            if radius < step * 0.5 {
                continue;
            }
            // Row 0 of the image is the top, so cell row cj maps to the cell cells_y - 1 - cj from the bottom.
            let (cx, cy) = ((ci as f64 + 0.5) * s.pitch_mm, ((cells_y - 1 - cj) as f64 + 0.5) * s.pitch_mm);
            let (i0, i1) = (((cx - radius) / step).floor().max(0.0) as usize, (((cx + radius) / step).ceil() as usize).min(nx - 1));
            let (j0, j1) = (((cy - radius) / step).floor().max(0.0) as usize, (((cy + radius) / step).ceil() as usize).min(ny - 1));
            for j in j0..=j1 {
                for i in i0..=i1 {
                    let (x, y) = (i as f64 * step, j as f64 * step);
                    if (x - cx).hypot(y - cy) <= radius {
                        t[j * nx + i] = (s.base_thickness + s.dot_height) as f32;
                    }
                }
            }
        }
        ctl.check()?;
    }
    let m = heightfield(&t, nx, ny, step, step, 0.0).map_err(FabError::Invalid)?;
    finish(m, "Halftone", "Halftone", s.usable, vec![format!("{cells_x} × {cells_y} dots at {:.2} mm pitch", s.pitch_mm)], ctl)
}

#[cfg(test)]
mod tests {
    use super::*;
    use image::{ImageBuffer, ImageEncoder, Luma};

    fn png(w: u32, h: u32, f: impl Fn(u32, u32) -> u8) -> Vec<u8> {
        let img: ImageBuffer<Luma<u8>, Vec<u8>> = ImageBuffer::from_fn(w, h, |x, y| Luma([f(x, y)]));
        let mut out = Vec::new();
        image::codecs::png::PngEncoder::new(&mut out).write_image(img.as_raw(), w, h, image::ExtendedColorType::L8).unwrap();
        out
    }

    #[test]
    fn decode_resample_and_blur() {
        let bytes = png(40, 20, |x, _| if x < 20 { 0 } else { 255 });
        let g = decode(&bytes).unwrap();
        assert_eq!((g.width, g.height), (40, 20));
        let r = g.resample(4, 2);
        assert_eq!(r.data[0], 0.0);
        assert_eq!(r.data[3], 1.0);
        // blur smooths the edge
        let b = g.blur(2);
        assert!(b.data[19] > 0.0 && b.data[19] < 1.0);
        assert!(decode(b"not an image").is_err());
    }

    #[test]
    fn lithophane_is_thicker_where_the_image_is_dark() {
        let bytes = png(100, 50, |x, _| (x * 255 / 99) as u8); // dark on the left, bright on the right
        let mut s = LithophaneSettings::new(DVec3::splat(250.0));
        s.width_mm = 50.0;
        s.pixel_mm = 0.5;
        let r = lithophane(&bytes, &s, &Control::none()).unwrap();
        assert!(r.report.is_valid(), "{:?}", r.report);
        let m = &r.parts[0].piece.mesh;
        let b = m.bounds().unwrap();
        assert!((b.size().x - 50.0).abs() < 1e-3 && (b.size().y - 25.0).abs() < 0.6);
        // Thickness at the dark edge ~3.2, at the bright edge ~0.8.
        let top_at = |x: f32| m.vertices.iter().filter(|v| (v.x - x).abs() < 0.01 && v.z > 0.5).map(|v| v.z).fold(0.0f32, f32::max);
        assert!((top_at(0.0) - 3.2).abs() < 0.15, "{}", top_at(0.0));
        assert!((top_at(50.0) - 0.8).abs() < 0.15, "{}", top_at(50.0));
        // Inverting flips it.
        let inv = lithophane(&bytes, &LithophaneSettings { invert: true, ..s.clone() }, &Control::none()).unwrap();
        let mi = &inv.parts[0].piece.mesh;
        let t0 = mi.vertices.iter().filter(|v| v.x.abs() < 0.01 && v.z > 0.5).map(|v| v.z).fold(0.0f32, f32::max);
        assert!((t0 - 0.8).abs() < 0.15, "{t0}");
    }

    #[test]
    fn image_relief_rises_where_bright_and_halftone_dots_scale_with_darkness() {
        let bytes = png(60, 60, |_, y| if y < 30 { 255 } else { 0 }); // bright top half, dark bottom half
        let mut s = ImageReliefSettings::new(DVec3::splat(250.0));
        s.width_mm = 30.0;
        s.pixel_mm = 0.5;
        s.blur_radius = 0;
        let r = relief_from_image(&bytes, &s, &Control::none()).unwrap();
        assert!(r.report.is_valid(), "{:?}", r.report);
        let b = r.parts[0].piece.bounds;
        assert!((b.max.z - 6.5).abs() < 1e-3, "bright areas rise to base + relief height: {}", b.max.z);

        let mut h = HalftoneSettings::new(DVec3::splat(250.0));
        h.width_mm = 30.0;
        h.pitch_mm = 3.0;
        let r = halftone(&bytes, &h, &Control::none()).unwrap();
        assert!(r.report.is_valid(), "{:?}", r.report);
        let m = &r.parts[0].piece.mesh;
        let raised = |lo: f32, hi: f32| m.vertices.iter().filter(|v| v.y >= lo && v.y <= hi && v.z > 1.5).count();
        // The bottom half of the image (y small) is dark: lots of raised samples; the top half is white: none.
        assert!(raised(0.0, 14.0) > 100, "{}", raised(0.0, 14.0));
        assert_eq!(raised(16.0, 30.0), 0);
    }

    #[test]
    fn invalid_settings_and_sizes_are_rejected() {
        let bytes = png(10, 10, |_, _| 128);
        let usable = DVec3::splat(250.0);
        assert!(lithophane(
            &bytes,
            &LithophaneSettings { min_thickness: 3.0, max_thickness: 2.0, ..LithophaneSettings::new(usable) },
            &Control::none()
        )
        .is_err());
        assert!(lithophane(&bytes, &LithophaneSettings { pixel_mm: 0.0, ..LithophaneSettings::new(usable) }, &Control::none()).is_err());
        let e = lithophane(
            &bytes,
            &LithophaneSettings { width_mm: 1000.0, pixel_mm: 0.1, max_samples: 1000, ..LithophaneSettings::new(usable) },
            &Control::none(),
        )
        .unwrap_err();
        assert!(e.to_string().contains("too many"), "{e}");
        assert!(halftone(&bytes, &HalftoneSettings { samples_per_dot: 2, ..HalftoneSettings::new(usable) }, &Control::none()).is_err());
    }

    #[test]
    fn large_lithophanes_are_tiled_to_the_machine() {
        let bytes = png(50, 50, |x, y| ((x + y) * 2) as u8);
        let mut s = LithophaneSettings::new(DVec3::new(100.0, 100.0, 20.0));
        s.width_mm = 160.0;
        s.pixel_mm = 1.0;
        let r = lithophane(&bytes, &s, &Control::none()).unwrap();
        assert!(r.report.is_valid(), "{:?}", r.report);
        assert!(r.parts.len() >= 4, "{}", r.parts.len());
        assert!(r.report.notes.iter().any(|n| n.contains("tiled")));
    }
}
