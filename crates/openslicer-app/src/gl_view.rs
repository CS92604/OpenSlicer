//! OpenGL 3D viewport renderer, driven from egui through a paint callback.
//!
//! Meshes are uploaded once as indexed positions (no per-vertex normals);
//! the fragment shader derives flat face normals from screen-space derivatives,
//! so GPU memory is 12 bytes per vertex plus 12 bytes per triangle.

use eframe::egui_glow::{self, glow, glow::HasContext};
use glam::{Mat4, Vec3};
use openslicer_core::Mesh;
use std::collections::HashMap;
use std::sync::{Arc, Mutex};

#[repr(C)]
#[derive(Clone, Copy, bytemuck::Pod, bytemuck::Zeroable)]
pub struct FlatVertex {
    pub pos: [f32; 3],
    pub color: [f32; 4],
}

impl FlatVertex {
    pub fn new(pos: Vec3, color: [f32; 4]) -> Self {
        Self { pos: pos.to_array(), color }
    }
}

/// A batch of flat-coloured primitives.
pub struct FlatBatch {
    pub triangles: bool,
    pub verts: Vec<FlatVertex>,
}

pub struct DrawModel {
    pub id: u64,
    pub matrix: Mat4,
    pub selected: bool,
    pub mirrored: bool,
    /// Colour when not selected; `None` for the default grey.
    pub color: Option<[f32; 3]>,
}

/// Everything needed to draw one frame.
pub struct Frame {
    pub view: Mat4,
    pub proj: Mat4,
    pub underlay: Vec<FlatBatch>,
    pub models: Vec<DrawModel>,
    pub overlay: Vec<FlatBatch>,
}

struct GpuMesh {
    vao: glow::VertexArray,
    vbo: glow::Buffer,
    ibo: glow::Buffer,
    index_count: i32,
}

struct Gpu {
    mesh_prog: glow::Program,
    flat_prog: glow::Program,
    flat_vao: glow::VertexArray,
    flat_vbo: glow::Buffer,
    meshes: HashMap<u64, GpuMesh>,
}

#[derive(Default)]
struct State {
    gpu: Option<Gpu>,
    uploads: Vec<(u64, Arc<Mesh>)>,
    removals: Vec<u64>,
    /// Set when shader compilation fails; the message is surfaced in the UI.
    error: Option<String>,
}

/// Shared handle between the UI thread and the paint callback.
#[derive(Clone, Default)]
pub struct Renderer {
    state: Arc<Mutex<State>>,
}

const MESH_VS: &str = r#"#version 330 core
layout(location = 0) in vec3 pos;
uniform mat4 u_mvp;
uniform mat4 u_mv;
out vec3 v_pos;
void main() {
    v_pos = (u_mv * vec4(pos, 1.0)).xyz;
    gl_Position = u_mvp * vec4(pos, 1.0);
}"#;

const MESH_FS: &str = r#"#version 330 core
in vec3 v_pos;
uniform vec3 u_color;
uniform float u_flip;
out vec4 frag;
void main() {
    vec3 n = normalize(cross(dFdx(v_pos), dFdy(v_pos)));
    bool front = (gl_FrontFacing != (u_flip > 0.5));
    vec3 l1 = normalize(vec3(-0.4, 0.6, 0.8));
    vec3 l2 = normalize(vec3(0.6, -0.2, 0.4));
    float d1 = max(dot(n, l1), 0.0);
    float d2 = max(dot(n, l2), 0.0);
    float light = 0.30 + 0.60 * d1 + 0.22 * d2;
    // Faces seen from behind (inverted winding / open shells) are tinted orange.
    vec3 base = front ? u_color : vec3(0.95, 0.45, 0.12);
    vec3 col = base * light + vec3(0.10) * pow(d1, 24.0);
    frag = vec4(col, 1.0);
}"#;

const FLAT_VS: &str = r#"#version 330 core
layout(location = 0) in vec3 pos;
layout(location = 1) in vec4 col;
uniform mat4 u_mvp;
out vec4 v_col;
void main() {
    v_col = col;
    gl_Position = u_mvp * vec4(pos, 1.0);
}"#;

const FLAT_FS: &str = r#"#version 330 core
in vec4 v_col;
out vec4 frag;
void main() { frag = v_col; }"#;

unsafe fn compile(gl: &glow::Context, vs: &str, fs: &str) -> Result<glow::Program, String> {
    unsafe {
        let prog = gl.create_program()?;
        let mut shaders = Vec::new();
        for (kind, src) in [(glow::VERTEX_SHADER, vs), (glow::FRAGMENT_SHADER, fs)] {
            let sh = gl.create_shader(kind)?;
            gl.shader_source(sh, src);
            gl.compile_shader(sh);
            if !gl.get_shader_compile_status(sh) {
                return Err(format!("shader compile error: {}", gl.get_shader_info_log(sh)));
            }
            gl.attach_shader(prog, sh);
            shaders.push(sh);
        }
        gl.link_program(prog);
        if !gl.get_program_link_status(prog) {
            return Err(format!("shader link error: {}", gl.get_program_info_log(prog)));
        }
        for sh in shaders {
            gl.detach_shader(prog, sh);
            gl.delete_shader(sh);
        }
        Ok(prog)
    }
}

impl Gpu {
    unsafe fn new(gl: &glow::Context) -> Result<Self, String> {
        unsafe {
            let mesh_prog = compile(gl, MESH_VS, MESH_FS)?;
            let flat_prog = compile(gl, FLAT_VS, FLAT_FS)?;
            let flat_vao = gl.create_vertex_array()?;
            let flat_vbo = gl.create_buffer()?;
            gl.bind_vertex_array(Some(flat_vao));
            gl.bind_buffer(glow::ARRAY_BUFFER, Some(flat_vbo));
            let stride = std::mem::size_of::<FlatVertex>() as i32;
            gl.enable_vertex_attrib_array(0);
            gl.vertex_attrib_pointer_f32(0, 3, glow::FLOAT, false, stride, 0);
            gl.enable_vertex_attrib_array(1);
            gl.vertex_attrib_pointer_f32(1, 4, glow::FLOAT, false, stride, 12);
            gl.bind_vertex_array(None);
            Ok(Self { mesh_prog, flat_prog, flat_vao, flat_vbo, meshes: HashMap::new() })
        }
    }

    unsafe fn upload(&mut self, gl: &glow::Context, id: u64, mesh: &Mesh) -> Result<(), String> {
        unsafe {
            self.remove(gl, id);
            let vao = gl.create_vertex_array()?;
            let vbo = gl.create_buffer()?;
            let ibo = gl.create_buffer()?;
            gl.bind_vertex_array(Some(vao));
            gl.bind_buffer(glow::ARRAY_BUFFER, Some(vbo));
            gl.buffer_data_u8_slice(glow::ARRAY_BUFFER, bytemuck::cast_slice(&mesh.vertices), glow::STATIC_DRAW);
            gl.enable_vertex_attrib_array(0);
            gl.vertex_attrib_pointer_f32(0, 3, glow::FLOAT, false, 12, 0);
            gl.bind_buffer(glow::ELEMENT_ARRAY_BUFFER, Some(ibo));
            gl.buffer_data_u8_slice(glow::ELEMENT_ARRAY_BUFFER, bytemuck::cast_slice(&mesh.triangles), glow::STATIC_DRAW);
            gl.bind_vertex_array(None);
            self.meshes.insert(id, GpuMesh { vao, vbo, ibo, index_count: (mesh.triangles.len() * 3) as i32 });
            Ok(())
        }
    }

    unsafe fn remove(&mut self, gl: &glow::Context, id: u64) {
        if let Some(m) = self.meshes.remove(&id) {
            unsafe {
                gl.delete_vertex_array(m.vao);
                gl.delete_buffer(m.vbo);
                gl.delete_buffer(m.ibo);
            }
        }
    }

    unsafe fn draw_flat(&self, gl: &glow::Context, mvp: &Mat4, batches: &[FlatBatch]) {
        unsafe {
            gl.use_program(Some(self.flat_prog));
            let loc = gl.get_uniform_location(self.flat_prog, "u_mvp");
            gl.uniform_matrix_4_f32_slice(loc.as_ref(), false, &mvp.to_cols_array());
            gl.bind_vertex_array(Some(self.flat_vao));
            gl.bind_buffer(glow::ARRAY_BUFFER, Some(self.flat_vbo));
            for b in batches.iter().filter(|b| !b.verts.is_empty()) {
                gl.buffer_data_u8_slice(glow::ARRAY_BUFFER, bytemuck::cast_slice(&b.verts), glow::STREAM_DRAW);
                let mode = if b.triangles { glow::TRIANGLES } else { glow::LINES };
                gl.draw_arrays(mode, 0, b.verts.len() as i32);
            }
            gl.bind_vertex_array(None);
        }
    }
}

impl Renderer {
    pub fn upload(&self, id: u64, mesh: Arc<Mesh>) {
        self.state.lock().unwrap().uploads.push((id, mesh));
    }

    pub fn remove(&self, id: u64) {
        self.state.lock().unwrap().removals.push(id);
    }

    pub fn take_error(&self) -> Option<String> {
        self.state.lock().unwrap().error.take()
    }

    /// Builds the egui paint callback that draws `frame`.
    pub fn callback(&self, frame: Frame) -> Arc<egui_glow::CallbackFn> {
        let state = self.state.clone();
        Arc::new(egui_glow::CallbackFn::new(move |_info, painter| {
            let gl: &glow::Context = painter.gl();
            let mut st = state.lock().unwrap();
            if st.gpu.is_none() && st.error.is_none() {
                match unsafe { Gpu::new(gl) } {
                    Ok(g) => st.gpu = Some(g),
                    Err(e) => st.error = Some(e),
                }
            }
            let State { gpu, uploads, removals, error } = &mut *st;
            let Some(gpu) = gpu.as_mut() else { return };
            unsafe {
                for id in removals.drain(..) {
                    gpu.remove(gl, id);
                }
                for (id, mesh) in uploads.drain(..) {
                    if let Err(e) = gpu.upload(gl, id, &mesh) {
                        *error = Some(format!("could not upload model to the GPU: {e}"));
                    }
                }

                let view_proj = frame.proj * frame.view;
                gl.depth_mask(true);
                gl.disable(glow::DEPTH_TEST);
                gl.enable(glow::BLEND);
                gl.blend_func(glow::SRC_ALPHA, glow::ONE_MINUS_SRC_ALPHA);
                gpu.draw_flat(gl, &view_proj, &frame.underlay);

                gl.clear(glow::DEPTH_BUFFER_BIT);
                gl.enable(glow::DEPTH_TEST);
                gl.depth_func(glow::LEQUAL);
                gl.disable(glow::BLEND);
                gl.use_program(Some(gpu.mesh_prog));
                let u = |name| gl.get_uniform_location(gpu.mesh_prog, name);
                for m in &frame.models {
                    let Some(g) = gpu.meshes.get(&m.id) else { continue };
                    let mv = frame.view * m.matrix;
                    gl.uniform_matrix_4_f32_slice(u("u_mvp").as_ref(), false, &(frame.proj * mv).to_cols_array());
                    gl.uniform_matrix_4_f32_slice(u("u_mv").as_ref(), false, &mv.to_cols_array());
                    let color = surface_color(m.selected, m.color);
                    gl.uniform_3_f32(u("u_color").as_ref(), color[0], color[1], color[2]);
                    gl.uniform_1_f32(u("u_flip").as_ref(), if m.mirrored { 1.0 } else { 0.0 });
                    gl.bind_vertex_array(Some(g.vao));
                    gl.draw_elements(glow::TRIANGLES, g.index_count, glow::UNSIGNED_INT, 0);
                }
                gl.bind_vertex_array(None);

                gl.enable(glow::BLEND);
                gpu.draw_flat(gl, &view_proj, &frame.overlay);
                gl.disable(glow::DEPTH_TEST);
            }
        }))
    }
}

/// Colour of a model's surface: the highlight blue when selected (blended with its own colour, so parts stay
/// recognisable), its own colour, or the default grey.
pub fn surface_color(selected: bool, own: Option<[f32; 3]>) -> [f32; 3] {
    const HIGHLIGHT: [f32; 3] = [0.42, 0.72, 0.95];
    const GREY: [f32; 3] = [0.72, 0.74, 0.78];
    match (selected, own) {
        (true, Some(c)) => std::array::from_fn(|i| 0.45 * c[i] + 0.55 * HIGHLIGHT[i]),
        (true, None) => HIGHLIGHT,
        (false, Some(c)) => c,
        (false, None) => GREY,
    }
}

/// Distinct, readable part colours (pastel enough to keep shading visible).
pub const PART_PALETTE: [[f32; 3]; 12] = [
    [0.91, 0.55, 0.45],
    [0.45, 0.78, 0.55],
    [0.93, 0.80, 0.42],
    [0.62, 0.55, 0.90],
    [0.45, 0.75, 0.85],
    [0.90, 0.52, 0.75],
    [0.70, 0.82, 0.40],
    [0.95, 0.65, 0.35],
    [0.50, 0.62, 0.92],
    [0.80, 0.60, 0.50],
    [0.55, 0.85, 0.75],
    [0.85, 0.75, 0.90],
];

/// Appends a wireframe box.
pub fn push_box_lines(out: &mut Vec<FlatVertex>, min: Vec3, max: Vec3, color: [f32; 4]) {
    let c = |i: usize| {
        Vec3::new(if i & 1 == 0 { min.x } else { max.x }, if i & 2 == 0 { min.y } else { max.y }, if i & 4 == 0 { min.z } else { max.z })
    };
    for (a, b) in [(0, 1), (2, 3), (4, 5), (6, 7), (0, 2), (1, 3), (4, 6), (5, 7), (0, 4), (1, 5), (2, 6), (3, 7)] {
        out.push(FlatVertex::new(c(a), color));
        out.push(FlatVertex::new(c(b), color));
    }
}

/// Build plate: filled quad, minor/major grid lines and axes. `half` is half the plate size in X/Y.
pub fn plate_batches(half: (f32, f32), show_grid: bool) -> Vec<FlatBatch> {
    let (hx, hy) = half;
    let z = 0.0;
    let fill = [0.145, 0.15, 0.16, 1.0];
    let quad = [(-hx, -hy), (hx, -hy), (hx, hy), (-hx, -hy), (hx, hy), (-hx, hy)]
        .iter()
        .map(|&(x, y)| FlatVertex::new(Vec3::new(x, y, z), fill))
        .collect();
    let mut batches = vec![FlatBatch { triangles: true, verts: quad }];
    let mut lines = Vec::new();
    if show_grid {
        let minor = [1.0, 1.0, 1.0, 0.045];
        let major = [1.0, 1.0, 1.0, 0.10];
        let mut x = -((hx / 10.0).floor() * 10.0);
        while x <= hx + 1e-3 {
            let c = if (x / 50.0).fract().abs() < 1e-4 { major } else { minor };
            lines.push(FlatVertex::new(Vec3::new(x, -hy, z), c));
            lines.push(FlatVertex::new(Vec3::new(x, hy, z), c));
            x += 10.0;
        }
        let mut y = -((hy / 10.0).floor() * 10.0);
        while y <= hy + 1e-3 {
            let c = if (y / 50.0).fract().abs() < 1e-4 { major } else { minor };
            lines.push(FlatVertex::new(Vec3::new(-hx, y, z), c));
            lines.push(FlatVertex::new(Vec3::new(hx, y, z), c));
            y += 10.0;
        }
    }
    // Plate outline and origin axes.
    let edge = [0.55, 0.58, 0.62, 0.9];
    for (a, b) in [((-hx, -hy), (hx, -hy)), ((hx, -hy), (hx, hy)), ((hx, hy), (-hx, hy)), ((-hx, hy), (-hx, -hy))] {
        lines.push(FlatVertex::new(Vec3::new(a.0, a.1, z), edge));
        lines.push(FlatVertex::new(Vec3::new(b.0, b.1, z), edge));
    }
    let len = hx.min(hy) * 0.25;
    lines.push(FlatVertex::new(Vec3::ZERO, [0.95, 0.35, 0.35, 1.0]));
    lines.push(FlatVertex::new(Vec3::new(len, 0.0, 0.0), [0.95, 0.35, 0.35, 1.0]));
    lines.push(FlatVertex::new(Vec3::ZERO, [0.45, 0.85, 0.40, 1.0]));
    lines.push(FlatVertex::new(Vec3::new(0.0, len, 0.0), [0.45, 0.85, 0.40, 1.0]));
    batches.push(FlatBatch { triangles: false, verts: lines });
    batches
}

#[cfg(test)]
mod tests {
    #[test]
    fn part_colours_are_distinct_and_selection_stays_recognisable() {
        for (i, a) in PART_PALETTE.iter().enumerate() {
            for b in &PART_PALETTE[i + 1..] {
                let d: f32 = (0..3).map(|k| (a[k] - b[k]).abs()).sum();
                assert!(d > 0.15, "{a:?} and {b:?} are too alike");
            }
        }
        assert_eq!(surface_color(false, None), [0.72, 0.74, 0.78]);
        assert_eq!(surface_color(false, Some(PART_PALETTE[0])), PART_PALETTE[0]);
        assert_ne!(surface_color(true, Some(PART_PALETTE[0])), surface_color(true, Some(PART_PALETTE[1])));
    }

    use super::*;

    #[test]
    fn plate_has_quad_and_even_line_vertex_count() {
        let b = plate_batches((110.0, 110.0), true);
        assert_eq!(b[0].verts.len(), 6);
        assert!(b[1].verts.len() % 2 == 0);
        assert!(b[1].verts.len() > 40);
    }

    #[test]
    fn box_lines_have_twelve_edges() {
        let mut v = Vec::new();
        push_box_lines(&mut v, Vec3::ZERO, Vec3::ONE, [1.0; 4]);
        assert_eq!(v.len(), 24);
    }

    #[test]
    fn vertex_layout_matches_shader_offsets() {
        assert_eq!(std::mem::size_of::<FlatVertex>(), 28);
        assert_eq!(std::mem::size_of::<glam::Vec3>(), 12);
    }
}
