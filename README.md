# OpenSlicer

**Open-source, offline software for splitting oversized 3D models into printable parts, joining them with real printable connectors, and planning the assembly.**

OpenSlicer is written entirely in Rust. It runs locally, with no account, subscription, cloud service or network connection. It is intended as an open alternative to commercial large-model splitting tools such as LuBan 3D.

> **Project status: pre-alpha, working skeleton.**
> The desktop application launches and can open, inspect, transform, measure and export STL models. **Splitting, connectors, assembly planning, OBJ/3MF support, project files and undo/redo are not implemented yet.** Their buttons are visible but disabled and labelled "Not implemented yet". See [What works today](#what-works-today) and the [Roadmap](#roadmap). A feature is marked **Done** only once it is implemented and covered by automated tests on real geometry.

![OpenSlicer skeleton](docs/images/skeleton.png)

*The current skeleton showing a 400 mm torus that does not fit a 220 × 220 mm printer. The screenshot was captured on Linux under software OpenGL.*

OpenSlicer is an independent project. It is not affiliated with, endorsed by, or derived from the source code of LuBan 3D or any other commercial product.

---

## What works today

Everything below is implemented and exercised by automated tests or by running the real application:

- **STL import:** binary and ASCII, auto-detected by exact size (so binary files that begin with the word "solid" load correctly). NaN/infinite coordinates and truncated files are rejected with a clear error.
- **Vertex welding on import** within 1e-6 of the model's bounding-box diagonal, so floating-point noise in exported files does not create false open edges. Original coordinates are preserved.
- **Mesh inspection:** triangle and vertex counts, surface area, enclosed volume, open (boundary) edges, non-manifold edges, degenerate triangles, and detection of inverted normals. Back faces render orange in the viewport.
- **3D viewport:** OpenGL renderer with flat shading, build plate with grid, build-volume wireframe, axis triad, orbit / pan / zoom, view presets, click-to-select, and a Move tool that drags a model across the plate.
- **Transforms:** numeric position, rotation, scale, size, drop-to-plate, centre. Exports have the transform baked in (winding is corrected for mirrored models).
- **Printer profile:** editable build volume and margin with presets, saved between sessions. Every model is checked against the usable volume and over-size is reported per axis.
- **Export:** binary STL, written to a temporary file, **re-imported and compared** (triangle count, volume, bounds) before being atomically renamed into place. A failed export never replaces an existing file.
- **Background jobs:** opening and exporting run on worker threads with a progress indicator and a Cancel button. Cancelling discards the result and leaves no files behind; a phase already in progress finishes first.
- **Multiple models**, drag-and-drop of files onto the window, and command-line file arguments (`openslicer model.stl`).
- **Forced dark theme** styled after Windows 11 / 3D Builder: command bar along the bottom, scene list on the left, properties on the right. Segoe UI is used automatically when installed.

Measured on a 1,000,000-triangle (50 MB) STL under software OpenGL in a Linux container: loaded and displayed in under 3 seconds with about 260 MB resident memory, and reported the correct volume for the analytic shape.

### Keyboard shortcuts

| Key | Action |
|---|---|
| Ctrl+O / Ctrl+E | Open / export selected model |
| F | Fit view to selection (or plate) |
| 1 2 3 4 | Isometric / top / front / right view |
| Delete | Remove selected model |
| Esc | Back to Select tool |
| Left drag | Orbit (or move model with the Move tool) |
| Right / middle drag, Shift + left drag | Pan |
| Scroll | Zoom |
| Double-click | Fit view |

### Not tested yet

OpenSlicer has so far been built and run only on Linux (under Xvfb with Mesa software OpenGL). It targets Windows 11 and uses no Linux-specific code, but **it has not yet been run on Windows**, and the Windows installer and release pipeline do not exist. Window title-bar theming and the Segoe UI font path are untested.

---

## Goals

The finished application should let a user:

1. Import a real oversized model (STL, OBJ, 3MF).
2. Configure a printer profile (build volume, margins, tolerances).
3. Split the model into valid, printable components, automatically or with manual control.
4. Generate matching connectors (pins and sockets, dowels, dovetails, tabs and slots, puzzle joints) as real printable geometry.
5. Inspect the assembly with exploded views, part numbering and an assembly sequence.
6. Export every component, a manifest and printable assembly documentation.
7. Re-import the exported files and verify they are valid and reassemble into the original shape.

The headline target is processing large, complex, thin-walled shells at automotive scale (for example, a full-size hardtop for a Mazda Miata) on consumer printers. Whether this goal is met will be reported honestly, with measurements, in [docs/known-limitations.md](docs/known-limitations.md) once testing against such models exists.

### Principles

- **Geometry correctness over features.** The application must never report success when the resulting mesh is invalid. Every cut, connector and export is validated, and failures are reported plainly.
- **No fake functionality.** No placeholder buttons, simulated progress or stubs presented as finished features.
- **Fail safe.** Operations never modify the original model or a saved project in place. Writes are atomic.
- **Local and open.** No proprietary services. All dependencies must have licenses compatible with the project's distribution model.

---

## Planned feature set

### Model import and management
- STL (binary and ASCII), OBJ, and 3MF import and export.
- Multiple models per project.
- Measurement, scale, rotate, translate and orientation controls.
- Mesh inspection and validation.

### Splitting
- Automatic subdivision to fit a build volume.
- User-controlled splitting planes, grid splitting and configurable tile splitting.
- Overlap and cut-placement controls, with a cut-minimizing automatic mode.
- Support for curved, hollow and concave geometry.
- Layout preview and editing before parts are generated.
- Detection and reporting of empty, disconnected, invalid or non-fitting parts.
- Volume-conservation checks: pieces must reassemble to the original.

### Connectors
- Alignment pins and sockets, dowels, dovetails, interlocking tabs and slots, puzzle joints.
- Configurable shape, size, spacing, orientation and clearance.
- Automatic placement along shared boundaries, with manual adjustment and removal.
- Collision detection, thin or weak region detection, and optional reinforcement.

### Assembly planning
- Unique part IDs, labels and orientation markers.
- Interactive exploded view, part highlighting and mating-surface display.
- Assembly sequence, step diagrams, connector placement information and parts list.
- Printable assembly documentation export.

### Printers and constraints
- Saved printer profiles: build volume, margins, orientation constraints.
- Material- and printer-dependent joint clearances and minimum feature sizes.
- Validation that every part fits the selected profile.

### Mesh integrity
- Non-manifold edge, open boundary and degenerate triangle detection.
- Self-intersection detection where practical.
- Validation of Boolean results and generated connector geometry.
- Round-trip verification of exported files.

### Application
- Native desktop GUI with a large 3D viewport, light and dark themes, undo and redo, keyboard shortcuts and persistent preferences.
- Background processing with progress reporting and cancellation, so the UI stays responsive.
- Native project files, autosave and recovery, batch export with naming templates, and a JSON manifest.

---

## Architecture

A Cargo workspace, so the geometry code can be tested without the GUI. Crates that exist today:

| Crate | Responsibility |
|---|---|
| `openslicer-core` | Indexed triangle mesh, bounds, measurements, topology checks, welding, ray picking |
| `openslicer-io` | STL reader and writer |
| `openslicer-app` | Desktop application (`openslicer` binary): UI, viewport renderer, background jobs, preferences |

Planned crates as features land: `openslicer-geom` (plane cutting with capping, Booleans, self-intersection checks), `openslicer-split`, `openslicer-joints`, `openslicer-assembly`, `openslicer-project`, and `openslicer-cli` for headless use and end-to-end tests.

The GUI uses `egui`/`eframe` with an OpenGL (glow) renderer. Meshes are uploaded to the GPU as indexed positions only; face normals are derived in the fragment shader, so a model costs 12 bytes per vertex plus 12 bytes per triangle of GPU memory. OpenGL 3.3 or newer is required; the app retries without anti-aliasing if the driver cannot provide 4× MSAA.

---

## Roadmap

| Area | Status |
|---|---|
| Workspace, license files | Done |
| CI workflow (format, clippy, tests, release build) | In progress: written, not yet run on GitHub |
| Core mesh and math | In progress: basic mesh, measurement, welding, picking |
| STL I/O | Done for binary and ASCII STL. OBJ and 3MF are Planned |
| Mesh validation | In progress: open edges, non-manifold edges, degenerate triangles, inverted normals. Self-intersection is Planned |
| Plane cutting with capping | Planned |
| Automatic and manual splitting | Planned |
| Connector generation | Planned |
| Assembly planning and documentation | Planned |
| Printer profiles | In progress: one editable profile with presets and fit checking. Named, multiple saved profiles are Planned |
| Project files, autosave and recovery | Planned (preferences are persisted; projects are not) |
| Desktop GUI | In progress: skeleton described above. Undo/redo, searchable settings and light theme are not planned for the skeleton |
| Windows installer and release automation | Planned |
| Large-scale (automotive) validation | Planned |

Status values are **Planned**, **In progress** and **Done**.

---

## Building

Requirements: a current stable Rust toolchain (<https://rustup.rs>). On Linux you also need the runtime libraries winit and OpenGL load dynamically: X11 or Wayland, `libxkbcommon` (and `libxkbcommon-x11` for X11), and an OpenGL 3.3 driver (Mesa works). No extra system packages are needed on Windows.

```sh
git clone https://github.com/CS92604/OpenSlicer.git
cd OpenSlicer
cargo build --release
cargo test --workspace
cargo run --release -p openslicer-app -- path/to/model.stl
```

Windows installer packaging does not exist yet.

---

## License

OpenSlicer is dual-licensed under **MIT OR Apache-2.0**, at your option. See [LICENSE-MIT](LICENSE-MIT) and [LICENSE-APACHE](LICENSE-APACHE). A complete third-party dependency and license disclosure (`docs/dependencies.md`) has not been written yet; all current direct dependencies (`eframe`/`egui`, `glam`, `bytemuck`, `serde`, `thiserror`, `rfd`, `ron`) are MIT and/or Apache-2.0 licensed, but transitive dependencies have not yet been audited.

---

## Documentation (planned)

- `docs/user-guide.md`: using the application
- `docs/developer-guide.md`: architecture and contributing
- `docs/project-format.md`: native project file format
- `docs/export-formats.md`: exported files and manifest schema
- `docs/dependencies.md`: dependencies and their licenses
- `docs/known-limitations.md`: honest limitations and measured performance

---

## Contributing

Contributions are welcome. Run `cargo fmt --all`, `cargo clippy --workspace --all-targets -- -D warnings` and `cargo test --workspace` before opening a pull request. Priorities are geometry correctness, test coverage on real meshes, and clear error reporting.
