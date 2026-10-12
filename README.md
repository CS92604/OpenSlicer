# OpenSlicer

**Open-source, offline software for splitting oversized 3D models into printable parts, joining them with real printable connectors, and planning the assembly.**

OpenSlicer is written entirely in Rust. It runs locally, with no account, subscription, cloud service or network connection. It is intended as an open alternative to commercial large-model splitting tools such as LuBan 3D.

> **Project status: pre-alpha, design phase.**
> This repository currently contains the project definition only. No feature listed below is implemented yet. The status table in [Roadmap](#roadmap) is updated as each feature lands, and a feature is marked **Done** only once it is implemented and covered by automated tests on real geometry. Anything not marked Done is not available.

OpenSlicer is an independent project. It is not affiliated with, endorsed by, or derived from the source code of LuBan 3D or any other commercial product.

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

## Planned architecture

A Cargo workspace of focused crates, so the geometry engine can be used and tested without the GUI:

| Crate | Responsibility |
|---|---|
| `openslicer-core` | Math types, indexed triangle mesh, spatial acceleration structures, tolerances |
| `openslicer-io` | STL, OBJ and 3MF readers and writers, export validation |
| `openslicer-geom` | Mesh validation and repair diagnostics, plane cutting with cap generation, Boolean operations |
| `openslicer-split` | Build-volume fitting, grid, tile and manual cut planning, part validation |
| `openslicer-joints` | Connector geometry generation, placement, clearance and collision checks |
| `openslicer-assembly` | Part numbering, mating relationships, sequencing, documentation generation |
| `openslicer-project` | Native project format, autosave, recovery, printer profiles, manifest |
| `openslicer-app` | Desktop GUI, viewport, job system, undo and redo, preferences |
| `openslicer-cli` | Headless interface for scripting and for automated end-to-end testing |

The GUI is planned to use `egui`/`eframe` on `wgpu`, which targets Windows 11 first and keeps macOS and Linux practical. Final dependency choices will be recorded in [docs/dependencies.md](docs/dependencies.md) together with their licenses.

---

## Roadmap

| Area | Status |
|---|---|
| Workspace, CI, license files | Planned |
| Core mesh and math | Planned |
| STL / OBJ / 3MF I/O | Planned |
| Mesh validation | Planned |
| Plane cutting with capping | Planned |
| Automatic and manual splitting | Planned |
| Connector generation | Planned |
| Assembly planning and documentation | Planned |
| Printer profiles | Planned |
| Project files and recovery | Planned |
| Desktop GUI | Planned |
| Windows installer and release automation | Planned |
| Large-scale (automotive) validation | Planned |

Status values are **Planned**, **In progress** and **Done**.

---

## Building

Nothing buildable exists yet. Once the workspace lands, the intended workflow is:

```sh
# Requirements: a current stable Rust toolchain (https://rustup.rs)
git clone https://github.com/CS92604/OpenSlicer.git
cd OpenSlicer
cargo build --release
cargo test --workspace
cargo run --release -p openslicer-app
```

Windows 11 is the primary target. Windows installer packaging and release automation will be documented here when they exist.

---

## License

OpenSlicer is intended to be released under the **MIT OR Apache-2.0** dual license, the common choice in the Rust ecosystem. The `LICENSE-MIT` and `LICENSE-APACHE` files and the third-party license disclosure have not yet been added to the repository. They will land with the initial workspace, before any code is accepted. Until they exist, no license is granted.

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

Contributions will be welcome once the initial workspace exists. Priorities are geometry correctness, test coverage on real meshes, and clear error reporting.
