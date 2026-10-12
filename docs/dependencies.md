# Dependencies

Direct dependencies of the workspace. Transitive dependencies have **not** been audited; run `cargo tree` / `cargo deny` before redistributing.

| Crate | Use | License |
|---|---|---|
| glam, bytemuck | math, GPU buffers | MIT OR Apache-2.0 |
| serde, serde_json | settings, manifests, projects | MIT OR Apache-2.0 |
| sha2 | SHA-256 for manifests and project integrity | MIT OR Apache-2.0 |
| thiserror, rustc-hash, rayon | utilities | MIT OR Apache-2.0 |
| earcutr | polygon triangulation for extrusions | ISC |
| spade | constrained Delaunay triangulation for cut caps | MIT OR Apache-2.0 |
| i_overlay | 2D polygon Booleans | MIT |
| **boolmesh** | 3D mesh Booleans (connectors, drain holes, wireframes) | **MPL-2.0** |
| image | PNG/JPEG decoding for image tools | MIT OR Apache-2.0 |
| zip, roxmltree | 3MF and project files | MIT; MIT OR Apache-2.0 |
| truck-stepio, truck-meshalgo (with ruststep) | STEP import: parsing and tessellating exact B-reps | Apache-2.0 |
| winresource (Windows builds only) | embeds the application icon in the exe | MIT |
| clap | command-line parsing | MIT OR Apache-2.0 |
| eframe / egui, rfd, ron | desktop app | MIT OR Apache-2.0 |

## MPL-2.0 disclosure

`boolmesh` is licensed under the Mozilla Public License 2.0, a file-level copyleft. OpenSlicer links it unmodified. If you modify boolmesh files and distribute binaries, you must make those modified files available under MPL-2.0. This does not change the licence of OpenSlicer's own code (MIT OR Apache-2.0).
