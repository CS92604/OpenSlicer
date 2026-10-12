# Known limitations

Honest status as of this commit.

## Not done
- **The desktop GUI covers splitting, connectors, results and export only.** Alternative construction methods, image tools, hollowing, Booleans, repair, per-connector editing, manual cut-plane placement and assembly step playback are CLI-only or missing. The exploded-view slider was implemented but could not be exercised under the automated Xvfb session, so it is untested. The GUI cannot save or open `.osproj` projects yet, and the project format does not store generated parts or assembly relationships. There is no thin-wall check beyond connector fit. Splitting is not multithreaded beyond what the Boolean library does internally. Undo/redo, a light theme, searchable settings and project autosave in the GUI are not implemented (the project file *format* and loader exist in `openslicer-project`).
- **Windows: built by CI, never run by a person.** The release is a single static `openslicer.exe` (no installer). The GUI was only run on Linux under software OpenGL.
- **No real-printer validation.** Connector clearances are presets, not measured fits. Print a test joint first.
- **No automotive-scale validation.** A Miata-hardtop-class model has not been processed. Measured so far on generated geometry: a 1M-triangle plane cut with full verification took about 0.8 s; a boolmesh union on a 1M-triangle mesh about 4.5 s; a 1M-triangle STL loads in the app in under 3 s using about 260 MB. Thin, curved, self-intersecting real-world shells have not been tried and may expose problems.
- Interactive features not implemented: layout editing in the viewport, exploded 3D view, part highlighting. Assembly diagrams exist only in the exported HTML (static SVG, painter's-algorithm rendering with vertex-clustered display meshes).

## Behaviour to know about
- Connector validation needs closed, manifold pieces. Pieces that fail are reported, not silently skipped; a split with problems exits with code 3.
- Nesting packs by bounding rectangle, not true contour.
- Alternative methods approximate the model (blocks, plates, relief, wireframe); the report states the fill ratio and, where defined, a deviation bound.
- Self-intersection detection is skipped on meshes above 500k triangles.
- `boolmesh` is MPL-2.0 (see dependencies.md).
