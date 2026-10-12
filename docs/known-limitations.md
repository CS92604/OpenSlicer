# Known limitations

Honest status for 0.1.0-beta.1. The README's "Limits and what is not built yet" section is the summary; this file adds detail.

## Not built yet
- **Interactive editing:** no movable cutting planes, live preview, visual face/cut selection, viewport layout editing, per-connector editing, or undo/redo.
- **Thin curved shells:** no surface-following connectors; no topology-aware cutting that avoids tiny awkward sections; no reinforcement.
- **GUI coverage:** repair, hollowing, Booleans, image tools and alternative construction methods exist only in `openslicer-cli`. No GUI batch run over several models, no assembly step playback, no part colouring, no light theme, no searchable settings.
- **Validation:** no dedicated thin-wall check (only connector-fit checks); no tolerance checks of dimensions; the assembly check looks for interpenetration only.
- **Projects:** `.osproj` stores models, their positions, the printer and the split settings. Generated parts and assembly relationships are not stored; run the split again after reopening.
- **Performance:** the pipeline is single-threaded apart from the Boolean library's own parallelism. The viewport has no level-of-detail.
- **Packaging:** one unsigned `openslicer.exe` only; SmartScreen may warn. No installer by design.

## Weak spots
- Pins, dowels, tabs, puzzle, pyramid and claw need a wall of roughly 4.3 mm or more (feature size + clearance + minimum wall on both sides). Thinner walls are skipped and reported.
- Lip and lap joints on curved thin walls often fail Boolean verification and are skipped and reported. On the stock dome, a lap-joint split connected 2 of 4 interfaces; on the stock panel, 1 to 3 of 22.
- Dovetails can be refused (neck too narrow) or fail the volume check on large solid parts; the piece is left without connectors and reported.
- One tested layout (stock wing, Ender-3 volume, auto-tiling at 75°) left one piece without connectors because its volume differed by 6.6 mm³ (0.0008%) from the expectation. The cause is not understood.
- Nesting packs by bounding rectangle, not true contour.
- Self-intersection detection is skipped on meshes above 500k triangles.
- Alternative construction methods approximate the model; the report states the fill ratio and, where defined, a deviation bound.
- Printer presets are published figures entered from memory and not re-checked against each datasheet.
- `boolmesh` is MPL-2.0 (see dependencies.md).

## Not verified
- **Windows:** built and tested by GitHub Actions; no human has run the `.exe`. Title-bar theming and the Segoe UI font path are untested.
- **Real prints:** clearances are presets, not measured fits.
- **Automotive scale:** no real hardtop-class model has been processed. Measured on generated geometry: a 1M-triangle plane cut with full verification about 0.8 s; a `boolmesh` union on a 1M-triangle mesh about 4.5 s; a 1M-triangle STL loads in under 3 s using about 260 MB.
- **GUI:** exercised on Linux under Xvfb with software OpenGL using scripted mouse input (open, stock import, printer presets, split, exploded view, autosave and restore). There is no automated GUI test suite.

## Behaviour to know about
- Connector validation needs closed, manifold pieces. Pieces that fail are reported, never silently skipped; a split with problems exits with code 3.
- Boolean results are cleaned of zero-area triangles caused by duplicate vertices before they are verified; results that are still not closed and manifold are rejected.
- The autosave is written every 10 seconds while the scene changes, on a background thread, to the application data folder. Starting with files on the command line skips the restore prompt.
