# Known limitations

Honest status for 0.1.0-beta.1. The README's "Limits and what is not built yet" section is the summary; this file adds detail.

## Not built yet
- **Interactive editing:** manual planes are axis-based with one tilt; no free 3D rotation handle, face picking, per-connector editing, or editing of cuts after a split (re-run it).
- **Thin curved shells:** no surface-following lap joints or lips that reliably work on 3-4 mm curved walls (the finger joint is the fallback); no topology-aware cutting that avoids tiny awkward sections; no reinforcement.
- **GUI coverage:** repair, hollowing, Booleans, image tools and alternative construction methods exist only in `openslicer-cli`. No GUI batch run over several models, no light theme, no searchable settings, no hand placement of labels.
- **Validation:** no dedicated thin-wall check; no tolerance checks of dimensions; the assembly check looks for interpenetration only.
- **Projects:** `.osproj` stores models, positions, the printer and the split settings. Generated parts, assembly relationships and the undo history are not stored; run the split again after reopening.
- **Performance:** the pipeline is single-threaded apart from the Boolean library's own parallelism. The viewport has no level-of-detail.
- **Packaging:** one unsigned `openslicer.exe` only; SmartScreen may warn. No installer by design.

## Weak spots
- Finger joints need walls of about 1.5 mm or more and a seam long enough for two strips. They are rectangular, with no taper or snap, and lock the pieces against sliding along the seam and in the cut direction but not across it, so they need glue or a clip. For thick solid parts pins or dovetails are the better choice.
- Pins, dowels, tabs, puzzle, pyramid and claw need a wall of roughly 4.3 mm or more. Thinner walls fall back to a finger joint (policy `Comb`, the default) or are skipped (policy `Skip`/`ShrinkToFit` in a settings file).
- Lip and lap joints often fail Boolean verification on curved thin walls; the fallback applies and is noted.
- Dovetails can be refused (neck too narrow) or fail the volume check on large solid parts.
- One tested layout (stock wing, Ender-3 volume, auto-tiling at 75°) left one piece without connectors because its volume differed by 6.6 mm³ (0.0008%) from the expectation. The cause is not understood.
- Labels need a flat patch (within 0.3 mm over the text area) and enough material behind it; the search shrinks the text down to 3 mm characters. Parts that cannot take a label are listed and left unchanged.
- Nesting packs by bounding rectangle, not true contour.
- Self-intersection detection is skipped on meshes above 500k triangles.
- Alternative construction methods approximate the model; the report states the fill ratio and, where defined, a deviation bound.
- Printer presets are published figures entered from memory and not re-checked against each datasheet.
- `boolmesh` is MPL-2.0 (see dependencies.md).

## Not verified
- **Windows:** built and tested by GitHub Actions; no human has run the `.exe`. Title-bar theming and the Segoe UI font path are untested.
- **Real prints:** clearances are presets, not measured fits.
- **Automotive scale:** no real hardtop-class model has been processed. Measured on generated geometry: a 1M-triangle plane cut with full verification about 0.8 s; a `boolmesh` union on a 1M-triangle mesh about 4.5 s; a 1M-triangle STL loads in under 3 s using about 260 MB.
- **GUI:** exercised on Linux under Xvfb with software OpenGL using scripted mouse input (open, stock import, printer presets, split, manual plane dragging, part colours, assembly playback, undo through the menu, exploded view, autosave and restore). Keyboard shortcuts could not be exercised there (no key events reach the window without a window manager) and are untested. There is no automated GUI test suite.

## Behaviour to know about
- Connector validation needs closed, manifold pieces. Pieces that fail are reported, never silently skipped; a split with problems exits with code 3.
- Boolean results are cleaned of zero-area triangles and near-duplicate vertices (collapsed edges shorter than twice the importer's weld distance) before they are verified, so parts survive an export and re-import; results that are still not closed and manifold are rejected.
- The autosave is written every 10 seconds while the scene changes, on a background thread, to the application data folder. Starting with files on the command line skips the restore prompt.
