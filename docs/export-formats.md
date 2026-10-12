# Export and project formats

## Export folder

`openslicer-cli split` and `fab`/`image` write, into an empty folder:

- one mesh file per part (`.stl` binary, `.obj`, `.3mf` or `.step`, a faceted B-rep with one face per triangle, in millimetres), named by `--name-template` (`{model}`, `{name}`, `{id}`);
- optionally one file per build plate (`*_plate_NN.*`) when nesting;
- `*_instructions.html` for splits: overview, parts list, connector table and one diagram per assembly step;
- `*_outline.svg` (millimetres) for flat stock from `fab stack`;
- `manifest.json`.

### manifest.json (version 1)

`manifest_version`, `generator`, `model_name`, `format`, `print_orientation`, `method` (`split`, `stack`, …), `printer` (name, build volume, margin, usable volume, clearance, material), `parts[]`, `plates[]`, `outlines[]`, `interfaces[]`, `connectors[]`, `assembly[]` (steps), `instructions`, `assembly_csv` (name of the CSV table of assembly steps, if written), and `validation` (`valid`, `problems`, `notes`, source volume, cut count).

Each part has `id`, `name`, `kind` (`piece` or `loose`), `label`, `file`, `sha256`, `triangles`, `volume_mm3`, `bounds_min/max`, `size`, `watertight`, `fits_printer`, and, when exported in print orientation, `print_rotation` (row-major 3×3) and `print_offset` such that `exported = R · assembly + offset`.

`openslicer-cli verify <folder>` re-imports every file and checks hash, triangle count, volume, bounds, watertightness, build-volume fit (print-orientation exports) and that plates fit. An export that fails verification at write time has its manifest removed and reports an error.

## Project files (`.osproj`)

A zip archive: `project.json` (version, name, printer profile, split settings, model list with SHA-256) and `models/NN_name.stl`. Saves are atomic and read back before the existing file is replaced. Results are not stored; they are regenerated from the models and settings. Files from a newer format version are refused.


## Assembly CSV

`<model>_assembly.csv` has one row per assembly step in assembly order, then one row per loose part (dowel). Columns:
`step, part, type, file, label_on_part, label_status, attaches_to, contact_area_mm2, connectors, connector_types, needs_glue, loose_parts, starts_subassembly, size_x_mm, size_y_mm, size_z_mm, volume_cm3, print_plate, fits_printer`.
Fields containing commas or quotes are quoted per RFC 4180; list-valued fields (`attaches_to`, `connectors`, `loose_parts`) are space-separated. `label_on_part` is the text engraved or raised on the part (for example `P07-3`) and `label_status` says where it went or why none was placed. `needs_glue` is `yes` when a joint in that step is a finger joint, which does not lock the pieces together across the seam. Each part's `label` also appears in `manifest.json`.

## Project files

`project.json` in an `.osproj` stores for each model its name, STL file, checksum and (optionally) its placement `transform` (`position`, `rotation_deg`, `scale`), plus the printer and split settings, including the part label options.
