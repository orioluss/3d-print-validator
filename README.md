# 3D Print Validator

A fast command-line tool that checks whether a 3D model is suitable for printing before you waste filament or resin on a failed print.

Detects the most common issues: walls too thin for your nozzle, open meshes, ghost bodies, non-manifold edges, and degenerate faces. Outputs a clear 0–100 score with colour-coded feedback and concrete fix suggestions.

## Supported formats

| Format | Extension |
|--------|-----------|
| STL    | `.stl`    |
| STEP   | `.stp`, `.step` *(requires cadquery)* |
| OBJ    | `.obj`    |
| PLY    | `.ply`    |
| 3MF    | `.3mf`    |
| OFF    | `.off`    |

## Installation

```bash
pip install trimesh numpy rtree

# Only needed for STEP files:
pip install cadquery
```

## Usage

```bash
# Basic check (FDM, 0.4 mm nozzle)
python print3d_check.py model.stl

# Specify technology
python print3d_check.py model.stl --tech resin
python print3d_check.py model.stl --tech sls

# Custom nozzle diameter
python print3d_check.py model.stl --nozzle 0.6

# More accurate thickness measurement (more samples = slower)
python print3d_check.py model.stl --samples 5000

# JSON output for scripting / CI pipelines
python print3d_check.py model.stl --json
```

## Example output

```
──────────────────────────────────────────────────────────
  🖨  3D Print Validator
──────────────────────────────────────────────────────────
  File      : bracket.stl
  Technology: FDM (filament)
  Nozzle    : 0.4 mm
  Analysis  : 3.1s
──────────────────────────────────────────────────────────

  ██████████████████████████████  100/100
  ✔ PRINTABLE

  Dimensions
  ········································
  X: 40.00 mm  Y: 30.00 mm  Z: 8.00 mm
  Volume: 6872.2 mm³  |  Surface area: 3840.5 mm²
  Faces: 1,560  |  Vertices: 780

  Mesh Integrity
  ········································
  ✔ Watertight (closed mesh)
  ✔ Consistent normals
  ✔ No non-manifold edges

  Wall Thickness
  ········································
  Minimum : 3.999 mm
  Median  : 7.999 mm
  Mean    : 9.896 mm
  Maximum : 42.734 mm
  Minimum threshold (FDM (filament)): 1.2 mm

  ✔ Wall thickness is adequate for printing.

  Recommendations
  ········································
  ✔ Model looks good. Always do a final check in your slicer.

──────────────────────────────────────────────────────────
```

## Scoring

| Score | Verdict |
|-------|---------|
| 75–100 | ✔ Printable |
| 45–74 | ⚠ Printable with caution |
| 0–44 | ✘ Not printable |

Penalties are applied for: open mesh (−20), inconsistent normals (−10), non-manifold edges (up to −15), open edges (up to −10), ghost bodies (up to −10), degenerate faces (up to −5), and walls below the technology threshold (up to −30).

## Technology profiles

| Flag | Technology | Min wall | Recommended wall |
|------|-----------|----------|-----------------|
| `fdm` *(default)* | FDM filament | 1.2 mm | 2.0 mm |
| `resin` | SLA / MSLA resin | 0.5 mm | 1.0 mm |
| `sls` | SLS powder | 0.8 mm | 1.5 mm |

## How thickness is measured

Wall thickness is estimated by shooting inward ray casts from randomly sampled surface points and recording the distance to the opposite face. This is fast and works well for most shell-like geometry; very complex internal structures may give approximate results.

## Common fixes

| Issue | Tool | Steps |
|-------|------|-------|
| Open mesh / holes | Meshmixer | Analysis → Inspector → Auto Repair All |
| Ghost bodies | Meshmixer | Edit → Separate Shells → delete small ones |
| Non-manifold edges | Blender | Edit Mode → Mesh → Cleanup → Merge by Distance |
| Walls too thin | CAD software | Thicken walls to the recommended minimum |

## Requirements

- Python 3.10+
- trimesh ≥ 4.0
- numpy ≥ 1.24
- rtree ≥ 1.0
- cadquery ≥ 2.4 *(optional, for STEP files)*
