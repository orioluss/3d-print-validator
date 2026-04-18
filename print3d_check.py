#!/usr/bin/env python3
"""
print3d_check.py — 3D model validator for printability
Supports: .stl, .stp/.step, .obj, .ply, .3mf, .off

Usage:
    python print3d_check.py model.stl
    python print3d_check.py model.stp --nozzle 0.6
    python print3d_check.py model.stl --tech resin
    python print3d_check.py model.stl --json

Dependencies:
    pip install trimesh numpy cadquery rtree
"""

import sys
import os
import argparse
import time
import json
import numpy as np

# ──────────────────────────────────────────────
# CONFIGURATION
# ──────────────────────────────────────────────
GHOST_BODY_THRESHOLD_MM = 1.0  # Bodies smaller than this are considered "ghost"

# ──────────────────────────────────────────────
# TECHNOLOGY PROFILES
# ──────────────────────────────────────────────
TECH_PROFILES = {
    "fdm": {
        "name":          "FDM (filament)",
        "wall_min_mm":   1.2,
        "wall_ok_mm":    2.0,
        "detail_min_mm": 0.4,
        "overhang_deg":  45,
    },
    "resin": {
        "name":          "Resin (SLA/MSLA)",
        "wall_min_mm":   0.5,
        "wall_ok_mm":    1.0,
        "detail_min_mm": 0.1,
        "overhang_deg":  30,
    },
    "sls": {
        "name":          "SLS (powder)",
        "wall_min_mm":   0.8,
        "wall_ok_mm":    1.5,
        "detail_min_mm": 0.3,
        "overhang_deg":  90,   # no supports needed
    },
}

# ──────────────────────────────────────────────
# ANSI COLOURS
# ──────────────────────────────────────────────
class C:
    RESET  = "\033[0m"
    BOLD   = "\033[1m"
    GREEN  = "\033[92m"
    YELLOW = "\033[93m"
    RED    = "\033[91m"
    CYAN   = "\033[96m"
    GRAY   = "\033[90m"
    WHITE  = "\033[97m"

def ok(txt):   return f"{C.GREEN}✔{C.RESET} {txt}"
def warn(txt): return f"{C.YELLOW}⚠{C.RESET} {txt}"
def err(txt):  return f"{C.RED}✘{C.RESET} {txt}"
def info(txt): return f"{C.CYAN}→{C.RESET} {txt}"

# ──────────────────────────────────────────────
# MODEL LOADING
# ──────────────────────────────────────────────
def load_model(path: str):
    """Load any supported format and return a trimesh mesh."""
    ext = os.path.splitext(path)[1].lower()

    if ext in (".stp", ".step"):
        return _load_step(path)

    import trimesh
    mesh = trimesh.load(path, force="mesh")
    if isinstance(mesh, trimesh.Scene):
        mesh = trimesh.util.concatenate(list(mesh.geometry.values()))
    return mesh


def _load_step(path: str):
    """Convert STEP → mesh via CadQuery."""
    try:
        import cadquery as cq
    except ImportError:
        sys.exit(
            f"{C.RED}ERROR:{C.RESET} cadquery is required to read STEP files.\n"
            "  Install it with:  pip install cadquery"
        )

    import trimesh
    import tempfile

    result = cq.importers.importStep(path)
    bb = result.val().BoundingBox()

    # Auto-detect inch units: STEP files from some CAD tools store geometry
    # in inches while STL exports use mm, producing a ~25.4× size mismatch.
    max_dim = max(bb.xmax - bb.xmin, bb.ymax - bb.ymin, bb.zmax - bb.zmin)
    scale = 1.0 / 25.4 if max_dim > 500 else 1.0
    if scale != 1.0:
        result = cq.Workplane().add(result.val().scale(scale))

    with tempfile.NamedTemporaryFile(suffix=".stl", delete=False) as tmp:
        tmp_path = tmp.name

    try:
        cq.exporters.export(
            result, tmp_path,
            exportType="STL",
            tolerance=0.05,
            angularTolerance=0.5,
        )
        mesh = trimesh.load(tmp_path, force="mesh")
    finally:
        try:
            os.unlink(tmp_path)
        except FileNotFoundError:
            pass  # File already deleted or never created

    return mesh


# ──────────────────────────────────────────────
# ANALYSIS
# ──────────────────────────────────────────────
def analyse(mesh, tech: dict, nozzle_mm: float | None, samples: int = 2000):
    """Run all checks and return a results dict plus the main body mesh."""
    results = {}

    # ── Dimensions ──
    results["dims_mm"]    = mesh.extents.tolist()
    results["volume_mm3"] = float(mesh.volume) if mesh.is_volume else None
    results["area_mm2"]   = float(mesh.area)
    results["faces"]      = len(mesh.faces)
    results["vertices"]   = len(mesh.vertices)

    # ── Separate bodies ──
    bodies      = mesh.split(only_watertight=False)
    main        = max(bodies, key=lambda b: len(b.faces))
    ghost_bodies = [b for b in bodies if b is not main and b.extents.max() < GHOST_BODY_THRESHOLD_MM]
    extra_bodies = [b for b in bodies if b is not main and b.extents.max() >= GHOST_BODY_THRESHOLD_MM]

    results["num_bodies"]        = len(bodies)
    results["ghost_bodies"]      = len(ghost_bodies)
    results["extra_real_bodies"] = len(extra_bodies)

    # ── Mesh integrity ──
    results["is_watertight"]      = bool(main.is_watertight)
    results["normals_ok"]         = bool(main.is_winding_consistent)

    edge_counts = np.bincount(main.edges_unique_inverse)
    results["non_manifold_edges"] = int(np.sum(edge_counts > 2))
    results["open_edges"]         = int(np.sum(edge_counts == 1))

    # ── Degenerate faces ──
    results["degenerate_faces"] = int((main.area_faces < 1e-6).sum())

    # ── Wall thickness ──
    results["thickness"] = _measure_thickness(main, samples)

    # ── Score ──
    results["score"] = _score(results, tech)

    return results, main


def _measure_thickness(mesh, n_samples: int) -> dict:
    """Estimate wall thickness via inward ray casting."""
    import trimesh
    try:
        points, face_idx = trimesh.sample.sample_surface(mesh, n_samples)
        normals = mesh.face_normals[face_idx]

        origins = points - normals * 0.001
        locs, ray_idx, _ = mesh.ray.intersects_location(
            ray_origins=origins,
            ray_directions=-normals,
            multiple_hits=False,
        )

        if len(locs) == 0:
            return {}

        raw = np.linalg.norm(locs - origins[ray_idx], axis=1)
        t = raw[raw > 0.05]   # discard near-zero hits (self-intersection)

        if len(t) == 0:
            return {}

        return {
            "min_mm":    float(t.min()),
            "mean_mm":   float(t.mean()),
            "median_mm": float(np.median(t)),
            "max_mm":    float(t.max()),
            "p10_mm":    float(np.percentile(t, 10)),
            "samples":   int(len(t)),
        }
    except (ValueError, RuntimeError, ImportError) as e:
        return {}


def _score(r: dict, tech: dict) -> int:
    """Compute a 0–100 printability score."""
    score = 100
    pmin  = tech["wall_min_mm"]

    # Mesh integrity penalties
    if not r["is_watertight"]:          score -= 20
    if not r["normals_ok"]:             score -= 10
    if r["non_manifold_edges"] > 0:     score -= min(15, r["non_manifold_edges"] * 2)
    if r["open_edges"] > 0:             score -= min(10, r["open_edges"])
    if r["ghost_bodies"] > 0:           score -= min(10, r["ghost_bodies"] * 3)
    if r["degenerate_faces"] > 0:       score -= min(5,  r["degenerate_faces"])

    # Wall thickness penalties
    t = r.get("thickness", {})
    if t:
        med = t.get("median_mm", pmin)
        if med < pmin * 0.5:    score -= 30
        elif med < pmin:        score -= 15
        elif med < pmin * 1.5:  score -= 5

    return max(0, score)


# ──────────────────────────────────────────────
# OUTPUT FORMATTING
# ──────────────────────────────────────────────
def print_report(path: str, r: dict, tech: dict, nozzle: float | None, elapsed: float):
    score = r["score"]

    # Header
    print()
    print(f"{C.BOLD}{'─'*58}{C.RESET}")
    print(f"{C.BOLD}  🖨  3D Print Validator{C.RESET}")
    print(f"{'─'*58}")
    print(f"  File      : {C.WHITE}{os.path.basename(path)}{C.RESET}")
    print(f"  Technology: {tech['name']}")
    if nozzle:
        print(f"  Nozzle    : {nozzle} mm")
    print(f"  Analysis  : {elapsed:.1f}s")
    print(f"{'─'*58}")

    # Score bar
    bar_len   = 30
    filled    = round(score / 100 * bar_len)
    bar_col   = C.GREEN if score >= 75 else (C.YELLOW if score >= 45 else C.RED)
    bar       = f"{bar_col}{'█' * filled}{C.GRAY}{'░' * (bar_len - filled)}{C.RESET}"
    verdict   = ("✔ PRINTABLE" if score >= 75
                 else "⚠ PRINTABLE WITH CAUTION" if score >= 45
                 else "✘ NOT PRINTABLE")
    vcol      = C.GREEN if score >= 75 else (C.YELLOW if score >= 45 else C.RED)
    print(f"\n  {bar}  {C.BOLD}{vcol}{score}/100{C.RESET}")
    print(f"  {C.BOLD}{vcol}{verdict}{C.RESET}\n")

    # ── Dimensions ──
    _section("Dimensions")
    d = r["dims_mm"]
    print(f"  X: {d[0]:.2f} mm  Y: {d[1]:.2f} mm  Z: {d[2]:.2f} mm")
    if r["volume_mm3"]:
        print(f"  Volume: {r['volume_mm3']:.1f} mm³  |  Surface area: {r['area_mm2']:.1f} mm²")
    print(f"  Faces: {r['faces']:,}  |  Vertices: {r['vertices']:,}")

    # ── Mesh integrity ──
    _section("Mesh Integrity")
    print(f"  {ok('Watertight (closed mesh)') if r['is_watertight'] else err('Open mesh — slicer may fail or produce holes')}")
    print(f"  {ok('Consistent normals') if r['normals_ok'] else warn('Inconsistent normals — possible face inversions')}")

    nm = r["non_manifold_edges"]
    print(f"  {ok('No non-manifold edges') if nm == 0 else warn(f'{nm} non-manifold edge(s) — ambiguous geometry')}")

    oe = r["open_edges"]
    if oe > 0:
        print(f"  {warn(f'{oe} open edge(s) — holes in the mesh')}")

    gb = r["ghost_bodies"]
    if gb > 0:
        print(f"  {warn(f'{gb} ghost body/bodies (< 1 mm) — remove before slicing')}")

    eb = r["extra_real_bodies"]
    if eb > 0:
        print(f"  {info(f'{eb} additional body/bodies (≥ 1 mm) — check if intentional')}")

    dg = r["degenerate_faces"]
    if dg > 0:
        print(f"  {warn(f'{dg} degenerate face(s) (zero area)')}")

    # ── Wall thickness ──
    _section("Wall Thickness")
    t    = r.get("thickness", {})
    pmin = tech["wall_min_mm"]
    pok  = tech["wall_ok_mm"]

    if not t:
        print(f"  {warn('Could not measure thickness (geometry too simple or open mesh)')}")
    else:
        print(f"  Minimum : {_thick_line(t['min_mm'],    pmin, pok)}")
        print(f"  Median  : {_thick_line(t['median_mm'], pmin, pok)}")
        print(f"  Mean    : {_thick_line(t['mean_mm'],   pmin, pok)}")
        print(f"  Maximum : {t['max_mm']:.3f} mm")
        print(f"  Minimum threshold ({tech['name']}): {pmin} mm")

        med = t["median_mm"]
        if med < pmin:
            print(f"\n  {err('Most walls are below the minimum printable thickness.')}")
            print(f"  {info('The slicer will skip or merge thin zones → gaps in the print.')}")
        elif med < pok:
            print(f"\n  {warn('Some walls may be too thin. Verify the result in your slicer.')}")
        else:
            print(f"\n  {ok('Wall thickness is adequate for printing.')}")

    # ── Recommendations ──
    _section("Recommendations")
    _recommendations(r, tech, nozzle)

    print(f"\n{'─'*58}\n")


def _thick_line(val: float, pmin: float, pok: float) -> str:
    col = C.GREEN if val >= pok else (C.YELLOW if val >= pmin else C.RED)
    tag = "" if val >= pok else (" ← BORDERLINE" if val >= pmin else " ← CRITICAL")
    return f"{col}{val:.3f} mm{C.RESET}{C.GRAY}{tag}{C.RESET}"


def _section(title: str):
    print(f"\n  {C.BOLD}{C.CYAN}{title}{C.RESET}")
    print(f"  {'·'*40}")


def _recommendations(r: dict, tech: dict, nozzle: float | None):
    recs = []

    if not r["is_watertight"] or r["open_edges"] > 0:
        recs.append(("✘", "Fix the mesh: Meshmixer → Analysis → Inspector → Auto Repair All"))

    if r["ghost_bodies"] > 0:
        recs.append(("✘", "Remove ghost bodies: Meshmixer → Separate Shells → delete the tiny ones"))

    t    = r.get("thickness", {})
    pmin = tech["wall_min_mm"]
    if t and t.get("median_mm", 99) < pmin:
        recs.append(("✘", f"Thicken walls in your CAD to ≥ {pmin} mm"))
        recs.append(("→", "Alternative: switch to resin printing (SLA) for small, detailed parts"))
        recs.append(("→", "Alternative: scale the model up (×2 or more) if dimensions allow"))

    if r["non_manifold_edges"] > 0:
        recs.append(("⚠", "Non-manifold edges: import into Blender → Mesh → Cleanup → Merge by Distance"))

    if not recs:
        recs.append(("✔", "Model looks good. Always do a final check in your slicer."))

    for symbol, text in recs:
        col = C.GREEN if symbol == "✔" else (C.RED if symbol == "✘" else C.CYAN)
        print(f"  {col}{symbol}{C.RESET} {text}")


# ──────────────────────────────────────────────
# ENTRY POINT
# ──────────────────────────────────────────────
def main():
    parser = argparse.ArgumentParser(
        description="Validate whether a 3D model is suitable for printing.",
        formatter_class=argparse.RawTextHelpFormatter,
    )
    parser.add_argument(
        "file",
        help="Path to the 3D model (.stl, .stp, .step, .obj, .ply, .3mf, .off)",
    )
    parser.add_argument(
        "--tech",
        choices=TECH_PROFILES.keys(),
        default="fdm",
        help="Printing technology: fdm (default), resin, sls",
    )
    parser.add_argument(
        "--nozzle",
        type=float,
        default=None,
        help="Nozzle diameter in mm (default: 0.4 for FDM)",
    )
    parser.add_argument(
        "--samples",
        type=int,
        default=2000,
        help="Number of ray-cast samples for thickness measurement (default: 2000)",
    )
    parser.add_argument(
        "--json",
        action="store_true",
        help="Output results as JSON (useful for scripting or CI integration)",
    )
    args = parser.parse_args()

    if not os.path.exists(args.file):
        sys.exit(f"{C.RED}ERROR:{C.RESET} File not found: {args.file}")

    if args.samples <= 0 or args.samples > 50000:
        sys.exit(f"{C.RED}ERROR:{C.RESET} Samples must be between 1 and 50000")

    tech   = TECH_PROFILES[args.tech]
    nozzle = args.nozzle or (0.4 if args.tech == "fdm" else None)

    print(f"\n{C.GRAY}Loading model...{C.RESET}", end="", flush=True)
    t0 = time.time()

    try:
        mesh = load_model(args.file)
    except (ValueError, RuntimeError, ImportError, OSError) as e:
        sys.exit(f"\n{C.RED}ERROR loading file:{C.RESET} {e}")

    print(f"\r{C.GRAY}Analysing...    {C.RESET}", end="", flush=True)

    try:
        results, _ = analyse(mesh, tech, nozzle, samples=args.samples)
    except (ValueError, RuntimeError, ImportError) as e:
        sys.exit(f"\n{C.RED}ERROR during analysis:{C.RESET} {e}")

    elapsed = time.time() - t0
    print(f"\r{' ' * 30}\r", end="")

    if args.json:
        results["file"]       = args.file
        results["technology"] = args.tech
        results["time_s"]     = round(elapsed, 2)
        print(json.dumps(results, indent=2, ensure_ascii=False))
    else:
        print_report(args.file, results, tech, nozzle, elapsed)


if __name__ == "__main__":
    main()
