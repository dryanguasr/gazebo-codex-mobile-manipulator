#!/usr/bin/env python3
"""Convert a real Poppy STEP assembly to metre-scale STL with Gmsh/OpenCASCADE.

The official STL files are deliberately loaded only *after* STEP tessellation,
as a validation reference. They are never conversion inputs.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import shutil
import subprocess
import tempfile
from pathlib import Path

import numpy as np

from mesh_io import mesh_summary, read_stl, transform_triangles, write_binary_stl


STEP_RELATIVE = Path("source/hardware/STEP/base.step")
REFERENCE_STLS = (
    Path("source/hardware/STL/base.stl"),
    Path("source/hardware/STL/disk_support.stl"),
    Path("source/hardware/STL/support_camera.stl"),
)
VARIANTS = {"coarse": 1.0, "fine": 0.5}
SCALE_TO_METRES = 0.001
BOUNDS_TOLERANCE_M = 0.001
VOLUME_RELATIVE_TOLERANCE = 0.02


def digest(path: Path) -> str:
    value = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            value.update(block)
    return value.hexdigest()


def rounded(summary: dict[str, object]) -> dict[str, object]:
    result = dict(summary)
    for key in ("bounds_min", "bounds_max", "extents"):
        result[key] = [round(float(value), 9) for value in result[key]]
    for key in ("surface_area", "signed_volume"):
        result[key] = round(float(result[key]), 12)
    return result


def require_gmsh(candidate: str | None) -> str:
    executable = candidate or shutil.which("gmsh")
    if not executable:
        raise RuntimeError(
            "Gmsh is required for CASE B. Install it with: sudo apt install gmsh"
        )
    return executable


def gmsh_version(executable: str) -> str:
    completed = subprocess.run(
        [executable, "-version"], check=True, capture_output=True, text=True
    )
    return completed.stdout.strip()


def compare(step_mesh: np.ndarray, reference: np.ndarray) -> dict[str, object]:
    step_summary = mesh_summary(step_mesh)
    reference_summary = mesh_summary(reference)
    bounds_error = np.maximum(
        np.abs(np.asarray(step_summary["bounds_min"]) - np.asarray(reference_summary["bounds_min"])),
        np.abs(np.asarray(step_summary["bounds_max"]) - np.asarray(reference_summary["bounds_max"])),
    )
    extent_error = np.abs(
        np.asarray(step_summary["extents"]) - np.asarray(reference_summary["extents"])
    )
    reference_volume = abs(float(reference_summary["signed_volume"]))
    volume_error = abs(
        abs(float(step_summary["signed_volume"])) - reference_volume
    ) / reference_volume
    same_orientation = (
        float(step_summary["signed_volume"]) * float(reference_summary["signed_volume"])
        > 0
    )
    passed = (
        max(bounds_error) <= BOUNDS_TOLERANCE_M
        and max(extent_error) <= BOUNDS_TOLERANCE_M
        and volume_error <= VOLUME_RELATIVE_TOLERANCE
        and same_orientation
        and step_summary["triangles"] > 0
    )
    return {
        "status": "PASS" if passed else "FAIL",
        "step_mesh": rounded(step_summary),
        "official_stl_reference": rounded(reference_summary),
        "max_bounds_error_m": round(float(max(bounds_error)), 9),
        "max_extent_error_m": round(float(max(extent_error)), 9),
        "volume_relative_error": round(float(volume_error), 9),
        "orientation_consistent": same_orientation,
        "tolerances": {
            "bounds_and_extents_m": BOUNDS_TOLERANCE_M,
            "volume_relative": VOLUME_RELATIVE_TOLERANCE,
        },
    }


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Tessellate the Poppy base STEP assembly and validate it."
    )
    parser.add_argument("--gmsh", help="Gmsh executable; defaults to PATH")
    parser.add_argument("--asset-root", type=Path)
    parser.add_argument("--results", type=Path)
    args = parser.parse_args()

    root = Path(__file__).resolve().parents[2]
    asset_root = args.asset_root or root / "src/mobile_manipulator/meshes/poppy_ergo_jr"
    results = args.results or root / "results/verified/cad_step_conversion/summary.json"
    step_path = asset_root / STEP_RELATIVE
    output_root = asset_root / "source/derived_step"
    executable = require_gmsh(args.gmsh)
    if not step_path.is_file():
        raise RuntimeError(f"STEP input is missing: {step_path}")

    # This is the only conversion input. Reference STL is read afterwards.
    reference = np.concatenate(
        [
            transform_triangles(read_stl(asset_root / relative), scale=SCALE_TO_METRES)
            for relative in REFERENCE_STLS
        ]
    )
    variants: dict[str, object] = {}
    output_root.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix="poppy_step_tessellation_") as temporary:
        temporary_root = Path(temporary)
        for name, scale in VARIANTS.items():
            raw_output = temporary_root / f"base_step_{name}_raw.stl"
            command = [
                executable,
                str(step_path),
                "-2",
                "-clscale",
                str(scale),
                "-format",
                "stl",
                "-o",
                str(raw_output),
                "-v",
                "2",
            ]
            completed = subprocess.run(
                command, capture_output=True, text=True, check=False
            )
            if completed.returncode or not raw_output.is_file():
                raise RuntimeError(
                    f"Gmsh failed for {name}:\n{completed.stdout}\n{completed.stderr}"
                )
            converted = transform_triangles(read_stl(raw_output), scale=SCALE_TO_METRES)
            output = output_root / f"base_step_gmsh_{name}.stl"
            write_binary_stl(
                output,
                converted,
                f"Poppy base STEP tessellation via Gmsh; {name}; metres",
            )
            report = compare(converted, reference)
            report.update(
                {
                    "mesh_path": str(output.relative_to(asset_root)),
                    "mesh_sha256": digest(output),
                    "mesh_bytes": output.stat().st_size,
                    "gmsh_characteristic_length_factor": scale,
                    "gmsh_warnings": [
                        line for line in completed.stderr.splitlines()
                        if "Warning" in line
                    ],
                }
            )
            variants[name] = report

    passed = variants['fine']['status'] == 'PASS'
    summary = {
        "status": "PASS" if passed else "FAIL",
        "case": "B: STEP/B-rep without a prepared mesh",
        "conversion_input_only": str(STEP_RELATIVE),
        "official_stl_role": "validation reference only; never passed to Gmsh",
        "source_step_sha256": digest(step_path),
        "source_step_declared_unit": "metre (AP214 SI_UNIT in source)",
        "observed_gmsh_coordinate_scale": "millimetre-like numeric coordinates",
        "applied_output_scale_to_metres": SCALE_TO_METRES,
        "tool": {"name": "Gmsh with OpenCASCADE", "version": gmsh_version(executable)},
        "format": "binary STL",
        "variants": variants,
        "selected_for_teaching": "fine",
        "explanation": (
            "Lower characteristic-length factor yields more triangles and more "
            "accurate bounds, at greater meshing/runtime cost."
        ),
    }
    results.parent.mkdir(parents=True, exist_ok=True)
    results.write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    if not passed:
        raise RuntimeError(f"STEP conversion validation failed; see {results}")
    print(f"STEP conversion passed; wrote {results}")


if __name__ == "__main__":
    main()
