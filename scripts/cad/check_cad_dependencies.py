#!/usr/bin/env python3
"""Report the declared dependencies of the reproducible CAD pipeline."""

from __future__ import annotations

import importlib
import shutil


def main() -> None:
    missing = []
    for module, package in (
        ("numpy", "python3-numpy"),
        ("scipy", "python3-scipy"),
    ):
        try:
            loaded = importlib.import_module(module)
            print(f"PASS  Python module {module} {loaded.__version__}")
        except ImportError:
            missing.append(package)
            print(f"MISSING  Python module {module}; install: sudo apt install {package}")

    gmsh = shutil.which("gmsh")
    if gmsh:
        print(f"PASS  STEP tessellator: {gmsh}")
    else:
        print("OPTIONAL FOR CASE A / REQUIRED FOR CASE B  gmsh; install: sudo apt install gmsh")

    if missing:
        raise SystemExit("CAD preflight failed: install the missing required packages above")
    print("CAD preflight passed. Case B additionally requires gmsh.")
