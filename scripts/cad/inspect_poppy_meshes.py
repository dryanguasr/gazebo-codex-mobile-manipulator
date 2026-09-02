#!/usr/bin/env python3
"""Inspect raw Poppy STL geometry without relying on a prebuilt robot model."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from mesh_io import mesh_summary, read_stl


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        'mesh_root',
        type=Path,
        help='Directory containing the official hardware/STL tree',
    )
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()

    report = {
        str(path.relative_to(args.mesh_root)): mesh_summary(read_stl(path))
        for path in sorted(args.mesh_root.rglob('*.stl'))
    }
    rendered = json.dumps(report, indent=2, sort_keys=True) + '\n'
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(rendered, encoding='utf-8')
    print(rendered, end='')


if __name__ == '__main__':
    main()
