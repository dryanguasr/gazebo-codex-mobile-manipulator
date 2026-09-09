#!/usr/bin/env python3
"""Compare every Gazebo-loaded triangle vertex against evaluated official CAD."""
import argparse
import json
from pathlib import Path
import shlex
import subprocess
import tempfile

from build_gazebo_visuals import ASSETS, LINK_COLORS, REPO
from collada_io import read_collada_instances
import numpy as np


def audit(visual_dir):
    flags = shlex.split(subprocess.check_output(
        ['pkg-config', '--cflags', '--libs', 'gz-common5-graphics'], text=True
    ))
    results = []
    with tempfile.TemporaryDirectory(prefix='poppy-runtime-audit-') as temporary:
        work = Path(temporary)
        reader = work / 'reader'
        subprocess.run(
            ['g++', str(REPO / 'tools/audit_runtime_mesh.cc'), '-o', str(reader)] + flags,
            check=True,
        )
        paths = [visual_dir / (name + '.dae') for name in LINK_COLORS]
        loaded = json.loads(subprocess.check_output(
            [str(reader), '--dump', str(work / 'vertices')] + [str(p) for p in paths],
            text=True,
        ))
        for mesh in loaded['meshes']:
            source = ASSETS / 'official' / Path(mesh['path']).name
            expected = read_collada_instances(source)
            submeshes = mesh['submeshes']
            errors = []
            records = []
            if len(expected) != len(submeshes):
                errors.append('instance_count')
            for original, native in zip(expected, submeshes):
                data = np.fromfile(
                    work / 'vertices' / native['dump'], dtype=np.float64
                ).reshape(-1, 3, 3)
                maximum = None
                if data.shape != original.triangles.shape:
                    errors.append(native['name'] + ': triangle_count')
                else:
                    maximum = float(np.max(np.abs(data - original.triangles)))
                    if maximum > 1e-10:
                        errors.append(native['name'] + ': vertex_position')
                if native['normal_count'] != native['vertex_count']:
                    errors.append(native['name'] + ': missing_normals')
                records.append({
                    key: value for key, value in {
                        **native, 'source_ancestry': original.node_id,
                        'maximum_vertex_error_m': maximum,
                    }.items() if key != 'dump'
                })
            results.append({
                'mesh': Path(mesh['path']).name,
                'status': 'PASS' if not errors else 'FAIL',
                'errors': errors, 'instances': records,
            })
    return {
        'reader': 'gz::common::MeshManager (installed gz-common5-graphics)',
        'comparison': (
            'every triangle vertex versus transformed official source; tolerance 1e-10 m'
        ),
        'visual_dir': str(visual_dir),
        'status': 'PASS' if all(r['status'] == 'PASS' for r in results) else 'FAIL',
        'meshes': results,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--visual-dir', type=Path, default=ASSETS / 'gazebo_inspection')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    result = audit(args.visual_dir)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2) + '\n')
    print(f"Gazebo runtime vertex audit {result['status']}: {len(result['meshes'])} meshes")
    return 0 if result['status'] == 'PASS' else 1


if __name__ == '__main__':
    raise SystemExit(main())
