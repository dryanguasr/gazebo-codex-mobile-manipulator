#!/usr/bin/env python3
"""Create link-local metre-scale meshes from pinned Poppy Ergo Jr STL files."""

from __future__ import annotations

import hashlib
import json
import platform
from pathlib import Path

import numpy as np
import scipy

from mesh_io import (
    convex_hull_triangles,
    mesh_summary,
    read_stl,
    transform_triangles,
    write_binary_stl,
)


REPOSITORY = 'https://github.com/poppy-project/poppy-ergo-jr'
COMMIT = '97ce599be8c717843c45ebf48341f2ebf8f250b3'
SOURCE_FILES = {
    'base.stl': 'c3150095267a94d0df530167b9bb22d22d00ec74918969fdbf33aa83f77ca63b',
    'disk_support.stl': '45941750fa44a9b625d471589e919e69b5d0e881c435da8f733b7bb797fe8827',
    'horn2horn.stl': 'cc6d40d692c6d12e4ad16f9566cf90b6e401c1a46848564d8dda83072ed75dbb',
    'long_U.stl': '8f6b02d1be0517bb018c28496cc8d4129be730b838f8db11ea8e93f3711fe0fe',
    'short_U.stl': 'aeb94f32e01d08db7a00f137fec3b970a2d7247b4b3447dea5c84f4382c1efd1',
    'side2side.stl': '0527075781cdb4221049aa5d0fef7d218e132986196dadcee7b82862985c4d02',
    'support_camera.stl': 'f26f269ffd4ca92fc5d4615e3f7bdcd20519e8a5909ce94ace47295ecbd72969',
    'tools/gripper-fixation.stl': '52663bd2b410975eaf59751bdbd981dd51ce24b9356cf13acf8508e975b61c4c',
    'tools/gripper-fixed_part.stl': '78d22d532083d36677f54f75575d5bd9a2208222eafda29a12f7cb0144b368a6',
    'tools/gripper-rotative_part.stl': 'c4afe084b66d845d64fa19e9c82c457b171f6f85538500aa532bec783ca1eac6',
}
STEP_FILES = {
    'base.step': 'c6f222b8cb2bd227412fad26ee7f5eeb8a1c56c1555b45d4d3072bd74c164e5a',
    'U_parts.step': '099ea2c65bb806e280d8e2a06b30f002be026a2dc6f72785559c794e6248c056',
    'lateral_parts.step': 'f4a3494e56e9543655446a23b73b3e583146f86940e4dd88ba044188f36d28ed',
    'tools/gripper.step': 'f67540cef54d364f57d19afdf3019e4fe453319d8034a073f4e73564e3c8e9ee',
}
RX_MINUS_90 = np.array(
    [[1.0, 0.0, 0.0], [0.0, 0.0, 1.0], [0.0, -1.0, 0.0]]
)
RZ_MINUS_90 = np.array(
    [[0.0, 1.0, 0.0], [-1.0, 0.0, 0.0], [0.0, 0.0, 1.0]]
)


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def rounded_summary(triangles: np.ndarray) -> dict[str, object]:
    summary = mesh_summary(triangles)
    for key in ('bounds_min', 'bounds_max', 'extents'):
        summary[key] = [round(float(value), 7) for value in summary[key]]
    for key in ('surface_area', 'signed_volume'):
        summary[key] = round(float(summary[key]), 12)
    return summary


def main() -> None:
    root = Path(__file__).resolve().parents[2]
    asset_root = root / 'src/mobile_manipulator/meshes/poppy_ergo_jr'
    source_root = asset_root / 'source/hardware/STL'
    visual_root = asset_root / 'visual'
    collision_root = asset_root / 'collision'

    source_meshes: dict[str, np.ndarray] = {}
    source_report = {}
    for relative, expected in SOURCE_FILES.items():
        path = source_root / relative
        actual = sha256(path)
        if actual != expected:
            raise RuntimeError(
                f'checksum mismatch for {relative}: {actual} != {expected}'
            )
        triangles = read_stl(path)
        source_meshes[relative] = triangles
        source_report[relative] = {
            'sha256': actual,
            'original_path': f'hardware/STL/{relative}',
            'units': 'millimetres',
            'geometry': rounded_summary(triangles),
        }

    step_report = {}
    step_root = asset_root / 'source/hardware/STEP'
    for relative, expected in STEP_FILES.items():
        path = step_root / relative
        actual = sha256(path)
        if actual != expected:
            raise RuntimeError(
                f'checksum mismatch for STEP {relative}: {actual} != {expected}'
            )
        step_report[relative] = {
            'sha256': actual,
            'original_path': f'hardware/STEP/{relative}',
            'declared_units': 'metres',
            'role': 'B-rep audit/reference; the official STL export drives meshes',
        }

    base = transform_triangles(source_meshes['base.stl'], scale=0.001)
    base_bounds = base.reshape((-1, 3))
    base_shift = np.array(
        [
            -(base_bounds[:, 0].min() + base_bounds[:, 0].max()) / 2,
            -(base_bounds[:, 1].min() + base_bounds[:, 1].max()) / 2,
            -base_bounds[:, 2].min(),
        ]
    )
    base += base_shift

    link_1 = transform_triangles(source_meshes['long_U.stl'], scale=0.001)
    link_2 = np.concatenate(
        [
            transform_triangles(source_meshes['horn2horn.stl'], scale=0.001),
            transform_triangles(source_meshes['side2side.stl'], scale=0.001),
        ]
    )
    link_3 = transform_triangles(source_meshes['short_U.stl'], scale=0.001)
    link_4 = link_2.copy()

    fixation = transform_triangles(
        source_meshes['tools/gripper-fixation.stl'],
        scale=0.001,
        rotation=RX_MINUS_90,
    )
    fixed_jaw = transform_triangles(
        source_meshes['tools/gripper-fixed_part.stl'],
        scale=0.001,
        rotation=RZ_MINUS_90,
        translation=np.array([0.0, 0.0, 0.058]),
    )
    link_5 = np.concatenate([fixation, fixed_jaw])
    link_6 = transform_triangles(
        source_meshes['tools/gripper-rotative_part.stl'],
        scale=0.001,
        rotation=RZ_MINUS_90,
    )
    camera_support = transform_triangles(
        source_meshes['support_camera.stl'], scale=0.001
    )

    visuals = {
        'poppy_mount.stl': base,
        'poppy_link_1.stl': link_1,
        'poppy_link_2.stl': link_2,
        'poppy_link_3.stl': link_3,
        'poppy_link_4.stl': link_4,
        'poppy_link_5.stl': link_5,
        'poppy_link_6.stl': link_6,
        'poppy_camera_support.stl': camera_support,
    }
    for name, triangles in visuals.items():
        write_binary_stl(
            visual_root / name,
            triangles,
            f'Poppy Ergo Jr derived visual: {name}; metres',
        )

    collisions = {
        'poppy_mount_convex.stl': convex_hull_triangles(base, voxel_size=0.001),
        'poppy_link_6_convex.stl': convex_hull_triangles(link_6, voxel_size=0.001),
    }
    for name, triangles in collisions.items():
        write_binary_stl(
            collision_root / name,
            triangles,
            f'Poppy Ergo Jr simplified collision: {name}; metres',
        )

    transformations = {
        'poppy_mount.stl': {
            'inputs': ['base.stl'],
            'scale': 0.001,
            'translation_m': base_shift.tolist(),
            'reason': 'convert mm to m, centre XY, and put the mesh bottom at z=0',
        },
        'poppy_link_1.stl': {
            'inputs': ['long_U.stl'],
            'scale': 0.001,
            'reason': 'raw origin already coincides with the first horn axis',
        },
        'poppy_link_2.stl': {
            'inputs': ['horn2horn.stl', 'side2side.stl'],
            'scale': 0.001,
            'reason': 'paired bracket halves share the official assembly frame',
        },
        'poppy_link_3.stl': {
            'inputs': ['short_U.stl'],
            'scale': 0.001,
            'reason': 'preserve the measured 44-54 mm offset from its parent axis',
        },
        'poppy_link_4.stl': {
            'inputs': ['horn2horn.stl', 'side2side.stl'],
            'scale': 0.001,
            'reason': 'same repeated paired bracket as link 2',
        },
        'poppy_link_5.stl': {
            'inputs': [
                'tools/gripper-fixation.stl',
                'tools/gripper-fixed_part.stl',
            ],
            'scale': 0.001,
            'rotations': {
                'gripper-fixation.stl': 'Rx(-90 deg): CAD -Y becomes link +Z',
                'gripper-fixed_part.stl': 'Rz(-90 deg): CAD +Y becomes tool +X',
            },
            'translation_m': {
                'gripper-fixed_part.stl': [0.0, 0.0, 0.058]
            },
            'reason': 'place the fixed jaw on the parent side of motor m6',
        },
        'poppy_link_6.stl': {
            'inputs': ['tools/gripper-rotative_part.stl'],
            'scale': 0.001,
            'rotation': 'Rz(-90 deg): CAD +Y becomes tool +X',
            'reason': 'make the original motor-axis pivot the link frame',
        },
        'poppy_camera_support.stl': {
            'inputs': ['support_camera.stl'],
            'scale': 0.001,
            'reason': 'audited derivative; retained for teaching/reference',
        },
        'poppy_mount_convex.stl': {
            'input': 'visual/poppy_mount.stl',
            'operation': 'SciPy Qhull convex hull after 1 mm vertex voxelization',
        },
        'poppy_link_6_convex.stl': {
            'input': 'visual/poppy_link_6.stl',
            'operation': 'SciPy Qhull convex hull after 1 mm vertex voxelization',
        },
    }

    manifest = {
        'source_repository': REPOSITORY,
        'source_commit': COMMIT,
        'hardware_license': 'CC BY-SA 4.0',
        'pipeline': {
            'script': 'scripts/cad/prepare_poppy_assets.py',
            'python': platform.python_version(),
            'numpy': np.__version__,
            'scipy': scipy.__version__,
            'output_units': 'metres',
            'manual_geometry_edits': False,
        },
        'source': source_report,
        'step_reference': step_report,
        'transformations': transformations,
        'visual': {
            name: rounded_summary(triangles)
            for name, triangles in visuals.items()
        },
        'collision': {
            name: rounded_summary(triangles)
            for name, triangles in collisions.items()
        },
    }
    (asset_root / 'asset_manifest.json').write_text(
        json.dumps(manifest, indent=2, sort_keys=True) + '\n',
        encoding='utf-8',
    )


if __name__ == '__main__':
    main()
