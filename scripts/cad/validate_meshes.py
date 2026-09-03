#!/usr/bin/env python3
"""Validate source provenance, derived meshes, mesh paths, and inertias."""

from __future__ import annotations

import hashlib
import json
import xml.etree.ElementTree as ET
from pathlib import Path

from mesh_io import mesh_summary, read_stl


def require(condition: bool, message: str) -> None:
    if not condition:
        raise RuntimeError(message)


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def main() -> None:
    root = Path(__file__).resolve().parents[2]
    asset_root = root / 'src/mobile_manipulator/meshes/poppy_ergo_jr'
    manifest = json.loads(
        (asset_root / 'asset_manifest.json').read_text(encoding='utf-8')
    )
    source_root = asset_root / 'source/hardware/STL'

    for relative, metadata in manifest['source'].items():
        path = source_root / relative
        require(path.is_file(), f'missing pinned source: {relative}')
        require(
            sha256(path) == metadata['sha256'],
            f'source checksum changed: {relative}',
        )

    step_root = asset_root / 'source/hardware/STEP'
    for relative, metadata in manifest['step_reference'].items():
        path = step_root / relative
        require(path.is_file(), f'missing pinned STEP source: {relative}')
        require(
            sha256(path) == metadata['sha256'],
            f'STEP source checksum changed: {relative}',
        )

    step_results_path = root / 'results/verified/cad_step_conversion/summary.json'
    require(step_results_path.is_file(), 'missing STEP conversion summary; run convert_step_example.py')
    step_results = json.loads(step_results_path.read_text(encoding='utf-8'))
    require(step_results.get('status') == 'PASS', 'STEP conversion summary is not PASS')
    require(
        step_results.get('official_stl_role') == 'validation reference only; never passed to Gmsh',
        'STEP conversion must not use official STL as conversion input',
    )
    selected = step_results.get('selected_for_teaching')
    variant = step_results.get('variants', {}).get(selected)
    require(variant is not None, 'STEP conversion selected variant is missing')
    require(variant.get('status') == 'PASS', 'selected STEP tessellation failed comparison')
    converted_path = asset_root / variant['mesh_path']
    require(converted_path.is_file(), f'missing STEP-derived mesh: {converted_path}')
    converted = mesh_summary(read_stl(converted_path))
    require(
        converted['triangles'] == variant['step_mesh']['triangles'],
        'STEP-derived mesh triangle count differs from summary',
    )
    require(
        0.005 < max(converted['extents']) < 0.20,
        'STEP-derived mesh has implausible scale; verify source units and 0.001 conversion',
    )
    require(variant['orientation_consistent'], 'STEP-derived mesh orientation differs from STL reference')
    require(
        variant['max_extent_error_m'] <= variant['tolerances']['bounds_and_extents_m'],
        'STEP-derived mesh dimensions exceed declared tolerance',
    )

    for category in ('visual', 'collision'):
        for name, expected in manifest[category].items():
            path = asset_root / category / name
            require(path.is_file(), f'missing {category} mesh: {name}')
            actual = mesh_summary(read_stl(path))
            require(
                actual['triangles'] == expected['triangles'],
                f'triangle count changed for {category}/{name}',
            )
            require(
                max(actual['extents']) < 0.20,
                f'{category}/{name} still appears to use millimetres',
            )
            require(
                max(actual['extents']) > 0.005,
                f'{category}/{name} is implausibly small',
            )

    visual_triangles = manifest['visual']['poppy_link_6.stl']['triangles']
    collision = manifest['collision']['poppy_link_6_convex.stl']
    require(
        collision['triangles'] < visual_triangles * 0.05,
        'link 6 collision mesh was not substantially simplified',
    )
    require(collision['watertight'], 'link 6 collision hull is not watertight')

    xacro = root / 'src/mobile_manipulator/urdf/mobile_manipulator.urdf.xacro'
    tree = ET.parse(xacro)
    collision_kinds = [
        list(geometry)[0].tag
        for geometry in tree.findall('.//collision/geometry')
        if list(geometry)
    ]
    require('box' in collision_kinds, 'collision strategy regression: no primitive boxes')
    require('mesh' in collision_kinds, 'collision strategy regression: no convex mesh')
    xacro_prefix = 'file://$(find mobile_manipulator)/'
    package_prefix = 'package://mobile_manipulator/'
    for mesh in tree.findall('.//mesh'):
        uri = mesh.attrib['filename']
        if uri.startswith(xacro_prefix):
            relative = uri.removeprefix(xacro_prefix)
        elif uri.startswith(package_prefix):
            relative = uri.removeprefix(package_prefix)
        else:
            raise RuntimeError(f'unexpected mesh URI scheme: {uri}')
        require(
            (root / 'src/mobile_manipulator' / relative).is_file(),
            f'Xacro mesh does not exist: {uri}',
        )

    for link in tree.findall('.//link'):
        inertial = link.find('inertial')
        if inertial is None:
            continue
        mass_text = inertial.find('mass').attrib['value']
        if not mass_text[0].isdigit():
            continue
        mass = float(mass_text)
        inertia = inertial.find('inertia').attrib
        ixx, iyy, izz = (
            float(inertia['ixx']),
            float(inertia['iyy']),
            float(inertia['izz']),
        )
        require(mass > 0, f'non-positive mass on {link.attrib.get("name")}')
        require(
            ixx > 0 and iyy > 0 and izz > 0,
            f'non-positive principal inertia on {link.attrib.get("name")}',
        )
        require(
            ixx + iyy >= izz
            and ixx + izz >= iyy
            and iyy + izz >= ixx,
            f'invalid inertia triangle on {link.attrib.get("name")}',
        )

    ratio = collision['triangles'] / visual_triangles
    print(
        'CAD mesh validation passed: '
        f'link 6 collision/visual triangle ratio={ratio:.4f}'
    )


if __name__ == '__main__':
    main()
