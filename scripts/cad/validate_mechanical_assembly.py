#!/usr/bin/env python3
"""Validate the Poppy mechanical chain beyond URDF syntax and joint feedback."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[2]
XACRO_PATH = (
    REPO_ROOT
    / 'src'
    / 'mobile_manipulator'
    / 'urdf'
    / 'mobile_manipulator.urdf.xacro'
)
RESULT_PATH = (
    REPO_ROOT
    / 'results'
    / 'verified'
    / 'mechanical_assembly'
    / 'summary.json'
)
OFFICIAL_URDF_COMMIT = '7eb32bd385afa11dea5e6a6b6a4a86a0243aaa2b'
OFFICIAL_URDF_URL = 'https://github.com/poppy-project/poppy_ergo_jr_description'
ASSEMBLY_GUIDE_URL = (
    'https://docs.poppy-project.org/en/assembly-guides/ergo-jr/'
    'mechanical-construction'
)
TOLERANCE = 1.0e-9

EXPECTED_JOINTS = {
    'poppy_m1_joint': {
        'label': 'm1',
        'parent': 'poppy_mount_link',
        'child': 'poppy_link_1',
        'xyz': [0.0, 0.0, 0.0327993216120967],
        'rpy': [0.0, 0.0, 0.0],
        'axis': [0.0, 0.0, 1.0],
    },
    'poppy_m2_joint': {
        'label': 'm2',
        'parent': 'poppy_link_1',
        'child': 'poppy_link_2',
        'xyz': [0.0, 0.0, 0.0240006783879033],
        'rpy': [0.0, -math.pi / 2.0, 0.0],
        'axis': [0.0, 0.0, -1.0],
    },
    'poppy_m3_joint': {
        'label': 'm3',
        'parent': 'poppy_link_2',
        'child': 'poppy_link_3',
        'xyz': [0.054, 0.0, 0.0],
        'rpy': [0.0, 0.0, 0.0],
        'axis': [0.0, 0.0, -1.0],
    },
    'poppy_m4_joint': {
        'label': 'm4',
        'parent': 'poppy_link_3',
        'child': 'poppy_link_4',
        'xyz': [0.045, 0.0, 0.0],
        'rpy': [0.0, -math.pi / 2.0, 0.0],
        'axis': [0.0, 0.0, -1.0],
    },
    'poppy_m5_joint': {
        'label': 'm5',
        'parent': 'poppy_link_4',
        'child': 'poppy_link_5',
        'xyz': [0.0, -0.048, 0.0],
        'rpy': [0.0, -math.pi / 2.0, 0.0],
        'axis': [0.0, 0.0, 1.0],
    },
    'poppy_m6_joint': {
        'label': 'm6',
        'parent': 'poppy_link_5',
        'child': 'poppy_link_6',
        'xyz': [0.0, -0.058, 0.0],
        'rpy': [0.0, -math.pi / 2.0, 0.0],
        'axis': [0.0, 0.0, -1.0],
    },
}

POSES = {
    'home': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    'pose_1': [0.35, -0.45, 0.40, -0.35, 0.25, 0.15],
    'pose_2': [-0.45, 0.35, -0.30, 0.45, -0.35, 0.75],
}


def values(element: ET.Element | None, name: str, default: list[float]) -> list[float]:
    if element is None or name not in element.attrib:
        return default
    return [float(value) for value in element.attrib[name].split()]


def rpy_matrix(rpy: list[float]) -> np.ndarray:
    roll, pitch, yaw = rpy
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]], dtype=float)
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]], dtype=float)
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]], dtype=float)
    return rz @ ry @ rx


def origin_transform(xyz: list[float], rpy: list[float]) -> np.ndarray:
    transform = np.eye(4)
    transform[:3, :3] = rpy_matrix(rpy)
    transform[:3, 3] = xyz
    return transform


def axis_rotation(axis: list[float], angle: float) -> np.ndarray:
    vector = np.asarray(axis, dtype=float)
    vector /= np.linalg.norm(vector)
    x, y, z = vector
    c, s = math.cos(angle), math.sin(angle)
    cross = 1.0 - c
    rotation = np.array(
        [
            [c + x * x * cross, x * y * cross - z * s, x * z * cross + y * s],
            [y * x * cross + z * s, c + y * y * cross, y * z * cross - x * s],
            [z * x * cross - y * s, z * y * cross + x * s, c + z * z * cross],
        ]
    )
    transform = np.eye(4)
    transform[:3, :3] = rotation
    return transform


def quaternion_xyzw(rotation: np.ndarray) -> list[float]:
    trace = float(np.trace(rotation))
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        q = [
            (rotation[2, 1] - rotation[1, 2]) / s,
            (rotation[0, 2] - rotation[2, 0]) / s,
            (rotation[1, 0] - rotation[0, 1]) / s,
            0.25 * s,
        ]
    else:
        diagonal = np.diag(rotation)
        index = int(np.argmax(diagonal))
        if index == 0:
            s = math.sqrt(1.0 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2]) * 2
            q = [
                0.25 * s,
                (rotation[0, 1] + rotation[1, 0]) / s,
                (rotation[0, 2] + rotation[2, 0]) / s,
                (rotation[2, 1] - rotation[1, 2]) / s,
            ]
        elif index == 1:
            s = math.sqrt(1.0 + rotation[1, 1] - rotation[0, 0] - rotation[2, 2]) * 2
            q = [
                (rotation[0, 1] + rotation[1, 0]) / s,
                0.25 * s,
                (rotation[1, 2] + rotation[2, 1]) / s,
                (rotation[0, 2] - rotation[2, 0]) / s,
            ]
        else:
            s = math.sqrt(1.0 + rotation[2, 2] - rotation[0, 0] - rotation[1, 1]) * 2
            q = [
                (rotation[0, 2] + rotation[2, 0]) / s,
                (rotation[1, 2] + rotation[2, 1]) / s,
                0.25 * s,
                (rotation[1, 0] - rotation[0, 1]) / s,
            ]
    q_array = np.asarray(q)
    q_array /= np.linalg.norm(q_array)
    return q_array.round(9).tolist()


def close(actual: list[float], expected: list[float]) -> bool:
    return bool(np.allclose(actual, expected, atol=TOLERANCE, rtol=0.0))


def resolve_mesh_uri(uri: str) -> Path | None:
    prefix = 'file://$(find mobile_manipulator)/'
    if not uri.startswith(prefix):
        return None
    return REPO_ROOT / 'src' / 'mobile_manipulator' / uri[len(prefix):]


def fk(joints: dict[str, ET.Element], positions: list[float]) -> np.ndarray:
    transform = np.eye(4)
    for (joint_name, expected), position in zip(EXPECTED_JOINTS.items(), positions):
        joint = joints[joint_name]
        origin = joint.find('origin')
        xyz = values(origin, 'xyz', [0.0, 0.0, 0.0])
        rpy = values(origin, 'rpy', [0.0, 0.0, 0.0])
        axis = values(joint.find('axis'), 'xyz', expected['axis'])
        transform = transform @ origin_transform(xyz, rpy) @ axis_rotation(axis, position)
    tip_joint = joints['poppy_moving_tip_joint']
    tip_origin = tip_joint.find('origin')
    return transform @ origin_transform(
        values(tip_origin, 'xyz', [0.0, 0.0, 0.0]),
        values(tip_origin, 'rpy', [0.0, 0.0, 0.0]),
    )


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument('--xacro', type=Path, default=XACRO_PATH)
    parser.add_argument('--output', type=Path, default=RESULT_PATH)
    args = parser.parse_args()

    root = ET.parse(args.xacro).getroot()
    joints = {joint.attrib['name']: joint for joint in root.findall('joint')}
    checks: list[dict[str, object]] = []
    failures: list[str] = []

    joint_audit: list[dict[str, object]] = []
    for name, expected in EXPECTED_JOINTS.items():
        joint = joints.get(name)
        if joint is None:
            failures.append(f'{name}: joint missing')
            continue
        origin = joint.find('origin')
        actual = {
            'parent': joint.find('parent').attrib['link'],
            'child': joint.find('child').attrib['link'],
            'xyz': values(origin, 'xyz', [0.0, 0.0, 0.0]),
            'rpy': values(origin, 'rpy', [0.0, 0.0, 0.0]),
            'axis': values(joint.find('axis'), 'xyz', [1.0, 0.0, 0.0]),
        }
        passed = (
            actual['parent'] == expected['parent']
            and actual['child'] == expected['child']
            and close(actual['xyz'], expected['xyz'])
            and close(actual['rpy'], expected['rpy'])
            and close(actual['axis'], expected['axis'])
        )
        if not passed:
            failures.append(f'{name}: transform/axis differs from audited official chain')
        joint_audit.append(
            {
                'joint': expected['label'],
                'name': name,
                'actual': actual,
                'official': {
                    key: expected[key]
                    for key in ('parent', 'child', 'xyz', 'rpy', 'axis')
                },
                'distance_to_next_axis_m': round(float(np.linalg.norm(expected['xyz'])), 12),
                'status': 'PASS' if passed else 'FAIL',
            }
        )

    expected_meshes = [
        f'poppy_link_{index}.stl' for index in range(1, 7)
    ] + ['poppy_mount.stl']
    source_text = args.xacro.read_text(encoding='utf-8')
    for mesh_name in expected_meshes:
        token = f'/visual/{mesh_name}'
        if token not in source_text:
            failures.append(f'{mesh_name}: visual mesh URI missing')
    if 'scale=' in '\n'.join(
        line for line in source_text.splitlines() if 'poppy_ergo_jr/visual/' in line
    ):
        failures.append('Poppy visual mesh has a URDF scale; physical 1:1 is required')

    broken_uris = []
    for mesh in root.findall('.//mesh'):
        filename = mesh.attrib.get('filename', '')
        resolved = resolve_mesh_uri(filename)
        if resolved is not None and not resolved.is_file():
            broken_uris.append({'uri': filename, 'resolved': str(resolved)})
    if broken_uris:
        failures.append(f'{len(broken_uris)} mesh URI(s) are broken')

    inertia_results = []
    for link in root.findall('link'):
        inertial = link.find('inertial')
        if inertial is None:
            continue
        mass = float(inertial.find('mass').attrib['value'])
        inertia = inertial.find('inertia').attrib
        matrix = np.array(
            [
                [float(inertia['ixx']), float(inertia['ixy']), float(inertia['ixz'])],
                [float(inertia['ixy']), float(inertia['iyy']), float(inertia['iyz'])],
                [float(inertia['ixz']), float(inertia['iyz']), float(inertia['izz'])],
            ]
        )
        eigenvalues = np.linalg.eigvalsh(matrix)
        passed = mass > 0.0 and bool(np.all(eigenvalues > 0.0))
        if not passed:
            failures.append(f"{link.attrib['name']}: non-positive mass or inertia")
        inertia_results.append(
            {
                'link': link.attrib['name'],
                'mass_kg': mass,
                'inertia_eigenvalues': eigenvalues.tolist(),
                'status': 'PASS' if passed else 'FAIL',
            }
        )

    pose_results = {}
    if all(name in joints for name in EXPECTED_JOINTS) and 'poppy_moving_tip_joint' in joints:
        for pose_name, positions in POSES.items():
            transform = fk(joints, positions)
            pose_results[pose_name] = {
                'joint_positions_rad': positions,
                'xyz_m_from_poppy_mount': transform[:3, 3].round(9).tolist(),
                'quaternion_xyzw': quaternion_xyzw(transform[:3, :3]),
            }
        pose_points = np.array(
            [pose_results[name]['xyz_m_from_poppy_mount'] for name in POSES]
        )
        pairwise = [
            float(np.linalg.norm(pose_points[i] - pose_points[j]))
            for i in range(3)
            for j in range(i + 1, 3)
        ]
        distinct = min(pairwise) > 0.02
        if not distinct:
            failures.append('FK landmarks do not move distinctly across the three poses')
        home_reach = float(np.linalg.norm(pose_points[0]))
        if not 0.15 < home_reach < 0.35:
            failures.append(f'home tool reach {home_reach:.3f} m is implausible')
        checks.extend(
            [
                {
                    'name': 'fk_poses_are_distinct',
                    'minimum_pairwise_tip_distance_m': min(pairwise),
                    'status': 'PASS' if distinct else 'FAIL',
                },
                {
                    'name': 'home_reach_is_plausible',
                    'reach_m': home_reach,
                    'accepted_range_m': [0.15, 0.35],
                    'status': 'PASS' if 0.15 < home_reach < 0.35 else 'FAIL',
                },
            ]
        )
    else:
        failures.append('cannot compute FK: one or more chain/tip joints are missing')

    expected_platform = {
        'base_length': 0.40,
        'base_width': 0.30,
        'base_height': 0.10,
        'base_mass': 6.0,
        'base_ixx': 0.050,
        'base_iyy': 0.085,
        'base_izz': 0.125,
        'wheel_radius': 0.070,
        'wheel_width': 0.045,
        'wheel_mass': 0.45,
        'wheel_ixx': 0.000627,
        'wheel_iyy': 0.001103,
        'wheel_izz': 0.000627,
        'wheel_x': 0.140,
        'wheel_y': 0.1725,
        'camera_x': 0.225,
        'camera_z': 0.050,
        'arm_mount_x': -0.030,
        'arm_mount_z': 0.050,
    }
    xacro_namespace = '{http://www.ros.org/wiki/xacro}property'
    properties = {
        item.attrib['name']: float(item.attrib['value'])
        for item in root.findall(xacro_namespace)
        if item.attrib.get('name') in expected_platform
    }
    property_failures = []
    for name, expected_value in expected_platform.items():
        actual_value = properties.get(name)
        if actual_value is None or not math.isclose(
            actual_value, expected_value, abs_tol=1.0e-9, rel_tol=0.0
        ):
            property_failures.append(
                f'{name}: expected {expected_value}, got {actual_value}'
            )
    if property_failures:
        failures.extend(f'compact platform {failure}' for failure in property_failures)

    base_inertia_expected = {
        'base_ixx': expected_platform['base_mass']
        * (expected_platform['base_width'] ** 2 + expected_platform['base_height'] ** 2)
        / 12.0,
        'base_iyy': expected_platform['base_mass']
        * (expected_platform['base_length'] ** 2 + expected_platform['base_height'] ** 2)
        / 12.0,
        'base_izz': expected_platform['base_mass']
        * (expected_platform['base_length'] ** 2 + expected_platform['base_width'] ** 2)
        / 12.0,
    }
    base_inertia_passed = all(
        math.isclose(properties.get(name, math.nan), value, abs_tol=5.0e-7)
        for name, value in base_inertia_expected.items()
    )
    if not base_inertia_passed:
        failures.append('compact base inertia does not match a homogeneous box')

    controller_path = (
        REPO_ROOT / 'src' / 'mobile_manipulator' / 'config' / 'controllers.yaml'
    )
    controller_text = controller_path.read_text(encoding='utf-8')
    controller_values = {}
    for name in ('wheel_radius', 'wheel_separation'):
        line = next(
            (
                candidate
                for candidate in controller_text.splitlines()
                if candidate.strip().startswith(f'{name}:')
            ),
            None,
        )
        controller_values[name] = (
            float(line.split(':', 1)[1].strip()) if line is not None else None
        )
    expected_separation = 2.0 * expected_platform['wheel_y']
    controller_passed = (
        math.isclose(
            controller_values.get('wheel_radius') or math.nan,
            expected_platform['wheel_radius'],
            abs_tol=1.0e-9,
        )
        and math.isclose(
            controller_values.get('wheel_separation') or math.nan,
            expected_separation,
            abs_tol=1.0e-9,
        )
    )
    if not controller_passed:
        failures.append('diff-drive wheel radius/separation differ from Xacro geometry')
    platform_audit = {
        'properties': properties,
        'expected': expected_platform,
        'base_inertia_formula_values': base_inertia_expected,
        'base_inertia_status': 'PASS' if base_inertia_passed else 'FAIL',
        'controller_values': controller_values,
        'expected_wheel_separation_m': expected_separation,
        'controller_geometry_status': 'PASS' if controller_passed else 'FAIL',
        'status': (
            'PASS'
            if not property_failures and base_inertia_passed and controller_passed
            else 'FAIL'
        ),
    }

    manifest_path = (
        REPO_ROOT
        / 'src'
        / 'mobile_manipulator'
        / 'meshes'
        / 'poppy_ergo_jr'
        / 'asset_manifest.json'
    )
    manifest = json.loads(manifest_path.read_text(encoding='utf-8'))
    visual_triangles = manifest['visual']['poppy_link_6.stl']['triangles']
    collision_triangles = manifest['collision']['poppy_link_6_convex.stl']['triangles']
    collision_ratio = collision_triangles / visual_triangles
    collision_passed = collision_ratio < 0.1
    if not collision_passed:
        failures.append('link 6 collision mesh is not sufficiently simplified')
    checks.append(
        {
            'name': 'link_6_collision_is_simplified',
            'visual_triangles': visual_triangles,
            'collision_triangles': collision_triangles,
            'ratio': collision_ratio,
            'status': 'PASS' if collision_passed else 'FAIL',
        }
    )

    result = {
        'status': 'PASS' if not failures else 'FAIL',
        'validator': 'scripts/cad/validate_mechanical_assembly.py',
        'model_scale': 'Poppy CAD geometry is 1:1 in metres',
        'reference': {
            'official_urdf_repository': OFFICIAL_URDF_URL,
            'official_urdf_commit': OFFICIAL_URDF_COMMIT,
            'official_assembly_guide': ASSEMBLY_GUIDE_URL,
        },
        'tolerance': {
            'joint_transform_absolute': TOLERANCE,
            'units': 'SI (metres, radians)',
        },
        'joint_audit': joint_audit,
        'checks': checks,
        'fk': pose_results,
        'inertias': inertia_results,
        'platform_audit': platform_audit,
        'broken_mesh_uris': broken_uris,
        'failures': failures,
        'visual_inspection_required': True,
        'note': (
            'PASS proves audited transforms, assets, inertias and FK landmarks; '
            'it does not replace human inspection of the rendered assembly.'
        ),
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2) + '\n', encoding='utf-8')
    print(
        f"Mechanical assembly validation {result['status']}: "
        f"{len(joint_audit)} joints, {len(pose_results)} FK poses, "
        f"{len(failures)} failure(s)"
    )
    for failure in failures:
        print(f'ERROR: {failure}', file=sys.stderr)
    return 0 if not failures else 1


if __name__ == '__main__':
    raise SystemExit(main())
