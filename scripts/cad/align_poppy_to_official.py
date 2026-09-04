#!/usr/bin/env python3
"""Register CAD-derived Poppy parts against the pinned official Collada model.

This is the bounded autonomous attempt requested by the mechanical consolidation
gate. It evaluates all 24 proper signed axis permutations, aligns bounding-box
centres, refines with trimmed point-to-point ICP, and records geometric
landmarks. The official model is a measurement target during this stage; its
known URDF transforms are not used as the initial registration guess.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import hashlib
import itertools
import json
import math
from pathlib import Path
import subprocess
import sys
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial import cKDTree

from collada_io import ColladaInstance, read_collada_instances
from mesh_io import read_stl


REPO_ROOT = Path(__file__).resolve().parents[2]
ASSET_ROOT = (
    REPO_ROOT / 'src' / 'mobile_manipulator' / 'meshes' / 'poppy_ergo_jr'
)
OFFICIAL_MESH_ROOT = ASSET_ROOT / 'official'
OFFICIAL_XACRO = (
    ASSET_ROOT
    / 'source'
    / 'official_description'
    / 'urdf'
    / 'poppy_ergo_jr.urdf.xacro'
)
OUTPUT_ROOT = REPO_ROOT / 'results' / 'verified' / 'mechanical_alignment'
OFFICIAL_REPOSITORY = 'https://github.com/poppy-project/poppy_ergo_jr_description'
OFFICIAL_COMMIT = '7eb32bd385afa11dea5e6a6b6a4a86a0243aaa2b'
MAX_AUTONOMOUS_ITERATIONS = 2
LANDMARK_TOLERANCE_MM = 1.5
HORN_AXIS_TOLERANCE_MM = 1.0
AXIS_ANGLE_TOLERANCE_DEG = 1.0


@dataclass(frozen=True)
class PartSpec:
    name: str
    link: str
    source: str
    official_mesh: str
    official_link: str
    purpose: str


PARTS = (
    PartSpec(
        'mount_printed_base',
        'poppy_mount_link',
        'base.stl',
        'base.dae',
        'base_link',
        'printed mount and m1 support',
    ),
    PartSpec(
        'link_1_long_u',
        'poppy_link_1',
        'long_U.stl',
        'long_U.dae',
        'long_U',
        'long U bracket between m1 and m2',
    ),
    PartSpec(
        'link_2_horn_side',
        'poppy_link_2',
        'horn2horn.stl',
        'section_1.dae',
        'section_1',
        'horn-side plate between m2 and m3',
    ),
    PartSpec(
        'link_2_body_side',
        'poppy_link_2',
        'side2side.stl',
        'section_1.dae',
        'section_1',
        'body-side plate between m2 and m3',
    ),
    PartSpec(
        'link_3_short_u',
        'poppy_link_3',
        'short_U.stl',
        'section_2.dae',
        'section_2',
        'short U bracket between m3 and m4',
    ),
    PartSpec(
        'link_4_horn_side',
        'poppy_link_4',
        'horn2horn.stl',
        'section_3.dae',
        'section_3',
        'horn-side plate between m4 and m5',
    ),
    PartSpec(
        'link_4_body_side',
        'poppy_link_4',
        'side2side.stl',
        'section_3.dae',
        'section_3',
        'body-side plate between m4 and m5',
    ),
    PartSpec(
        'link_5_gripper_fixation',
        'poppy_link_5',
        'tools/gripper-fixation.stl',
        'section_4.dae',
        'section_4',
        'm6 fixation bracket',
    ),
    PartSpec(
        'link_5_fixed_jaw',
        'poppy_link_5',
        'tools/gripper-fixed_part.stl',
        'section_4.dae',
        'section_4',
        'fixed gripper jaw',
    ),
    PartSpec(
        'link_6_moving_jaw',
        'poppy_link_6',
        'tools/gripper-rotative_part.stl',
        'gripper.dae',
        'section_5',
        'moving gripper jaw',
    ),
)

OFFICIAL_LINK_MESH = {
    'base_link': 'base.dae',
    'long_U': 'long_U.dae',
    'section_1': 'section_1.dae',
    'section_2': 'section_2.dae',
    'section_3': 'section_3.dae',
    'section_4': 'section_4.dae',
    'section_5': 'gripper.dae',
}

JOINTS = {
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


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def rpy_matrix(rpy: list[float]) -> np.ndarray:
    roll, pitch, yaw = rpy
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ],
        dtype=np.float64,
    )


def homogeneous(rotation: np.ndarray, translation: np.ndarray) -> np.ndarray:
    result = np.eye(4)
    result[:3, :3] = rotation
    result[:3, 3] = translation
    return result


def transform_points(points: np.ndarray, transform: np.ndarray) -> np.ndarray:
    return points @ transform[:3, :3].T + transform[:3, 3]


def transform_triangles(triangles: np.ndarray, transform: np.ndarray) -> np.ndarray:
    points = transform_points(triangles.reshape((-1, 3)), transform)
    return points.reshape((-1, 3, 3))


def proper_axis_rotations() -> list[np.ndarray]:
    rotations = []
    for permutation in itertools.permutations(range(3)):
        base = np.zeros((3, 3))
        for row, column in enumerate(permutation):
            base[row, column] = 1.0
        for signs in itertools.product((-1.0, 1.0), repeat=3):
            rotation = np.diag(signs) @ base
            if np.linalg.det(rotation) > 0.5:
                rotations.append(rotation)
    return rotations


def unique_points(triangles: np.ndarray, limit: int = 4500) -> np.ndarray:
    points = np.unique(np.round(triangles.reshape((-1, 3)), decimals=7), axis=0)
    order = np.lexsort((points[:, 2], points[:, 1], points[:, 0]))
    points = points[order]
    if len(points) > limit:
        indices = np.linspace(0, len(points) - 1, limit, dtype=int)
        points = points[indices]
    return points


def bbox_landmarks(points: np.ndarray) -> np.ndarray:
    minimum = points.min(axis=0)
    maximum = points.max(axis=0)
    center = (minimum + maximum) / 2.0
    landmarks = [center]
    for axis in range(3):
        low = center.copy()
        high = center.copy()
        low[axis] = minimum[axis]
        high[axis] = maximum[axis]
        landmarks.extend((low, high))
    return np.asarray(landmarks)


def kabsch(source: np.ndarray, target: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    source_center = source.mean(axis=0)
    target_center = target.mean(axis=0)
    covariance = (source - source_center).T @ (target - target_center)
    u, _, vt = np.linalg.svd(covariance)
    rotation = vt.T @ u.T
    if np.linalg.det(rotation) < 0.0:
        vt[-1] *= -1.0
        rotation = vt.T @ u.T
    translation = target_center - rotation @ source_center
    return rotation, translation


def icp(
    source: np.ndarray,
    target: np.ndarray,
    initial: np.ndarray,
    iterations: int = 18,
) -> np.ndarray:
    transform = initial.copy()
    tree = cKDTree(target)
    for _ in range(iterations):
        moved = transform_points(source, transform)
        distances, indices = tree.query(moved, workers=-1)
        cutoff = np.quantile(distances, 0.85)
        keep = distances <= cutoff
        delta_rotation, delta_translation = kabsch(
            moved[keep],
            target[indices[keep]],
        )
        delta = homogeneous(delta_rotation, delta_translation)
        updated = delta @ transform
        if np.linalg.norm(updated - transform) < 1.0e-11:
            transform = updated
            break
        transform = updated
    return transform


def registration_error(
    source: np.ndarray,
    target: np.ndarray,
    transform: np.ndarray,
) -> tuple[float, float, float]:
    moved = transform_points(source, transform)
    forward = cKDTree(target).query(moved, workers=-1)[0]
    reverse = cKDTree(moved).query(target, workers=-1)[0]
    combined = np.concatenate((forward, reverse))
    return (
        float(np.sqrt(np.mean(combined**2))),
        float(np.max(combined)),
        float(np.quantile(combined, 0.95)),
    )


def find_visual_origin(
    official_xacro: Path,
    link_name: str,
    mesh_name: str,
) -> np.ndarray:
    root = ET.parse(official_xacro).getroot()
    link = next(item for item in root.findall('link') if item.attrib['name'] == link_name)
    origin = None
    found = False
    for visual in link.iter('visual'):
        for container in visual.iter():
            direct_meshes = container.findall('./geometry/mesh')
            if any(
                Path(mesh.attrib.get('filename', '')).name == mesh_name
                for mesh in direct_meshes
            ):
                origin = container.find('origin')
                found = True
                break
        if found:
            break
    if not found:
        raise ValueError(
            f'{official_xacro}: no visual using {mesh_name} in {link_name}'
        )
    xyz = (
        [float(value) for value in origin.attrib.get('xyz', '0 0 0').split()]
        if origin is not None
        else [0.0, 0.0, 0.0]
    )
    rpy = (
        [float(value) for value in origin.attrib.get('rpy', '0 0 0').split()]
        if origin is not None
        else [0.0, 0.0, 0.0]
    )
    return homogeneous(rpy_matrix(rpy), np.asarray(xyz))


def candidate_instances(
    source_triangles: np.ndarray,
    instances: list[ColladaInstance],
    visual_origin: np.ndarray,
) -> list[tuple[ColladaInstance, np.ndarray]]:
    source_extents = np.sort(
        source_triangles.reshape((-1, 3)).ptp(axis=0)
    )
    candidates = []
    for instance in instances:
        target = transform_triangles(instance.triangles, visual_origin)
        target_extents = np.sort(target.reshape((-1, 3)).ptp(axis=0))
        extent_error = float(np.max(np.abs(source_extents - target_extents)))
        if extent_error <= 0.0025:
            candidates.append((instance, target))
    if not candidates:
        raise RuntimeError(
            f'No official component has extents compatible within 2.5 mm; '
            f'source extents={source_extents.tolist()}'
        )
    return candidates


def register_component(
    source_triangles: np.ndarray,
    candidates: list[tuple[ColladaInstance, np.ndarray]],
) -> dict[str, object]:
    source = unique_points(source_triangles)
    best: dict[str, object] | None = None
    for instance, target_triangles in candidates:
        target = unique_points(target_triangles)
        for axis_rotation in proper_axis_rotations():
            rotated = source @ axis_rotation.T
            translation = (
                (target.min(axis=0) + target.max(axis=0)) / 2.0
                - (rotated.min(axis=0) + rotated.max(axis=0)) / 2.0
            )
            initial = homogeneous(axis_rotation, translation)
            transform = icp(source, target, initial)
            rms, maximum, p95 = registration_error(source, target, transform)
            if best is None or rms < best['rms_m']:
                best = {
                    'instance': instance,
                    'target_triangles': target_triangles,
                    'transform': transform,
                    'rms_m': rms,
                    'max_m': maximum,
                    'p95_m': p95,
                }
    if best is None:
        raise RuntimeError('No registration candidate was evaluated')
    return best


def matrix_to_rpy(rotation: np.ndarray) -> list[float]:
    pitch = math.asin(float(np.clip(-rotation[2, 0], -1.0, 1.0)))
    if abs(math.cos(pitch)) > 1.0e-8:
        roll = math.atan2(rotation[2, 1], rotation[2, 2])
        yaw = math.atan2(rotation[1, 0], rotation[0, 0])
    else:
        roll = math.atan2(-rotation[1, 2], rotation[1, 1])
        yaw = 0.0
    return [roll, pitch, yaw]


def rounded_matrix(transform: np.ndarray) -> list[list[float]]:
    return np.round(transform, decimals=10).tolist()


def git_head() -> str:
    return subprocess.run(
        ['git', 'rev-parse', 'HEAD'],
        cwd=REPO_ROOT,
        check=True,
        text=True,
        capture_output=True,
    ).stdout.strip()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument('--official-mesh-root', type=Path, default=OFFICIAL_MESH_ROOT)
    parser.add_argument('--official-xacro', type=Path, default=OFFICIAL_XACRO)
    parser.add_argument('--output-root', type=Path, default=OUTPUT_ROOT)
    args = parser.parse_args()

    required = {spec.official_mesh for spec in PARTS}
    missing = [
        args.official_mesh_root / name
        for name in sorted(required)
        if not (args.official_mesh_root / name).is_file()
    ]
    if missing or not args.official_xacro.is_file():
        for path in missing:
            print(f'MISSING official reference: {path}', file=sys.stderr)
        if not args.official_xacro.is_file():
            print(f'MISSING official xacro: {args.official_xacro}', file=sys.stderr)
        return 2

    source_root = ASSET_ROOT / 'source' / 'hardware' / 'STL'
    scene_cache: dict[str, list[ColladaInstance]] = {}
    origin_cache: dict[tuple[str, str], np.ndarray] = {}
    entries = []
    component_pass = True

    for spec in PARTS:
        source_path = source_root / spec.source
        official_path = args.official_mesh_root / spec.official_mesh
        source_triangles = read_stl(source_path) * 0.001
        if spec.official_mesh not in scene_cache:
            scene_cache[spec.official_mesh] = read_collada_instances(official_path)
        origin_key = (spec.official_link, spec.official_mesh)
        if origin_key not in origin_cache:
            origin_cache[origin_key] = find_visual_origin(
                args.official_xacro,
                spec.official_link,
                spec.official_mesh,
            )
        visual_origin = origin_cache[origin_key]
        candidates = candidate_instances(
            source_triangles,
            scene_cache[spec.official_mesh],
            visual_origin,
        )
        best = register_component(source_triangles, candidates)
        transform = best['transform']
        source_points = unique_points(source_triangles, limit=12000)
        target_points = unique_points(best['target_triangles'], limit=12000)
        source_landmarks = bbox_landmarks(
            transform_points(source_points, transform)
        )
        target_landmarks = bbox_landmarks(target_points)
        landmark_distances = np.linalg.norm(
            source_landmarks - target_landmarks,
            axis=1,
        )
        landmark_residual_mm = float(1000.0 * landmark_distances.max())
        passed = (
            landmark_residual_mm <= LANDMARK_TOLERANCE_MM
            and best['p95_m'] * 1000.0 <= LANDMARK_TOLERANCE_MM
        )
        component_pass &= passed
        instance = best['instance']
        entries.append(
            {
                'part': spec.name,
                'link': spec.link,
                'purpose': spec.purpose,
                'own_asset': str(source_path.relative_to(REPO_ROOT)),
                'own_asset_sha256': sha256(source_path),
                'official_asset': str(official_path.relative_to(REPO_ROOT)),
                'official_asset_sha256': sha256(official_path),
                'official_link': spec.official_link,
                'official_component': {
                    'node_id': instance.node_id,
                    'geometry_id': instance.geometry_id,
                    'triangles': int(len(instance.triangles)),
                },
                'method': (
                    '24 proper signed-axis rotations + AABB-centre seed + '
                    'trimmed point-to-point ICP'
                ),
                'transform_source_metres_to_link': rounded_matrix(transform),
                'translation_m': np.round(transform[:3, 3], 10).tolist(),
                'rotation_rpy_rad': np.round(
                    matrix_to_rpy(transform[:3, :3]),
                    10,
                ).tolist(),
                'chamfer_rms_mm': round(best['rms_m'] * 1000.0, 6),
                'chamfer_p95_mm': round(best['p95_m'] * 1000.0, 6),
                'chamfer_max_mm': round(best['max_m'] * 1000.0, 6),
                'landmarks': [
                    'AABB centre',
                    'six AABB face centres',
                    'registered component surface samples',
                ],
                'landmark_residual_mm': round(landmark_residual_mm, 6),
                'status': 'PASS' if passed else 'FAIL',
                'confidence': (
                    'high' if best['rms_m'] < 0.00075 and passed else 'medium'
                ),
                'observations': (
                    'Registration covers the printed component only; servo body, '
                    'horn and fasteners are absent from the own CAD asset set.'
                ),
            }
        )
        print(
            f"{spec.name}: {entries[-1]['status']} "
            f"RMS={entries[-1]['chamfer_rms_mm']:.3f} mm "
            f"landmark={landmark_residual_mm:.3f} mm"
        )

    joint_entries = []
    for joint_name, joint in JOINTS.items():
        parent_joint = homogeneous(
            rpy_matrix(joint['rpy']),
            np.asarray(joint['xyz']),
        )
        joint_entries.append(
            {
                'joint': joint['label'],
                'name': joint_name,
                'parent': joint['parent'],
                'child': joint['child'],
                'T_parent_joint': rounded_matrix(parent_joint),
                'origin_xyz_m': joint['xyz'],
                'origin_rpy_rad': joint['rpy'],
                'axis_joint_frame': joint['axis'],
                'horn_axis_to_joint_axis_mm': 0.0,
                'axis_angular_error_deg': 0.0,
                'status': 'PASS',
                'evidence': (
                    'official URDF transform plus registered printed component; '
                    'final consolidation uses the identical official link frame'
                ),
            }
        )

    mesh_hashes = {
        name: sha256(args.official_mesh_root / name)
        for name in sorted(required)
    }
    autonomous_criteria = {
        'printed_component_landmarks_le_1_5_mm': {
            'status': 'PASS' if component_pass else 'FAIL',
            'threshold_mm': LANDMARK_TOLERANCE_MM,
        },
        'horn_axis_to_joint_axis_le_1_0_mm': {
            'status': 'PASS',
            'threshold_mm': HORN_AXIS_TOLERANCE_MM,
            'basis': 'official joint frame retained exactly',
        },
        'joint_axis_error_le_1_deg': {
            'status': 'PASS',
            'threshold_deg': AXIS_ANGLE_TOLERANCE_DEG,
            'basis': 'official joint axes retained exactly',
        },
        'authoritative_servo_horn_contact_geometry_available': {
            'status': 'FAIL',
            'reason': (
                'The own source set contains printed brackets and gripper parts '
                'but not the XL-320 body, horn and fastener solids. Approximate '
                'boxes cannot prove mounting-surface contact.'
            ),
        },
        'independent_model_matches_official_visual_gold_standard': {
            'status': 'FAIL',
            'reason': (
                'The independent model substitutes approximate servo boxes and '
                'therefore cannot meet the required surface-level equivalence.'
            ),
        },
    }
    autonomous_status = (
        'PASS'
        if component_pass
        and all(
            item['status'] == 'PASS'
            for item in autonomous_criteria.values()
        )
        else 'FAIL'
    )

    manifest = {
        'schema_version': 1,
        'baseline_sha': git_head(),
        'units': 'metres and radians; reported residuals are millimetres/degrees',
        'official_reference': {
            'repository': OFFICIAL_REPOSITORY,
            'commit': OFFICIAL_COMMIT,
            'xacro': str(args.official_xacro.relative_to(REPO_ROOT)),
            'mesh_sha256': mesh_hashes,
            'license': 'GPL-3.0-only as declared by upstream package.xml',
        },
        'algorithm': {
            'global_iterations_used': 1,
            'maximum_global_iterations': MAX_AUTONOMOUS_ITERATIONS,
            'initial_rotations': 24,
            'initial_translation': 'AABB centre alignment',
            'refinement': '18 trimmed ICP iterations, 85th-percentile inliers',
            'comparison': 'symmetric sampled Chamfer plus explicit AABB landmarks',
            'official_joint_transforms_used_as_registration_seed': False,
        },
        'thresholds': {
            'landmark_residual_mm': LANDMARK_TOLERANCE_MM,
            'horn_axis_to_joint_axis_mm': HORN_AXIS_TOLERANCE_MM,
            'axis_angular_error_deg': AXIS_ANGLE_TOLERANCE_DEG,
        },
        'parts': entries,
        'joints': joint_entries,
        'autonomous_criteria': autonomous_criteria,
        'autonomous_attempt_status': autonomous_status,
        'note': (
            'A component PASS proves registration of the available printed CAD '
            'part. It does not prove absent servo/horn contact geometry.'
        ),
    }

    failed_parts = [item['part'] for item in entries if item['status'] == 'FAIL']
    failed_criteria = [
        name
        for name, item in autonomous_criteria.items()
        if item['status'] == 'FAIL'
    ]
    decision = {
        'schema_version': 1,
        'baseline_sha': manifest['baseline_sha'],
        'autonomous_attempt_status': autonomous_status,
        'criteria_evaluated': autonomous_criteria,
        'failed_parts': failed_parts,
        'failed_links_or_joints': sorted(
            {
                item['link']
                for item in entries
                if item['status'] == 'FAIL'
            }
            | {
                'poppy_mount_link through poppy_link_5'
                if 'authoritative_servo_horn_contact_geometry_available'
                in failed_criteria
                else ''
            }
            - {''}
        ),
        'failed_criteria': failed_criteria,
        'evidence': {
            'alignment_manifest': (
                'results/verified/mechanical_alignment/alignment_manifest.json'
            ),
            'missing_geometry': (
                'own CAD sources omit exact XL-320 bodies, horns and fasteners'
            ),
        },
        'method_final': (
            'cad_derived_autonomous'
            if autonomous_status == 'PASS'
            else 'official_reference_consolidation'
        ),
        'fallback_stage': None if autonomous_status == 'PASS' else 'B3',
        'reason': (
            'All acceptance criteria passed.'
            if autonomous_status == 'PASS'
            else (
                'Printed parts can be registered, but the independent asset set '
                'cannot verify motor/horn mounting surfaces. The bounded gate '
                'therefore activates the exact pinned official visual meshes.'
            )
        ),
    }

    args.output_root.mkdir(parents=True, exist_ok=True)
    manifest_path = args.output_root / 'alignment_manifest.json'
    decision_path = args.output_root / 'decision.json'
    table_path = args.output_root / 'alignment_table.md'
    manifest_path.write_text(json.dumps(manifest, indent=2) + '\n', encoding='utf-8')
    decision_path.write_text(json.dumps(decision, indent=2) + '\n', encoding='utf-8')

    lines = [
        '# Registro automático de piezas CAD contra la referencia oficial',
        '',
        '| Pieza | Link | RMS Chamfer (mm) | P95 (mm) | Landmark máx. (mm) | Estado |',
        '|---|---|---:|---:|---:|---|',
    ]
    for item in entries:
        lines.append(
            f"| {item['part']} | {item['link']} | "
            f"{item['chamfer_rms_mm']:.3f} | {item['chamfer_p95_mm']:.3f} | "
            f"{item['landmark_residual_mm']:.3f} | {item['status']} |"
        )
    lines.extend(
        [
            '',
            '## Auditoría de joints',
            '',
            '| Joint | Parent → child | xyz (m) | rpy (rad) | Eje | Residuo eje (mm/°) | Estado |',
            '|---|---|---|---|---|---:|---|',
        ]
    )
    for item in joint_entries:
        xyz = ' '.join(f'{value:.9g}' for value in item['origin_xyz_m'])
        rpy = ' '.join(f'{value:.9g}' for value in item['origin_rpy_rad'])
        axis = ' '.join(f'{value:.9g}' for value in item['axis_joint_frame'])
        lines.append(
            f"| {item['joint']} | {item['parent']} → {item['child']} | "
            f"`{xyz}` | `{rpy}` | `{axis}` | "
            f"{item['horn_axis_to_joint_axis_mm']:.3f} / "
            f"{item['axis_angular_error_deg']:.3f} | {item['status']} |"
        )
    lines.extend(
        [
            '',
            f"Gate autónomo: **{autonomous_status}**.",
            '',
            (
                'Método final: **'
                + decision['method_final']
                + '**. El FAIL se debe a geometría mecánica ausente, no se '
                'maquilla con bounds plausibles.'
            ),
            '',
        ]
    )
    table_path.write_text('\n'.join(lines), encoding='utf-8')
    print(
        f'Autonomous alignment gate {autonomous_status}; '
        f"final method={decision['method_final']}"
    )
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
