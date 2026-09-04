#!/usr/bin/env python3
"""Validate the consolidated robot against the pinned official Poppy model."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np

from align_poppy_to_official import find_visual_origin, homogeneous, rpy_matrix


REPO_ROOT = Path(__file__).resolve().parents[2]
ASSET_ROOT = (
    REPO_ROOT / 'src' / 'mobile_manipulator' / 'meshes' / 'poppy_ergo_jr'
)
FINAL_XACRO = (
    REPO_ROOT
    / 'src'
    / 'mobile_manipulator'
    / 'urdf'
    / 'mobile_manipulator.urdf.xacro'
)
OFFICIAL_XACRO = (
    ASSET_ROOT
    / 'source'
    / 'official_description'
    / 'urdf'
    / 'poppy_ergo_jr.urdf.xacro'
)
OUTPUT_ROOT = REPO_ROOT / 'results' / 'verified' / 'mechanical_alignment'
POSITION_TOLERANCE_M = 1.0e-9
ORIENTATION_TOLERANCE_RAD = 1.0e-7

JOINT_MAP = (
    ('m1', 'poppy_m1_joint', 'long_U', 'poppy_link_1'),
    ('m2', 'poppy_m2_joint', 'section_1', 'poppy_link_2'),
    ('m3', 'poppy_m3_joint', 'section_2', 'poppy_link_3'),
    ('m4', 'poppy_m4_joint', 'section_3', 'poppy_link_4'),
    ('m5', 'poppy_m5_joint', 'section_4', 'poppy_link_5'),
    ('m6', 'poppy_m6_joint', 'section_5', 'poppy_link_6'),
)
LINK_MAP = (
    ('base_link', 'poppy_mount_link', 'base.dae'),
    ('long_U', 'poppy_link_1', 'long_U.dae'),
    ('section_1', 'poppy_link_2', 'section_1.dae'),
    ('section_2', 'poppy_link_3', 'section_2.dae'),
    ('section_3', 'poppy_link_4', 'section_3.dae'),
    ('section_4', 'poppy_link_5', 'section_4.dae'),
    ('section_5', 'poppy_link_6', 'gripper.dae'),
)
POSES = {
    'home': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    'pose_1': [0.35, -0.45, 0.40, -0.35, 0.25, 0.15],
    'pose_2': [-0.45, 0.35, -0.30, 0.45, -0.35, 0.75],
    'gripper_open': [0.35, -0.45, 0.40, -0.35, 0.25, 1.20],
    'gripper_closed': [0.35, -0.45, 0.40, -0.35, 0.25, 0.0],
}


def values(element: ET.Element | None, attribute: str, default: list[float]) -> list[float]:
    if element is None:
        return default
    return [
        float(value)
        for value in element.attrib.get(
            attribute,
            ' '.join(str(value) for value in default),
        ).split()
    ]


def conditional_child(
    element: ET.Element,
    child_name: str,
    branch: str = 'gripper',
) -> ET.Element | None:
    direct = element.find(child_name)
    if direct is not None:
        return direct
    for candidate in element:
        if candidate.tag.endswith('if') and branch in candidate.attrib.get('value', ''):
            child = candidate.find(child_name)
            if child is not None:
                return child
    return None


def joint_values(joint: ET.Element) -> dict[str, object]:
    origin = conditional_child(joint, 'origin')
    axis = conditional_child(joint, 'axis')
    parent = conditional_child(joint, 'parent')
    child = conditional_child(joint, 'child')
    return {
        'parent': parent.attrib['link'] if parent is not None else None,
        'child': child.attrib['link'] if child is not None else None,
        'xyz': values(origin, 'xyz', [0.0, 0.0, 0.0]),
        'rpy': values(origin, 'rpy', [0.0, 0.0, 0.0]),
        'axis': values(axis, 'xyz', [1.0, 0.0, 0.0]),
    }


def axis_rotation(axis: list[float], angle: float) -> np.ndarray:
    vector = np.asarray(axis, dtype=np.float64)
    vector /= np.linalg.norm(vector)
    x, y, z = vector
    c = math.cos(angle)
    s = math.sin(angle)
    d = 1.0 - c
    return np.array(
        [
            [c + x * x * d, x * y * d - z * s, x * z * d + y * s],
            [y * x * d + z * s, c + y * y * d, y * z * d - x * s],
            [z * x * d - y * s, z * y * d + x * s, c + z * z * d],
        ],
        dtype=np.float64,
    )


def rotation_error(a: np.ndarray, b: np.ndarray) -> float:
    relative = a.T @ b
    cosine = float(np.clip((np.trace(relative) - 1.0) / 2.0, -1.0, 1.0))
    return math.acos(cosine)


def pose_error(a: np.ndarray, b: np.ndarray) -> tuple[float, float]:
    return (
        float(np.linalg.norm(a[:3, 3] - b[:3, 3])),
        rotation_error(a[:3, :3], b[:3, :3]),
    )


def final_visual(link: ET.Element, mesh_name: str) -> tuple[str, np.ndarray]:
    for visual in link.findall('visual'):
        mesh = visual.find('./geometry/mesh')
        if mesh is None:
            continue
        filename = mesh.attrib.get('filename', '')
        if Path(filename).name != mesh_name:
            continue
        origin = visual.find('origin')
        xyz = values(origin, 'xyz', [0.0, 0.0, 0.0])
        rpy = values(origin, 'rpy', [0.0, 0.0, 0.0])
        return filename, homogeneous(rpy_matrix(rpy), np.asarray(xyz))
    raise ValueError(f'{link.attrib["name"]}: expected visual {mesh_name} not found')


def chain_poses(
    joints: dict[str, ET.Element],
    joint_names: list[str],
    positions: list[float],
) -> dict[str, np.ndarray]:
    transform = np.eye(4)
    poses: dict[str, np.ndarray] = {}
    for joint_name, position in zip(joint_names, positions):
        data = joint_values(joints[joint_name])
        origin = homogeneous(
            rpy_matrix(data['rpy']),
            np.asarray(data['xyz']),
        )
        motion = homogeneous(
            axis_rotation(data['axis'], position),
            np.zeros(3),
        )
        transform = transform @ origin @ motion
        poses[data['child']] = transform.copy()
    return poses


def tool_pose(
    poses: dict[str, np.ndarray],
    joint: ET.Element,
) -> np.ndarray:
    data = joint_values(joint)
    return poses[data['parent']] @ homogeneous(
        rpy_matrix(data['rpy']),
        np.asarray(data['xyz']),
    )


def rounded(transform: np.ndarray) -> list[list[float]]:
    return np.round(transform, decimals=10).tolist()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument('--final-xacro', type=Path, default=FINAL_XACRO)
    parser.add_argument('--official-xacro', type=Path, default=OFFICIAL_XACRO)
    parser.add_argument('--output-root', type=Path, default=OUTPUT_ROOT)
    args = parser.parse_args()

    final_root = ET.parse(args.final_xacro).getroot()
    official_root = ET.parse(args.official_xacro).getroot()
    final_links = {link.attrib['name']: link for link in final_root.findall('link')}
    final_joints = {joint.attrib['name']: joint for joint in final_root.findall('joint')}
    official_links = {
        link.attrib['name']: link for link in official_root.findall('link')
    }
    official_joints = {
        joint.attrib['name']: joint for joint in official_root.findall('joint')
    }

    failures: list[str] = []
    joint_rows = []
    for official_name, final_name, official_child, final_child in JOINT_MAP:
        official = joint_values(official_joints[official_name])
        final = joint_values(final_joints[final_name])
        translation_error = float(
            np.linalg.norm(np.asarray(official['xyz']) - np.asarray(final['xyz']))
        )
        orientation_error = rotation_error(
            rpy_matrix(official['rpy']),
            rpy_matrix(final['rpy']),
        )
        axis_error = float(
            math.degrees(
                math.acos(
                    float(
                        np.clip(
                            np.dot(official['axis'], final['axis'])
                            / (
                                np.linalg.norm(official['axis'])
                                * np.linalg.norm(final['axis'])
                            ),
                            -1.0,
                            1.0,
                        )
                    )
                )
            )
        )
        parent_ok = final['parent'] == (
            'poppy_mount_link'
            if official['parent'] == 'base_link'
            else next(
                local
                for upstream, local, _ in LINK_MAP
                if upstream == official['parent']
            )
        )
        child_ok = final['child'] == final_child and official['child'] == official_child
        passed = (
            translation_error <= POSITION_TOLERANCE_M
            and orientation_error <= ORIENTATION_TOLERANCE_RAD
            and axis_error <= math.degrees(ORIENTATION_TOLERANCE_RAD)
            and parent_ok
            and child_ok
        )
        if not passed:
            failures.append(f'{final_name}: joint differs from official reference')
        joint_rows.append(
            {
                'element': final_name,
                'official': official_name,
                'position_error_m': translation_error,
                'orientation_error_rad': orientation_error,
                'axis_error_deg': axis_error,
                'status': 'PASS' if passed else 'FAIL',
            }
        )

    visual_rows = []
    for official_name, final_name, mesh_name in LINK_MAP:
        official_transform = find_visual_origin(
            args.official_xacro,
            official_name,
            mesh_name,
        )
        final_filename, final_transform = final_visual(
            final_links[final_name],
            mesh_name,
        )
        position_error, orientation_error = pose_error(
            official_transform,
            final_transform,
        )
        uri_ok = '/official/' in final_filename
        passed = (
            position_error <= POSITION_TOLERANCE_M
            and orientation_error <= ORIENTATION_TOLERANCE_RAD
            and uri_ok
        )
        if not passed:
            failures.append(f'{final_name}: visual differs from official reference')
        visual_rows.append(
            {
                'element': final_name,
                'official': official_name,
                'mesh': mesh_name,
                'final_uri': final_filename,
                'position_error_m': position_error,
                'orientation_error_rad': orientation_error,
                'status': 'PASS' if passed else 'FAIL',
            }
        )

    final_joint_names = [local for _, local, _, _ in JOINT_MAP]
    official_joint_names = [official for official, _, _, _ in JOINT_MAP]
    fk_rows = []
    for pose_name, positions in POSES.items():
        official_poses = chain_poses(
            official_joints,
            official_joint_names,
            positions,
        )
        final_poses = chain_poses(
            final_joints,
            final_joint_names,
            positions,
        )
        for official_name, final_name, _ in LINK_MAP[1:]:
            position_error, orientation_error = pose_error(
                official_poses[official_name],
                final_poses[final_name],
            )
            passed = (
                position_error <= POSITION_TOLERANCE_M
                and orientation_error <= ORIENTATION_TOLERANCE_RAD
            )
            if not passed:
                failures.append(
                    f'{pose_name}/{final_name}: FK differs from official reference'
                )
            fk_rows.append(
                {
                    'pose': pose_name,
                    'element': final_name,
                    'official': official_name,
                    'position_error_m': position_error,
                    'orientation_error_rad': orientation_error,
                    'status': 'PASS' if passed else 'FAIL',
                }
            )

        official_tool = tool_pose(
            official_poses,
            official_joints['t7f'],
        )
        final_tool = tool_pose(
            {
                **final_poses,
                'poppy_fixed_tip': tool_pose(
                    final_poses,
                    final_joints['poppy_fixed_tip_joint'],
                ),
            },
            final_joints['poppy_tool_frame_joint'],
        )
        position_error, orientation_error = pose_error(
            official_tool,
            final_tool,
        )
        tool_passed = (
            position_error <= POSITION_TOLERANCE_M
            and orientation_error <= ORIENTATION_TOLERANCE_RAD
        )
        if not tool_passed:
            failures.append(f'{pose_name}/poppy_tool_frame: tool FK mismatch')
        fk_rows.append(
            {
                'pose': pose_name,
                'element': 'poppy_tool_frame',
                'official': 'fixed_tip',
                'position_error_m': position_error,
                'orientation_error_rad': orientation_error,
                'xyz_m_from_mount': np.round(
                    final_tool[:3, 3],
                    decimals=9,
                ).tolist(),
                'transform_from_mount': rounded(final_tool),
                'status': 'PASS' if tool_passed else 'FAIL',
            }
        )

    decision_path = args.output_root / 'decision.json'
    decision = json.loads(decision_path.read_text(encoding='utf-8'))
    decision_ok = (
        decision['autonomous_attempt_status'] == 'FAIL'
        and decision['method_final'] == 'official_reference_consolidation'
        and decision['fallback_stage'] == 'B3'
    )
    if not decision_ok:
        failures.append('decision.json does not record the required B3 fallback')

    result = {
        'schema_version': 1,
        'status': 'PASS' if not failures else 'FAIL',
        'model': str(args.final_xacro.relative_to(REPO_ROOT)),
        'reference': str(args.official_xacro.relative_to(REPO_ROOT)),
        'tolerances': {
            'position_m': POSITION_TOLERANCE_M,
            'orientation_rad': ORIENTATION_TOLERANCE_RAD,
        },
        'decision_gate_status': 'PASS' if decision_ok else 'FAIL',
        'joint_comparison': joint_rows,
        'visual_comparison': visual_rows,
        'fk_comparison': fk_rows,
        'tool_frame': {
            'name': 'poppy_tool_frame',
            'fixed_to': 'poppy_link_5 via poppy_fixed_tip',
            'does_not_move_with_gripper': True,
        },
        'failures': failures,
    }
    args.output_root.mkdir(parents=True, exist_ok=True)
    output_path = args.output_root / 'official_vs_final.json'
    table_path = args.output_root / 'official_vs_final.md'
    output_path.write_text(json.dumps(result, indent=2) + '\n', encoding='utf-8')

    lines = [
        '# Modelo oficial vs. modelo final consolidado',
        '',
        '| Elemento | Modelo oficial | Modelo final | Error posición | Error orientación | Estado |',
        '|---|---|---|---:|---:|---|',
    ]
    for row in visual_rows:
        lines.append(
            f"| visual {row['element']} | {row['mesh']} | mismo DAE fijado | "
            f"{row['position_error_m']:.3e} m | "
            f"{row['orientation_error_rad']:.3e} rad | {row['status']} |"
        )
    for row in joint_rows:
        lines.append(
            f"| joint {row['element']} | {row['official']} | transform auditado | "
            f"{row['position_error_m']:.3e} m | "
            f"{row['orientation_error_rad']:.3e} rad | {row['status']} |"
        )
    for row in fk_rows:
        if row['element'] == 'poppy_tool_frame':
            lines.append(
                f"| tool/{row['pose']} | fixed_tip | poppy_tool_frame | "
                f"{row['position_error_m']:.3e} m | "
                f"{row['orientation_error_rad']:.3e} rad | {row['status']} |"
            )
    lines.extend(['', f"Resultado: **{result['status']}**.", ''])
    table_path.write_text('\n'.join(lines), encoding='utf-8')
    print(
        f"Official vs final validation {result['status']}: "
        f"{len(joint_rows)} joints, {len(visual_rows)} visuals, "
        f"{len(fk_rows)} FK rows"
    )
    for failure in failures:
        print(f'ERROR: {failure}')
    return 0 if not failures else 1


if __name__ == '__main__':
    raise SystemExit(main())
