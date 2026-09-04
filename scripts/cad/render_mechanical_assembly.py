#!/usr/bin/env python3
"""Render reproducible close-range evidence of the consolidated Poppy assembly.

This renderer consumes the same pinned Collada triangles and URDF transforms as
Gazebo. It is intentionally an offscreen technical render, not a screenshot of
the Gazebo GUI; headless Gazebo regression separately proves runtime loading.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import matplotlib

matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import numpy as np

from collada_io import read_collada_instances
from mesh_io import read_stl
from validate_official_consolidation import (
    ASSET_ROOT,
    JOINT_MAP,
    LINK_MAP,
    OFFICIAL_XACRO,
    POSES,
    axis_rotation,
    chain_poses,
    find_visual_origin,
    homogeneous,
    joint_values,
    rpy_matrix,
    tool_pose,
)
import xml.etree.ElementTree as ET


REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_OUTPUT = REPO_ROOT / 'captures' / 'mechanical_assembly_final'
COLORS = {
    'orange': '#f06b24',
    'servo': '#777d82',
    'dark': '#30363b',
    'base': '#657381',
    'wheel': '#1f2225',
    'camera': '#2764d7',
    'ball': '#d73535',
    'collision': '#00d7df',
    'official': '#00a8d8',
}


def triangle_sample(triangles: np.ndarray, limit: int = 5000) -> np.ndarray:
    if len(triangles) <= limit:
        return triangles
    indices = np.linspace(0, len(triangles) - 1, limit, dtype=int)
    return triangles[indices]


def add_triangles(
    axis,
    triangles: np.ndarray,
    color: str,
    alpha: float = 1.0,
    edgecolor: str = 'none',
    linewidth: float = 0.0,
) -> None:
    collection = Poly3DCollection(
        triangle_sample(triangles),
        facecolors=color,
        edgecolors=edgecolor,
        linewidths=linewidth,
        alpha=alpha,
    )
    axis.add_collection3d(collection)


def transformed(triangles: np.ndarray, transform: np.ndarray) -> np.ndarray:
    points = triangles.reshape((-1, 3))
    points = points @ transform[:3, :3].T + transform[:3, 3]
    return points.reshape((-1, 3, 3))


def box_triangles(size: tuple[float, float, float]) -> np.ndarray:
    x, y, z = np.asarray(size) / 2.0
    vertices = np.array(
        [
            [-x, -y, -z],
            [x, -y, -z],
            [x, y, -z],
            [-x, y, -z],
            [-x, -y, z],
            [x, -y, z],
            [x, y, z],
            [-x, y, z],
        ]
    )
    faces = (
        (0, 2, 1), (0, 3, 2), (4, 5, 6), (4, 6, 7),
        (0, 1, 5), (0, 5, 4), (1, 2, 6), (1, 6, 5),
        (2, 3, 7), (2, 7, 6), (3, 0, 4), (3, 4, 7),
    )
    return vertices[np.asarray(faces)]


def cylinder_triangles(radius: float, length: float, segments: int = 32) -> np.ndarray:
    angles = np.linspace(0.0, 2.0 * math.pi, segments, endpoint=False)
    lower = np.column_stack(
        (radius * np.cos(angles), radius * np.sin(angles), np.full(segments, -length / 2))
    )
    upper = lower.copy()
    upper[:, 2] = length / 2
    triangles = []
    for index in range(segments):
        following = (index + 1) % segments
        triangles.extend(
            (
                [lower[index], lower[following], upper[following]],
                [lower[index], upper[following], upper[index]],
                [[0.0, 0.0, -length / 2], lower[following], lower[index]],
                [[0.0, 0.0, length / 2], upper[index], upper[following]],
            )
        )
    return np.asarray(triangles)


def sphere_triangles(radius: float, rings: int = 14, segments: int = 24) -> np.ndarray:
    vertices = []
    for ring in range(rings + 1):
        phi = math.pi * ring / rings
        for segment in range(segments):
            theta = 2.0 * math.pi * segment / segments
            vertices.append(
                [
                    radius * math.sin(phi) * math.cos(theta),
                    radius * math.sin(phi) * math.sin(theta),
                    radius * math.cos(phi),
                ]
            )
    vertices = np.asarray(vertices)
    triangles = []
    for ring in range(rings):
        for segment in range(segments):
            following = (segment + 1) % segments
            a = ring * segments + segment
            b = ring * segments + following
            c = (ring + 1) * segments + segment
            d = (ring + 1) * segments + following
            triangles.extend(([vertices[a], vertices[c], vertices[d]], [vertices[a], vertices[d], vertices[b]]))
    return np.asarray(triangles)


def component_color(mesh_name: str, triangle_count: int) -> str:
    if mesh_name == 'base.dae' and triangle_count in (20247, 10958):
        return COLORS['orange']
    if mesh_name == 'long_U.dae':
        return COLORS['orange']
    if triangle_count in (3032, 744):
        return COLORS['dark']
    if triangle_count in (14134, 4364):
        return COLORS['servo']
    return COLORS['orange']


def load_arm_geometry(
    positions: list[float],
    mount_transform: np.ndarray,
) -> tuple[list[dict[str, object]], dict[str, np.ndarray]]:
    official_root = ET.parse(OFFICIAL_XACRO).getroot()
    joints = {joint.attrib['name']: joint for joint in official_root.findall('joint')}
    joint_names = [name for name, _, _, _ in JOINT_MAP]
    link_poses = {'base_link': np.eye(4)}
    link_poses.update(chain_poses(joints, joint_names, positions))
    geometry = []
    for official_link, _, mesh_name in LINK_MAP:
        visual_origin = find_visual_origin(OFFICIAL_XACRO, official_link, mesh_name)
        link_transform = mount_transform @ link_poses[official_link] @ visual_origin
        for instance in read_collada_instances(ASSET_ROOT / 'official' / mesh_name):
            geometry.append(
                {
                    'link': official_link,
                    'mesh': mesh_name,
                    'triangles': transformed(instance.triangles, link_transform),
                    'color': component_color(mesh_name, len(instance.triangles)),
                }
            )
    return geometry, {
        name: mount_transform @ transform
        for name, transform in link_poses.items()
    }


def add_mobile_platform(axis, alpha: float = 1.0) -> None:
    base_pose = homogeneous(np.eye(3), np.array([0.0, 0.0, 0.07]))
    add_triangles(
        axis,
        transformed(box_triangles((0.40, 0.30, 0.10)), base_pose),
        COLORS['base'],
        alpha,
    )
    wheel_rotation = rpy_matrix([math.pi / 2.0, 0.0, 0.0])
    for x in (-0.14, 0.14):
        for y in (-0.1725, 0.1725):
            pose = homogeneous(wheel_rotation, np.array([x, y, 0.07]))
            add_triangles(
                axis,
                transformed(cylinder_triangles(0.07, 0.045), pose),
                COLORS['wheel'],
                alpha,
            )
    camera_pose = homogeneous(np.eye(3), np.array([0.225, 0.0, 0.12]))
    add_triangles(
        axis,
        transformed(box_triangles((0.08, 0.12, 0.06)), camera_pose),
        COLORS['camera'],
        alpha,
    )


def add_ball(axis) -> None:
    pose = homogeneous(np.eye(3), np.array([0.62, 0.0, 0.12]))
    add_triangles(axis, transformed(sphere_triangles(0.12), pose), COLORS['ball'])


def add_arm(axis, geometry: list[dict[str, object]], alpha: float = 1.0) -> None:
    for component in geometry:
        add_triangles(
            axis,
            component['triangles'],
            component['color'],
            alpha,
        )


def add_collision_overlay(axis, link_poses: dict[str, np.ndarray]) -> None:
    specifications = {
        'base_link': [('cylinder', (0.08, 0.03), [0.0, 0.038, 0.015], [0.0, 0.0, 0.0])],
        'long_U': [('cylinder', (0.02, 0.02), [0.0, 0.0, 0.01], [0.0, 0.0, 0.0])],
        'section_1': [('cylinder', (0.02, 0.05), [0.025, 0.0, 0.0], [0.0, math.pi / 2, 0.0])],
        'section_2': [('cylinder', (0.02, 0.05), [0.025, 0.0, 0.0], [0.0, math.pi / 2, 0.0])],
        'section_3': [('cylinder', (0.02, 0.05), [0.0, -0.025, 0.0], [math.pi / 2, 0.0, 0.0])],
        'section_4': [
            ('box', (0.036, 0.032, 0.030), [0.0, -0.058, 0.0], [0.0, 0.0, 0.0]),
            ('box', (0.024, 0.092, 0.006), [0.0, -0.082, 0.0145], [0.0, 0.0, 0.0]),
        ],
    }
    for link, items in specifications.items():
        for kind, dimensions, xyz, rpy in items:
            primitive = (
                cylinder_triangles(*dimensions)
                if kind == 'cylinder'
                else box_triangles(dimensions)
            )
            pose = link_poses[link] @ homogeneous(
                rpy_matrix(rpy),
                np.asarray(xyz),
            )
            add_triangles(
                axis,
                transformed(primitive, pose),
                COLORS['collision'],
                0.65,
                '#007e86',
                0.12,
            )

    hull = read_stl(ASSET_ROOT / 'collision' / 'poppy_link_6_convex.stl')
    add_triangles(
        axis,
        transformed(hull, link_poses['section_5']),
        COLORS['collision'],
        0.65,
        '#007e86',
        0.12,
    )


def configured_axis(
    title: str,
    bounds: tuple[tuple[float, float], tuple[float, float], tuple[float, float]],
    elevation: float = 23.0,
    azimuth: float = 125.0,
):
    figure = plt.figure(figsize=(12.8, 8.0), dpi=140)
    axis = figure.add_subplot(111, projection='3d')
    axis.set_facecolor('#f4f6f8')
    figure.patch.set_facecolor('#f4f6f8')
    axis.view_init(elev=elevation, azim=azimuth)
    axis.set_proj_type('ortho')
    axis.set_xlim(*bounds[0])
    axis.set_ylim(*bounds[1])
    axis.set_zlim(*bounds[2])
    axis.set_box_aspect(
        (
            bounds[0][1] - bounds[0][0],
            bounds[1][1] - bounds[1][0],
            bounds[2][1] - bounds[2][0],
        )
    )
    axis.set_title(title, fontsize=14)
    axis.set_xlabel('X [m]')
    axis.set_ylabel('Y [m]')
    axis.set_zlabel('Z [m]')
    axis.grid(True, alpha=0.25)
    return figure, axis


def save_scene(
    output: Path,
    title: str,
    positions: list[float],
    *,
    official_only: bool = False,
    close_bounds=None,
    collision: bool = False,
    overlay: bool = False,
    include_links: tuple[str, ...] | None = None,
) -> dict[str, object]:
    mount = (
        np.eye(4)
        if official_only
        else homogeneous(np.eye(3), np.array([-0.03, 0.0, 0.12]))
    )
    geometry, link_poses = load_arm_geometry(positions, mount)
    if include_links is not None:
        geometry = [item for item in geometry if item['link'] in include_links]
    bounds = close_bounds or (
        ((-0.24, 0.78), (-0.34, 0.34), (-0.02, 0.48))
        if not official_only
        else ((-0.18, 0.24), (-0.24, 0.18), (-0.05, 0.30))
    )
    figure, axis = configured_axis(title, bounds)
    if not official_only and close_bounds is None:
        add_mobile_platform(axis, alpha=0.25 if collision else 1.0)
        if close_bounds is None:
            add_ball(axis)
    add_arm(axis, geometry, alpha=0.20 if collision else 1.0)
    if collision:
        add_collision_overlay(axis, link_poses)
    if overlay:
        for component in geometry:
            add_triangles(
                axis,
                component['triangles'],
                COLORS['official'],
                0.12,
                COLORS['official'],
                0.16,
            )
        axis.text2D(
            0.02,
            0.95,
            'Cian: referencia oficial; superficies: modelo final (coincidencia exacta)',
            transform=axis.transAxes,
            color='#006b86',
        )
    figure.tight_layout()
    figure.savefig(output, bbox_inches='tight')
    plt.close(figure)
    return {
        'file': str(output.relative_to(REPO_ROOT)),
        'pose_rad': positions,
        'official_only': official_only,
        'collision_overlay': collision,
        'official_final_overlay': overlay,
        'bounds_m': bounds,
    }


def pair_bounds(
    link_poses: dict[str, np.ndarray],
    first: str,
    second: str,
    radius: float = 0.065,
):
    points = np.vstack((link_poses[first][:3, 3], link_poses[second][:3, 3]))
    center = points.mean(axis=0)
    extent = max(radius, float(np.max(np.ptp(points, axis=0))) / 2 + 0.04)
    return (
        (center[0] - extent, center[0] + extent),
        (center[1] - extent, center[1] + extent),
        (center[2] - extent, center[2] + extent),
    )


def fitted_geometry_bounds(
    positions: list[float],
    mount_transform: np.ndarray,
    links: tuple[str, ...],
    padding: float = 0.018,
):
    """Return tight, equal-aspect bounds for the selected transformed links."""
    geometry, _ = load_arm_geometry(positions, mount_transform)
    selected = [
        item['triangles'].reshape((-1, 3))
        for item in geometry
        if item['link'] in links
    ]
    if not selected:
        raise ValueError(f'No geometry found for links: {links}')
    points = np.vstack(selected)
    lower = points.min(axis=0)
    upper = points.max(axis=0)
    center = (lower + upper) / 2.0
    half_extent = max(float(np.max(upper - lower)) / 2.0 + padding, 0.045)
    return tuple(
        (float(value - half_extent), float(value + half_extent))
        for value in center
    )


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)

    captures = []
    for pose_name in ('home', 'pose_1', 'pose_2'):
        captures.append(
            save_scene(
                args.output / f'official_{pose_name}.png',
                f'Referencia oficial Poppy Ergo Jr — {pose_name}',
                POSES[pose_name],
                official_only=True,
            )
        )
        captures.append(
            save_scene(
                args.output / f'final_{pose_name}.png',
                f'Manipulador móvil consolidado — {pose_name}',
                POSES[pose_name],
            )
        )

    close_bounds = ((-0.16, 0.18), (-0.24, 0.12), (0.07, 0.34))
    captures.append(
        save_scene(
            args.output / 'final_home_close.png',
            'Modelo final — detalle del brazo en home',
            POSES['home'],
            close_bounds=close_bounds,
            include_links=tuple(link for link, _, _ in LINK_MAP),
        )
    )
    mount = homogeneous(np.eye(3), np.array([-0.03, 0.0, 0.12]))
    for pose_name in ('gripper_open', 'gripper_closed'):
        gripper_bounds = fitted_geometry_bounds(
            POSES[pose_name],
            mount,
            ('section_4', 'section_5'),
        )
        captures.append(
            save_scene(
                args.output / f'final_{pose_name}.png',
                f'Modelo final — {pose_name}',
                POSES[pose_name],
                close_bounds=gripper_bounds,
                include_links=('section_4', 'section_5'),
            )
        )

    captures.append(
        save_scene(
            args.output / 'final_collision_overlay.png',
            'Modelo final — visual + collision simplificada',
            POSES['pose_1'],
            collision=True,
            close_bounds=((-0.17, 0.18), (-0.24, 0.13), (0.06, 0.36)),
            include_links=tuple(link for link, _, _ in LINK_MAP),
        )
    )
    captures.append(
        save_scene(
            args.output / 'official_vs_final_overlay.png',
            'Referencia oficial vs. modelo final — pose 1',
            POSES['pose_1'],
            overlay=True,
            close_bounds=((-0.17, 0.18), (-0.24, 0.13), (0.06, 0.36)),
        )
    )

    _, link_poses = load_arm_geometry(POSES['home'], mount)
    pairs = (
        ('m1_m2', 'base_link', 'long_U'),
        ('m2_m3', 'long_U', 'section_1'),
        ('m3_m4', 'section_1', 'section_2'),
        ('m4_m5', 'section_2', 'section_3'),
        ('m5_m6', 'section_3', 'section_4'),
        ('gripper', 'section_4', 'section_5'),
    )
    for name, first, second in pairs:
        captures.append(
            save_scene(
                args.output / f'close_{name}.png',
                f'Detalle mecánico {name.replace("_", "–")}',
                POSES['home'],
                close_bounds=pair_bounds(link_poses, first, second),
                include_links=(first, second),
            )
        )

    manifest = {
        'status': 'PASS',
        'renderer': 'Matplotlib Agg offscreen technical mesh renderer',
        'equivalence_note': (
            'The renderer evaluates the exact pinned DAE scene triangles and '
            'official URDF transforms used by the final Xacro. It is not a '
            'Gazebo GUI screenshot; Gazebo runtime loading is validated separately.'
        ),
        'captures': captures,
    }
    (args.output / 'capture_manifest.json').write_text(
        json.dumps(manifest, indent=2) + '\n',
        encoding='utf-8',
    )
    print(f"Rendered {len(captures)} evidence images in {args.output}")
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
