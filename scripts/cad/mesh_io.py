#!/usr/bin/env python3
"""Small, dependency-light STL helpers used by the Poppy CAD pipeline."""

from __future__ import annotations

import struct
from pathlib import Path

import numpy as np


def read_stl(path: Path) -> np.ndarray:
    """Return STL triangles as an (n, 3, 3) float64 array."""
    data = path.read_bytes()
    if len(data) >= 84:
        triangle_count = struct.unpack_from('<I', data, 80)[0]
        if 84 + triangle_count * 50 == len(data):
            record = np.dtype(
                [
                    ('normal', '<f4', (3,)),
                    ('vertices', '<f4', (3, 3)),
                    ('attribute', '<u2'),
                ]
            )
            return np.frombuffer(data, dtype=record, offset=84)[
                'vertices'
            ].astype(np.float64)

    vertices = []
    for line in data.decode('ascii').splitlines():
        fields = line.strip().split()
        if len(fields) == 4 and fields[0].lower() == 'vertex':
            vertices.append([float(value) for value in fields[1:]])
    if not vertices or len(vertices) % 3:
        raise ValueError(f'{path} is not a supported binary or ASCII STL')
    return np.asarray(vertices, dtype=np.float64).reshape((-1, 3, 3))


def write_binary_stl(path: Path, triangles: np.ndarray, label: str) -> None:
    """Write triangles as deterministic binary STL with recomputed normals."""
    triangles = np.asarray(triangles, dtype=np.float32)
    edge_a = triangles[:, 1] - triangles[:, 0]
    edge_b = triangles[:, 2] - triangles[:, 0]
    normals = np.cross(edge_a, edge_b)
    lengths = np.linalg.norm(normals, axis=1)
    nonzero = lengths > 0
    normals[nonzero] /= lengths[nonzero, None]
    normals[~nonzero] = 0

    record = np.zeros(
        len(triangles),
        dtype=np.dtype(
            [
                ('normal', '<f4', (3,)),
                ('vertices', '<f4', (3, 3)),
                ('attribute', '<u2'),
            ]
        ),
    )
    record['normal'] = normals
    record['vertices'] = triangles
    header = label.encode('ascii', errors='replace')[:80].ljust(80, b' ')
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open('wb') as stream:
        stream.write(header)
        stream.write(struct.pack('<I', len(triangles)))
        stream.write(record.tobytes())


def transform_triangles(
    triangles: np.ndarray,
    *,
    scale: float = 1.0,
    rotation: np.ndarray | None = None,
    translation: np.ndarray | None = None,
) -> np.ndarray:
    result = np.asarray(triangles, dtype=np.float64) * scale
    if rotation is not None:
        result = result @ np.asarray(rotation, dtype=np.float64).T
    if translation is not None:
        result = result + np.asarray(translation, dtype=np.float64)
    return result


def unique_vertices(triangles: np.ndarray, decimals: int = 7) -> np.ndarray:
    points = np.asarray(triangles, dtype=np.float64).reshape((-1, 3))
    return np.unique(np.round(points, decimals=decimals), axis=0)


def mesh_summary(triangles: np.ndarray) -> dict[str, object]:
    triangles = np.asarray(triangles, dtype=np.float64)
    points = triangles.reshape((-1, 3))
    minimum = points.min(axis=0)
    maximum = points.max(axis=0)
    cross = np.cross(
        triangles[:, 1] - triangles[:, 0],
        triangles[:, 2] - triangles[:, 0],
    )
    area = float(0.5 * np.linalg.norm(cross, axis=1).sum())
    signed_volume = float(
        np.einsum(
            'ij,ij->i',
            triangles[:, 0],
            np.cross(triangles[:, 1], triangles[:, 2]),
        ).sum()
        / 6.0
    )

    rounded = np.round(points, decimals=9)
    _, inverse = np.unique(rounded, axis=0, return_inverse=True)
    faces = inverse.reshape((-1, 3))
    edges = np.sort(
        np.concatenate(
            [faces[:, [0, 1]], faces[:, [1, 2]], faces[:, [2, 0]]],
            axis=0,
        ),
        axis=1,
    )
    _, counts = np.unique(edges, axis=0, return_counts=True)
    return {
        'triangles': int(len(triangles)),
        'vertices': int(len(np.unique(rounded, axis=0))),
        'bounds_min': minimum.tolist(),
        'bounds_max': maximum.tolist(),
        'extents': (maximum - minimum).tolist(),
        'surface_area': area,
        'signed_volume': signed_volume,
        'boundary_edges': int(np.count_nonzero(counts == 1)),
        'nonmanifold_edges': int(np.count_nonzero(counts > 2)),
        'watertight': bool(np.all(counts == 2)),
    }


def convex_hull_triangles(
    triangles: np.ndarray, voxel_size: float | None = None
) -> np.ndarray:
    """Build an outward-oriented convex hull using SciPy/Qhull."""
    from scipy.spatial import ConvexHull

    points = unique_vertices(triangles)
    if voxel_size:
        points = np.unique(np.round(points / voxel_size) * voxel_size, axis=0)
    hull = ConvexHull(points)
    center = points[hull.vertices].mean(axis=0)
    faces = points[hull.simplices].copy()
    normals = np.cross(faces[:, 1] - faces[:, 0], faces[:, 2] - faces[:, 0])
    face_centers = faces.mean(axis=1)
    inward = np.einsum('ij,ij->i', normals, face_centers - center) < 0
    faces[inward, 1], faces[inward, 2] = (
        faces[inward, 2].copy(),
        faces[inward, 1].copy(),
    )
    return faces
