#!/usr/bin/env python3
"""Audit contact faces against transformed, unmodified official Poppy CAD."""
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / 'scripts/cad'))
from collada_io import read_collada_instances  # noqa: E402
import numpy as np  # noqa: E402
from validate_mechanical_assembly import origin_transform  # noqa: E402


def surface_distance(point, triangles):
    """Exact point/triangle distance including edges; ignores degenerate faces."""
    a, b, c = triangles[:, 0], triangles[:, 1], triangles[:, 2]
    u, v, w = b - a, c - a, point - a
    uu = np.einsum('ij,ij->i', u, u)
    uv = np.einsum('ij,ij->i', u, v)
    vv = np.einsum('ij,ij->i', v, v)
    wu = np.einsum('ij,ij->i', w, u)
    wv = np.einsum('ij,ij->i', w, v)
    den = uu * vv - uv * uv
    valid = den > 1e-20
    s = np.divide(vv * wu - uv * wv, den, out=np.zeros_like(den), where=valid)
    t = np.divide(uu * wv - uv * wu, den, out=np.zeros_like(den), where=valid)
    interior = valid & (s >= 0) & (t >= 0) & (s + t <= 1)
    distances = np.full(len(a), np.inf)
    projection = a + s[:, None] * u + t[:, None] * v
    distances[interior] = np.linalg.norm(projection[interior] - point, axis=1)
    for first, last in ((a, b), (b, c), (c, a)):
        edge = last - first
        length2 = np.einsum('ij,ij->i', edge, edge)
        fraction = np.clip(np.divide(
            np.einsum('ij,ij->i', point - first, edge),
            length2, out=np.zeros_like(length2), where=length2 > 1e-20,
        ), 0, 1)
        closest = first + fraction[:, None] * edge
        distances = np.minimum(distances, np.linalg.norm(closest - point, axis=1))
    return float(distances.min())


def audit():
    assets = REPO / 'src/mobile_manipulator/meshes/poppy_ergo_jr/official'
    urdf = ET.parse(REPO / 'src/mobile_manipulator/urdf/mobile_manipulator.urdf.xacro')
    results = {}
    for label, link, mesh, node, axis, sign in (
        ('fixed', 'poppy_link_5', 'section_4.dae', '/ID22', 2, -1),
        ('moving', 'poppy_link_6', 'gripper.dae', '/ID21', 0, 1),
    ):
        element = urdf.find(f".//link[@name='{link}']")
        visual_origin = element.find('visual/origin')
        xyz = [float(v) for v in visual_origin.get('xyz', '0 0 0').split()]
        rpy = [float(v) for v in visual_origin.get('rpy', '0 0 0').split()]
        transform = origin_transform(xyz, rpy)
        mesh_data = next(
            item.triangles for item in read_collada_instances(assets / mesh)
            if item.node_id.endswith(node)
        )
        triangles = mesh_data @ transform[:3, :3].T + transform[:3, 3]
        collision = element.find(f".//collision[@name='poppy_{label}_finger_collision']")
        center = np.array([float(v) for v in collision.find('origin').get('xyz').split()])
        size = np.array([float(v) for v in collision.find('geometry/box').get('size').split()])
        face_center = center.copy()
        face_center[axis] += sign * size[axis] / 2
        tangent_axes = [i for i in range(3) if i != axis]
        distances = []
        for first in (-0.3, 0, 0.3):
            for second in (-0.3, 0, 0.3):
                point = face_center.copy()
                point[tangent_axes[0]] += first * size[tangent_axes[0]]
                point[tangent_axes[1]] += second * size[tangent_axes[1]]
                distances.append(surface_distance(point, triangles))
        maximum = max(distances)
        assert maximum < 0.00075, (label, maximum)
        results[label] = {
            'inner_face_center_m': face_center.tolist(),
            'samples': len(distances),
            'max_distance_to_visible_CAD_m': maximum,
        }
    assert results['fixed']['inner_face_center_m'][2] == 0.014
    assert results['moving']['inner_face_center_m'][0] == -0.014
    results['status'] = 'passed'
    return results


if __name__ == '__main__':
    print(json.dumps(audit(), indent=2))
