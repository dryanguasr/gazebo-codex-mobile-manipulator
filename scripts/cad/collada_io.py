#!/usr/bin/env python3
"""Dependency-light Collada triangle reader for mechanical reference meshes.

The official Poppy description stores each rigid section as a Collada scene made
from instanced geometry. Gazebo understands that scene directly, but a numerical
alignment tool needs evaluated triangles and each instance transform. This
module implements the Collada 1.4 features used by the pinned Poppy assets.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np


@dataclass(frozen=True)
class ColladaInstance:
    """One evaluated geometry instance in scene coordinates."""

    node_id: str
    geometry_id: str
    transform: np.ndarray
    triangles: np.ndarray


def _url_id(value: str) -> str:
    if not value.startswith('#'):
        raise ValueError(f'Only local Collada references are supported: {value}')
    return value[1:]


def _matrix(element: ET.Element | None) -> np.ndarray:
    if element is None or not (element.text or '').strip():
        return np.eye(4)
    values = np.fromstring(element.text, sep=' ', dtype=np.float64)
    if values.size != 16:
        raise ValueError(f'Expected 16 values in Collada matrix, got {values.size}')
    return values.reshape((4, 4))


def _transform_triangles(triangles: np.ndarray, transform: np.ndarray) -> np.ndarray:
    points = np.asarray(triangles, dtype=np.float64).reshape((-1, 3))
    homogeneous = np.column_stack((points, np.ones(len(points))))
    transformed = homogeneous @ transform.T
    return transformed[:, :3].reshape((-1, 3, 3))


def _read_geometry(mesh: ET.Element, namespace: str) -> np.ndarray:
    sources: dict[str, np.ndarray] = {}
    for source in mesh.findall(f'{namespace}source'):
        array = source.find(f'{namespace}float_array')
        accessor = source.find(f'{namespace}technique_common/{namespace}accessor')
        if array is None or accessor is None:
            continue
        stride = int(accessor.attrib.get('stride', '1'))
        values = np.fromstring(array.text or '', sep=' ', dtype=np.float64)
        if values.size % stride:
            raise ValueError(f"Malformed source {source.attrib.get('id', '<unknown>')}")
        sources[source.attrib['id']] = values.reshape((-1, stride))

    vertices_sources: dict[str, str] = {}
    for vertices in mesh.findall(f'{namespace}vertices'):
        position = next(
            (
                item
                for item in vertices.findall(f'{namespace}input')
                if item.attrib.get('semantic') == 'POSITION'
            ),
            None,
        )
        if position is not None:
            vertices_sources[vertices.attrib['id']] = _url_id(position.attrib['source'])

    output: list[np.ndarray] = []
    for primitive_name in ('triangles', 'polylist'):
        for primitive in mesh.findall(f'{namespace}{primitive_name}'):
            inputs = primitive.findall(f'{namespace}input')
            vertex_input = next(
                (item for item in inputs if item.attrib.get('semantic') == 'VERTEX'),
                None,
            )
            if vertex_input is None:
                continue
            stride = 1 + max(int(item.attrib.get('offset', '0')) for item in inputs)
            vertex_offset = int(vertex_input.attrib.get('offset', '0'))
            vertex_source = vertices_sources[_url_id(vertex_input.attrib['source'])]
            positions = sources[vertex_source][:, :3]
            indices = np.fromstring(
                primitive.findtext(f'{namespace}p', default=''),
                sep=' ',
                dtype=np.int64,
            )
            if indices.size % stride:
                raise ValueError(f'Malformed {primitive_name} index stream')
            vertex_indices = indices.reshape((-1, stride))[:, vertex_offset]
            if primitive_name == 'triangles':
                if vertex_indices.size % 3:
                    raise ValueError('Collada triangle index count is not divisible by 3')
                output.append(positions[vertex_indices].reshape((-1, 3, 3)))
                continue

            counts = np.fromstring(
                primitive.findtext(f'{namespace}vcount', default=''),
                sep=' ',
                dtype=np.int64,
            )
            cursor = 0
            polygons: list[np.ndarray] = []
            for count in counts:
                polygon = positions[vertex_indices[cursor : cursor + count]]
                cursor += int(count)
                for index in range(1, len(polygon) - 1):
                    polygons.append(
                        np.array([polygon[0], polygon[index], polygon[index + 1]])
                    )
            if polygons:
                output.append(np.asarray(polygons))

    if not output:
        return np.empty((0, 3, 3), dtype=np.float64)
    return np.concatenate(output)


def read_collada_instances(path: Path) -> list[ColladaInstance]:
    """Evaluate a Collada visual scene and return its triangle instances."""

    root = ET.parse(path).getroot()
    namespace_uri = root.tag.partition('}')[0].lstrip('{')
    namespace = f'{{{namespace_uri}}}' if namespace_uri else ''
    unit_element = root.find(f'{namespace}asset/{namespace}unit')
    unit_scale = (
        float(unit_element.attrib.get('meter', '1'))
        if unit_element is not None
        else 1.0
    )

    geometries: dict[str, np.ndarray] = {}
    for geometry in root.findall(
        f'{namespace}library_geometries/{namespace}geometry'
    ):
        mesh = geometry.find(f'{namespace}mesh')
        if mesh is not None:
            geometries[geometry.attrib['id']] = (
                _read_geometry(mesh, namespace) * unit_scale
            )

    nodes: dict[str, ET.Element] = {
        node.attrib['id']: node
        for node in root.findall(
            f'{namespace}library_nodes/{namespace}node'
        )
        if 'id' in node.attrib
    }
    visual_scenes = {
        scene.attrib['id']: scene
        for scene in root.findall(
            f'{namespace}library_visual_scenes/{namespace}visual_scene'
        )
    }
    scene_instance = root.find(
        f'{namespace}scene/{namespace}instance_visual_scene'
    )
    if scene_instance is None:
        raise ValueError(f'{path}: Collada scene has no instance_visual_scene')
    scene = visual_scenes[_url_id(scene_instance.attrib['url'])]

    result: list[ColladaInstance] = []

    def visit(
        node: ET.Element,
        parent: np.ndarray,
        ancestry: tuple[str, ...],
    ) -> None:
        node_id = node.attrib.get('id', node.attrib.get('name', 'anonymous'))
        transform = parent @ _matrix(node.find(f'{namespace}matrix'))
        current_ancestry = ancestry + (node_id,)
        for instance in node.findall(f'{namespace}instance_geometry'):
            geometry_id = _url_id(instance.attrib['url'])
            triangles = geometries[geometry_id]
            result.append(
                ColladaInstance(
                    node_id='/'.join(current_ancestry),
                    geometry_id=geometry_id,
                    transform=transform.copy(),
                    triangles=_transform_triangles(triangles, transform),
                )
            )
        for child in node.findall(f'{namespace}node'):
            visit(child, transform, current_ancestry)
        for instance in node.findall(f'{namespace}instance_node'):
            reference = _url_id(instance.attrib['url'])
            visit(
                nodes[reference],
                transform,
                current_ancestry + (f'instance:{reference}',),
            )

    for node in scene.findall(f'{namespace}node'):
        visit(node, np.eye(4), ())
    return result


def read_collada(path: Path) -> np.ndarray:
    """Return all evaluated scene triangles as one array."""

    instances = read_collada_instances(path)
    if not instances:
        return np.empty((0, 3, 3), dtype=np.float64)
    return np.concatenate(
        [instance.triangles for instance in instances]
    )
