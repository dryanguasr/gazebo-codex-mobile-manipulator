#!/usr/bin/env python3
"""Bake official Collada instances for Gazebo and add inspection materials.

The upstream files stay byte-for-byte intact. Unique geometry and node names
avoid Gazebo reusing the first transform of repeated library-node geometries.
No joint, visual origin, triangle topology, or collision is changed.
"""
import argparse
import hashlib
import json
from pathlib import Path
import re
import xml.etree.ElementTree as ET

from collada_io import read_collada_instances
import numpy as np

REPO = Path(__file__).resolve().parents[2]
ASSETS = REPO / 'src/mobile_manipulator/meshes/poppy_ergo_jr'
COLLADA_NS = 'http://www.collada.org/2005/11/COLLADASchema'
LINK_COLORS = {
    'base': (0.24, 0.30, 0.37),
    'long_U': (0.08, 0.38, 0.72),
    'section_1': (0.04, 0.58, 0.66),
    'section_2': (0.88, 0.57, 0.08),
    'section_3': (0.48, 0.27, 0.68),
    'section_4': (0.16, 0.64, 0.32),
    'gripper': (0.80, 0.12, 0.40),
}
HARDWARE_COLORS = {
    'servo': (0.15, 0.18, 0.23),
    'horn': (0.67, 0.72, 0.78),
    'fastener': (0.48, 0.54, 0.61),
}


def role_and_name(instance, names):
    ancestry = [names.get(part, part) for part in instance.node_id.split('/')]
    name = ancestry[-1]
    if any('rivet' in part.lower() for part in ancestry):
        return 'fastener', name
    if name == 'body' and any('XL-320' in part for part in ancestry):
        return 'servo', name
    if name == 'horn':
        return 'horn', name
    return 'printed', name


def float_text(values):
    return ' '.join(format(float(v), '.12g') for v in np.asarray(values).ravel())


def add_source(mesh, name, data):
    source = ET.SubElement(mesh, 'source', id=name)
    array = ET.SubElement(
        source, 'float_array', id=name + '-array', count=str(data.size)
    )
    array.text = float_text(data)
    accessor = ET.SubElement(
        ET.SubElement(source, 'technique_common'), 'accessor',
        source='#' + name + '-array', count=str(len(data)), stride='3',
    )
    for axis in 'XYZ':
        ET.SubElement(accessor, 'param', name=axis, type='float')


def build_one(source, destination):
    namespace = {'c': COLLADA_NS}
    original = ET.parse(source)
    names = {
        node.get('id'): node.get('name', node.get('id'))
        for node in original.findall('.//c:library_nodes/c:node', namespace)
    }
    root = ET.Element('COLLADA', version='1.4.1', xmlns=COLLADA_NS)
    asset = ET.SubElement(root, 'asset')
    contributor = ET.SubElement(asset, 'contributor')
    ET.SubElement(contributor, 'author').text = 'Poppy Project; derived Gazebo inspection asset'
    ET.SubElement(asset, 'unit', name='meter', meter='1')
    ET.SubElement(asset, 'up_axis').text = 'Z_UP'
    effects = ET.SubElement(root, 'library_effects')
    materials = ET.SubElement(root, 'library_materials')
    colors = {'printed': LINK_COLORS[source.stem], **HARDWARE_COLORS}
    for role, color in colors.items():
        effect = ET.SubElement(effects, 'effect', id='effect-' + role)
        profile = ET.SubElement(effect, 'profile_COMMON')
        technique = ET.SubElement(profile, 'technique', sid='common')
        phong = ET.SubElement(technique, 'phong')
        for key, rgba in (
            ('emission', (0, 0, 0, 1)),
            ('ambient', tuple(c * 0.25 for c in color) + (1,)),
            ('diffuse', color + (1,)),
            ('specular', (0.08, 0.08, 0.08, 1)),
        ):
            ET.SubElement(ET.SubElement(phong, key), 'color').text = float_text(rgba)
        ET.SubElement(ET.SubElement(phong, 'shininess'), 'float').text = '12'
        material = ET.SubElement(materials, 'material', id='material-' + role)
        ET.SubElement(material, 'instance_effect', url='#effect-' + role)

    geometries = ET.SubElement(root, 'library_geometries')
    visual_scene = ET.SubElement(
        ET.SubElement(root, 'library_visual_scenes'), 'visual_scene', id='scene'
    )
    records = []
    for index, instance in enumerate(read_collada_instances(source)):
        role, part_name = role_and_name(instance, names)
        unique_name = source.stem + '_' + str(index).zfill(3) + '_' + re.sub(
            r'[^a-zA-Z0-9_]+', '_', part_name
        )
        triangles = instance.triangles
        points, vertex_indices = np.unique(
            triangles.reshape(-1, 3), axis=0, return_inverse=True
        )
        face_normals = np.cross(
            triangles[:, 1] - triangles[:, 0], triangles[:, 2] - triangles[:, 0]
        )
        lengths = np.linalg.norm(face_normals, axis=1)
        face_normals = np.divide(
            face_normals, lengths[:, None], out=np.zeros_like(face_normals),
            where=lengths[:, None] > 1e-20,
        )
        normals, normal_indices = np.unique(
            np.round(face_normals, 9), axis=0, return_inverse=True
        )
        mesh = ET.SubElement(
            ET.SubElement(geometries, 'geometry', id=unique_name, name=unique_name), 'mesh'
        )
        add_source(mesh, unique_name + '-positions', points)
        add_source(mesh, unique_name + '-normals', normals)
        vertices = ET.SubElement(mesh, 'vertices', id=unique_name + '-vertices')
        ET.SubElement(
            vertices, 'input', semantic='POSITION', source='#' + unique_name + '-positions'
        )
        primitive = ET.SubElement(
            mesh, 'triangles', count=str(len(triangles)), material='surface'
        )
        ET.SubElement(
            primitive, 'input', semantic='VERTEX', offset='0',
            source='#' + unique_name + '-vertices',
        )
        ET.SubElement(
            primitive, 'input', semantic='NORMAL', offset='1',
            source='#' + unique_name + '-normals',
        )
        indices = np.column_stack((vertex_indices, np.repeat(normal_indices, 3)))
        ET.SubElement(primitive, 'p').text = ' '.join(str(v) for v in indices.ravel())
        node = ET.SubElement(visual_scene, 'node', id='node-' + unique_name, name=unique_name)
        geometry = ET.SubElement(node, 'instance_geometry', url='#' + unique_name)
        binding = ET.SubElement(ET.SubElement(geometry, 'bind_material'), 'technique_common')
        ET.SubElement(
            binding, 'instance_material', symbol='surface', target='#material-' + role
        )
        records.append({
            'name': unique_name, 'source_ancestry': instance.node_id,
            'role': role, 'triangle_count': len(triangles),
            'min_m': points.min(axis=0).tolist(), 'max_m': points.max(axis=0).tolist(),
            'vertex_mean_m': points.mean(axis=0).tolist(),
        })
    ET.SubElement(ET.SubElement(root, 'scene'), 'instance_visual_scene', url='#scene')
    ET.indent(root, space='  ')
    destination.parent.mkdir(parents=True, exist_ok=True)
    ET.ElementTree(root).write(destination, encoding='utf-8', xml_declaration=True)
    return {
        'source': str(source.relative_to(REPO)),
        'source_sha256': hashlib.sha256(source.read_bytes()).hexdigest(),
        'output': str(destination.relative_to(REPO)),
        'output_sha256': hashlib.sha256(destination.read_bytes()).hexdigest(),
        'instances': records,
    }


def verify_baked_geometry(source, baked):
    """Check every triangle vertex, not only overall bounding boxes."""
    reference = read_collada_instances(source)
    actual = read_collada_instances(baked)
    if len(reference) != len(actual):
        raise ValueError(f'{baked}: instance count changed')
    if len({item.geometry_id for item in actual}) != len(actual):
        raise ValueError(f'{baked}: repeated geometry IDs can trigger the loader bug')
    maximum_error = 0.0
    for original, generated in zip(reference, actual):
        if original.triangles.shape != generated.triangles.shape:
            raise ValueError(f'{baked}: triangle topology changed')
        error = float(np.max(np.abs(original.triangles - generated.triangles)))
        maximum_error = max(maximum_error, error)
    if maximum_error > 1e-10:
        raise ValueError(f'{baked}: vertex error {maximum_error} m')
    return {
        'instance_count': len(reference),
        'triangle_count': sum(len(item.triangles) for item in reference),
        'maximum_vertex_error_m': maximum_error,
        'status': 'PASS',
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir', type=Path, default=ASSETS / 'gazebo_inspection')
    args = parser.parse_args()
    records = [
        build_one(ASSETS / 'official' / (name + '.dae'), args.output_dir / (name + '.dae'))
        for name in LINK_COLORS
    ]
    manifest = {
        'license': 'GPL-3.0-only (derivative of official Poppy DAE assets)',
        'operation': 'bake instance transforms; unique geometry names; explicit normals/materials',
        'joint_or_collision_changes': False,
        'palette': {'links': LINK_COLORS, 'hardware': HARDWARE_COLORS},
        'meshes': records,
    }
    (args.output_dir / 'manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
    print(f'Generated {len(records)} inspection meshes in {args.output_dir}')


if __name__ == '__main__':
    main()
