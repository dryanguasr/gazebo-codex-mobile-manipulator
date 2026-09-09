"""Guard the Gazebo render-asset fix against duplicate instance transforms."""
import importlib.util
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import numpy as np
import pytest

REPO = Path(__file__).resolve().parents[3]
CAD = REPO / 'scripts/cad'
if str(CAD) not in sys.path:
    sys.path.insert(0, str(CAD))
spec = importlib.util.spec_from_file_location(
    'build_gazebo_visuals', CAD / 'build_gazebo_visuals.py'
)
builder = importlib.util.module_from_spec(spec)
spec.loader.exec_module(builder)


@pytest.mark.parametrize('name', list(builder.LINK_COLORS))
def test_baked_visual_preserves_every_official_triangle_vertex(name):
    result = builder.verify_baked_geometry(
        builder.ASSETS / 'official' / (name + '.dae'),
        builder.ASSETS / 'gazebo_inspection' / (name + '.dae'),
    )
    assert result['maximum_vertex_error_m'] < 1e-10
    assert result['status'] == 'PASS'


def test_wrist_servo_instances_are_not_superimposed():
    from collada_io import read_collada_instances

    instances = read_collada_instances(
        builder.ASSETS / 'gazebo_inspection/section_4.dae'
    )
    bodies = [i for i in instances if i.geometry_id.endswith('_body')]
    assert len(bodies) == 2
    centers = [
        (i.triangles.min(axis=(0, 1)) + i.triangles.max(axis=(0, 1))) / 2
        for i in bodies
    ]
    assert np.linalg.norm(centers[1] - centers[0]) == pytest.approx(0.040, abs=1e-10)
    assert bodies[0].geometry_id != bodies[1].geometry_id


def test_runtime_visuals_use_unique_geometry_ids_and_explicit_normals():
    ns = {'c': builder.COLLADA_NS}
    for name in builder.LINK_COLORS:
        root = ET.parse(builder.ASSETS / 'gazebo_inspection' / (name + '.dae'))
        assert not root.findall('.//c:instance_node', ns)
        instances = root.findall('.//c:instance_geometry', ns)
        assert len({i.get('url') for i in instances}) == len(instances)
        for triangle in root.findall('.//c:triangles', ns):
            semantics = {i.get('semantic') for i in triangle.findall('c:input', ns)}
            assert semantics == {'VERTEX', 'NORMAL'}


def test_inspection_palette_separates_fingers_and_servo_bodies():
    fixed = np.array(builder.LINK_COLORS['section_4'])
    moving = np.array(builder.LINK_COLORS['gripper'])
    servo = np.array(builder.HARDWARE_COLORS['servo'])
    assert np.linalg.norm(fixed - moving) > 0.6
    assert np.linalg.norm(fixed - servo) > 0.4
    assert np.linalg.norm(moving - servo) > 0.4
