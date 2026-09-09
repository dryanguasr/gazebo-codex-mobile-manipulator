"""Regression tests for CAD-aligned, unassisted gripping."""
import importlib.util
import math
from pathlib import Path
from types import SimpleNamespace as NS
import xml.etree.ElementTree as ET

from mobile_manipulator.pick_place_attach_gate import cylinder_tilt_deg
from mobile_manipulator.pick_place_supervisor import PickPlaceSupervisor
import numpy as np
import pytest
import yaml

REPO = Path(__file__).resolve().parents[3]


def test_contact_surfaces_follow_transformed_official_cad():
    spec = importlib.util.spec_from_file_location(
        'audit_gripper_contact', REPO / 'tools/audit_gripper_contact.py'
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    assert module.audit()['status'] == 'passed'


def test_cylinder_decomposition_preserves_volume_mass_and_sensor_coverage():
    root = ET.parse(REPO / 'src/mobile_manipulator/worlds/pick_object.sdf')
    link = root.find('model/link')
    intervals = []
    for collision in link.findall('collision'):
        cylinder = collision.find('geometry/cylinder')
        assert float(cylinder.findtext('radius')) == pytest.approx(0.015)
        z = float(collision.findtext('pose').split()[2])
        half = float(cylinder.findtext('length')) / 2
        intervals.append((z - half, z + half))
    intervals.sort()
    assert intervals[0][0] == pytest.approx(-0.0225)
    assert intervals[-1][1] == pytest.approx(0.0225)
    for first, second in zip(intervals, intervals[1:]):
        assert first[1] == pytest.approx(second[0])
    assert float(link.findtext('inertial/mass')) == pytest.approx(0.030)
    assert {c.get('name') for c in link.findall('collision')} == {
        c.text for c in link.findall('sensor/contact/collision')
    }


@pytest.mark.parametrize('pose', ['lift', 'transfer', 'place'])
def test_transport_keeps_closing_instead_of_replaying_contact_angle(pose):
    configuration = {'close': [0.0] * 6, pose: [0.0] * 5 + [0.6]}
    node = NS(
        joints={'poppy_m6_joint': 0.04},
        get_parameter=lambda key: NS(value=configuration[key]),
    )
    result = PickPlaceSupervisor.positions(node, pose)
    assert result[5] == 0.0
    assert result[5] < node.joints['poppy_m6_joint']


def test_late_goal_result_cannot_complete_replacement_motion():
    node = NS(motion_generation=2, motion_done=False, motion_error=None)
    future = NS(result=lambda: pytest.fail('stale result must not be read'))
    PickPlaceSupervisor.goal_result(node, future, generation=1)
    assert not node.motion_done
    assert node.motion_error is None


def test_late_goal_acceptance_is_cancelled():
    cancellations = []
    handle = NS(
        accepted=True, cancel_goal_async=lambda: cancellations.append(True)
    )
    node = NS(motion_generation=2)
    PickPlaceSupervisor.goal_response(
        node, NS(result=lambda: handle), generation=1
    )
    assert cancellations == [True]


@pytest.mark.parametrize(
    'rotation, expected',
    [
        (NS(x=0.0, y=0.0, z=0.0, w=1.0), 0.0),
        (NS(x=1.0, y=0.0, z=0.0, w=1.0), 90.0),
        (NS(x=0.0, y=0.0, z=0.0, w=0.0), math.inf),
        (NS(x=math.nan, y=0.0, z=0.0, w=1.0), math.inf),
    ],
)
def test_object_tilt_is_normalized_and_fail_closed(rotation, expected):
    assert cylinder_tilt_deg(rotation) == pytest.approx(expected)


def test_transport_waypoints_keep_gripper_vertical():
    configuration = yaml.safe_load(
        (REPO / 'src/mobile_manipulator/config/pick_place_a1.yaml').read_text()
    )['pick_place_supervisor']['ros__parameters']
    # With m4=0, m2+m3+m5=0 preserves the jaw's vertical axis even
    # during joint-space interpolation, not just at the endpoints.
    for first, second in (('grasp', 'lift'), ('lift', 'transfer'),
                          ('transfer', 'place')):
        for fraction in np.linspace(0, 1, 51):
            q = (
                np.array(configuration[first]) * (1 - fraction)
                + np.array(configuration[second]) * fraction
            )
            assert q[3] == pytest.approx(0.0, abs=1e-7)
            assert q[1] + q[2] + q[4] == pytest.approx(0.0, abs=1e-7)


@pytest.mark.parametrize('verified, age_s', [(False, 0.0), (True, 0.6)])
def test_loss_or_stale_heartbeat_stops_transport(verified, age_s):
    import time

    failures = []
    parameters = {
        'grasp_mode': 'physical_contact', 'wall_watchdog_s': 3.0,
        'base_drift_limit_m': 0.01, 'base_speed_limit_mps': 0.01,
    }
    node = NS(
        state='LIFT', last_sim_ns=1_000_000_000,
        last_sim_progress_wall=time.monotonic(),
        sim_ns=lambda: 1_000_000_000,
        get_clock=lambda: NS(now=lambda: NS(to_msg=lambda: None)),
        zero_pub=NS(publish=lambda _: None),
        gate_command=lambda: None,
        cancel_requested=False,
        base_metrics=lambda: (0.0, 0.0),
        physical_feedback_ns=int((1.0 - age_s) * 1e9),
        physical_grasp_verified=verified,
        grasp_lost_since_ns=700_000_000,
        get_parameter=lambda key: NS(value=parameters[key]),
        recovery_tick=lambda: None,
    )

    def fail(reason):
        failures.append(reason)
        node.state = 'RECOVER'

    # ROS messages reject a None stamp; supply a real clock stamp.
    from builtin_interfaces.msg import Time
    node.get_clock = lambda: NS(now=lambda: NS(to_msg=lambda: Time(sec=1)))
    node.fail = fail
    PickPlaceSupervisor.tick(node)
    assert failures == ['physical_grasp_lost_or_stale']
    assert node.state == 'RECOVER'


def test_physical_recovery_does_not_open_or_sweep_home():
    calls = []
    node = NS(
        get_parameter=lambda _: NS(value='physical_contact'),
        start_motion=lambda pose, duration_s: calls.append(pose),
        motion_done=False, motion_error=None,
        elapsed_sim=lambda: 0.1,
    )
    PickPlaceSupervisor.recovery_tick(node)
    assert calls == ['recovery_hold']
