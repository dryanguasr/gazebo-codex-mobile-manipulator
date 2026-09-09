"""Route, safety and world contracts for the complete mobile exercise."""

import math
from pathlib import Path
from types import SimpleNamespace as NS
import xml.etree.ElementTree as ET

from mobile_manipulator.mobile_transport import (
    model_planar_pose, PlanarRoute, quaternion_distance_deg, relative_quaternion,
    slew, SmoothRoute, TRANSPORT_STATES, world_to_odom, wrap_angle,
)
from mobile_manipulator.pick_place_attach_gate import RETENTION_STATES
from mobile_manipulator.pick_place_evaluator import expected_fixed_jaw_approach
from mobile_manipulator.pick_place_supervisor import (
    MOBILE_STATE_SEQUENCE, PickPlaceSupervisor,
)
import pytest
import yaml


ROOT = Path(__file__).resolve().parents[3]


def test_route_reaches_separated_station_and_rotates_with_bounded_commands():
    cfg = yaml.safe_load(
        (ROOT / 'src/mobile_manipulator/config/pick_place_mobile.yaml').read_text()
    )
    values = cfg['pick_place_supervisor']['ros__parameters']['route']
    route = PlanarRoute([values[i:i + 3] for i in range(0, len(values), 3)])
    x = y = yaw = previous_v = previous_w = distance = 0.0
    dt = 0.02
    for _ in range(9000):
        v, w, done = route.step((x, y, yaw), dt)
        assert abs(v) <= 0.06
        assert abs(w) <= 0.2
        # Waypoint stop snaps only negligible residual velocities to zero.
        assert abs(v - previous_v) <= 0.0021
        assert abs(w - previous_w) <= 0.0061
        distance += abs(v) * dt
        x += v * math.cos(yaw) * dt
        y += v * math.sin(yaw) * dt
        yaw = wrap_angle(yaw + w * dt)
        previous_v, previous_w = v, w
        if done:
            break
    assert done
    assert math.dist((x, y), (0.838, 0.738)) < 0.005
    assert abs(wrap_angle(yaw - math.pi / 2)) < 0.005
    assert distance >= 1.3
    assert route.index == 4


@pytest.mark.parametrize('dt', [0.0, 0.02, 10.0])
def test_slew_never_jumps_on_clock_gaps(dt):
    assert 0 <= slew(0, 1, 0.025, dt) <= 0.0025 + 1e-12


def test_world_odom_transform_corrects_skid_without_teleporting_base():
    actual = (0.85, 0.65, math.pi / 2)
    wheels = (0.8, 0.6, 1.8)
    x, y, yaw = world_to_odom(actual, wheels)
    reconstructed = (
        x + math.cos(yaw) * wheels[0] - math.sin(yaw) * wheels[1],
        y + math.sin(yaw) * wheels[0] + math.cos(yaw) * wheels[1],
        wrap_angle(yaw + wheels[2]),
    )
    assert reconstructed == pytest.approx(actual)


def test_model_pose_rejects_ambiguous_messages():
    assert model_planar_pose(NS(transforms=[])) is None
    assert model_planar_pose(NS(transforms=[None, None])) is None


def test_grid_is_visual_only_and_stations_are_over_a_meter_apart():
    world = ET.parse(
        ROOT / 'src/mobile_manipulator/worlds/pick_and_place_mobile.sdf'
    ).getroot().find('world')
    grid = world.find("model[@name='inspection_grid_20cm']")
    assert len(grid.findall('link/visual')) == 82
    assert not grid.findall('.//collision')
    pick = list(map(float, world.find("model[@name='pick_support']/pose").text.split()))
    place = list(map(float, world.find("model[@name='place_support']/pose").text.split()))
    assert math.dist(pick[:2], place[:2]) > 1.4
    assert place[:2] == pytest.approx([0.838 + 0.225, 0.738 - 0.044])


def test_retention_monitoring_covers_driving_and_docking():
    assert all(state in RETENTION_STATES for state in TRANSPORT_STATES)
    assert MOBILE_STATE_SEQUENCE.index('HOLD') < MOBILE_STATE_SEQUENCE.index('NAVIGATE')
    assert MOBILE_STATE_SEQUENCE.index('DOCK_BASE') < MOBILE_STATE_SEQUENCE.index('LOWER')


def test_stale_localization_stops_before_driving():
    failures = []
    commands = []
    node = NS(
        state='NAVIGATE', sim_ns=lambda: 2_000_000_000,
        world_base=(0, 0, 0), world_base_ns=1_000_000_000,
        odom_rx_ns=2_000_000_000, publish_base=lambda: commands.append((0, 0)),
        fail=failures.append,
    )
    PickPlaceSupervisor.normal_tick(node)
    assert commands == [(0, 0)]
    assert failures == ['base_localization_or_odometry_stale']


def test_mobile_recovery_holds_closed_instead_of_releasing_over_floor():
    calls = []
    node = NS(
        mobile=True,
        get_parameter=lambda _: NS(value='attach_conditioned'),
        start_motion=lambda pose, duration_s: calls.append(pose),
        motion_done=False, motion_error=None, elapsed_sim=lambda: 0.1,
    )
    PickPlaceSupervisor.recovery_tick(node)
    assert calls == ['recovery_hold']


def test_orientation_metric_detects_spin_but_ignores_common_base_rotation():
    identity = (0, 0, 0, 1)
    yaw90 = (0, 0, math.sin(math.pi / 4), math.cos(math.pi / 4))
    assert quaternion_distance_deg(relative_quaternion(yaw90, yaw90), identity) < 1e-6
    assert quaternion_distance_deg(
        relative_quaternion(identity, yaw90), identity
    ) == pytest.approx(90)
    assert quaternion_distance_deg(identity, (0, 0, 0, -1)) == pytest.approx(0)


def test_joint_loss_during_navigation_enters_recovery():
    import time

    failures = []
    node = NS(
        mobile=True, attached=False, state='NAVIGATE',
        last_sim_ns=1_000_000_000, last_sim_progress_wall=time.monotonic(),
        sim_ns=lambda: 1_000_000_000, gate_command=lambda: None,
        cancel_requested=False, base_metrics=lambda: (0.8, 0.06),
        get_parameter=lambda key: NS(value={
            'grasp_mode': 'attach_conditioned', 'wall_watchdog_s': 3.0,
        }[key]),
        recovery_tick=lambda: None,
    )

    def fail(reason):
        failures.append(reason)
        node.state = 'RECOVER'

    node.fail = fail
    PickPlaceSupervisor.tick(node)
    assert failures == ['temporary_joint_lost_during_transport']
    assert node.state == 'RECOVER'


def test_station_collision_immediately_stops_mobile_supervisor():
    failures = []
    node = NS(
        mobile=True, emit=lambda *args, **kwargs: None, fail=failures.append,
    )
    contact = NS(
        collision1=NS(name='place_support::support_link::place_support_collision'),
        collision2=NS(name='mobile_manipulator::front_right_link::collision'),
    )
    PickPlaceSupervisor.support_contact_callback(node, NS(contacts=[contact]))
    assert failures == ['robot_contacted_station']


def test_mobile_delivery_uses_proven_grasp_pose_and_wheel_clearance():
    cfg = yaml.safe_load(
        (ROOT / 'src/mobile_manipulator/config/pick_place_mobile.yaml').read_text()
    )
    arm = cfg['pick_place_supervisor']['ros__parameters']
    assert arm['place'] == arm['close']
    assert arm['release_open'] == arm['grasp']
    # 225 mm station lateral offset minus half support width and outer wheel face.
    clearance = 0.225 - 0.032 / 2 - (0.1725 + 0.045 / 2)
    assert clearance >= 0.014 - 1e-12


def test_route_compensates_turn_translation_measured_from_skid_steer():
    route = PlanarRoute([
        (0.838, 0, 0), (0.838, 0, math.pi / 2),
        (0.838, 0.35, math.pi / 2), (0.838, 0.738, math.pi / 2),
    ])
    x = y = yaw = 0.0
    dt = 0.01
    offset = 0.09
    for _ in range(18000):
        v, w, done = route.step((x, y, yaw), dt)
        x += (v * math.cos(yaw) - w * offset * math.sin(yaw)) * dt
        y += (v * math.sin(yaw) + w * offset * math.cos(yaw)) * dt
        yaw = wrap_angle(yaw + w * dt)
        if done:
            break
    assert done
    assert math.dist((x, y), (0.838, 0.738)) < 0.005
    assert abs(wrap_angle(yaw - math.pi / 2)) < 0.005
    assert route.pivot_offset[0] == pytest.approx(offset, abs=0.002)


def test_orientation_marks_do_not_change_object_contact_or_inertia():
    original = ET.parse(
        ROOT / 'src/mobile_manipulator/worlds/pick_object.sdf'
    ).getroot().find('model/link')
    mobile = ET.parse(
        ROOT / 'src/mobile_manipulator/worlds/pick_object_mobile.sdf'
    ).getroot().find('model/link')

    def signature(element):
        return (element.tag, element.attrib, (element.text or '').strip(),
                [signature(child) for child in element])

    assert signature(original.find('inertial')) == signature(mobile.find('inertial'))
    assert [signature(c) for c in original.findall('collision')] == [
        signature(c) for c in mobile.findall('collision')
    ]
    assert len(mobile.findall('visual')) == 3


def test_supported_piece_may_meet_fixed_jaw_at_end_of_approach():
    args = ('APPROACH', 'poppy_fixed_finger_collision',
            (0, 0, 0.1625), (0, 0, 0.1625), (0, 0, 0.18), 1.2, 0.0)
    assert expected_fixed_jaw_approach(*args)
    assert not expected_fixed_jaw_approach(
        args[0], 'poppy_moving_finger_collision', *args[2:]
    )
    assert not expected_fixed_jaw_approach(
        *args[:2], (0.02, 0, 0.1625), *args[3:]
    )
    assert not expected_fixed_jaw_approach(*args[:6], 0.06)
    assert not expected_fixed_jaw_approach(*args[:4], (0, 0, 0.3), *args[5:])


@pytest.mark.parametrize('pivot', [0.0, 0.05, 0.09, 0.14])
def test_smooth_route_continuous_corner_and_precise_docking(pivot):
    route = SmoothRoute((0.838, 0.738, math.pi / 2))
    x = y = yaw = previous_v = previous_w = 0.0
    moving_corner = []
    for tick in range(4500):
        v, w, done = route.step((x, y, yaw), 0.02)
        assert 0 <= v <= 0.10
        assert abs(w) <= 0.45
        assert abs(v - previous_v) <= 0.00101
        assert abs(w - previous_w) <= 0.00601
        if 0.2 < yaw < 1.3:
            moving_corner.append(v)
        x += (v * math.cos(yaw) - w * pivot * math.sin(yaw)) * 0.02
        y += (v * math.sin(yaw) + w * pivot * math.cos(yaw)) * 0.02
        yaw = wrap_angle(yaw + w * 0.02)
        previous_v, previous_w = v, w
        if done:
            break
    assert done
    assert tick * 0.02 < 30
    assert moving_corner and min(moving_corner) > 0.035
    assert math.dist((x, y), (0.838, 0.738)) <= 0.004
    assert abs(wrap_angle(yaw - math.pi / 2)) <= 0.0041
    assert route.step((x, y, yaw), 0.02) == (0, 0, True)


@pytest.mark.parametrize('goal', [(0, 0, 0), (0.838, 0.738, 0),
                                  (math.nan, 0.738, math.pi / 2)])
def test_smooth_route_rejects_unsupported_geometry(goal):
    with pytest.raises(ValueError):
        SmoothRoute(goal)


def test_transport_pose_is_folded_before_driving_and_keeps_vertical_wrist():
    cfg = yaml.safe_load(
        (ROOT / 'src/mobile_manipulator/config/pick_place_mobile.yaml').read_text()
    )['pick_place_supervisor']['ros__parameters']
    assert cfg['smooth_transport']
    assert cfg['motion_duration_s'] == 2.2
    pose = cfg['transport_pose']
    assert pose == pytest.approx([-0.05806242, 0.15, -1.05, 0, 0.9, 0])
    assert pose[1] + pose[2] + pose[4] == pytest.approx(0)
    assert MOBILE_STATE_SEQUENCE.index('FOLD') + 1 == MOBILE_STATE_SEQUENCE.index('NAVIGATE')
    assert 'FOLD' in RETENTION_STATES


def test_fold_reaches_measured_pose_before_base_can_move():
    transitions = []
    observations = []
    node = NS(
        state='FOLD', motion_done=True, motion_error=None,
        start_motion=lambda pose: observations.append(pose),
        joints_at=lambda pose, ignore_m6: ignore_m6 and pose == 'transport_pose',
        transition=transitions.append,
    )
    PickPlaceSupervisor.normal_tick(node)
    assert observations == ['transport_pose']
    assert transitions == ['NAVIGATE']


def test_mobile_arm_trajectory_requests_zero_endpoint_derivatives():
    goals = []
    future = NS(add_done_callback=lambda callback: None)
    action = NS(
        server_is_ready=lambda: True,
        send_goal_async=lambda goal, **kwargs: goals.append(goal) or future,
    )
    node = NS(
        mobile=True, motion_started=False, motion_retry_after_wall=0,
        action=action, positions=lambda name: [0.0] * 6,
        get_parameter=lambda name: NS(value={
            'joint_names': [f'poppy_m{i}_joint' for i in range(1, 7)],
            'motion_duration_s': 2.2,
        }[name]),
        motion_attempt=0, motion_generation=0, emit=lambda *args, **kwargs: None,
        feedback=lambda feedback: None,
    )
    PickPlaceSupervisor.start_motion(node, 'transport_pose')
    point = goals[0].trajectory.points[-1]
    assert list(point.velocities) == [0.0] * 6
    assert list(point.accelerations) == [0.0] * 6
    assert point.time_from_start.sec + point.time_from_start.nanosec / 1e9 == pytest.approx(2.2)


def test_fold_kinematics_reduce_extension_without_lowering_load(monkeypatch):
    import importlib.util

    import numpy as np

    monkeypatch.syspath_prepend(str(ROOT / 'scripts/cad'))
    spec = importlib.util.spec_from_file_location(
        'fold_fk_audit', ROOT / 'scripts/cad/validate_mechanical_assembly.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    root = ET.parse(ROOT / 'src/mobile_manipulator/urdf/mobile_manipulator.urdf.xacro')
    joints = {j.get('name'): j for j in root.findall('joint')}
    cfg = yaml.safe_load(
        (ROOT / 'src/mobile_manipulator/config/pick_place_mobile.yaml').read_text()
    )['pick_place_supervisor']['ros__parameters']
    extended, folded = np.array(cfg['lift']), np.array(cfg['transport_pose'])
    before, after = (module.fixed_tool_fk(joints, q) for q in (extended, folded))
    assert np.linalg.norm(after[:2, 3]) < 0.53 * np.linalg.norm(before[:2, 3])
    for fraction in np.linspace(0, 1, 101):
        transform = module.fixed_tool_fk(joints, extended + fraction * (folded - extended))
        assert transform[2, 3] >= before[2, 3] - 1e-6
        assert transform[:3, :3] == pytest.approx(before[:3, :3], abs=1e-7)
    # Peak velocity of a zero-velocity/acceleration quintic is 1.875*delta/T.
    for index in range(5):
        limit = float(joints[f'poppy_m{index + 1}_joint'].find('limit').get('velocity'))
        assert 1.875 * abs(folded[index] - extended[index]) / 2.2 < limit
