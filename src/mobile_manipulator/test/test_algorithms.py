import math
from types import SimpleNamespace

from mobile_manipulator.ball_detector import (
    depth_to_range,
    estimate_sphere_distance,
    focal_length_from_fov,
)
from mobile_manipulator.metrics_logger import (
    reference_range_from_transform,
    ReferenceUnavailable,
    select_fresh_sample,
    summarize_rows,
)
from mobile_manipulator.pick_place_attach_gate import (
    evaluate_attach_gate,
    extract_single_model_pose,
)
from mobile_manipulator.pick_place_evaluator import (
    json_safe,
    update_spatial_stability,
)
from mobile_manipulator.pick_place_initializer import reset_ready
from mobile_manipulator.pick_place_supervisor import (
    may_retry_goal_rejection,
    NOMINAL_STATE_SEQUENCE,
    recovery_lowering_pose,
    recovery_terminal_state,
    state_has_timed_out,
    wall_watchdog_expired,
)
from mobile_manipulator.target_trajectory import target_position
from mobile_manipulator.visual_tracker import clamp
import pytest


def test_focal_length_matches_gazebo_camera():
    focal_length = focal_length_from_fov(640, math.pi / 3.0)
    assert focal_length == pytest.approx(554.256, abs=0.01)


def test_sphere_distance_inverts_angular_radius():
    focal_length = 554.256
    sphere_radius = 0.12
    expected_distance = 2.0
    apparent_radius = focal_length * sphere_radius / math.sqrt(
        expected_distance**2 - sphere_radius**2
    )
    assert estimate_sphere_distance(
        apparent_radius, focal_length, sphere_radius
    ) == pytest.approx(expected_distance)


def test_off_axis_depth_is_converted_to_range():
    assert depth_to_range(
        2.0,
        pixel_x=420.0,
        pixel_y=240.0,
        focal_x_px=500.0,
        focal_y_px=500.0,
        principal_x_px=320.0,
        principal_y_px=240.0,
    ) == pytest.approx(2.0 * math.sqrt(1.04))


def test_target_trajectory_is_deterministic_and_two_dimensional():
    first = target_position(7.0, 'moving', 2.0, 0.45, 0.65, 0.25)
    second = target_position(7.0, 'moving', 2.0, 0.45, 0.65, 0.25)
    assert first == second
    assert first[0] != pytest.approx(2.0)
    assert first[1] != pytest.approx(0.0)
    assert target_position(100.0, 'static', 2.0, 0.45, 0.65, 0.25) == (
        2.0,
        0.0,
    )


def test_five_lobe_clover_completes_two_fast_laps_per_minute():
    def position(elapsed_s):
        return target_position(
            elapsed_s,
            'trefoil',
            2.0,
            0.45,
            0.65,
            0.25,
            trefoil_radius_m=1.15,
            trefoil_lap_period_s=30.0,
            trefoil_lobes=5,
        )

    assert position(0.0) == pytest.approx(position(30.0))
    assert position(0.0) == pytest.approx(position(60.0))
    samples = [position(index * 0.05) for index in range(601)]
    x_values = [point[0] for point in samples]
    y_values = [point[1] for point in samples]
    path_length_m = sum(
        math.hypot(
            current[0] - previous[0],
            current[1] - previous[1],
        )
        for previous, current in zip(samples, samples[1:])
    )
    assert max(x_values) - min(x_values) > 1.8
    assert max(y_values) - min(y_values) > 1.8
    assert path_length_m > 11.0


def test_clamp():
    assert clamp(2.0, -1.0, 1.0) == 1.0
    assert clamp(-2.0, -1.0, 1.0) == -1.0
    assert clamp(0.25, -1.0, 1.0) == 0.25


def tracking_transform(stamp_ns, translation):
    return SimpleNamespace(
        header=SimpleNamespace(
            stamp=SimpleNamespace(
                sec=stamp_ns // 1_000_000_000,
                nanosec=stamp_ns % 1_000_000_000,
            )
        ),
        transform=SimpleNamespace(
            translation=SimpleNamespace(
                x=translation[0],
                y=translation[1],
                z=translation[2],
            ),
            rotation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
        ),
    )


def test_tracking_reference_follows_tf_when_camera_extrinsic_changes():
    query_ns = 10_000_000_000
    target_xyz = (2.0, 0.0, 0.12)
    original, _ = reference_range_from_transform(
        target_xyz,
        tracking_transform(query_ns, (-0.225, 0.0, -0.12)),
        query_ns,
        max_transform_age_s=0.1,
    )
    changed, _ = reference_range_from_transform(
        target_xyz,
        tracking_transform(query_ns, (-0.325, 0.0, -0.12)),
        query_ns,
        max_transform_age_s=0.1,
    )
    assert original == pytest.approx(1.775)
    assert changed == pytest.approx(1.675)
    assert changed != pytest.approx(original)


def test_tracking_reference_rejects_missing_transform():
    with pytest.raises(ReferenceUnavailable, match='transform_unavailable'):
        reference_range_from_transform(
            (2.0, 0.0, 0.12),
            None,
            query_ns=10_000_000_000,
            max_transform_age_s=0.1,
        )


def test_tracking_reference_rejects_stale_transform():
    with pytest.raises(ReferenceUnavailable, match='transform_stale'):
        reference_range_from_transform(
            (2.0, 0.0, 0.12),
            tracking_transform(9_000_000_000, (-0.225, 0.0, -0.12)),
            query_ns=10_000_000_000,
            max_transform_age_s=0.1,
        )


def test_tracking_reference_rejects_stale_or_future_target_data():
    samples = [(9_000_000_000, 'old')]
    with pytest.raises(ReferenceUnavailable, match='target_pose_stale'):
        select_fresh_sample(
            samples,
            query_ns=10_000_000_000,
            max_age_s=0.25,
        )
    with pytest.raises(ReferenceUnavailable, match='target_pose_from_future'):
        select_fresh_sample(
            [(11_000_000_000, 'future')],
            query_ns=10_000_000_000,
            max_age_s=0.25,
        )


def test_metric_summary_counts_valid_samples():
    rows = [
        {
            'elapsed_s': float(index),
            'valid_detection': index != 6,
            'estimation_error_m': 0.1,
            'horizontal_error': 0.05,
            'distance_target_error_m': 0.1,
            'linear_command_mps': 0.2,
            'angular_command_radps': 0.0,
            'target_x_m': 2.0 + index * 0.1,
            'target_y_m': index * 0.2,
            'robot_x_m': index * 0.05,
            'robot_y_m': 0.0,
        }
        for index in range(10)
    ]
    summary = summarize_rows(rows, warmup_s=5.0, target_tolerance_m=0.2)
    assert summary['samples_after_warmup'] == 5
    assert summary['valid_detections'] == 4
    assert summary['detection_rate_percent'] == pytest.approx(80.0)
    assert summary['robot_displacement_m'] == pytest.approx(0.2)


def model_pose_transform(x, y=0.0, z=0.0):
    return SimpleNamespace(
        transform=SimpleNamespace(
            translation=SimpleNamespace(x=x, y=y, z=z)
        )
    )


def test_model_pose_requires_unambiguous_model_specific_pose_v():
    message = SimpleNamespace(
        transforms=[
            model_pose_transform(99.0),
            model_pose_transform(1.0, 2.0, 3.0),
        ]
    )
    assert extract_single_model_pose(message) == (1.0, 2.0, 3.0)
    assert extract_single_model_pose(
        SimpleNamespace(transforms=[model_pose_transform(1.0)])
    ) is None
    assert extract_single_model_pose(
        SimpleNamespace(
            transforms=[
                model_pose_transform(1.0),
                model_pose_transform(2.0),
                model_pose_transform(3.0),
            ]
        )
    ) is None


def valid_attach_snapshot():
    return {
        'state': 'VERIFY_GRASP',
        'object_present': True,
        'fixed_contact_fresh': True,
        'moving_contact_fresh': True,
        'contact_persistence_s': 0.15,
        'geometry_error_m': 0.006,
        'geometry_angle_deg': 3.0,
        'close_commanded': True,
        'm6_rad': 0.70,
        'base_stopped': True,
        'tf_valid': True,
    }


GATE_LIMITS = {
    'contact_persistence_s': 0.15,
    'position_tolerance_m': 0.020,
    'angle_tolerance_deg': 5.0,
    'close_feedback_max_rad': 0.78,
}


def test_attach_gate_accepts_only_the_complete_contract():
    allowed, reasons = evaluate_attach_gate(
        valid_attach_snapshot(), GATE_LIMITS
    )
    assert allowed
    assert reasons == []


@pytest.mark.parametrize(
    ('changes', 'expected_reason'),
    [
        ({'state': 'CLOSE'}, 'state_not_authorized'),
        ({'object_present': False}, 'object_missing'),
        ({'fixed_contact_fresh': False}, 'fixed_contact_missing_or_stale'),
        ({'moving_contact_fresh': False}, 'moving_contact_missing_or_stale'),
        (
            {'contact_persistence_s': 0.149},
            'bilateral_contact_not_persistent',
        ),
        ({'geometry_error_m': 0.021}, 'geometry_position'),
        ({'geometry_error_m': 0.120}, 'geometry_position'),
        ({'geometry_angle_deg': 5.1}, 'geometry_orientation'),
        ({'close_commanded': False}, 'close_not_commanded'),
        ({'m6_rad': 0.79}, 'close_feedback'),
        ({'base_stopped': False}, 'base_not_stopped'),
        ({'tf_valid': False}, 'tf_invalid'),
    ],
)
def test_attach_gate_fails_closed_for_each_negative(changes, expected_reason):
    snapshot = valid_attach_snapshot()
    snapshot.update(changes)
    allowed, reasons = evaluate_attach_gate(snapshot, GATE_LIMITS)
    assert not allowed
    assert expected_reason in reasons


def test_proximity_without_bilateral_contacts_never_authorizes_attach():
    snapshot = valid_attach_snapshot()
    snapshot.update({
        'fixed_contact_fresh': False,
        'moving_contact_fresh': False,
        'contact_persistence_s': 0.0,
    })
    allowed, reasons = evaluate_attach_gate(snapshot, GATE_LIMITS)
    assert not allowed
    assert set(reasons) >= {
        'fixed_contact_missing_or_stale',
        'moving_contact_missing_or_stale',
        'bilateral_contact_not_persistent',
    }


def test_post_release_stability_requires_continuous_spatial_envelope():
    target = (0.044, -0.213)
    anchor, since, best = update_spatial_stability(
        None, None, 0.0, 10.0, (0.044, -0.213, 0.1625),
        target, 0.030, 0.0005, False,
    )
    anchor, since, best = update_spatial_stability(
        anchor, since, best, 12.1, (0.0442, -0.213, 0.1625),
        target, 0.030, 0.0005, False,
    )
    assert best == pytest.approx(2.1)
    anchor, since, best = update_spatial_stability(
        anchor, since, best, 13.0, (0.045, -0.213, 0.1625),
        target, 0.030, 0.0005, False,
    )
    assert since == pytest.approx(13.0)
    assert best == pytest.approx(2.1)
    anchor, since, best = update_spatial_stability(
        anchor, since, best, 14.0, (0.045, -0.213, 0.1625),
        target, 0.030, 0.0005, True,
    )
    assert anchor is None
    assert since is None
    assert best == pytest.approx(2.1)


def test_json_safe_recursively_replaces_nonfinite_measurements():
    result = json_safe({
        'lift_m': math.nan,
        'placement_error_m': math.inf,
        'samples': [1.0, -math.inf],
    })
    assert result == {
        'lift_m': None, 'placement_error_m': None, 'samples': [1.0, None],
    }


def test_initializer_requires_free_object_feedback_and_fresh_pose():
    assert reset_ready(True, 'detached', 0.25, True)
    assert not reset_ready(True, 'attached', 0.25, True)
    assert not reset_ready(True, 'detached', 0.25, False)
    assert reset_ready(False, None, 0.25, True)


def test_nominal_state_machine_sequence_is_exact_and_terminal():
    assert NOMINAL_STATE_SEQUENCE == (
        'IDLE', 'FREEZE_BASE', 'OPEN', 'PREGRASP', 'APPROACH', 'CLOSE',
        'VERIFY_GRASP', 'LIFT', 'HOLD', 'TRANSFER', 'LOWER', 'RELEASE',
        'RETREAT', 'DONE',
    )


def test_controller_unavailable_reaches_timeout_policy():
    assert not state_has_timed_out(15.0, 15.0)
    assert state_has_timed_out(15.001, 15.0)


def test_wall_watchdog_detects_a_stalled_simulation_clock():
    assert not wall_watchdog_expired(3.0, 3.0)
    assert wall_watchdog_expired(3.001, 3.0)


def test_goal_rejection_retry_is_bounded():
    assert may_retry_goal_rejection(1, 1)
    assert not may_retry_goal_rejection(2, 1)


def test_cancel_during_transport_lowers_before_release_and_cancels():
    assert recovery_lowering_pose(True, True) == 'place'
    assert recovery_lowering_pose(True, False) == 'grasp'
    assert recovery_lowering_pose(False, True) is None
    assert recovery_terminal_state(True) == 'CANCELLED'
    assert recovery_terminal_state(False) == 'FAILED'
