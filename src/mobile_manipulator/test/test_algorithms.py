import math

import pytest

from mobile_manipulator.ball_detector import (
    depth_to_range,
    estimate_sphere_distance,
    focal_length_from_fov,
)
from mobile_manipulator.metrics_logger import summarize_rows
from mobile_manipulator.target_trajectory import target_position
from mobile_manipulator.visual_tracker import clamp


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
