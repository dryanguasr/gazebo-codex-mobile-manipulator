#!/usr/bin/env python3
import json
import math
import re
import sys
from pathlib import Path


JOINTS = [
    'poppy_m1_joint',
    'poppy_m2_joint',
    'poppy_m3_joint',
    'poppy_m4_joint',
    'poppy_m5_joint',
    'poppy_m6_joint',
]
POSES = {
    'pose_1': dict(zip(JOINTS, [0.25, -0.35, 0.30, -0.25, 0.20, 0.45])),
    'pose_2': dict(zip(JOINTS, [-0.20, 0.30, -0.25, 0.35, -0.30, 0.85])),
}
TOLERANCE_RAD = 0.03


def require(condition, message):
    if not condition:
        raise RuntimeError(message)


def odom_xy(path):
    text = path.read_text(encoding='utf-8')
    match = re.search(
        r'pose:\s+pose:\s+position:\s+x:\s*([-+0-9.eE]+)'
        r'\s+y:\s*([-+0-9.eE]+)',
        text,
    )
    require(match is not None, f'Could not parse odometry: {path}')
    return float(match.group(1)), float(match.group(2))


def tf_xy(path):
    text = path.read_text(encoding='utf-8')
    match = re.search(
        r'Translation: \[([-+0-9.eE]+), ([-+0-9.eE]+),',
        text,
    )
    require(match is not None, f'Could not parse TF: {path}')
    return float(match.group(1)), float(match.group(2))


def joint_positions(path):
    text = path.read_text(encoding='utf-8')
    names_match = re.search(r'name:\n((?:- [^\n]+\n)+)', text)
    positions_match = re.search(
        r'position:\n((?:- [-+.0-9a-zA-Z]+\n)+)',
        text,
    )
    require(names_match is not None, f'Could not parse joint names: {path}')
    require(
        positions_match is not None,
        f'Could not parse joint positions: {path}',
    )
    names = re.findall(r'^- (.+)$', names_match.group(1), re.MULTILINE)
    positions = [
        float(value)
        for value in re.findall(
            r'^- ([-+.0-9eE]+)$',
            positions_match.group(1),
            re.MULTILINE,
        )
    ]
    require(len(names) == len(positions), 'Joint name/position size mismatch')
    return dict(zip(names, positions))


def validate_pose(results, label, expected):
    observed = joint_positions(results / f'joint_states_after_{label}.txt')
    errors = {}
    for joint, target in expected.items():
        require(joint in observed, f'Joint missing from {label}: {joint}')
        errors[joint] = abs(observed[joint] - target)
        require(
            errors[joint] < TOLERANCE_RAD,
            f'{joint} failed {label}: target={target}, '
            f'observed={observed[joint]}, error={errors[joint]}',
        )
    return observed, errors


def main():
    results = Path(sys.argv[1])
    initial_positions = joint_positions(results / 'joint_states.txt')
    for joint in JOINTS:
        require(joint in initial_positions, f'Expected Poppy joint missing: {joint}')

    before_x, before_y = odom_xy(results / 'odom_before.txt')
    after_x, after_y = odom_xy(results / 'odom_after.txt')
    displacement = math.hypot(after_x - before_x, after_y - before_y)
    require(displacement > 0.20, f'Base displacement too small: {displacement}')
    tf_after_x, tf_after_y = tf_xy(results / 'tf_after.txt')
    require(
        math.hypot(tf_after_x - after_x, tf_after_y - after_y) < 0.02,
        'TF translation does not match final odometry',
    )

    camera_info = (results / 'camera_info.txt').read_text(encoding='utf-8')
    focal_match = re.search(r'k:\s*\n- ([-+0-9.eE]+)', camera_info)
    require(focal_match is not None, 'CameraInfo focal length was not found')
    focal_length_px = float(focal_match.group(1))
    require(540.0 < focal_length_px < 570.0, 'Unexpected camera focal length')

    measurement = (results / 'ball_measurement.txt').read_text(
        encoding='utf-8'
    )
    distance_match = re.search(r'z:\s*([-+0-9.eE]+)', measurement)
    require(distance_match is not None, 'Ball distance was not numeric')
    estimated_distance_m = float(distance_match.group(1))
    require(
        1.0 < estimated_distance_m < 2.5,
        f'Unexpected ball distance: {estimated_distance_m}',
    )

    pose_results = {}
    for label, expected in POSES.items():
        observed, errors = validate_pose(results, label, expected)
        pose_results[label] = {
            'commanded_rad': expected,
            'observed_rad': {joint: observed[joint] for joint in JOINTS},
            'absolute_error_rad': errors,
            'max_absolute_error_rad': max(errors.values()),
        }

    for joint in JOINTS:
        delta = abs(
            pose_results['pose_2']['observed_rad'][joint]
            - pose_results['pose_1']['observed_rad'][joint]
        )
        require(delta > 0.15, f'{joint} appears blocked across the two poses')

    summary = {
        'status': 'passed',
        'odometry_topic': '/base_controller/odom',
        'tf': 'odom -> base_footprint',
        'initial_position_m': {'x': before_x, 'y': before_y},
        'final_position_m': {'x': after_x, 'y': after_y},
        'final_tf_translation_m': {'x': tf_after_x, 'y': tf_after_y},
        'base_displacement_m': displacement,
        'camera_focal_length_px': focal_length_px,
        'initial_estimated_ball_distance_m': estimated_distance_m,
        'arm_joint_names': JOINTS,
        'arm_tolerance_rad': TOLERANCE_RAD,
        'arm_pose_results': pose_results,
    }
    (results / 'summary.json').write_text(
        json.dumps(summary, indent=2) + '\n',
        encoding='utf-8',
    )


if __name__ == '__main__':
    main()
