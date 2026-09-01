#!/usr/bin/env python3
import json
import math
import re
import sys
from pathlib import Path


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
    require(names_match is not None, 'Could not parse joint names')
    require(positions_match is not None, 'Could not parse joint positions')
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


def main():
    results = Path(sys.argv[1])
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

    positions = joint_positions(results / 'joint_states_after_arm.txt')
    expected_positions = {
        'arm_base_yaw': 0.2,
        'shoulder_pitch': -0.3,
        'elbow_pitch': 0.5,
        'wrist_pitch': -0.2,
        'left_finger_joint': 0.02,
        'right_finger_joint': -0.02,
    }
    for joint, expected in expected_positions.items():
        require(joint in positions, f'Joint missing from state: {joint}')
        require(
            abs(positions[joint] - expected) < 0.03,
            f'{joint} did not reach the commanded position',
        )

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
        'arm_commanded_positions_rad_or_m': expected_positions,
    }
    (results / 'summary.json').write_text(
        json.dumps(summary, indent=2) + '\n',
        encoding='utf-8',
    )


if __name__ == '__main__':
    main()
