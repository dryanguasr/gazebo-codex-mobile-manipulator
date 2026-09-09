#!/usr/bin/env python3
"""Audit a nominal mobile run using both its result and raw time-series samples."""

import argparse
import csv
import json
import math
from pathlib import Path


def audit(directory):
    result = json.loads((directory / 'run.json').read_text())
    with (directory / 'samples.csv').open(newline='') as stream:
        rows = list(csv.DictReader(stream))
    samples = []
    for row in rows:
        try:
            values = [float(row[key]) for key in (
                'sim_time_s', 'base_x_m', 'base_y_m', 'base_yaw_rad',
                'object_x_m', 'object_y_m', 'object_z_m',
                'grasp_x_m', 'grasp_y_m', 'grasp_z_m',
            )]
        except (KeyError, ValueError):
            continue
        if all(math.isfinite(v) for v in values):
            samples.append((row, values))
    if not samples:
        return {'status': 'failed', 'reason': 'No finite mobile samples'}
    path_length = sum(
        math.dist(a[1][1:3], b[1][1:3]) for a, b in zip(samples, samples[1:])
    )
    yaw_change = abs(math.atan2(
        math.sin(samples[-1][1][3] - samples[0][1][3]),
        math.cos(samples[-1][1][3] - samples[0][1][3]),
    ))
    navigation = [
        (row, values) for row, values in samples
        if row['state'] in ('NAVIGATE', 'DOCK_BASE')
    ]
    max_error = max(
        (math.dist(v[4:7], v[7:10]) for _, v in navigation), default=math.inf
    )
    phases = [t['to_state'] for t in result['transitions']]
    expected = [
        'FREEZE_BASE', 'OPEN', 'PREGRASP', 'APPROACH', 'CLOSE',
        'VERIFY_GRASP', 'LIFT', 'HOLD', 'FOLD', 'NAVIGATE', 'DOCK_BASE',
        'TRANSFER', 'LOWER', 'RELEASE', 'RETREAT', 'DONE',
    ]
    folded_run = 'FOLD' in phases or 'transport_posture_samples' in result
    if not folded_run:
        expected.remove('FOLD')
    # Evaluator startup may occur after FREEZE_BASE; manipulation must be complete.
    if phases and phases[0] == 'OPEN':
        expected = expected[1:]
    posture = [-0.05806242, 0.15, -1.05, 0.0, 0.9]
    posture_errors = [
        abs(float(row.get(f'm{i}_rad', 'nan')) - target)
        for row, _ in navigation for i, target in enumerate(posture, 1)
    ]
    criteria = {
        'runtime_all_criteria': result['status'] == 'passed'
        and all(result['criteria'].values()),
        'actual_mobile_sequence': phases == expected,
        'raw_folded_posture_during_transport': not folded_run or (
            bool(posture_errors)
            and all(math.isfinite(v) and v <= 0.035 for v in posture_errors)
        ),
        'raw_base_path_at_least_1_3m': path_length >= 1.3,
        'raw_base_yaw_change_at_least_80deg': yaw_change >= math.radians(80),
        'raw_retention_samples': len(navigation) >= 100,
        'raw_joint_kept_during_transport': bool(navigation)
        and all(row['attached'] == 'True' for row, _ in navigation),
        'raw_retention_error_under_8mm': max_error <= 0.008,
        'one_conditional_attach_and_release': result['attach_count'] == 1
        and result['detach_count'] == 1 and result['simulator_assisted'],
        'orientation_measured_and_fixed': result['relative_orientation_samples'] >= 100
        and result['maximum_relative_rotation_deg'] <= 1.0,
        'station_separation_over_1_3m': math.dist(
            result['initial_object_pose_m'][:2], result['final_object_pose_m'][:2]
        ) > 1.3,
    }
    return {
        'status': 'passed' if all(criteria.values()) else 'failed',
        'criteria': criteria,
        'folded_posture_expected': folded_run,
        'raw_base_path_m': path_length,
        'raw_base_yaw_change_deg': math.degrees(yaw_change),
        'raw_navigation_samples': len(navigation),
        'raw_max_retention_error_m': max_error,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory', type=Path)
    args = parser.parse_args()
    report = audit(args.directory)
    print(json.dumps(report, indent=2))
    return 0 if report['status'] == 'passed' else 1


if __name__ == '__main__':
    raise SystemExit(main())
