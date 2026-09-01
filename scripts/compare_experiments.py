#!/usr/bin/env python3
import json
import sys
from pathlib import Path


def require(condition, message):
    if not condition:
        raise RuntimeError(message)


def load_summary(output_dir, label):
    path = output_dir / f'{label}_summary.json'
    return json.loads(path.read_text(encoding='utf-8'))


def main():
    output_dir = Path(sys.argv[1])
    baseline = load_summary(output_dir, 'A')
    tracking = load_summary(output_dir, 'B')

    for label, summary in (('A', baseline), ('B', tracking)):
        require(
            summary['samples_after_warmup'] >= 400,
            f'{label}: insufficient samples',
        )
        require(
            summary['detection_rate_percent'] >= 90.0,
            f'{label}: detection rate below 90%',
        )
        require(
            summary['target_x_span_m'] >= 0.7,
            f'{label}: longitudinal target motion not demonstrated',
        )
        require(
            summary['target_y_span_m'] >= 0.7,
            f'{label}: lateral target motion not demonstrated',
        )
        require(
            summary['distance_estimation_mae_m'] <= 0.15,
            f'{label}: camera distance MAE exceeds 0.15 m',
        )

    require(
        baseline['command_active_percent'] <= 1.0,
        'A: baseline unexpectedly contains active commands',
    )
    require(
        baseline['robot_displacement_m'] <= 0.05,
        'A: baseline robot moved unexpectedly',
    )
    require(
        tracking['command_active_percent'] >= 80.0,
        'B: visual controller was not active often enough',
    )
    require(
        tracking['robot_displacement_m'] >= 0.25,
        'B: robot displacement is too small',
    )
    require(
        tracking['horizontal_error_rms'] <= 0.10,
        'B: horizontal tracking RMS exceeds 0.10',
    )
    require(
        tracking['steady_state_target_error_mae_m'] <= 0.20,
        'B: steady-state distance error exceeds 0.20 m',
    )
    require(
        tracking['target_distance_error_mae_m']
        < 0.5 * baseline['target_distance_error_mae_m'],
        'B: tracking did not halve the baseline distance error',
    )

    improvement_percent = 100.0 * (
        1.0
        - tracking['target_distance_error_mae_m']
        / baseline['target_distance_error_mae_m']
    )
    comparison = {
        'status': 'passed',
        'definition': {
            'A': 'moving target, perception and metrics active, tracking disabled',
            'B': 'same moving target, perception, metrics and tracking active',
        },
        'distance_error_improvement_percent': improvement_percent,
        'A': baseline,
        'B': tracking,
    }
    (output_dir / 'comparison.json').write_text(
        json.dumps(comparison, indent=2) + '\n',
        encoding='utf-8',
    )
    (output_dir / 'comparison.txt').write_text(
        '\n'.join(
            [
                'A/B visual tracking experiment: PASS',
                f'Distance-error improvement: {improvement_percent:.1f}%',
                (
                    'A target-distance MAE: '
                    f"{baseline['target_distance_error_mae_m']:.4f} m"
                ),
                (
                    'B target-distance MAE: '
                    f"{tracking['target_distance_error_mae_m']:.4f} m"
                ),
                (
                    'B steady-state MAE: '
                    f"{tracking['steady_state_target_error_mae_m']:.4f} m"
                ),
                (
                    'B horizontal RMS: '
                    f"{tracking['horizontal_error_rms']:.4f}"
                ),
            ]
        )
        + '\n',
        encoding='utf-8',
    )


if __name__ == '__main__':
    main()
