#!/usr/bin/env python3
"""Plot measured base trajectories from two pick-and-place CSV recordings."""

import argparse
import csv
import math
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402


def trajectory(path):
    with path.open(newline='') as stream:
        rows = list(csv.DictReader(stream))
    points = []
    for row in rows:
        try:
            x, y = float(row['base_x_m']), float(row['base_y_m'])
        except (KeyError, ValueError):
            continue
        if math.isfinite(x) and math.isfinite(y):
            points.append((x, y))
    return points


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--baseline', type=Path, required=True)
    parser.add_argument('--current', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    fig, axis = plt.subplots(figsize=(7, 6))
    for path, label, color in (
        (args.baseline, 'Anterior: parada y giro', '#64748b'),
        (args.current, 'Actual: curva continua', '#007c91'),
    ):
        points = trajectory(path)
        axis.plot([p[0] for p in points], [p[1] for p in points],
                  label=label, color=color, linewidth=2.3)
    axis.scatter([-.044, 1.063], [-.225, .694], marker='s', s=70,
                 color=['#3182ce', '#38a169'], label='Pedestales')
    axis.scatter([0, .838], [0, .738], marker='o', s=35, color='#111827')
    axis.set(xlabel='X mundial [m]', ylabel='Y mundial [m]',
             title='Trayectoria real del origen del carro')
    axis.set_aspect('equal')
    axis.grid(alpha=.25)
    axis.legend(loc='lower right')
    fig.tight_layout()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.output, dpi=160)
    plt.close(fig)


if __name__ == '__main__':
    main()
