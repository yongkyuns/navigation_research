"""Render the original synthetic-evaluation chart without selecting plot colors."""
from __future__ import annotations
import argparse
import json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np


def plot(summary_path: Path, output: Path) -> None:
    summary = json.loads(summary_path.read_text())
    scenarios = [
        ('parallel_bias', 'Persistent GNSS bias'),
        ('turn_outage', 'Turn during GNSS outage'),
        ('ambiguous_parallel', 'Unresolved parallel roads'),
        ('map_offset', 'Map shifted by 6 m'),
        ('offroad_leave', 'Departure from mapped road'),
        ('missing_tiles', 'Missing map tiles'),
    ]
    baseline = [summary[key]['fallback_rmse_m']['mean'] for key, _ in scenarios]
    proposed = [summary[key]['output_rmse_m']['mean'] for key, _ in scenarios]
    fig, ax = plt.subplots(figsize=(10, 5.8))
    y = np.arange(len(scenarios))
    ax.barh(y - 0.18, baseline, height=0.34, label='Map-free navigation')
    ax.barh(y + 0.18, proposed, height=0.34, label='Route-conditioned output / fallback')
    for i, (a, b) in enumerate(zip(baseline, proposed)):
        ax.text(a + 0.15, i - 0.18, f'{a:.2f}', va='center', fontsize=9)
        ax.text(b + 0.15, i + 0.18, f'{b:.2f}', va='center', fontsize=9)
    ax.set_yticks(y, [label for _, label in scenarios])
    ax.invert_yaxis()
    ax.set_xlim(0, max(baseline + proposed) * 1.18)
    ax.set_xlabel('Mean per-run horizontal RMSE (metres)')
    ax.set_title('Synthetic evaluation: 10 held-out seeds per scenario\nNot M9 / LSM6DSO field measurements')
    ax.legend(loc='lower right')
    fig.tight_layout()
    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output, dpi=180, bbox_inches='tight')
    plt.close(fig)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('summary', type=Path)
    parser.add_argument('output', type=Path)
    args = parser.parse_args()
    plot(args.summary, args.output)
