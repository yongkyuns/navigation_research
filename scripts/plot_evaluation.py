"""Render a synthetic-evaluation chart without assigning custom colors."""
from pathlib import Path
import argparse
import json
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np


def plot(summary_path: Path, output: Path) -> None:
    summary = json.loads(summary_path.read_text())
    scenarios = [('parallel_bias', 'Persistent GNSS bias'), ('turn_outage', 'Turn during GNSS outage'),
                 ('ambiguous_parallel', 'Unresolved parallel roads'), ('map_offset', 'Map shifted by 6 m'),
                 ('offroad_leave', 'Departure from mapped road'), ('missing_tiles', 'Missing map tiles')]
    a = [summary[key]['fallback_rmse_m']['mean'] for key, _ in scenarios]
    b = [summary[key]['output_rmse_m']['mean'] for key, _ in scenarios]
    fig, ax = plt.subplots(figsize=(10, 5.8))
    y = np.arange(len(scenarios))
    ax.barh(y - .18, a, height=.34, label='Map-free navigation')
    ax.barh(y + .18, b, height=.34, label='Route-conditioned output / fallback')
    for i, (left, right) in enumerate(zip(a, b)):
        ax.text(left + .15, i - .18, f'{left:.2f}', va='center', fontsize=9)
        ax.text(right + .15, i + .18, f'{right:.2f}', va='center', fontsize=9)
    ax.set_yticks(y, [label for _, label in scenarios])
    ax.invert_yaxis()
    ax.set_xlim(0, max(a + b) * 1.18)
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
