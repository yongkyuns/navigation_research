"""Run the original 60-case evaluation in a NEW directory, preserving evidence."""
from __future__ import annotations
import argparse
import csv
import json
import math
import os
from pathlib import Path
import platform
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
PROJECT = ROOT / 'routefusion_prototype'


def close(actual, expected, label: str) -> None:
    if isinstance(expected, (int, float)):
        if not isinstance(actual, (int, float)) or not math.isclose(
                actual, expected, rel_tol=1e-9, abs_tol=1e-9):
            raise ValueError(f'{label}: {actual!r} != {expected!r}')
    elif actual != expected:
        raise ValueError(f'{label}: {actual!r} != {expected!r}')


def run(args: list[str], log: Path) -> None:
    env = dict(os.environ, OPENBLAS_NUM_THREADS='1', OMP_NUM_THREADS='1',
               MPLBACKEND='Agg', PYTHONDONTWRITEBYTECODE='1')
    with log.open('w') as stream:
        result = subprocess.run(args, cwd=PROJECT, env=env,
                                stdout=stream, stderr=subprocess.STDOUT)
    print(log.read_text(), end='')
    if result.returncode:
        raise RuntimeError(f'Command failed; see {log}')


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=ROOT / 'runs/reproduction')
    args = parser.parse_args()
    output = args.output.resolve()
    if output == PROJECT or PROJECT in output.parents:
        raise ValueError('Reruns must not modify the frozen project directory')
    output.mkdir(parents=True, exist_ok=False)
    run([sys.executable, str(ROOT / 'scripts/verify_snapshot.py')], output / 'snapshot.txt')
    run([sys.executable, '-m', 'pytest', '-q'], output / 'tests.txt')
    results = output / 'results'
    run([sys.executable, '-m', 'routefusion.simulate', '--seeds', '10',
         '--start-seed', '100', '--save-traces', '--output', str(results)],
        output / 'simulation.txt')
    expected = json.loads((PROJECT / 'results/metrics.json').read_text())
    actual = json.loads((results / 'metrics.json').read_text())
    key = lambda item: (item['scenario'], item['seed'])
    baseline = {key(item): item for item in expected}
    candidate = {key(item): item for item in actual}
    if len(candidate) != len(actual) or baseline.keys() != candidate.keys():
        raise ValueError('Scenario/seed coverage mismatch')
    for case, reference in baseline.items():
        row = candidate[case]
        if row.keys() != reference.keys():
            raise ValueError(f'Metric schema changed for {case}')
        for field in reference:
            if field != 'runtime_s':
                close(row[field], reference[field], f'{case}/{field}')
    replay = output / 'replay.jsonl'
    run([sys.executable, '-m', 'routefusion.replay',
         str(results / 'parallel_bias_example'), '--output', str(replay)],
        output / 'replay.txt')
    with (results / 'parallel_bias_seed100.csv').open(newline='') as stream:
        trace = list(csv.DictReader(stream))
    played = [json.loads(line) for line in replay.read_text().splitlines()]
    if len(trace) != len(played):
        raise ValueError('Replay length mismatch')
    for row, state in zip(trace, played):
        close(state['t'], float(row['t']), 'replay time')
        close(state['status'], row['status'], 'replay status')
        close(state['route_group'], row['route_group'] or None, 'replay route')
        close(state['position'][0], float(row['output_e']), 'replay east')
        close(state['position'][1], float(row['output_n']), 'replay north')
    run([sys.executable, '-m', 'routefusion.replay',
         str(PROJECT / 'results/parallel_bias_example'), '--output', str(output / 'frozen-replay.jsonl')],
        output / 'frozen-replay.txt')
    frozen = [json.loads(line) for line in (PROJECT / 'results/replay_check.jsonl').read_text().splitlines()]
    regenerated = [json.loads(line) for line in (output / 'frozen-replay.jsonl').read_text().splitlines()]
    if len(frozen) != len(regenerated):
        raise ValueError('Frozen replay length mismatch')
    for reference, state in zip(frozen, regenerated):
        close(state['status'], reference['status'], 'frozen replay status')
        close(state['route_edge'], reference['route_edge'], 'frozen replay edge')
        for axis in range(2):
            close(state['position'][axis], reference['position'][axis], 'frozen replay position')
            for other in range(2):
                close(state['covariance'][axis][other], reference['covariance'][axis][other], 'frozen replay covariance')
    run([sys.executable, str(ROOT / 'scripts/plot_evaluation.py'),
         str(results / 'summary.json'), str(output / 'synthetic_evaluation.png')],
        output / 'plot.txt')
    import numpy
    import scipy
    import matplotlib
    import PIL
    result = {
        'scenario_seed_runs': len(actual), 'epochs': sum(row['epochs'] for row in actual),
        'non_timing_metrics_match': True, 'relative_tolerance': 1e-9,
        'absolute_tolerance': 1e-9, 'replayed_outputs': len(played),
        'frozen_replayed_outputs': len(frozen),
        'frozen_evidence_modified': False, 'synthetic_only': True,
        'environment': {'python': sys.version, 'platform': platform.platform(),
                        'numpy': numpy.__version__, 'scipy': scipy.__version__,
                        'matplotlib': matplotlib.__version__, 'Pillow': PIL.__version__},
        'note': 'Fresh runtime measurements are under results/; historical timings are not compared.',
    }
    (output / 'verification.json').write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
