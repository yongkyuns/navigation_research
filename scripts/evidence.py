"""Shared verification helpers; historical records and fresh runs stay separate."""
from __future__ import annotations
import csv
import hashlib
import json
import math
import os
from pathlib import Path, PurePosixPath
import platform
import subprocess
import sys
import zlib

ROOT = Path(__file__).resolve().parents[1]
PROJECT = ROOT / 'routefusion_prototype'
TOLERANCE = 1e-9


def sha(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def original_records() -> tuple[dict, dict[str, bytes]]:
    ref = json.loads((ROOT / 'provenance/original-numerical-record.json').read_text())
    records = {}
    for name, item in ref['files'].items():
        path = PurePosixPath(name)
        if path.is_absolute() or len(path.parts) != 1 or name in ('.', '..'):
            raise ValueError(f'Unsafe provenance filename: {name}')
        preserved = ROOT / 'provenance' / name
        if preserved.exists():
            data = preserved.read_bytes()
        else:
            if item.get('encoding') != 'zlib-parts' or not item.get('parts'):
                raise ValueError(f'Historical record missing: {name}')
            chunks = []
            for part in item['parts']:
                if PurePosixPath(part).name != part or part in ('.', '..'):
                    raise ValueError('Unsafe transfer-part filename')
                chunks.append((ROOT / 'provenance/import-parts' / part).read_bytes())
            data = zlib.decompress(b''.join(chunks))
        if sha(data) != item['sha256']:
            raise ValueError(f'Historical record checksum mismatch: {name}')
        records[name] = data
    return ref, records


def check_sources() -> int:
    ref, _ = original_records()
    for name, digest in ref['source_sha256'].items():
        path = PurePosixPath(name)
        if path.is_absolute() or '..' in path.parts:
            raise ValueError('Unsafe source manifest entry')
        if sha((PROJECT / name).read_bytes()) != digest:
            raise ValueError(f'Original source/document changed: {name}')
    return len(ref['source_sha256'])


def close(actual, expected, label='value') -> None:
    if isinstance(expected, bool) or expected is None or isinstance(expected, str):
        if type(actual) is not type(expected) or actual != expected:
            raise ValueError(f'{label}: {actual!r} != {expected!r}')
    elif isinstance(expected, (int, float)):
        if (isinstance(actual, bool) or not isinstance(actual, (int, float))
                or not math.isfinite(actual) or not math.isfinite(expected)
                or not math.isclose(actual, expected, rel_tol=TOLERANCE, abs_tol=TOLERANCE)):
            raise ValueError(f'{label}: {actual!r} != {expected!r}')
    elif isinstance(expected, dict):
        if not isinstance(actual, dict) or actual.keys() != expected.keys():
            raise ValueError(f'{label}: dictionary keys differ')
        for key in expected:
            close(actual[key], expected[key], f'{label}/{key}')
    elif isinstance(expected, list):
        if not isinstance(actual, list) or len(actual) != len(expected):
            raise ValueError(f'{label}: list length differs')
        for index, (a, b) in enumerate(zip(actual, expected)):
            close(a, b, f'{label}/{index}')
    else:
        raise TypeError(f'Unsupported comparison type: {type(expected)}')


def compare_metrics(actual: list[dict], expected: list[dict]) -> None:
    def keyed(rows):
        result = {(r['scenario'], r['seed']): r for r in rows}
        if len(result) != len(rows):
            raise ValueError('Duplicate scenario/seed')
        return result
    candidate, reference = keyed(actual), keyed(expected)
    if candidate.keys() != reference.keys():
        raise ValueError('Scenario/seed coverage differs')
    for key, row in reference.items():
        if candidate[key].keys() != row.keys():
            raise ValueError(f'{key}: metric schema differs')
        for field in row:
            if field != 'runtime_s':
                close(candidate[key][field], row[field], f'{key}/{field}')


def compare_summary(actual: dict, expected: dict) -> None:
    strip = lambda d: {s: {k: v for k, v in values.items() if k != 'runtime_s'}
                       for s, values in d.items()}
    close(strip(actual), strip(expected), 'summary')


def execute(command: list[str], log: Path) -> None:
    env = dict(os.environ, OPENBLAS_NUM_THREADS='1', OMP_NUM_THREADS='1',
               MPLBACKEND='Agg', PYTHONDONTWRITEBYTECODE='1',
               PYTHONPATH=str(PROJECT) + os.pathsep + str(ROOT))
    with log.open('w') as stream:
        result = subprocess.run(command, cwd=ROOT, env=env,
                                stdout=stream, stderr=subprocess.STDOUT)
    print(log.read_text(), end='', flush=True)
    if result.returncode:
        raise RuntimeError(f'Command failed; see {log}')


def evaluate(output: Path) -> dict:
    output = output.resolve()
    if output == PROJECT or PROJECT in output.parents:
        raise ValueError('Fresh evaluations must not overwrite the published project')
    check_sources()
    _, records = original_records()
    output.mkdir(parents=True, exist_ok=False)
    results = output / 'results'
    results.mkdir()
    execute([sys.executable, '-m', 'pytest', '-q', str(PROJECT / 'tests'), str(ROOT / 'tests')],
            results / 'tests.txt')
    execute([sys.executable, '-m', 'routefusion.simulate', '--seeds', '10',
             '--start-seed', '100', '--save-traces', '--output', str(results)],
            results / 'evaluation_console.txt')
    metrics = json.loads((results / 'metrics.json').read_text())
    compare_metrics(metrics, json.loads(records['original-metrics.json']))
    compare_summary(json.loads((results / 'summary.json').read_text()),
                    json.loads(records['original-summary.json']))
    scenarios = sorted({m['scenario'] for m in metrics})
    replays = results / 'replays'
    replays.mkdir()
    replayed = 0
    for name in scenarios:
        path = replays / f'{name}.jsonl'
        execute([sys.executable, '-m', 'routefusion.replay', str(results / f'{name}_example'),
                 '--output', str(path)], output / f'replay-{name}.txt')
        played = [json.loads(line) for line in path.read_text().splitlines()]
        with (results / f'{name}_seed100.csv').open(newline='') as stream:
            rows = list(csv.DictReader(stream))
        if len(played) != len(rows):
            raise ValueError(f'{name}: replay length differs')
        for row, state in zip(rows, played):
            close(state['t'], float(row['t']), 'replay time')
            close(state['status'], row['status'], 'replay status')
            close(state['route_group'], row['route_group'] or None, 'replay road')
            close(state['position'], [float(row['output_e']), float(row['output_n'])], 'replay position')
            close(state['fallback_position'], [float(row['fallback_e']), float(row['fallback_n'])],
                  'replay fallback')
        published = PROJECT / 'results/replays' / path.name
        if published.exists():
            reference = [json.loads(line) for line in published.read_text().splitlines()]
            close(played, reference, f'{name}/published replay')
        replayed += len(played)
    # Keep the original documented replay path usable, while testing all six cases.
    (results / 'replay_check.jsonl').write_bytes((replays / 'parallel_bias.jsonl').read_bytes())
    execute([sys.executable, str(ROOT / 'scripts/plot_evaluation.py'),
             str(results / 'summary.json'), str(results / 'figures/synthetic_evaluation.png')],
            output / 'plot.txt')
    import numpy, scipy, matplotlib, PIL
    environment = {'python': sys.version, 'platform': platform.platform(),
                   'numpy': numpy.__version__, 'scipy': scipy.__version__,
                   'matplotlib': matplotlib.__version__, 'Pillow': PIL.__version__,
                   'kind': 'Fresh synthetic reproduction; not the original experiment runtime',
                   'github_sha': os.environ.get('GITHUB_SHA'),
                   'github_run_id': os.environ.get('GITHUB_RUN_ID')}
    (results / 'environment.json').write_text(json.dumps(environment, indent=2) + '\n')
    verification = {'scenario_seed_runs': len(metrics), 'epochs': sum(m['epochs'] for m in metrics),
                    'original_non_timing_metrics_match': True, 'original_summary_matches': True,
                    'relative_tolerance': TOLERANCE, 'absolute_tolerance': TOLERANCE,
                    'replayed_outputs': replayed, 'original_source_files_verified': check_sources(),
                    'synthetic_only': True, 'timing_fields': 'Fresh measurements, never overwritten with historical timings'}
    (results / 'verification.json').write_text(json.dumps(verification, indent=2) + '\n')
    print(json.dumps(verification, indent=2), flush=True)
    return verification
