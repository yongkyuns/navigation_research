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


def validate_symmetric_tie(trace: Path, observed_rate: float) -> None:
    """Qualify only the known diagnostic tie; never exempt navigation errors.

    In this frozen scenario road centerlines are at north=0 and north=26 m.
    Every fallback fix must remain on the 13 m bisector and the estimator must
    abstain. This allows a nearest-road diagnostic to choose either equal road.
    """
    with trace.open(newline='') as stream:
        rows = list(csv.DictReader(stream))
    if len(rows) != 120:
        raise ValueError('Tie qualification needs all 120 original scenario epochs')
    for i, row in enumerate(rows, 1):
        close(float(row['t']), float(i), 'tie epoch')
        if (not math.isfinite(float(row['fallback_n']))
                or abs(float(row['fallback_n']) - 13.) > 1e-10
                or row['status'] != 'AMBIGUOUS' or row['route_group']
                or row['true_group'] != 'main'
                or row['naive_group'] not in ('main', 'parallel')):
            raise ValueError('Nearest-road discrepancy is not an unresolved symmetric tie')
        close(float(row['top_weight']), .5, 'equal route weights')
    rate = sum(r['naive_group'] == r['true_group'] for r in rows) / len(rows)
    close(observed_rate, rate, 'tie diagnostic versus trace')


def compare_metrics(actual: list[dict], expected: list[dict], traces: Path | None = None) -> list[dict]:
    def keyed(rows):
        result = {(r['scenario'], r['seed']): r for r in rows}
        if len(result) != len(rows):
            raise ValueError('Duplicate scenario/seed')
        return result
    candidate, reference = keyed(actual), keyed(expected)
    if candidate.keys() != reference.keys():
        raise ValueError('Scenario/seed coverage differs')
    diagnostics = []
    for key, row in reference.items():
        if candidate[key].keys() != row.keys():
            raise ValueError(f'{key}: metric schema differs')
        for field in row:
            if field == 'runtime_s':
                continue
            try:
                close(candidate[key][field], row[field], f'{key}/{field}')
            except ValueError:
                if traces is None or key[0] != 'ambiguous_parallel' or field != 'naive_correct_rate':
                    raise
                validate_symmetric_tie(traces / f'{key[0]}_seed{key[1]}.csv', candidate[key][field])
                diagnostics.append({'scenario': key[0], 'seed': key[1], 'field': field,
                                    'original': row[field], 'reproduced': candidate[key][field],
                                    'reason': 'Validated exact-bisector nearest-road tie; navigation remains ambiguous'})
    return diagnostics


def compare_summary(actual: dict, expected: dict, tie_diagnostics=()) -> None:
    if tie_diagnostics:
        # Recompute the diagnostic aggregate from the exact original per-run values
        # plus only the trace-qualified replacements. Do not exempt the summary.
        import numpy as np
        _, records = original_records()
        rates = {r['seed']: r['naive_correct_rate']
                 for r in json.loads(records['original-metrics.json'])
                 if r['scenario'] == 'ambiguous_parallel'}
        for diagnostic in tie_diagnostics:
            rates[diagnostic['seed']] = diagnostic['reproduced']
        values = list(rates.values())
        aggregate = {'mean': float(np.mean(values)), 'min': float(np.min(values)),
                     'max': float(np.max(values))}
        close(actual['ambiguous_parallel']['naive_correct_rate'], aggregate,
              'qualified nearest-road summary')
    def strip(d):
        return {scenario: {k: v for k, v in values.items()
                           if k != 'runtime_s' and not (tie_diagnostics
                           and scenario == 'ambiguous_parallel' and k == 'naive_correct_rate')}
                for scenario, values in d.items()}
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
    diagnostics = compare_metrics(metrics, json.loads(records['original-metrics.json']), results)
    compare_summary(json.loads((results / 'summary.json').read_text()),
                    json.loads(records['original-summary.json']), diagnostics)
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
                    'primary_navigation_metrics_match_original': True,
                    'all_non_timing_metrics_match_original': not diagnostics,
                    'tie_sensitive_baseline_diagnostics': diagnostics,
                    'relative_tolerance': TOLERANCE, 'absolute_tolerance': TOLERANCE,
                    'replayed_outputs': replayed, 'original_source_files_verified': check_sources(),
                    'synthetic_only': True, 'timing_fields': 'Fresh measurements, never overwritten with historical timings'}
    (results / 'verification.json').write_text(json.dumps(verification, indent=2) + '\n')
    print(json.dumps(verification, indent=2), flush=True)
    return verification
