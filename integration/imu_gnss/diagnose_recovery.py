"""Read-only analysis of existing evaluate_relocalization.py evidence.

No native estimator is imported or run. The original case file supplies the
unchanged corridor tolerance, withholding boundary and sampled-hold duration.
A selected road, an inside native position and a completed hold are distinct.
"""
from __future__ import annotations

import argparse
from bisect import bisect_left
import csv
import hashlib
import json
import math
from pathlib import Path
import re
from typing import Any


def digest(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def inventory(root: Path) -> dict[str, str]:
    return {str(p.relative_to(root)): digest(p)
            for p in sorted(root.rglob('*')) if p.is_file()}


def boolean(value: Any) -> bool:
    if value is True or value == 'True':
        return True
    if value is False or value == 'False':
        return False
    raise ValueError(f'Invalid CSV boolean: {value!r}')


def number(value: Any) -> float:
    result = float(value)
    if not math.isfinite(result):
        raise ValueError('Nonfinite diagnostic input')
    return result


def samples(rows: list[dict]) -> list[dict]:
    """Match the evaluator: last/post-map row owns duplicate timestamps."""
    unique = {}
    previous = -math.inf
    for row in rows:
        t = number(row['elapsed_s'])
        if t < 0 or t < previous:
            raise ValueError('Nonchronological diagnostic input')
        previous = t
        unique[t] = row
    if len(unique) < 2:
        raise ValueError('At least two distinct output epochs are required')
    return list(unique.values())


def intervals(times: list[float], flags: list[bool]) -> list[dict]:
    """Describe accepted samples, never interpolate a passing hold endpoint."""
    if len(times) != len(flags):
        raise ValueError('Mismatched times and flags')
    result = []
    i = 0
    while i < len(times):
        if not flags[i]:
            i += 1
            continue
        j = i
        while j + 1 < len(times) and flags[j + 1]:
            j += 1
        result.append({'first_s': times[i], 'last_sample_s': times[j],
                       'sample_span_s': times[j] - times[i],
                       'next_rejected_sample_s': times[j + 1] if j + 1 < len(times) else None})
        i = j + 1
    return result


def accepted_metrics(rows: list[dict], cfg: dict) -> dict:
    data = samples(rows)
    times = [number(r['elapsed_s']) for r in data]
    tolerance = number(cfg['inside_tolerance_m'])
    hold = number(cfg['hold_duration_s'])
    boundary = number(cfg['withheld_gnss_duration_s'])
    if tolerance < 0 or hold <= 0 or boundary <= 0:
        raise ValueError('Invalid unchanged case policy')
    selected = [boolean(r['auto_latched']) and boolean(r['auto_selected_on_supplied_route'])
                for r in data]
    accepted = [ok and number(r['auto_route_excess_m']) <= tolerance
                for ok, r in zip(selected, data)]
    spans = intervals(times, accepted)
    # Use the same floating-point comparison as searchsorted(t_start + hold).
    first_hold = next((s['first_s'] for s in spans
                       if s['last_sample_s'] >= s['first_s'] + hold), None)
    # A nominal endpoint before the boundary is not enough: the first
    # accepted sample confirming the hold must also precede that boundary.
    completion = (None if first_hold is None else
                  times[bisect_left(times, first_hold + hold)])
    durations = [b - a for a, b in zip(times, times[1:])]
    before = [max(0., min(b, boundary) - a) for a, b in zip(times, times[1:])]
    if sum(before) <= 0:
        raise ValueError('Trace does not cover the withholding window')
    last_entry = next((i + 1 for i in range(len(accepted) - 1, -1, -1)
                       if not accepted[i]), 0)
    stays = (times[last_entry] if last_entry < len(times)
             and times[-1] - times[last_entry] >= hold else None)
    metrics = {'inside_fraction': sum(d for d, ok in zip(durations, accepted) if ok) / sum(durations),
               'first_10s_inside_fraction': sum(d for d, ok in zip(before, accepted) if ok) / sum(before),
               'first_5s_hold_entry_s': first_hold, 'entry_then_stays_until_end_s': stays}
    first_selected = next((t for t, ok in zip(times, selected) if ok), None)
    first_accepted = spans[0]['first_s'] if spans else None
    last_before = max((i for i, t in enumerate(times) if t < boundary), default=None)
    interruption_status = None
    if spans and spans[0]['next_rejected_sample_s'] is not None:
        interruption_status = next(r['auto_status'] for r in data
                                   if number(r['elapsed_s']) == spans[0]['next_rejected_sample_s'])
    return {'metrics': metrics, 'first_5s_hold_completion_sample_s': completion,
            'first_selected_route_s': first_selected,
            'first_accepted_native_sample_s': first_accepted,
            'accepted_sample_intervals': spans,
            'first_interruption_status': interruption_status,
            'last_sample_before_boundary_s': times[last_before] if last_before is not None else None,
            'last_excess_before_boundary_m': number(data[last_before]['auto_route_excess_m'])
                if last_before is not None else None,
            'wrong_route_selected_samples': sum(boolean(r['auto_latched'])
                and not boolean(r['auto_selected_on_supplied_route']) for r in data),
            'entry_before_boundary_with_full_sampled_hold': first_hold is not None and first_hold < boundary,
            'hold_completed_before_boundary': completion is not None and completion < boundary}


def classification(result: dict, boundary: float) -> str:
    if result['entry_before_boundary_with_full_sampled_hold']:
        return 'passes_original_entry_and_hold_metric'
    first = result['first_accepted_native_sample_s']
    if first is None:
        return 'no_accepted_native_corridor_sample'
    if first >= boundary:
        return 'late_native_corridor_entry'
    if result['first_interruption_status'] == 'JUNCTION_AMBIGUOUS':
        return 'junction_interrupts_sampled_hold'
    return 'other_incomplete_sampled_hold'


def verify_metrics(actual: dict, expected: dict) -> None:
    for key, value in actual.items():
        target = expected[key]
        if value is None or target is None:
            equal = value is None and target is None
        else:
            equal = math.isclose(value, number(target), rel_tol=0., abs_tol=1e-12)
        if not equal:
            raise ValueError(f'Retained metric mismatch: {key}: {value!r} != {target!r}')


def read_csv(path: Path) -> list[dict]:
    with path.open(newline='', encoding='utf-8') as stream:
        return list(csv.DictReader(stream))


def analyze(evaluation: Path, cases: Path) -> dict:
    before = inventory(evaluation)
    cfg = json.loads(cases.read_text())
    qualification = json.loads((evaluation / 'qualification.json').read_text())
    pairs = json.loads((evaluation / 'paired-summary.json').read_text())
    if (qualification.get('experiment_integrity_passed') is not True
            or qualification.get('bias_model_enabled') is not False
            or qualification.get('initial_covariance_changed_before_comparison') is not False):
        raise ValueError('Expected qualified, unaugmented native recovery evidence')
    expected_ids = {f"{r['name']}__{target}__native__{feed}"
                    for r in cfg['routes']
                    for target, feed in [('clean', 'original_gnss')] +
                    [(f'cross{int(t)}', feed) for t in cfg['cross_track_targets_m']
                     for feed in ('original_gnss', 'withhold_first_10s')]}
    actual_ids = [pair['recovery']['id'] for pair in pairs]
    if (len(actual_ids) != len(set(actual_ids)) or set(actual_ids) != expected_ids
            or len(pairs) != qualification['paired_cases']):
        raise ValueError('Missing, duplicated or unexpected paired cases')
    records = []
    verified = 0
    for pair in pairs:
        case_id = pair['recovery']['id']
        if not re.fullmatch(r'[A-Za-z0-9_-]+', case_id):
            raise ValueError('Unsafe evidence case ID')
        traces = {}
        summaries = {}
        for variant in ('original', 'recovery'):
            summary = pair[variant]
            if (summary['id'] != case_id or summary['native_error_states'] != 18
                    or summary['bias_model_enabled'] is not False
                    or summary['fallback_bit_identical'] is not True
                    or summary['input_arrays_unchanged'] is not True):
                raise ValueError('Invalid comparison provenance')
            if json.loads((evaluation / variant / (case_id + '.json')).read_text()) != summary:
                raise ValueError('Per-case and paired summaries disagree')
            traces[variant] = read_csv(evaluation / variant / (case_id + '.csv'))
            summaries[variant] = accepted_metrics(traces[variant], cfg)
            verify_metrics(summaries[variant]['metrics'], summary['automatic_accepted_native_latch'])
            verified += 1
        left, right = traces['original'], traces['recovery']
        if len(left) != len(right):
            raise ValueError('Paired output coverage differs')
        for a, b in zip(left, right):
            fields = [k for k in a if k.startswith('before_') or k in ('t_s', 'elapsed_s')]
            if any(a[k] != b[k] for k in fields):
                raise ValueError('Retained map-free output differs across paired arms')
        summary = pair['recovery']
        result = summaries['recovery']
        factors = read_csv(evaluation / 'recovery' / (case_id + '__factors.csv'))
        activation = summary['auto_relocalization_t']
        record = {'id': case_id, 'route': summary['route']['name'],
                  'target_cross_m': summary['target_cross_m'], 'gnss_mode': summary['gnss_mode'],
                  'activation_elapsed_s': None if activation is None else activation - summary['start_s'],
                  **result, 'classification': classification(result, cfg['withheld_gnss_duration_s']),
                  'original_accepted_metrics': summaries['original']['metrics'],
                  'paired_trace_byte_identical': digest(evaluation / 'original' / (case_id + '.csv'))
                     == digest(evaluation / 'recovery' / (case_id + '.csv')),
                  'initial_known_route_factor_log_probability': factors[0].get('log_interval_probability'),
                  'factor_log_scope': 'supplied-route candidate only; not automatic branch telemetry'}
        records.append(record)
    clean = []
    fields = ('t_s', 'status', 'latched', 'edge', 'base_e', 'base_n',
              'candidate_e', 'candidate_n', 'p_ee', 'p_en', 'p_nn')
    for item in qualification['clean_full_runs']:
        name = item['dataset']
        if not re.fullmatch(r'[A-Za-z0-9_-]+', name):
            raise ValueError('Unsafe clean dataset ID')
        a = read_csv(evaluation / 'clean' / 'original' / (name + '.csv'))
        b = read_csv(evaluation / 'clean' / 'recovery' / (name + '.csv'))
        if len(a) != len(b) or len(a) != item['sampled_epochs']:
            raise ValueError('Clean coverage differs')
        if any(x[k] != y[k] for x, y in zip(a, b) for k in fields):
            raise ValueError('Clean logged navigation fields differ')
        clean.append({'dataset': name, 'sampled_epochs': len(a)})
    if len(clean) != 4:
        raise ValueError('Expected all four full clean recordings')
    outcomes = []
    for magnitude in sorted({abs(number(t)) for t in cfg['cross_track_targets_m']}):
        subset = [r for r in records if r['target_cross_m'] is not None
                  and abs(r['target_cross_m']) == magnitude
                  and r['gnss_mode'] == 'withhold_first_10s']
        outcomes.append({'magnitude_m': magnitude, 'cases': len(subset),
                         'passes': sum(r['entry_before_boundary_with_full_sampled_hold'] for r in subset),
                         'complete_pre_boundary_holds': sum(r['hold_completed_before_boundary'] for r in subset)})
    if inventory(evaluation) != before:
        raise ValueError('Evidence changed during read-only diagnosis')
    return {'schema_version': 1, 'analysis_kind': 'posthoc_retained_evidence_only',
            'native_estimator_executed': False, 'evidence_unchanged': True,
            'evaluation_execution_commit': qualification['execution_commit'],
            'retained_native_library_sha256': qualification['native_library_sha256'],
            'retained_source_sha256': qualification['source_sha256'],
            'case_config_sha256': digest(cases),
            'unchanged_metric_policy': {k: cfg[k] for k in ('inside_tolerance_m',
                'hold_duration_s', 'withheld_gnss_duration_s')},
            'metrics_recomputed_and_verified': verified,
            'clean_logged_fields_rechecked': clean,
            'outcomes': outcomes, 'cases': records, 'input_sha256': before,
            'limits': ['No fresh native build or replay is claimed.',
                       'Road identities are supplied development fixtures, not surveyed truth.',
                       'Automatic per-branch weights, offsets, covariance and gate reasons were not logged.',
                       'JUNCTION_AMBIGUOUS is an observed policy decision, not proof abstention was unavoidable.',
                       'Factor CSVs describe the supplied-route arm, not automatic alternatives.']}


def report(result: dict) -> str:
    lines = ['# Automatic recovery: retained-trace diagnosis', '',
             'Read-only post-hoc analysis; no native estimator execution or policy change.', '',
             '| Fixture | Offset (m) | Recovery (s) | Road selected (s) | First accepted inside sample (s) | Qualified hold entry (s) | Observed outcome |',
             '|---|---:|---:|---:|---:|---:|---|']
    def fmt(value):
        return 'none' if value is None else f'{value:.6f}'
    for r in sorted(result['cases'], key=lambda x: x['id']):
        if r['gnss_mode'] != 'withhold_first_10s':
            continue
        lines.append('| ' + ' | '.join([r['route'], f"{r['target_cross_m']:+.0f}",
            fmt(r['activation_elapsed_s']), fmt(r['first_selected_route_s']),
            fmt(r['first_accepted_native_sample_s']), fmt(r['metrics']['first_5s_hold_entry_s']),
            r['classification']]) + ' |')
    lines += ['', f"Recomputed all {result['metrics_recomputed_and_verified']} retained automatic metrics; all match.",
              'All four clean-recording comparisons and paired logged map-free outputs match.', '',
              'The original five-second metric requires an accepted sample at or after the endpoint.',
              'An interval ending at the next rejected sample is not a passing sampled hold.', '',
              '## Limits', ''] + result['limits']
    return '\n'.join(lines) + '\n'


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--evaluation', type=Path, required=True)
    parser.add_argument('--cases', type=Path, default=Path(__file__).parent / 'route_latch' / 'cases.json')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    evaluation, output = args.evaluation.resolve(), args.output.resolve()
    if output == evaluation or output.is_relative_to(evaluation) or output.exists():
        parser.error('Output must be a new directory outside the retained evidence')
    result = analyze(evaluation, args.cases)
    output.mkdir(parents=True)
    (output / 'diagnostics.json').write_text(json.dumps(result, indent=2, allow_nan=False) + '\n')
    (output / 'REPORT.md').write_text(report(result))
    print(json.dumps({'verified_metrics': result['metrics_recomputed_and_verified'],
                      'outcomes': result['outcomes']}, indent=2))


if __name__ == '__main__':
    main()
