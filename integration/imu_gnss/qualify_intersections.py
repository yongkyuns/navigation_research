"""Qualify new replay artifacts against the pinned pre-intersection baseline.

Changed map candidates are reported, not treated as independent truth. Original
baseline navigation, inputs, initialization and output coverage must not change.
"""
from pathlib import Path
import argparse
import hashlib
import json
import math
import subprocess
import xml.etree.ElementTree as ET

HERE = Path(__file__).resolve().parent


def equal(actual, expected, path):
    if isinstance(expected, dict):
        if not isinstance(actual, dict) or actual.keys() != expected.keys():
            raise AssertionError('Schema changed: ' + path)
        for key in expected:
            equal(actual[key], expected[key], path + '/' + key)
    elif isinstance(expected, (int, float)) and not isinstance(expected, bool):
        if not math.isclose(actual, expected, abs_tol=1e-5, rel_tol=1e-7):
            raise AssertionError(f'{path}: {actual} != {expected}')
    elif actual != expected:
        raise AssertionError(f'{path}: {actual!r} != {expected!r}')


def qualify(results, previous):
    old = {m['id']: m for m in json.loads((previous / 'baseline/summary.json').read_text())}
    new = {m['id']: m for m in json.loads((results / 'baseline/summary.json').read_text())}
    if new.keys() != old.keys():
        raise AssertionError('Dataset coverage changed')
    for key, row in new.items():
        equal(row['status'], old[key].get('status', 'REPLAYED'), key + '/status')
        if row['status'] != 'REPLAYED':
            continue  # reject-reason filesystem prefixes depend on checkout location
        for field in ('input_sha256', 'rows', 'initialization_t_s', 'sampled_epochs',
                      'ready_fraction', 'usable_fraction', 'nonfinite_outputs',
                      'reference_discrepancy', 'gnss_agreement_not_accuracy', 'reanchors'):
            equal(row[field], old[key][field], key + '/' + field)
        if row['nonfinite_outputs']:
            raise AssertionError('Nonfinite baseline output')
    comparisons = []
    for category in ('map', 'stress-outage30', 'stress-bias20'):
        before = {m['id']: m for m in json.loads((previous / category / 'summary.json').read_text())}
        after = json.loads((results / category / 'summary.json').read_text())
        if {m['id'] for m in after} != before.keys():
            raise AssertionError('Paired experiment coverage changed')
        for row in after:
            if (not row['map_free_exactly_matches_independent_replay']
                    or row['negative_position_covariance_epochs']
                    or row['max_branches'] > row['configured_cap']):
                raise AssertionError('Paired replay invariant failed: ' + row['id'])
            equal(row['base_reference_discrepancy'], before[row['id']]['base_reference_discrepancy'],
                  category + '/' + row['id'] + '/base')
            comparisons.append({
                'category': category, 'id': row['id'],
                'previous_latch_fraction': before[row['id']]['latch_fraction_available'],
                'new_latch_fraction': row['latch_fraction_available'],
                'status_counts': row['status_counts'], 'recovery_count': row['recovery_count'],
                'baseline_discrepancy': row['base_reference_discrepancy'],
                'previous_candidate_discrepancy': before[row['id']]['candidate_reference_discrepancy'],
                'new_candidate_discrepancy': row['candidate_reference_discrepancy'],
                'stress_baseline_discrepancy': row['stress_window_base_discrepancy'],
                'stress_candidate_discrepancy': row['stress_window_candidate_discrepancy'],
                'map_step_ms': row['map_step_ms'], 'max_pruned_mass': row['max_pruned_mass']})
    suites = list(ET.parse(results / 'integration-tests.xml').getroot().iter('testsuite'))
    if not suites or any(int(s.get(k, 0)) for s in suites for k in ('failures', 'errors', 'skipped')):
        raise AssertionError('Integration tests failed, missing, or skipped')
    tests = sum(int(s.get('tests', 0)) for s in suites)
    if tests < 39:
        raise AssertionError('Intersection regression tests were not collected')
    record = {
        'git_revision': subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=HERE, text=True).strip(),
        'upstream_revision': '8a34212c7befe30f8d3eae4ad708cc0dcd4aa0d6',
        'tests_passed': tests, 'datasets_replayed': sum(v['status'] == 'REPLAYED' for v in new.values()),
        'baseline_unchanged': True, 'paired_fallback_bit_identical': True,
        'reference_independence': 'unverified; all distances are reference discrepancies',
        'road_label_accuracy_measured': False, 'output_mode': 'shadow/map-free',
        'experiments': comparisons,
        'source_sha256': {p.relative_to(HERE).as_posix(): hashlib.sha256(p.read_bytes()).hexdigest()
                          for p in sorted(HERE.rglob('*.py'))},
    }
    (results / 'intersection-qualification.json').write_text(json.dumps(record, indent=2) + '\n')
    return record


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--results', type=Path, required=True)
    parser.add_argument('--previous', type=Path, default=HERE / 'evidence')
    args = parser.parse_args()
    print(json.dumps(qualify(args.results, args.previous), indent=2))
