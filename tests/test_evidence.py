"""Publication-tool guards; these do not change the original 24 algorithm tests."""
import copy
import pytest
from scripts.evidence import close, compare_metrics, original_records, check_sources


def test_original_records_and_source_hashes():
    _, records = original_records()
    assert len(records) == 4
    assert check_sources() == 17


def test_metric_comparison_ignores_only_runtime():
    expected = [dict(scenario='test', seed=100, runtime_s=1., output_rmse_m=2.)]
    actual = copy.deepcopy(expected)
    actual[0]['runtime_s'] = 1000.
    compare_metrics(actual, expected)
    actual[0]['output_rmse_m'] += .01
    with pytest.raises(ValueError):
        compare_metrics(actual, expected)


def test_missing_duplicate_or_extra_metric_rejected():
    rows = [dict(scenario='test', seed=100, runtime_s=1., score=2.)]
    for candidate in [[], rows + rows, [dict(rows[0], unexpected=0)]]:
        with pytest.raises(ValueError):
            compare_metrics(candidate, rows)


def test_nonfinite_values_and_status_changes_rejected():
    for value in [float('nan'), float('inf'), True]:
        with pytest.raises(ValueError):
            close(value, 1.)
    with pytest.raises(ValueError):
        close('LATCHED', 'AMBIGUOUS')


def tie_trace(path, north=13.):
    import csv
    with path.open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=['t', 'fallback_n', 'status',
                                'route_group', 'true_group', 'naive_group', 'top_weight'])
        writer.writeheader()
        for t in range(1, 121):
            writer.writerow(dict(t=t, fallback_n=north, status='AMBIGUOUS', route_group='',
                                 true_group='main', naive_group='main', top_weight=.5))


def test_symmetric_baseline_tie_is_qualified_not_hidden(tmp_path):
    ref = [dict(scenario='ambiguous_parallel', seed=109, runtime_s=1.,
                naive_correct_rate=.95, output_rmse_m=13.)]
    actual = [dict(ref[0], naive_correct_rate=1.)]
    with pytest.raises(ValueError):
        compare_metrics(actual, ref)
    tie_trace(tmp_path / 'ambiguous_parallel_seed109.csv')
    diagnostics = compare_metrics(actual, ref, tmp_path)
    assert len(diagnostics) == 1 and diagnostics[0]['original'] == .95
    actual[0]['output_rmse_m'] = 12.
    with pytest.raises(ValueError):
        compare_metrics(actual, ref, tmp_path)


def test_non_tied_baseline_difference_is_not_exempt(tmp_path):
    ref = [dict(scenario='ambiguous_parallel', seed=109, runtime_s=1., naive_correct_rate=.95)]
    actual = [dict(ref[0], naive_correct_rate=1.)]
    tie_trace(tmp_path / 'ambiguous_parallel_seed109.csv', north=13.001)
    with pytest.raises(ValueError):
        compare_metrics(actual, ref, tmp_path)
