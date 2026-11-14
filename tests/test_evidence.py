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
