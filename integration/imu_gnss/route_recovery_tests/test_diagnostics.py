"""Offline diagnostic metric regressions; no native runtime is required."""
import importlib.util
from pathlib import Path

import pytest

SPEC = importlib.util.spec_from_file_location(
    'diagnose_recovery', Path(__file__).parents[1] / 'diagnose_recovery.py')
diag = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(diag)
CFG = {'inside_tolerance_m': .001, 'hold_duration_s': 5.,
       'withheld_gnss_duration_s': 10.}


def row(t, *, latched=True, on_route=True, excess=0., status='LATCHED'):
    return {'elapsed_s': str(t), 'auto_latched': str(latched),
            'auto_selected_on_supplied_route': str(on_route),
            'auto_route_excess_m': str(excess), 'auto_status': status}


def test_duplicate_timestamp_is_owned_by_post_map_row():
    data = [row(0., excess=4.), row(0.), row(5.), row(10.)]
    result = diag.accepted_metrics(data, CFG)
    assert result['metrics']['first_5s_hold_entry_s'] == 0.
    assert result['metrics']['inside_fraction'] == 1.


@pytest.mark.parametrize('change', [dict(latched=False), dict(on_route=False), dict(excess=.002)])
def test_coordinate_or_road_label_alone_is_not_success(change):
    result = diag.accepted_metrics([row(0., **change), row(5., **change), row(10., **change)], CFG)
    assert result['first_accepted_native_sample_s'] is None
    assert result['metrics']['first_5s_hold_entry_s'] is None


def test_junction_dropout_is_not_interpolated_into_a_pass():
    data = [row(0., latched=False), row(5.025), row(9.950),
            row(10.050, latched=False, status='JUNCTION_AMBIGUOUS'), row(11., latched=False)]
    result = diag.accepted_metrics(data, CFG)
    assert result['metrics']['first_5s_hold_entry_s'] is None
    assert result['accepted_sample_intervals'][0]['sample_span_s'] == pytest.approx(4.925)
    assert diag.classification(result, 10.) == 'junction_interrupts_sampled_hold'


def test_successful_hold_crossing_boundary_is_not_a_pure_withholding_hold():
    data = [row(0., latched=False), row(5.025), row(9.950), row(10.050), row(11.)]
    result = diag.accepted_metrics(data, CFG)
    assert result['entry_before_boundary_with_full_sampled_hold']
    assert not result['hold_completed_before_boundary']


def test_early_entry_interrupted_later_remains_a_failed_pre_boundary_hold():
    data = [row(0., latched=False), row(9.), row(12.9),
            row(13., latched=False, status='JUNCTION_AMBIGUOUS'), row(33.), row(39.)]
    result = diag.accepted_metrics(data, CFG)
    assert result['first_accepted_native_sample_s'] == 9.
    assert result['metrics']['first_5s_hold_entry_s'] == 33.
    assert not result['entry_before_boundary_with_full_sampled_hold']
    assert diag.classification(result, 10.) == 'junction_interrupts_sampled_hold'


def test_selected_road_can_have_late_coordinate_entry():
    data = [row(0., latched=False), row(3., excess=.5), row(9.95, excess=.018),
            row(10.05), row(16.)]
    result = diag.accepted_metrics(data, CFG)
    assert result['first_selected_route_s'] == 3.
    assert result['first_accepted_native_sample_s'] == 10.05
    assert diag.classification(result, 10.) == 'late_native_corridor_entry'


def test_time_weighted_residence_does_not_condition_on_successful_samples():
    data = [row(0., latched=False), row(3.), row(5.), row(10.)]
    result = diag.accepted_metrics(data, CFG)
    assert result['metrics']['first_10s_inside_fraction'] == .7


@pytest.mark.parametrize('value', ['', 'false', '0', '1', None])
def test_missing_or_malformed_booleans_fail_closed(value):
    with pytest.raises(ValueError):
        diag.boolean(value)


def test_out_of_order_epochs_rejected():
    with pytest.raises(ValueError):
        diag.accepted_metrics([row(0.), row(2.), row(1.)], CFG)


@pytest.mark.parametrize('value', ['nan', 'inf', '-inf'])
def test_nonfinite_times_rejected(value):
    with pytest.raises(ValueError):
        diag.accepted_metrics([row(0.), row(value)], CFG)


def test_retained_metric_mismatch_is_an_error():
    with pytest.raises(ValueError, match='Retained metric mismatch'):
        diag.verify_metrics({'first_5s_hold_entry_s': None}, {'first_5s_hold_entry_s': 5.})


def test_exact_hold_endpoint_required():
    short = diag.accepted_metrics([row(0.), row(4.999)], CFG)
    exact = diag.accepted_metrics([row(0.), row(5.)], CFG)
    assert short['metrics']['first_5s_hold_entry_s'] is None
    assert exact['metrics']['first_5s_hold_entry_s'] == 0.


def test_whole_interval_not_only_endpoints_required():
    result = diag.accepted_metrics([row(0.), row(2., latched=False), row(5.)], CFG)
    assert result['metrics']['first_5s_hold_entry_s'] is None


@pytest.mark.parametrize('sample,completion,before_boundary', [
    (9.989, 11., False),
    (9.99, 9.99, True),
    (10., 10., False),
    (10.05, 10.05, False),
])
def test_hold_completion_requires_an_observed_sample_before_boundary(
        sample, completion, before_boundary):
    data = [row(0., latched=False), row(4.99), row(9.98), row(sample), row(11.)]
    result = diag.accepted_metrics(data, CFG)
    assert result['entry_before_boundary_with_full_sampled_hold']
    assert result['hold_completed_before_boundary'] is before_boundary
    assert result['first_5s_hold_completion_sample_s'] == completion


def test_incomplete_hold_has_no_completion_sample():
    result = diag.accepted_metrics([row(0.), row(4.999)], CFG)
    assert not result['hold_completed_before_boundary']
    assert result['first_5s_hold_completion_sample_s'] is None
