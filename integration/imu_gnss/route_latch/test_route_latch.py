"""Regression tests for a measurement-only experiment, not a navigation change."""
import json
import os
from pathlib import Path
import numpy as np
import pytest
from evaluate import (Library, RoadBank, Branch, load_inputs, checkpoint,
                      perturb, factor_diagnostic, metrics, compile_osm,
                      supplied_edge, geometry_metrics)

HERE=Path(__file__).resolve().parent
CFG=json.loads((HERE/'cases.json').read_text())

@pytest.fixture
def roads():
    p=Path(os.environ['ROUTE_LATCH_MAP_CACHE'])
    return compile_osm(p/'osm-roads.json.gz',p/'provenance.json')

@pytest.fixture
def native():
    lib=Library(os.environ['ROUTEFUSION_NATIVE_LIBRARY'])
    assert not hasattr(lib.lib,'rf_new_bias')
    return lib

@pytest.fixture
def initialized(native):
    route=CFG['routes'][0]
    imu,g,_=load_inputs(Path(os.environ['ROUTEFUSION_DATASETS'])/route['dataset'])
    nav,t=checkpoint(native,imu,g,route)
    yield nav,t,route
    nav.close()


def test_source_is_native_18_state_without_bias_symbol(native, initialized):
    n,t,r=initialized
    assert n.snapshot().P.shape==(18,18)
    assert not hasattr(n.snapshot(),'gnss_bias')


def test_supplied_routes_are_connected_in_independent_graph(roads):
    for route in CFG['routes']:
        ids=[e for _,e in route['segments']]
        assert all(b in roads.store.edges[a].successors for a,b in zip(ids,ids[1:]))


def test_supplied_edge_depends_on_time_not_estimated_position():
    route=CFG['routes'][0]
    assert supplied_edge(route,140)==route['segments'][0][1]
    assert supplied_edge(route,160)==route['segments'][1][1]


@pytest.mark.parametrize('target',[-20.,-10.,10.,20.])
def test_offset_is_injected_once_before_cloning_and_keeps_covariance(roads,initialized,target):
    nav,t,route=initialized
    P=nav.snapshot().P.copy()
    m=perturb(nav,roads,supplied_edge(route,t),target,'native')
    a=nav.clone();b=nav.clone()
    try:
        np.testing.assert_array_equal(a.snapshot().raw,b.snapshot().raw)
        np.testing.assert_allclose(a.snapshot().P,P,atol=1e-8,rtol=1e-7)
        assert abs(m['actual_initial_cross_m']-target)<1e-4
    finally:a.close();b.close()


def test_twenty_metre_native_gate_rejects_without_changing_state(roads,initialized):
    nav,t,route=initialized;eid=supplied_edge(route,t)
    perturb(nav,roads,eid,20.,'native')
    helper=RoadBank(nav.clone(),roads,recovery=False);helper._version=roads.store.version
    b=Branch(eid,nav,np.zeros(2),0.)
    before=nav.snapshot().raw.copy()
    try:
        assert factor_diagnostic(b,helper)['reason']=='INTERVAL_REJECTED'
        assert not np.isfinite(helper.condition(b))
        np.testing.assert_array_equal(nav.snapshot().raw,before)
    finally:helper.base.close()


def test_declared_common_uncertainty_allows_actual_native_map_correction(roads,initialized):
    nav,t,route=initialized;eid=supplied_edge(route,t)
    perturb(nav,roads,eid,20.,'additional_10m_cross_track_sigma')
    control=nav.clone();helper=RoadBank(nav.clone(),roads,recovery=False);helper._version=roads.store.version
    b=Branch(eid,nav,np.zeros(2),0.)
    before=control.snapshot().raw.copy()
    try:
        assert factor_diagnostic(b,helper)['reason']=='APPLIED'
        helper.condition(b)
        p=nav.snapshot().pose(roads.frame)[0]
        pc=control.snapshot().pose(roads.frame)[0]
        assert geometry_metrics(p,roads,route,eid)['route_excess_m']<1e-6
        assert geometry_metrics(pc,roads,route,eid)['route_excess_m']>13.99
        np.testing.assert_array_equal(control.snapshot().raw,before)
        assert np.linalg.eigvalsh(nav.snapshot().P).min()>-1e-6
    finally:control.close();helper.base.close()


def test_clean_inside_corridor_is_not_forced_to_centerline(roads,initialized):
    nav,t,route=initialized;eid=supplied_edge(route,t)
    helper=RoadBank(nav.clone(),roads,recovery=False);helper._version=roads.store.version
    b=Branch(eid,nav,np.zeros(2),0.)
    before=roads.store.edges[eid].project(nav.snapshot().pose(roads.frame)[0]).cross
    try:
        helper.condition(b)
        after=roads.store.edges[eid].project(nav.snapshot().pose(roads.frame)[0]).cross
        assert abs(after)>2.0
        assert abs(after-before)<.1
    finally:helper.base.close()


def test_duplicate_zero_time_pre_update_has_no_residence_duration():
    rows=[{'elapsed_s':0.,'x_route_excess_m':14.},
          {'elapsed_s':0.,'x_route_excess_m':0.},
          {'elapsed_s':5.,'x_route_excess_m':0.},
          {'elapsed_s':10.,'x_route_excess_m':0.}]
    out=metrics(rows,'x_',CFG)
    assert out['initial_route_excess_m']==14.
    assert out['inside_fraction']==1.
    assert out['first_5s_hold_entry_s']==0.
    assert out['route_excess_rms_m']==0.


def test_transient_entry_is_not_sustained_latch():
    rows=[{'elapsed_s':float(t),'x_route_excess_m':0. if t in (2,3,4) else 2.} for t in range(12)]
    out=metrics(rows,'x_',CFG)
    assert out['first_5s_hold_entry_s'] is None
    assert out['entry_then_stays_until_end_s'] is None


def test_known_route_evaluator_never_imports_bias_adapter_or_reference():
    import inspect,evaluate
    source=inspect.getsource(evaluate)
    assert 'from native_bias' not in source and 'load_reference(' not in source
    assert 'helper.condition(branch)' in source


def test_good_fallback_position_does_not_count_as_accepted_native_latch():
    from evaluate import accepted_native_latch
    rows=[{'elapsed_s':float(t),'auto_route_excess_m':0.,'auto_latched':False,
           'auto_selected_on_supplied_route':False} for t in range(12)]
    m=accepted_native_latch(rows,CFG)
    assert m['inside_fraction']==0. and m['first_5s_hold_entry_s'] is None
    for row in rows:row.update(auto_latched=True,auto_selected_on_supplied_route=True)
    m=accepted_native_latch(rows,CFG)
    assert m['inside_fraction']==1. and m['first_5s_hold_entry_s']==0.
