from pathlib import Path
import sys
import math
import numpy as np
import pytest
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT),str(ROOT/'tests')]
from route_recovery import (RelocalizationConfig, LossEvidence, broaden_position,
    inconsistent_edges, RelocalizingRoadBank, TrustedRouteConditioner)
from road_bank import RoadBank, Branch
from intersections import MissingTopology
from test_intersections import FakeNav, Snapshot, Roads
from routefusion.tiles import Edge

class IncrementNav(FakeNav):
    """Test double preserves map correction when subsequent motion arrives."""
    def __init__(self,p=(0.,20.),v=(10.,0.)):
        super().__init__(p,v)
        self.P=np.eye(18)*.01
        self.P[6:9,6:9]=np.eye(3)*.25**2
        self.t=0.
    def clone(self):
        n=IncrementNav(self.p,self.v)
        n.P=self.P.copy();n.t=self.t;n.count=self.count
        return n
    def imu(self,row):
        assert row[0]>self.t
        self.p+=self.v*(row[0]-self.t)
        self.t=row[0];self.count+=1


def straight(parallel=False):
    es=[Edge('a',[[-200,0],[500,0]],(),6.)]
    if parallel:es.append(Edge('b',[[-200,40],[500,40]],(),6.))
    return Roads(es)


def test_prior_addition_does_not_move_mean_or_change_velocity_uncertainty():
    n=IncrementNav();roads=straight();s=n.snapshot()
    broaden_position(n,roads.frame,10.)
    q=n.snapshot()
    np.testing.assert_array_equal(q.position,s.position)
    np.testing.assert_array_equal(q.velocity,s.velocity)
    delta=q.P-s.P
    expected=np.zeros((18,18));expected[6,6]=expected[7,7]=100.
    np.testing.assert_allclose(delta,expected,atol=1e-10)
    assert np.linalg.eigvalsh(q.P).min()>0
    n.close()

@pytest.mark.parametrize('value',[-1,0,float('nan'),float('inf')])
def test_invalid_sigma_is_rejected_without_mutation(value):
    n=IncrementNav();s=n.snapshot().raw.copy()
    with pytest.raises(ValueError):broaden_position(n,straight().frame,value)
    np.testing.assert_array_equal(s,n.snapshot().raw);n.close()


def test_repeated_confirmations_needed_and_only_one_episode():
    e=LossEvidence(RelocalizationConfig())
    assert not e.observe(0,[0,20],['a'],'v1')
    assert not e.observe(1,[10,20],['a'],'v1')
    assert e.observe(2,[20,20],['a'],'v1')
    assert e.activation_t==2
    assert not e.observe(100,[100,20],['a'],'v1')


def test_no_confirmation_from_static_reuse_gap_or_map_change():
    e=LossEvidence(RelocalizationConfig())
    for t in range(4):assert not e.observe(t,[0,20],['a'],'v1')
    assert not e.observe(9,[9,20],['a'],'v1')
    assert e.count==1
    assert not e.observe(10,[20,20],['a'],'v2')
    assert e.invalidated
    assert not e.observe(11,[30,20],['a'],'v2')


def test_alternative_route_fit_prevents_loss_inference():
    r=straight(True);n=IncrementNav(p=(0,40))
    assert inconsistent_edges(n.snapshot(),r,RelocalizationConfig())==()
    n.close()

@pytest.mark.parametrize('p,v', [((0,60),(10,0)),((0,20),(0,0)),((0,20),(0,10)),((600,20),(10,0))])
def test_out_of_range_stopped_crossing_and_outside_segment_do_not_trigger(p,v):
    n=IncrementNav(p,v)
    assert inconsistent_edges(n.snapshot(),straight(),RelocalizationConfig())==()
    n.close()


def test_automatic_parallel_ambiguity_is_not_collapsed_to_one_road():
    bank=RelocalizingRoadBank(IncrementNav(),straight(True));control=bank.base.clone()
    try:
        out=bank.step(0.)
        for t in range(1,9):
            row=np.array([float(t),0,0,0,0,0,0])
            bank.imu(row);control.imu(row);out=bank.step(float(t))
            assert not out['latched']
            np.testing.assert_array_equal(bank.base.snapshot().raw,control.snapshot().raw)
        assert bank.loss.used and bank.loss.activation_t==2.
        assert {b.edge for b in bank.branches}=={'a','b'}
        assert .49<out['top_weight']<.51
    finally:bank.reset();bank.base.close();control.close()


def test_single_road_automatic_recovery_corrects_a_native_hypothesis_only():
    bank=RelocalizingRoadBank(IncrementNav(),straight());control=bank.base.clone()
    try:
        bank.step(0.)
        for t in range(1,8):
            row=np.array([float(t),0,0,0,0,0,0]);bank.imu(row);control.imu(row);out=bank.step(float(t))
        assert bank.loss.activation_t==2.
        assert out['latched']
        best=max(bank.branches,key=lambda b:b.logw)
        assert abs(best.nav.snapshot().p[1])<6.
        np.testing.assert_array_equal(bank.base.snapshot().raw,control.snapshot().raw)
        assert bank.base.snapshot().p[1]==20.
    finally:bank.reset();bank.base.close();control.close()


def test_missing_map_never_activates_or_corrects():
    roads=straight();roads.store.loaded.clear()
    bank=RelocalizingRoadBank(IncrementNav(),roads)
    try:
        for t in range(5):
            if t:bank.imu(np.array([t,0,0,0,0,0,0.]))
            out=bank.step(float(t))
            assert out['status']=='MAP_UNAVAILABLE'
        assert not bank.loss.used and bank.base.snapshot().p[1]==20.
    finally:bank.reset();bank.base.close()


def test_clean_road_default_and_opt_in_paths_are_equal():
    r=straight();base=IncrementNav(p=(0,1))
    old=RoadBank(base.clone(),r,recovery=False);new=RelocalizingRoadBank(base.clone(),r)
    try:
        for t in range(6):
            if t:
                row=np.array([float(t),0,0,0,0,0,0]);old.imu(row);new.imu(row)
            a,b=old.step(float(t)),new.step(float(t))
            assert a['status']==b['status']
            np.testing.assert_array_equal(a['conditional_position'],b['conditional_position'])
        assert not new.loss.used
    finally:old.reset();new.reset();old.base.close();new.base.close();base.close()


def test_known_route_changed_map_rejected_before_native_mutation():
    roads=straight();base=IncrementNav();b=Branch('a',base.clone(),np.zeros(2),0.)
    helper=RoadBank(base,roads,recovery=False);helper._version=roads.store.version
    recovery=TrustedRouteConditioner(roads);snapshot=b.nav.snapshot().raw.copy()
    roads.store.version='new'
    with pytest.raises(MissingTopology):recovery.condition(b,helper,base,0.)
    np.testing.assert_array_equal(snapshot,b.nav.snapshot().raw)
    b.nav.close();base.close()
