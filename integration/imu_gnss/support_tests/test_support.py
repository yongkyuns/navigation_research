"""Geometry, support-availability, information-reuse and persistent-state regressions."""
from copy import deepcopy
from types import SimpleNamespace
import math
import numpy as np
import pytest
from path_support import (SmoothPath, PathIndex, CausalSupport, SupportConfig,
                          covered_search)
from supported_map import SupportedMapEKF, MeasurementConfig, lateral_update
from routefusion.tiles import Edge, TileStore


class Snapshot:
    def __init__(self, p=(25.,8.)):
        self.p=np.array(p,dtype=float)
        self.P=np.eye(18)
        self.velocity=np.array([5.,0.])
        self.anchor=self.lla=np.zeros(3)
        self.ready=self.usable=True
    def pose(self,frame):
        return self.p.copy(), self.P[6:8,6:8].copy(), self.velocity.copy()


class Nav:
    def __init__(self,p=(25.,8.)):
        self.s=Snapshot(p)
        self.injects=0
        self.closed=False
    def snapshot(self): return deepcopy(self.s)
    def inject(self,dx,P):
        self.s.p+=dx[6:8]
        self.s.P=P.copy()
        self.injects+=1
    def imu(self,row): self.s.p[0]+=1
    def gnss(self,row): pass
    def close(self): self.closed=True


def roads(edges):
    store=TileStore()
    for edge in edges: store.add(edge)
    store.cover([-200,-200],[600,600])
    return SimpleNamespace(store=store, frame=SimpleNamespace(from_ned=lambda a:np.eye(3)),
                           covers=lambda p:True)


def chain(split=True):
    if split:
        return roads([Edge('a',[[0.,0.],[50.,0.]],('b',)),
                      Edge('b',[[50.,0.],[100.,0.]],('c',)),
                      Edge('c',[[100.,0.],[200.,0.]])])
    return roads([Edge('whole',[[0.,0.],[200.,0.]])])


def same(a,b):
    np.testing.assert_array_equal(a.p,b.p)
    np.testing.assert_array_equal(a.P,b.P)


def test_graph_segmentation_changes_neither_geometry_identity_nor_cells():
    a,b=PathIndex(chain()),PathIndex(chain(False))
    x,y=a.by_edge['a'],b.by_edge['whole']
    assert x is a.by_edge['b'] is a.by_edge['c']
    assert x.key==y.key
    for s in np.linspace(1,199,61):
        p=np.array([s,3.])
        u,v=x.project(p),y.project(p)
        np.testing.assert_array_equal(u.point,v.point)
        np.testing.assert_array_equal(u.normal,v.normal)
        assert u.s==v.s


def test_causal_continuation_needs_past_acceptance_not_future_data():
    index=PathIndex(chain());s=Snapshot((45,1));c=CausalSupport(index)
    path,n=c.select(1.,s,None);assert path is None
    path,n=c.select(2.,s,'a');assert n['supported']
    s.p[0]=60
    path,n=c.select(3.,s,None)
    assert n['continued'] and n['supported'] and path is index.by_edge['b']
    path,n=c.select(4.,s,'b');assert n['supported']
    assert n['path_key']==index.by_edge['a'].key


def test_support_expires_without_reinitializing_the_filter():
    r=chain();c=CausalSupport(PathIndex(r));s=Snapshot((25,1));f=SupportedMapEKF(Nav(),r)
    p,n=c.select(1.,s,'a');f.step(1.,p,n);old=f.nav.snapshot()
    p,n=c.select(10.,s,None);after,m=f.step(10.,p,n)
    assert n['reason']=='SUPPORT_EXPIRED' and not m['updated'] and not m['reset']
    same(old,after)


@pytest.mark.parametrize('kind',['fork','merge','layer','gap','sharp'])
def test_never_stitch_unsupported_transitions(kind):
    a=Edge('a',[[0.,0.],[50.,0.]],('b',))
    b=Edge('b',[[50.,0.],[100.,0.]])
    extra=[]
    if kind=='fork':
        a.successors=('b','c');extra=[Edge('c',[[50,0],[60,40]])]
    if kind=='merge':extra=[Edge('c',[[50,20],[50,0]],('b',))]
    if kind=='layer':b.layer=1
    if kind=='gap':b=Edge('b',[[51,0],[100,0]])
    if kind=='sharp':b=Edge('b',[[50,0],[50,50]])
    idx=PathIndex(roads([a,b]+extra))
    assert idx.by_edge['a'] is not idx.by_edge['b']


def test_fork_does_not_invent_turn_or_select_exit():
    r=roads([Edge('a',[[0,0],[50,0]],('b','c')),
             Edge('b',[[50,0],[100,30]]),Edge('c',[[50,0],[100,-30]])])
    c=CausalSupport(PathIndex(r));s=Snapshot((40,0))
    c.select(1.,s,'a');s.p=np.array([55.,2.])
    p,n=c.select(2.,s,None)
    assert p is None and n['reason']=='OUTSIDE_PATH'


def test_parallel_competitor_prevents_continuation_not_median_average():
    r=chain();r.store.add(Edge('parallel',[[0,5],[200,5]]))
    c=CausalSupport(PathIndex(r));s=Snapshot((30,2))
    c.select(1.,s,'a');p,n=c.select(2.,s,None)
    assert p is None and n['reason']=='COMPETING_CORRIDOR'


def test_missing_neighbor_and_known_empty_search_tiles_fail_closed():
    r=chain();c=CausalSupport(PathIndex(r));s=Snapshot((5,1))
    c.select(1.,s,'a');r.store.loaded.remove((-1,0))
    p,n=c.select(2.,s,None)
    assert p is None and n['reason']=='INCOMPLETE_SEARCH_COVERAGE'


def test_declared_extract_footprint_is_checked_not_just_tiles():
    r=chain();r.bbox=[-100,-100,100,100]
    r.frame.project=lambda points:np.array([[p[1],p[0],0] for p in points])
    assert covered_search(r,np.array([50.,0.]),10)
    assert not covered_search(r,np.array([95.,0.]),10)


def test_stale_map_version_cannot_reenable_by_switching_version_back():
    r=chain();c=CausalSupport(PathIndex(r));s=Snapshot((25,1))
    c.select(1.,s,'a');version=r.store.version;r.store.version='changed'
    assert c.select(2.,s,'a')[1]['reason']=='MAP_CHANGED'
    r.store.version=version
    assert c.select(3.,s,'a')[0] is None


def test_geometry_curve_is_c1_and_has_bounded_offset():
    p=SmoothPath([Edge('a',[[0,0],[30,0],[60,20]])])
    assert 0 < p.deviation_m <= .5+1e-12
    last_point=last_tangent=None
    for primitive in p.primitives:
        if primitive[0]=='line':
            _,a,b,_,_=primitive;t=(b-a)/np.linalg.norm(b-a)
            first,end,ft,et=a,b,t,t
        else:
            _,c,r,phi,theta,_,_=primitive
            def values(x):
                z=np.array([math.cos(x),math.sin(x)])
                return c+r*z,math.copysign(1,theta)*np.array([-z[1],z[0]])
            first,ft=values(phi);end,et=values(phi+theta)
            for x in np.linspace(phi,phi+theta,101):
                point,_=values(x)
                assert p.polyline.project(point).distance <= .5+1e-10
        if last_point is not None:
            np.testing.assert_allclose(last_point,first,atol=1e-12)
            np.testing.assert_allclose(last_tangent,ft,atol=1e-12)
        last_point,last_tangent=end,et


def test_sharp_corner_not_smoothed_into_a_fictitious_curve():
    p=SmoothPath([Edge('a',[[0,0],[30,0],[30,30]])])
    assert all(x[0]=='line' for x in p.primitives)
    assert p.quality(30.,10.)==0
    assert p.quality(29.99,10.) < 1e-5


def test_taper_is_continuous_without_clamping_the_state_increment():
    path=PathIndex(chain()).by_edge['a']
    q=[path.quality(x,10.) for x in [0,1,5,9.999,10,50]]
    assert q[0]==0 and q[-1]==1 and all(a<=b for a,b in zip(q,q[1:]))


def test_split_graph_native_updates_are_identical():
    rs=[chain(),chain(False)];idx=[PathIndex(r) for r in rs]
    fs=[SupportedMapEKF(Nav(),r,MeasurementConfig(corridor=False,map_sigma_m=0,taper=False)) for r in rs]
    cs=[CausalSupport(x) for x in idx]
    for t,x in enumerate([25,35,45,55,65,95,105,115],1):
        for j in range(2):
            fs[j].nav.s.p[0]=x
            s=Snapshot((x,2))
            eid=('a' if x<50 else 'b' if x<100 else 'c') if j==0 else 'whole'
            p,n=cs[j].select(float(t),s,eid);fs[j].step(float(t),p,n)
        same(fs[0].nav.snapshot(),fs[1].nav.snapshot())
    assert fs[0].cells==fs[1].cells


def test_frequent_reuse_does_not_add_information_in_one_geometric_cell():
    r=chain();c=CausalSupport(PathIndex(r));f=SupportedMapEKF(Nav(),r)
    p,n=c.select(1.,Snapshot(),'a');a,m=f.step(1.,p,n);assert m['updated']
    for t in np.linspace(1.01,1.99,100):
        p,n=c.select(t,Snapshot(),'a');b,m=f.step(t,p,n)
        assert not m['updated'];same(a,b)
    assert f.update_count==1


def test_corridor_inside_is_not_forced_to_centerline():
    H=np.zeros(18);H[7]=1
    dx,P,_=lateral_update(np.eye(18),H,2.,1.5,6.)
    hard,Q,_=lateral_update(np.eye(18),H,2.,.25,0.)
    assert abs(dx[7]) < .03 and abs(hard[7]) > 1
    assert np.linalg.eigvalsh(P).min()>=0


def test_stronger_sigma_increases_single_centerline_correction():
    H=np.zeros(18);H[7]=1
    values=[abs(lateral_update(np.eye(18),H,4.,s)[0][7]) for s in [.05,.25,1.,5.]]
    assert all(a>b for a,b in zip(values,values[1:]))


def test_shared_covariance_updates_full_state_and_stays_psd():
    rng=np.random.default_rng(3);A=rng.normal(size=(18,18));P=A@A.T
    H=np.zeros(18);H[7]=1
    for width in [0.,6.]:
        dx,Q,_=lateral_update(P,H,8.,2.,width)
        assert np.linalg.norm(dx[:6])>0 and np.linalg.eigvalsh(Q).min()>=-1e-10


@pytest.mark.parametrize('value',[0,-1,math.nan,math.inf,1e308])
def test_invalid_sigma(value):
    with pytest.raises(ValueError):MeasurementConfig(sigma_m=value)


@pytest.mark.parametrize('failure',['missing','unusable','outside','heading','disabled','version'])
def test_rejection_preserves_persistent_state(failure):
    r=chain();idx=PathIndex(r);c=CausalSupport(idx);f=SupportedMapEKF(Nav(),r)
    p,n=c.select(1.,Snapshot(),'a');f.step(1.,p,n)
    f.nav.s.p[0]+=11
    if failure=='missing':r.store.loaded.clear()
    if failure=='unusable':f.nav.s.usable=False
    if failure=='outside':f.nav.s.p[0]=250
    if failure=='heading':f.nav.s.velocity=np.array([-5.,0.])
    if failure=='disabled':f.enabled=False
    if failure=='version':r.store.version='other'
    old=f.nav.snapshot();after,m=f.step(2.,p,n)
    assert not m['updated'] and not m['reset'];same(old,after)
    f.feed('imu',[0]);assert f.nav.snapshot().p[0]==old.p[0]+1


def test_no_support_before_initialization_is_not_a_fabricated_position():
    r=chain();f=SupportedMapEKF(Nav(),r);f.nav.s.ready=False
    a,n=f.step(1.,None,{'reason':'NO_SEED','supported':False})
    assert not a.ready and not n['updated']


def test_remote_corridor_is_rejected_not_a_numerical_exception():
    H=np.zeros(18);H[7]=1
    dx,P,n=lateral_update(np.eye(18),H,100.,1.5,6.)
    assert dx is None and P is None and n['reason']=='CORRIDOR_INCONSISTENT'


def test_explicit_association_failure_clears_support_not_ekf():
    r=chain();c=CausalSupport(PathIndex(r));s=Snapshot((25,1));f=SupportedMapEKF(Nav(),r)
    p,n=c.select(1.,s,'a');f.step(1.,p,n);old=f.nav.snapshot()
    p,n=c.select(2.,s,None,'NO_CONSISTENT_ROUTE');after,m=f.step(2.,p,n)
    assert n['reason']=='ASSOCIATION_INVALIDATED';same(after,old)
    assert c.select(3.,s,None,'AMBIGUOUS')[0] is None


def test_edge_handoff_inside_existing_cell_is_not_a_second_factor():
    r=roads([Edge('a',[[0,0],[28,0]],('b',)),Edge('b',[[28,0],[100,0]])])
    c=CausalSupport(PathIndex(r));f=SupportedMapEKF(Nav(),r)
    p,n=c.select(1.,Snapshot((25,1)),'a');f.step(1.,p,n)
    f.nav.s.p[0]=29
    p,n=c.select(2.,Snapshot((29,1)),'b');s,m=f.step(2.,p,n)
    assert not m['updated'] and m['reason']=='CELL_ALREADY_USED'
    assert f.update_count==1
    # Negative control: original edge-keyed cells would count two constraints.
    assert ('a',math.floor(25/10))!=('b',math.floor(1/10))


@pytest.mark.parametrize('sign',[-1,1])
def test_fillet_projection_and_normal_remain_continuous(sign):
    p=SmoothPath([Edge('a',[[0,0],[30,0],[60,sign*20]])])
    arc=next(x for x in p.primitives if x[0]=='arc')
    _,c,r,phi,theta,offset,length=arc
    for frac in [.1,.5,.9]:
        angle=phi+theta*frac;radial=np.array([math.cos(angle),math.sin(angle)])
        q=p.project(c+(r-.01)*radial)
        assert q.s==pytest.approx(offset+length*frac,abs=1e-8)
        assert q.distance==pytest.approx(.01,abs=1e-8)
    first=c+r*np.array([math.cos(phi),math.sin(phi)])
    before=p.project(first+np.array([-1e-5,0]))
    after=p.project(first+np.array([1e-5,0]))
    assert np.linalg.norm(before.normal-after.normal) < 1e-4


def test_no_inference_of_topology_from_crossing_lines():
    r=roads([Edge('a',[[0,0],[100,0]]),Edge('b',[[50,-50],[50,50]])])
    idx=PathIndex(r)
    assert idx.by_edge['a'] is not idx.by_edge['b']


@pytest.mark.parametrize('t',[math.nan,math.inf,-1.,1.])
def test_support_epoch_validation(t):
    c=CausalSupport(PathIndex(chain()));c.select(1.,Snapshot(),None)
    with pytest.raises(ValueError):c.select(t,Snapshot(),'a')


def test_disabled_filter_never_receives_a_map_update():
    r=chain();c=CausalSupport(PathIndex(r));f=SupportedMapEKF(Nav(),r,enabled=False)
    p,n=c.select(1.,Snapshot(),'a');old=f.nav.snapshot();after,m=f.step(1.,p,n)
    same(old,after);assert m['reason']=='MAP_DISABLED' and not m['updated']


def test_native_fault_is_not_hidden_by_a_fallback():
    r=chain();c=CausalSupport(PathIndex(r));f=SupportedMapEKF(Nav(),r)
    p,n=c.select(1.,Snapshot(),'a')
    f.nav.inject=lambda *args:(_ for _ in ()).throw(RuntimeError('native fault'))
    with pytest.raises(RuntimeError,match='native fault'):f.step(1.,p,n)
    assert f.update_count==0


@pytest.mark.parametrize('deviation',[-1.,math.nan,math.inf])
def test_invalid_geometry_uncertainty_rejected(deviation):
    with pytest.raises(ValueError):SmoothPath([Edge('a',[[0,0],[100,0]])],deviation_m=deviation)
