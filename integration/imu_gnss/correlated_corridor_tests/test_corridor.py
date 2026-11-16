"""Independent algebra, numerical quadrature and native-adapter policy checks."""
from copy import deepcopy
import math
import numpy as np
import pytest
from scipy.integrate import quad
from scipy.stats import norm
from correlated_corridor import (corridor_moments, NativeCorridorSchmidt, map_patch_id)
from native_schmidt import compose_joint
from schmidt_native_tests.test_native_schmidt import FakeNav, h, j
from supported_map import lateral_update


def prior():
    rng=np.random.default_rng(612)
    a=rng.normal(size=(20,20))*.2
    q=a@a.T+np.eye(20)
    return q[:18,:18],q[:18,18:],q[18:,18:]


@pytest.mark.parametrize('distance',[-8.,-2.,0.,2.,8.])
def test_interval_moments_match_independent_numerical_quadrature(distance):
    p,c,u=prior();g=np.r_[h(),j()];a=compose_joint(p,c,u)@g
    s=float(g@a+.5**2)
    pdf=lambda y: norm.pdf((y-distance)/math.sqrt(s))/math.sqrt(s)
    z=quad(pdf,-6,6,epsabs=1e-12)[0]
    mean=quad(lambda y:y*pdf(y),-6,6,epsabs=1e-12)[0]/z
    var=quad(lambda y:(y-mean)**2*pdf(y),-6,6,epsabs=1e-12)[0]/z
    out=corridor_moments(p,c,u,h(),j(),distance,6.,.5)
    assert out.reason=='APPLIED'
    np.testing.assert_allclose(out.dx,a[:18]*(mean-distance)/s,atol=2e-12)
    np.testing.assert_allclose(out.covariance,p+np.outer(a[:18],a[:18])*(var-s)/s**2,atol=2e-12)
    np.testing.assert_allclose(out.cross,c+np.outer(a[:18],a[18:])*(var-s)/s**2,atol=2e-12)
    assert out.conditional_variance==pytest.approx(var,abs=1e-11)


@pytest.mark.parametrize('distance',[0.,2.,7.])
def test_full_schmidt_gaussian_surrogate_joseph_equivalence(distance):
    p,c,u=prior();Q=compose_joint(p,c,u);g=np.r_[h(),j()]
    out=corridor_moments(p,c,u,h(),j(),distance,3.,.5)
    b=out.information_fraction;s=out.innovation_variance
    a=Q@g;k=np.r_[b*a[:18]/s,np.zeros(2)]
    A=np.eye(20)-np.outer(k,g)
    R=.25+s*(1./b-1.)
    expected=A@Q@A.T+R*np.outer(k,k)
    actual=compose_joint(out.covariance,out.cross,u)
    np.testing.assert_allclose(actual,expected,atol=1e-12)
    np.testing.assert_allclose(out.dx,k[:18]*(out.conditional_mean-distance)/b,atol=1e-12)
    assert np.linalg.eigvalsh(actual)[0]>-1e-12
    # Freezing U is an approximation: it does not claim the true joint posterior.
    conditional_u=u-b*np.outer(a[18:],a[18:])/s
    assert not np.allclose(conditional_u,u)


@pytest.mark.parametrize('distance',[-10.,0.,10.])
def test_zero_map_uncertainty_reduces_to_existing_corridor_update(distance):
    p=np.eye(18);c=np.zeros((18,2));u=np.zeros((2,2))
    dx,P,_=lateral_update(p,h(),distance,1.5,6.)
    q=corridor_moments(p,c,u,h(),j(),distance,6.,1.5)
    np.testing.assert_allclose(q.dx,dx,atol=1e-12)
    np.testing.assert_allclose(q.covariance,P,atol=1e-12)
    np.testing.assert_array_equal(q.cross,c)


def test_narrow_interval_approaches_gaussian_native_factor():
    f=NativeCorridorSchmidt(FakeNav(),'patch',np.eye(2)*16)
    p,c,u=f.nav.P.copy(),f.C.copy(),f.U.copy()
    out=corridor_moments(p,c,u,h(),j(),-4.,.001,.3)
    a=compose_joint(p,c,u)@np.r_[h(),j()];s=float(np.r_[h(),j()]@a+.09)
    np.testing.assert_allclose(out.dx,a[:18]*4/s,atol=1e-8)
    np.testing.assert_allclose(out.covariance,p-np.outer(a[:18],a[:18])/s,atol=1e-8)


def make(U=16.):return NativeCorridorSchmidt(FakeNav(),'patch',np.eye(2)*U)


def factor(f,cell='a',d=-4.,width=6.,sigma=.3):
    return f.interval_factor(patch_id='patch',cell=cell,cross_track_m=d,
                             half_width_m=width,h=h(),j=j(),local_sigma_m=sigma)


def test_shared_translation_is_not_averaged_away_across_new_cells():
    shared,naive=make(),make(0.)
    for i in range(100):
        factor(shared,str(i),shared.nav.x[6]-4.,.01,.3)
        factor(naive,str(i),naive.nav.x[6]-4.,.01,math.sqrt(16+.09))
    assert shared.nav.P[6,6]>2.
    assert naive.nav.P[6,6]<.2
    np.testing.assert_array_equal(shared.U,np.eye(2)*16)
    assert shared.map_updates==naive.map_updates==100


def test_duplicate_cell_preserves_entire_estimator_and_native_trace():
    f=make();assert factor(f)['applied']
    old=(f.nav.x.tobytes(),f.nav.P.tobytes(),f.C.tobytes(),f.U.tobytes(),deepcopy(f.nav.trace))
    q=factor(f,d=500)
    assert q['reason']=='ALREADY_USED' and not q['applied']
    assert (f.nav.x.tobytes(),f.nav.P.tobytes(),f.C.tobytes(),f.U.tobytes())==old[:4]
    np.testing.assert_array_equal(f.nav.trace[2],old[4][2]);assert f.map_updates==1


@pytest.mark.parametrize('case',['remote','wide','unusable','increment'])
def test_rejected_measurement_does_not_discard_corrected_history(case):
    f=make();factor(f);f.feed('imu',[.01]);old=(f.nav.x.copy(),f.nav.P.copy(),f.C.copy())
    kw=dict(cell='next')
    if case=='remote':kw['d']=1000.
    if case=='wide':kw['width']=1e6
    if case=='unusable':
        original=f.nav.snapshot
        def snap():
            s=original();s.usable=False;return s
        f.nav.snapshot=snap
    if case=='increment':
        f.nav.P[0,6]=f.nav.P[6,0]=.9
        f.nav.P[0,0]=100.
        old=(f.nav.x.copy(),f.nav.P.copy(),f.C.copy())
        q=f.interval_factor(patch_id='patch',cell='next',cross_track_m=-4.,half_width_m=.1,
                            h=h(),j=j(),local_sigma_m=.3,max_attitude_rad=1e-8)
    else:q=factor(f,**kw)
    assert not q['applied']
    for got,want in zip((f.nav.x,f.nav.P,f.C),old):np.testing.assert_array_equal(got,want)
    assert f.engaged and f.map_updates==1
    f.feed('imu',[.01]);assert f.imu_events==2


def test_correlated_moments_continue_through_native_sensor_and_reset_operators():
    f=make();factor(f)
    old=f.C.copy();f.feed('imu',[.1]);F=np.eye(18);F[6,3]=.1;F[7,4]=.1
    np.testing.assert_allclose(f.C,F@old,atol=1e-12)
    p=f.nav.P.copy();old=f.C.copy();gain=p@h()/(h()@p@h()+4.)
    f.feed('gnss',[2.,0.])
    np.testing.assert_allclose(f.C,(np.eye(18)-np.outer(gain,h()))@old,atol=1e-12)
    g=np.eye(18);g[0,6]=.02
    inject=f.nav.inject
    def reset(dx,P):
        inject(dx,g@P@g.T);f.nav.trace=(True,1,g)
    f.nav.inject=reset
    out=corridor_moments(f.nav.P,f.C,f.U,h(),j(),-4.,6.,.3)
    assert factor(f,'b')['applied']
    np.testing.assert_allclose(f.C,g@out.cross,atol=1e-12)


def test_post_native_failure_latches_invalidity():
    f=make();factor(f)
    def bad(_):f.nav.trace=(False,0,np.eye(18))
    f.nav.imu=bad
    with pytest.raises(RuntimeError,match='reseed'):f.feed('imu',[.01])
    assert f.failed
    with pytest.raises(RuntimeError,match='invalidated'):factor(f,'b')


def test_injection_failure_cannot_resume_an_uncertain_instance():
    f=make()
    def bad(*_):raise RuntimeError('injection failed')
    f.nav.inject=bad
    with pytest.raises(RuntimeError,match='injection'):factor(f)
    assert f.failed and not f.used_cells


def test_map_prior_is_owned_and_cannot_be_mutated_by_the_caller():
    original=np.eye(2)*16
    f=NativeCorridorSchmidt(FakeNav(),'patch',original);original[:]=0
    assert f.U[0,0]==16
    with pytest.raises(ValueError):f.U[0,0]=0


@pytest.mark.parametrize('parameter',['distance','width','sigma'])
@pytest.mark.parametrize('value',[math.nan,math.inf,-math.inf])
def test_nonfinite_measurements_fail_before_mutation(parameter,value):
    f=make();args={'d':1.,'width':6.,'sigma':.3};args[{'distance':'d'}.get(parameter,parameter)]=value
    with pytest.raises(ValueError):factor(f,**args)
    assert not f.engaged and f.map_updates==0


@pytest.mark.parametrize('width',[0.,-1.])
def test_nonpositive_corridor_rejected(width):
    with pytest.raises(ValueError):factor(make(),width=width)


def test_map_version_change_cannot_create_an_independent_prior():
    f=make();factor(f);old=f.C.copy()
    with pytest.raises(ValueError,match='patch'):
        f.interval_factor(patch_id='other',cell='new',cross_track_m=-4.,half_width_m=6.,h=h(),j=j(),local_sigma_m=.3)
    np.testing.assert_array_equal(f.C,old)


def test_patch_identity_depends_on_source_and_frame_not_edge_ids():
    a=map_patch_id('a'*64,[43.,-79.,0.])
    assert a==map_patch_id('a'*64,[43.,-79.,0.])
    assert a!=map_patch_id('b'*64,[43.,-79.,0.])
    assert a!=map_patch_id('a'*64,[43.01,-79.,0.])


@pytest.mark.parametrize('source,origin',[('',[0,0,0]),('z'*64,[0,0,0]),('a'*64,[0,0]),('a'*64,[0,0,math.nan])])
def test_invalid_map_identity_is_rejected(source,origin):
    with pytest.raises(ValueError):map_patch_id(source,origin)


def test_invalid_joint_covariance_rejected_before_injection():
    f=make();f.C[:]=100
    with pytest.raises(ValueError,match='covariance'):factor(f)
    assert not f.engaged


def test_matched_first_factor_does_not_double_count_map_uncertainty():
    shared,independent=make(2.25),make(0.)
    a=factor(shared,width=6.,sigma=.25)
    b=factor(independent,width=6.,sigma=math.sqrt(.25**2+2.25))
    assert a['applied'] and b['applied']
    np.testing.assert_allclose(shared.nav.x,independent.nav.x,atol=1e-12)
    np.testing.assert_allclose(shared.nav.P,independent.nav.P,atol=1e-12)
    assert np.linalg.norm(shared.C)>0 and np.linalg.norm(independent.C)==0


def test_world_frame_observation_and_spatial_key_ignore_corrected_arm():
    from evaluate_correlated_corridor import observation
    from path_support import PathIndex, CausalSupport
    from support_tests.test_support import chain, Snapshot
    r=chain();index=PathIndex(r);s=Snapshot((25,2));selector=CausalSupport(index)
    path,support=selector.select(1.,s,'a')
    first=observation(path,support,s,r,.25,10.)
    # Both comparison arms receive this one proposal; there is no corrected
    # navigation parameter from which to obtain a different evidence key.
    assert first['cell'].endswith(':2')
    np.testing.assert_array_equal(first['point'],[25.,0.])
    np.testing.assert_array_equal(first['normal'],[0.,1.])
    assert observation(None,{'supported':False},s,r,.25,10.) is None


def test_arbitrary_graph_split_does_not_create_extra_corridor_information():
    from evaluate_correlated_corridor import observation
    from path_support import PathIndex, CausalSupport
    from support_tests.test_support import chain, Snapshot
    observations=[]
    for split in [False,True]:
        r=chain(split);selector=CausalSupport(PathIndex(r));s=Snapshot((60,2))
        path,support=selector.select(1.,s,'b' if split else 'whole')
        observations.append(observation(path,support,s,r,.25,10.))
    assert observations[0]['cell']==observations[1]['cell']
    for k in ['point','normal']:np.testing.assert_array_equal(observations[0][k],observations[1][k])


def test_shifted_frame_keeps_one_fixed_map_prior_through_native_reanchor():
    f=make();factor(f);c=f.C.copy();u=f.U.copy()
    T=np.eye(18);rot=np.array([[0.,-1.,0.],[1.,0.,0.],[0.,0.,1.]])
    for b in [0,3,6]:T[b:b+3,b:b+3]=rot
    f.nav.P=T@f.nav.P@T.T;f.nav.trace=(True,1,T);f._take()
    np.testing.assert_allclose(f.C,T@c,atol=1e-12)
    np.testing.assert_array_equal(f.U,u)
    assert f.patch_id=='patch' and f.used_cells=={'a'}
