"""Contract tests: no stale map cross moments or artificial covariance collapse."""
import ctypes
import math
from types import SimpleNamespace
import numpy as np
import pytest

from native_schmidt import (NativeSchmidt, NativeTransition, compose_joint,
                            covariance_psd)


class TransitionFunction:
    def __init__(self,nav): self.nav=nav
    def __call__(self,handle,ptr,n):
        if n!=326: return -1
        tr=self.nav.trace
        self.nav.trace=(True,0,np.eye(18))
        buf=ctypes.cast(ptr,ctypes.POINTER(ctypes.c_double * 326)).contents
        buf[0]=float(tr[0]);buf[1]=tr[1]
        for i,x in enumerate(tr[2].flat):buf[i+2]=float(x)
        return 0


class FakeNav:
    def __init__(self):
        self.handle=1
        self.x=np.zeros(18)
        self.P=np.eye(18)*3
        self.trace=(True,0,np.eye(18))
        self.library=SimpleNamespace(lib=SimpleNamespace(rf_transition=TransitionFunction(self)))
        self.closed=False
    def snapshot(self):return SimpleNamespace(P=self.P.copy(),ready=True,usable=True,x=self.x.copy())
    def inject(self,dx,P):
        self.x+=dx
        self.P=P.copy()
        self.trace=(True,1,np.eye(18))
    def imu(self,row):
        F=np.eye(18)
        F[6,3]=float(row[0]);F[7,4]=float(row[0]);
        self.x=F@self.x
        self.P=F@self.P@F.T+np.diag([0.01]*18)
        self.trace=(True,1,F)
    def gnss(self,row):
        H=np.zeros(18);H[6]=1
        K=(self.P@H)/(float(H@self.P@H)+float(row[0])**2)
        A=np.eye(18)-np.outer(K,H)
        self.P=A@self.P@A.T+np.outer(K,K)*float(row[0])**2
        self.x=self.x+K*float(row[1])
        self.trace=(True,2,A)
    def close(self):self.closed=True


def create():
    return NativeSchmidt(FakeNav(), 'map-v1', np.diag([16.,16.]))


def h():
    x=np.zeros(18);x[6]=1
    return x


def j():return np.array([-1.,0.])


def factor(f,cell='a',residual=4.,gate=100):
    return f.factor(patch_id='map-v1',cell=cell,residual_m=residual,h=h(),j=j(),
                    local_sigma_m=.3,innovation_gate=gate)


def test_native_trace_symbol_is_mandatory_no_silent_approximation():
    nav=FakeNav();del nav.library.lib.rf_transition
    with pytest.raises(RuntimeError,match='rf_transition'):
        NativeSchmidt(nav,'map-v1',np.eye(2))


def test_one_map_factor_updates_native_18_state_and_cross_covariance():
    f=create();result=factor(f)
    assert result.applied and f.engaged and f.map_updates==1
    assert f.C[6,0]>0 # J=-1 gives positively correlated road offset
    assert f.nav.x[6]>0
    covariance_psd(compose_joint(f.nav.P,f.C,f.U))


def test_map_patch_shared_error_not_averaged_away():
    f=create()
    for i in range(150):factor(f,str(i),4.)
    assert f.U[0,0]==16
    assert f.nav.P[6,6]>1.
    assert f.nav.P[6,6]<3.
    assert np.linalg.eigvalsh(compose_joint(f.nav.P,f.C,f.U)).min()>-1e-5


def test_repeated_cell_is_strict_noop():
    f=create();factor(f)
    p=f.nav.snapshot();c=f.C.copy()
    r=factor(f,'a',1000.)
    assert r.reason=='ALREADY_USED' and not r.applied
    np.testing.assert_array_equal(f.C,c)
    np.testing.assert_array_equal(f.nav.snapshot().P,p.P)
    assert f.map_updates==1


def test_sensor_updates_apply_exact_native_composite_to_cross():
    f=create();factor(f)
    c=f.C.copy();f.feed('imu',[.05])
    F=np.eye(18);F[6,3]=.05;F[7,4]=.05
    np.testing.assert_allclose(f.C,F@c,atol=1e-12)
    c=f.C.copy();P=f.nav.P.copy();H=h();R=2.**2
    K=(P@H)/(H@P@H+R)
    A=np.eye(18)-np.outer(K,H)
    f.feed('gnss',[2.,0.])
    np.testing.assert_allclose(f.C,A@c,atol=1e-12)
    assert f.imu_events==1 and f.gnss_events==1
    assert not f.failed


def test_invalid_native_transition_fails_closed_after_map_update():
    f=create();factor(f)
    before=f.C.copy();f.nav.trace=(False,0,np.eye(18))
    with pytest.raises(RuntimeError,match='reseed'):
        f._take()
    assert f.failed
    np.testing.assert_array_equal(f.C,before)
    with pytest.raises(RuntimeError,match='invalidated'):
        f.feed('imu',[.01])


def test_native_initialization_reseed_is_harmless_before_first_factor():
    f=create();f.nav.trace=(False,0,np.eye(18))
    f._take();assert not f.failed and not f.engaged
    assert factor(f).applied


def test_invalid_native_map_identity_fails_without_mutation():
    f=create();p=f.nav.P.copy();c=f.C.copy()
    with pytest.raises(ValueError,match='independent patch'):
        f.factor(patch_id='other',cell='new',residual_m=2.,h=h(),j=j(),local_sigma_m=.3)
    np.testing.assert_array_equal(p,f.nav.P);np.testing.assert_array_equal(c,f.C)


def test_bad_innovation_rejected_without_consuming_cell():
    f=create();r=factor(f,'bad',1000.,gate=9.)
    assert r.reason=='INNOVATION_GATE' and not r.applied
    assert not f.used_cells and f.map_updates==0


def test_no_marginal_map_sigma_added_a_second_time():
    f=create();factor(f)
    c=f.C.copy();p=f.nav.P.copy()
    # Correct cross covariance prevents unbounded information accumulation.
    factor(f,'second')
    assert f.nav.P[6,6]>0.5
    assert abs(f.C[6,0])>=abs(c[6,0])-.1
    assert f.U[0,0]==16
    assert np.linalg.eigvalsh(compose_joint(p,c,f.U)).min()>-1e-5


@pytest.mark.parametrize('sigma',[0.,-1.,math.nan,math.inf])
def test_invalid_local_noise_rejected(sigma):
    f=create()
    with pytest.raises(ValueError):f.factor(patch_id='map-v1',cell='x',residual_m=1.,h=h(),j=j(),local_sigma_m=sigma)


@pytest.mark.parametrize('value',[math.nan,math.inf,-math.inf])
def test_invalid_residual_rejected(value):
    with pytest.raises(ValueError):factor(create(),residual=value)


def test_cross_covariance_map_mismatch_refused():
    with pytest.raises(ValueError):NativeSchmidt(FakeNav(),'x',[[-1,0],[0,1]])


def test_no_blind_map_update_if_native_unusable():
    f=create();f.nav.snapshot=lambda:SimpleNamespace(P=np.eye(18),ready=True,usable=False)
    assert factor(f).reason=='NATIVE_UNAVAILABLE'


def test_source_patch_rejects_non_matching_upstream():
    from prepare_native_transition import patch_filter,patch_facade,replace_once
    with pytest.raises(ValueError,match='one exact source anchor'):
        replace_once('abc','xyz','x','bad')
    with pytest.raises(ValueError):patch_filter('unexpected upstream source')
    with pytest.raises(ValueError):patch_facade('unexpected upstream source')


def test_native_schmidt_factor_matches_full_augmented_joseph():
    f=create()
    prior=compose_joint(f.nav.P,f.C,f.U)
    H=np.zeros((1,20));H[0,6]=1.;H[0,18]=-1.
    S=H@prior@H.T+np.array([[.09]])
    k=(prior[:18]@H.T)/float(S[0,0])
    joint_k=np.vstack((k,np.zeros((2,1))))
    A=np.eye(20)-joint_k@H
    expected=A@prior@A.T+joint_k@np.array([[.09]])@joint_k.T
    update=factor(f,'first',4.)
    assert update.applied
    np.testing.assert_allclose(compose_joint(f.nav.P,f.C,f.U),expected,rtol=0,atol=1e-11)


def test_stale_map_error_correlate_not_squared_on_narrow_corridor():
    f=create()
    p=f.nav.P[6,6]
    for i in range(200):
        r=factor(f,f'cell-{i}',residual=4.)
        assert r.applied
    # Three-dimensional map covariance remains exactly fixed; repeated cells
    # reduce only the genuinely independent 0.3 m local residual.
    assert f.nav.P[6,6] > 1.5
    assert f.nav.P[6,6] < p
    np.testing.assert_array_equal(f.U,np.diag([16.,16.]))


def test_native_reset_applies_to_cross_not_just_nav_marginal():
    f=create()
    f.nav.trace=(True,1,np.eye(18))
    factor(f,'a')
    c=f.C.copy()
    g=np.eye(18);g[0,1]=.02;g[2,0]=-.03
    f.nav.trace=(True,1,g)
    f._take()
    np.testing.assert_allclose(f.C,g@c,atol=1e-12)


def test_wrong_error_state_shape_is_rejected_before_injection():
    f=create();old=f.nav.P.copy()
    with pytest.raises(ValueError):
        f.factor(patch_id='map-v1',cell='b',residual_m=3.,h=[1.,0.],j=j(),local_sigma_m=.3)
    np.testing.assert_array_equal(f.nav.P,old)


def test_refused_factor_does_not_set_engaged_or_consume_native_trace():
    f=create()
    p=f.nav.P.copy()
    r=factor(f,'bad',300.,gate=9.)
    assert not r.applied and not f.engaged
    np.testing.assert_array_equal(f.nav.P,p)
    assert not f.used_cells
