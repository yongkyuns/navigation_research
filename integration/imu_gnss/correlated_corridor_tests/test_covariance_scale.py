"""Mixed-unit joint-covariance checks must not hide impossible map correlation."""
import numpy as np
import pytest

from correlated_corridor import NativeCorridorSchmidt, corridor_moments
from native_schmidt import compose_joint, covariance_psd
from schmidt_native_tests.test_native_schmidt import FakeNav, h, j


def hidden_indefinite_prior():
    p=np.eye(18)
    p[0,0]=1e-8
    p[8,8]=1e4
    c=np.zeros((18,2))
    c[0,0]=.002  # > sqrt(P[0,0]*U[0,0]) = 0.00015
    u=np.eye(2)*2.25
    return p,c,u


def test_large_unrelated_variance_cannot_hide_impossible_cross_correlation():
    p,c,u=hidden_indefinite_prior()
    q=compose_joint(p,c,u)
    minimum=np.linalg.eigvalsh(q)[0]
    # Negative control: the old absolute-unit global tolerance passes it.
    assert -2e-5*max(1.,np.diag(q).max()) < minimum < 0
    with pytest.raises(ValueError,match='variance-normalized'):
        covariance_psd(q,atol=2e-5)


def test_invalid_joint_prior_cannot_create_a_negative_posterior_variance():
    p,c,u=hidden_indefinite_prior()
    with pytest.raises(ValueError,match='variance-normalized'):
        corridor_moments(p,c,u,h(),j(),4.,.5,.25)


def test_invalid_corridor_prior_rejected_before_native_injection():
    p,c,u=hidden_indefinite_prior()
    nav=FakeNav();nav.P=p
    f=NativeCorridorSchmidt(nav,'patch',u,cross=c)
    old=(nav.x.tobytes(),nav.P.tobytes(),f.C.tobytes(),f.U.tobytes())
    with pytest.raises(ValueError,match='variance-normalized'):
        f.interval_factor(patch_id='patch',cell='first',cross_track_m=4.,
                          half_width_m=.5,h=h(),j=j(),local_sigma_m=.25)
    assert old==(nav.x.tobytes(),nav.P.tobytes(),f.C.tobytes(),f.U.tobytes())
    assert not f.used_cells and not f.engaged and f.map_updates==0


def test_invalid_post_sensor_cross_moments_cannot_resume():
    f=NativeCorridorSchmidt(FakeNav(),'patch',np.eye(2)*2.25)
    assert f.interval_factor(patch_id='patch',cell='first',cross_track_m=4.,
        half_width_m=.5,h=h(),j=j(),local_sigma_m=.25)['applied']
    p,c,_=hidden_indefinite_prior()
    def corrupt(_):
        f.nav.P=p.copy();f.C=c.copy();f.nav.trace=(True,1,np.eye(18))
    f.nav.imu=corrupt
    with pytest.raises(ValueError,match='variance-normalized'):
        f.feed('imu',[.1])
    assert f.failed
    with pytest.raises(RuntimeError,match='invalidated'):
        f.feed('gnss',[2.,0.])


@pytest.mark.parametrize('spread',[0.,3.,6.,9.])
@pytest.mark.parametrize('rank',[1,7,20])
def test_valid_mixed_scale_and_singular_covariances_are_not_modified(spread,rank):
    rng=np.random.default_rng(114)
    a=rng.normal(size=(20,rank))
    a/=np.linalg.norm(a,axis=1,keepdims=True)
    a*=np.logspace(-spread,spread,20)[:,None]
    q=a@a.T
    original=q.tobytes()
    covariance_psd(q,atol=2e-5)
    assert q.tobytes()==original


@pytest.mark.parametrize('exponent',[-9.,-6.,0.,6.,9.])
def test_invalid_correlation_rejected_independently_of_units(exponent):
    correlation=np.array([[1.,1.1,0.],[1.1,1.,0.],[0.,0.,1.]])
    scales=np.array([10.**exponent,10.**(-exponent),100.])
    q=correlation*scales[:,None]*scales[None,:]
    with pytest.raises(ValueError,match='covariance'):
        covariance_psd(q,atol=2e-5)


def test_pairwise_bounds_are_not_enough_for_joint_psd():
    q=np.full((3,3),-.75);np.fill_diagonal(q,1.)
    q*=1e-16
    assert np.max(np.abs(q-np.diag(np.diag(q)))) < q[0,0]
    with pytest.raises(ValueError,match='variance-normalized'):
        covariance_psd(q,atol=2e-5)


def test_zero_uncertainty_control_and_frozen_directions_are_valid():
    q=np.diag([0.,0.,1e-14,1.,1e6])
    covariance_psd(q,atol=2e-5)
    covariance_psd(np.zeros((20,20)),atol=2e-5)


@pytest.mark.parametrize('cross',[1e-16,-1e-16])
def test_exact_zero_variance_cannot_have_nonzero_correlation(cross):
    q=np.diag([0.,1.,1e5]);q[0,1]=q[1,0]=cross
    with pytest.raises(ValueError,match='Zero-variance'):
        covariance_psd(q,atol=2e-5)


def test_negative_variance_is_not_hidden_by_a_large_diagonal():
    with pytest.raises(ValueError,match='Negative variance'):
        covariance_psd(np.diag([-1e-9,1e6]),atol=2e-5)


def test_scaled_asymmetry_is_not_hidden_by_small_physical_units():
    q=np.diag([1e-12,1e-12,1e6]);q[0,1]=1e-14
    with pytest.raises(ValueError,match='Asymmetric variance-normalized'):
        covariance_psd(q,atol=2e-5)


@pytest.mark.parametrize('p',[np.array([]),np.zeros(3),np.zeros((0,0)),np.zeros((2,3))])
def test_malformed_covariance_has_explicit_validation_error(p):
    with pytest.raises(ValueError,match='square matrix'):
        covariance_psd(p)


@pytest.mark.parametrize('tol',[-1.,float('nan'),float('inf')])
def test_invalid_tolerance_is_not_a_backdoor_to_acceptance(tol):
    with pytest.raises(ValueError,match='tolerance'):
        covariance_psd(np.eye(3),atol=tol)
