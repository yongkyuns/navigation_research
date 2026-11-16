"""Exact linear Schmidt-map regression and negative controls."""
import copy
import math
import numpy as np
import pytest

from map_correlation import SchmidtMapReference, stable_patch_key


def model(nx=2,nm=1):
    return SchmidtMapReference(np.zeros(nx), np.diag([100.] + [4.]*(nx-1)),
                               np.eye(nm)*16.,map_id='road-v1')


def shapes(f):
    H=np.zeros((1,f.n));H[0,0]=1
    J=np.zeros((1,f.k));J[0,0]=1
    R=np.array([[.04]])
    return H,J,R


def joint_update(p,h,r,innovation,k):
    kh=np.zeros((p.shape[0],len(innovation)))
    kh[:k]=p[:k]@h.T@np.linalg.inv(h@p@h.T+r)
    a=np.eye(p.shape[0])-kh@h
    return .5*((a@p@a.T+kh@r@kh.T)+(a@p@a.T+kh@r@kh.T).T)


def test_exact_schmidt_joseph_with_correlated_prior_n18():
    rng=np.random.default_rng(12)
    a=rng.normal(size=(20,20));cov=a@a.T+.2*np.eye(20)
    x=rng.normal(size=18)
    f=SchmidtMapReference(x,cov[:18,:18],cov[18:,18:],C=cov[:18,18:],
                          map_id='shared')
    H=rng.normal(size=(2,18));J=rng.normal(size=(2,2));r=np.diag([.5,2.]);z=rng.normal(size=2)
    prior=f.joint_covariance();mu=f.x.copy();v=z-H@mu
    full=joint_update(prior,np.hstack((H,J)),r,v,18)
    k=prior[:18]@np.hstack((H,J)).T@np.linalg.inv(np.hstack((H,J))@prior@np.hstack((H,J)).T+r)
    result=f.road(z,H,J,r,observation_key='cell-0')
    assert result.applied
    np.testing.assert_allclose(f.x,mu+k@v,atol=2e-12,rtol=2e-12)
    np.testing.assert_allclose(f.joint_covariance(),full,atol=2e-11,rtol=1e-11)
    np.testing.assert_allclose(f.U,cov[18:,18:],atol=1e-11)
    np.testing.assert_array_equal(f.m,np.zeros(2))
    assert np.linalg.eigvalsh(f.joint_covariance()).min()>=-1e-11


def test_static_map_error_does_not_average_away():
    f=model(1,1);H,J,R=shapes(f)
    initial=f.P[0,0]
    f.road([5.],H,J,R,observation_key='cell0')
    first=f.P[0,0]
    assert 13.7 < first < 13.9
    assert f.C[0,0] < 0
    for i in range(1,100):
        f.road([5.],H,J,R,observation_key=f'cell{i}')
    # Repeating independent *small* shape residuals can reduce their own
    # noise, but it cannot eliminate the common map offset uncertainty.
    assert 13.7 < f.P[0,0] <= first
    assert f.P[0,0] > initial/10
    assert f.map_updates==100
    assert f.U[0,0]==pytest.approx(16.)
    assert np.linalg.eigvalsh(f.joint_covariance()).min()>=-1e-10


def test_naive_inflation_collapses_under_repeated_shared_map_error():
    f=model(1,1)
    h,j,r=shapes(f)
    for i in range(100):
        f.road([5.],h,j,r,observation_key=str(i))
    incorrect=1./(1./100.+100./(16.+.04))
    assert incorrect < .2
    assert f.P[0,0]/incorrect>60


def test_duplicate_geometry_observation_is_exact_noop():
    f=model();h,j,r=shapes(f)
    f.road([3],h,j,r,observation_key='a')
    snapshot=(f.x.copy(),f.joint_covariance().copy(),f.map_updates)
    x=f.road([90],h,j,r,observation_key='a')
    assert not x.applied and x.reason=='ALREADY_USED'
    np.testing.assert_array_equal(f.x,snapshot[0])
    np.testing.assert_array_equal(f.joint_covariance(),snapshot[1])
    assert f.map_updates==snapshot[2]


def test_sensor_observation_propagates_cross_moment():
    f=model();h,j,r=shapes(f)
    f.road([4],h,j,r,observation_key='a')
    prior=f.joint_covariance();H=np.array([[1.,0.]])
    full=np.hstack((H,np.zeros((1,1))))
    expect=joint_update(prior,full,np.array([[2.]]),np.array([1.]),2)
    result=f.sensor([f.x[0]+1],H,[[2.]])
    assert result.applied
    np.testing.assert_allclose(f.joint_covariance(),expect,atol=1e-12)


def test_transition_and_reset_covariance_obey_joint_augmented_reference():
    f=model();h,j,r=shapes(f)
    f.road([3],h,j,r,observation_key='a')
    prev=f.joint_covariance();F=np.array([[1,1.5],[0,1.]]);Q=np.diag([.3,.1]);G=np.array([[.99,.03],[0.,.98]])
    f.predict(F,Q,control=[2.,0.])
    A=np.eye(3);A[:2,:2]=F
    noise=np.zeros((3,3));noise[:2,:2]=Q
    expected=A@prev@A.T+noise
    np.testing.assert_allclose(f.joint_covariance(),expected,atol=1e-11)
    before=f.joint_covariance();A[:2,:2]=G
    f.reset(G)
    np.testing.assert_allclose(f.joint_covariance(),A@before@A.T,atol=1e-11)


def test_zero_sensor_cross_after_map_still_does_not_erase_map_memory():
    f=model();h,j,r=shapes(f)
    f.road([6],h,j,r,observation_key='a')
    assert f.C[0,0]<0
    f.predict([[1.,1.],[0.,1.]],[[.2,0],[0,.1]])
    f.sensor([0.],[[1.,0.]],[[1.]])
    assert f.C[0,0] < 0
    f.road([6],h,j,r,observation_key='b')
    assert np.linalg.eigvalsh(f.joint_covariance()).min()>-1e-9


def test_explicit_map_mismatch_fails_closed_without_change():
    f=model();before=f.joint_covariance();h,j,r=shapes(f)
    with pytest.raises(ValueError,match='Different map patch'):
        f.road([3],h,j,r,observation_key='same-cell',map_id='another')
    np.testing.assert_array_equal(f.joint_covariance(),before)
    assert f.map_updates==0


def test_chi_square_gate_skips_bad_map_and_leaves_mean_and_cov_exact():
    f=model();h,j,r=shapes(f)
    before=f.x.copy(),f.joint_covariance().copy()
    result=f.road([1000.],h,j,r,observation_key='bad',chi2_gate=9.)
    assert not result.applied and result.reason=='INNOVATION_GATE'
    np.testing.assert_array_equal(f.x,before[0])
    np.testing.assert_array_equal(f.joint_covariance(),before[1])
    assert f.used_observations==set()


def test_wrong_map_binding_cannot_produce_plausible_false_precision():
    f=model();h,j,r=shapes(f)
    f.road([8],h,j,r,observation_key='a')
    for i in range(1,30):
        f.road([8],h,j,r,observation_key=f'cell{i}')
    assert f.P[0,0] > 10.
    # If the mapper silently substituted another independent patch, the
    # covariance could collapse. A mismatch instead fails before mutation.
    with pytest.raises(ValueError):f.road([8],h,j,r,observation_key='x',map_id='elsewhere')


def test_orthogonal_road_normals_preserve_cross_axis_uncertainty():
    f=model(2,2);f.U=np.diag([9.,25.]);f._check_joint()
    H=np.eye(2);J=np.eye(2)
    f.road([5.,-3.],H,J,np.eye(2)*.2,observation_key='first')
    p=f.joint_covariance()
    assert np.linalg.eigvalsh(p).min()>=-1e-10
    assert f.P[0,0]>7.5 and f.P[1,1]>3.4
    assert f.C[0,0]<0 and f.C[1,1]<0


@pytest.mark.parametrize('invalid',[math.nan,math.inf,-math.inf])
def test_invalid_measurement_is_rejected_without_mutation(invalid):
    f=model();before=f.joint_covariance().copy()
    h,j,r=shapes(f)
    with pytest.raises(ValueError):f.road([invalid],h,j,r,observation_key='a')
    np.testing.assert_array_equal(f.joint_covariance(),before)


@pytest.mark.parametrize('bad',[[-1.],[math.nan]])
def test_invalid_map_noise_rejected(bad):
    f=model();h,j,_=shapes(f)
    with pytest.raises(ValueError):f.road([3.],h,j,[bad],observation_key='a')


def test_negative_map_covariance_rejected():
    with pytest.raises(ValueError,match='U'):
        SchmidtMapReference([0.],[[1.]],[[-1.]],map_id='a')


def test_joint_cross_covariance_inconsistent_rejected():
    with pytest.raises(ValueError,match='Inconsistent'):
        SchmidtMapReference([0.],[[1.]],[[1.]],C=[[2.]],map_id='a')


def test_patch_identity_stable_and_version_scoped():
    assert stable_patch_key('v1','geometry')==stable_patch_key('v1','geometry')
    assert stable_patch_key('v1','geometry')!=stable_patch_key('v2','geometry')
    assert stable_patch_key('v1','geometry')!=stable_patch_key('v1','different')
    with pytest.raises(ValueError):stable_patch_key('','geometry')


def test_state_and_map_mean_remain_distinct():
    f=SchmidtMapReference([0.],[[20.]],[[16.]],map_mean=[-2.],map_id='road')
    f.road([7],[[1]],[[1]],[[.1]],observation_key='cell')
    np.testing.assert_array_equal(f.m,[-2.])
    assert f.x[0]>0


def test_replay_reset_jacobian_is_required_not_silently_ignored():
    f=model();h,j,r=shapes(f);f.road([3],h,j,r,observation_key='a')
    with pytest.raises(ValueError,match='G'):
        f.reset([[1]])


def test_sensor_map_measurement_keeps_same_map_prior_on_multiple_updates():
    f=model();H,J,R=shapes(f)
    for i in range(20):
        f.road([2.],H,J,R,observation_key=f'{i}')
        f.sensor([0.],[[1.,0.]],[[3.]])
    assert f.U[0,0]==pytest.approx(16.)
    assert f.P[0,0]>0 and np.linalg.eigvalsh(f.joint_covariance()).min()>=-1e-10


def test_exact_zero_measurement_noise_is_allowed_if_innovation_is_positive():
    f=model();h,j,r=shapes(f)
    result=f.road([3.],h,j,[[0.]],observation_key='exact')
    assert result.applied


def test_controlled_biased_map_experiment_produces_same_seeded_result():
    from evaluate_map_correlation import evaluate
    a=evaluate(trials=6,seconds=9,seed=20261116)
    b=evaluate(trials=6,seconds=9,seed=20261116)
    assert a==b
    assert a['results']['GNSS only']['map_updates_per_trial']==0
    assert a['results']['correlated Schmidt map offset']['map_updates_per_trial']==9
    assert a['results']['correlated Schmidt map offset']['mean_predicted_sigma_m'] > 0


def test_no_fake_covariance_reduction_on_duplicate_map_cell():
    f=model();h,j,r=shapes(f)
    first=f.road([10.],h,j,r,observation_key=stable_patch_key('map-v1','cell-3'))
    assert first.applied
    prior=f.joint_covariance().copy()
    for _ in range(30):
        result=f.road([-100.],h,j,r,observation_key=stable_patch_key('map-v1','cell-3'))
        assert not result.applied
    np.testing.assert_array_equal(f.joint_covariance(),prior)


def test_discarding_cross_covariance_after_each_native_event_is_not_schmidt():
    correct=model(1,1)
    incorrectly_reset=model(1,1)
    H,J,R=shapes(correct)
    for cell in range(100):
        correct.road([4.],H,J,R,observation_key=f'{cell}')
        incorrectly_reset.C[:]=0
        incorrectly_reset.road([4.],H,J,R,observation_key=f'{cell}')
    assert correct.P[0,0] > 13.7
    assert incorrectly_reset.P[0,0] < .3
    assert correct.P[0,0]/incorrectly_reset.P[0,0] > 50
