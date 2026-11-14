import numpy as np
import pytest
from scipy.stats import truncnorm
from routefusion.filter import (N,PXY,VXY,YAW,MB,PlanarFilter,PVT,condition_interval,
                                interval_moments,mix_gaussians)

@pytest.mark.parametrize('mu,var,lo,hi',[(0,4,-1,1),(3,2,-1,2),(-5,3,-2,8),(0,1,5,6),(0,1,-6,-5)])
def test_scalar_interval_matches_scipy(mu,var,lo,hi):
    l,m,v=interval_moments(mu,var,lo,hi)
    a,b=(lo-mu)/np.sqrt(var),(hi-mu)/np.sqrt(var)
    expected=truncnorm.stats(a,b,loc=mu,scale=np.sqrt(var),moments='mv')
    np.testing.assert_allclose([m,v],expected,rtol=1e-7,atol=1e-9)
    assert l<=0

def test_interval_matches_monte_carlo_joint_moments():
    rng=np.random.default_rng(31)
    x=np.array([2.,-1.]); P=np.array([[4.,1.2],[1.2,3.]])
    H=np.array([1.,0.4]); offset=-0.2
    xn,Pn,ll=condition_interval(x,P,H,float(H@x+offset),-1,2,0.5)
    samples=rng.multivariate_normal(x,P,500_000)
    y=samples@H+offset+rng.normal(0,np.sqrt(0.5),len(samples))
    ok=(y>=-1)&(y<=2); selected=samples[ok]
    np.testing.assert_allclose(xn,selected.mean(axis=0),atol=0.025)
    np.testing.assert_allclose(Pn,np.cov(selected.T),atol=0.035)
    assert abs(np.exp(ll)-ok.mean())<0.003

def test_cross_track_constraint_does_not_invent_along_track_information():
    x=np.array([5.,4.]); P=np.diag([100.,25.])
    y,Q,_=condition_interval(x,P,np.array([0.,1.]),4.,-2,2)
    assert y[0]==x[0] and Q[0,0]==P[0,0]
    assert Q[1,1]<P[1,1]

def test_map_translation_uncertainty_is_not_fresh_measurement_noise():
    # p~N(0,100), m~N(0,9); the exact constraint p-m=0 leaves
    # Var(p)=100*9/(100+9), not zero. Check a narrow corridor approximation.
    x=np.zeros(2); P=np.diag([100.,9.]); H=np.array([1.,-1.])
    _,Q,_=condition_interval(x,P,H,0.,-0.001,0.001)
    assert abs(Q[0,0]-900/109)<1e-5

def test_mixture_retains_between_road_uncertainty():
    m,P=mix_gaussians([np.array([0.,-13.]),np.array([0.,13.])],
                      [np.eye(2),np.eye(2)],[.5,.5])
    np.testing.assert_allclose(m,[0,0])
    assert P[1,1]==170

def test_pvt_duplicate_rejected():
    f=PlanarFilter(np.zeros(N),np.eye(N))
    z=PVT(1,1,np.zeros(2),np.zeros(2),np.eye(4))
    f.update_pvt(z)
    with pytest.raises(ValueError): f.update_pvt(z)

def test_invalid_covariance_rejected():
    with pytest.raises(np.linalg.LinAlgError):
        PVT(1,1,np.zeros(2),np.zeros(2),-np.eye(4))

def test_predict_and_updates_keep_covariance_symmetric_positive():
    f=PlanarFilter(np.zeros(N),np.eye(N))
    for i in range(1,41):
        f.predict(np.array([0.1,0.02]),0.01,0.2)
        if i%5==0:
            f.nhc()
            f.update_pvt(PVT(i,i*.2,f.x[:2]+[1,2],f.x[2:4]+[.1,-.1],np.eye(4)))
        np.testing.assert_allclose(f.P,f.P.T,atol=1e-12)
        assert np.linalg.eigvalsh(f.P).min()>0
