"""Numerical and data-ownership checks for the opt-in receiver fault mode."""
import ctypes as C
import json
import os
from pathlib import Path
import sys
import numpy as np
import pytest
HERE=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(HERE),str(HERE/'tests')]
from native import Library, Frame, POINTER
from native_bias import BiasConfig, BiasLibrary, position_velocity_consistency
from datasets import events
from evaluate_bias import corrupt
from road_bank import RoadBank
from test_integration import recording, initialized
from test_intersections_native import native_bank


@pytest.fixture
def bias_library():
    path=os.environ.get('ROUTEFUSION_BIAS_LIBRARY')
    if not path:pytest.skip('Set ROUTEFUSION_BIAS_LIBRARY')
    return BiasLibrary(path)


def raw_rows():
    rows=np.zeros((5,14));rows[:,0]=np.arange(5)*.5
    rows[:,1]=43.47;rows[:,2]=-79.70;rows[:,3]=100.
    rows[:,7:10]=.2;rows[:,10:13]=.05;rows[:,13]=np.nan
    return rows


def test_constant_bias_is_not_new_motion_information():
    rows=raw_rows();shift=corrupt(rows,dict(fault='cold_start',metres=20))
    assert position_velocity_consistency(rows)[0]<1e-7
    assert position_velocity_consistency(shift)[0]<1e-7


def test_position_only_ramp_disagrees_with_velocity():
    rows=raw_rows();shift=corrupt(rows,dict(fault='position',metres=20,start=-1,end=10,ramp_s=5))
    distance,score=position_velocity_consistency(shift)
    assert 7.99<distance<8.01 and score>16


def test_coupled_position_velocity_ramp_is_not_detectable_by_consistency():
    rows=raw_rows();shift=corrupt(rows,dict(fault='position_velocity',metres=20,start=-1,end=10,ramp_s=5))
    distance,score=position_velocity_consistency(shift)
    assert distance<.001 and score<.001


def test_fault_injection_preserves_inputs_and_uncertainties():
    rows=raw_rows();before=rows.copy();out=corrupt(rows,dict(fault='position',metres=20,start=-1,end=10))
    np.testing.assert_array_equal(rows,before)
    np.testing.assert_array_equal(rows[:,3:],out[:,3:])


def test_configuration_rejects_nonfinite_and_unknown_policy():
    for values in [dict(tau_s=0),dict(shock_sigma_m=-1),dict(stationary_sigma_m=float('nan')),dict(activation='guess')]:
        with pytest.raises(ValueError):BiasConfig(**values)


def test_frozen_policy_matches_executed_defaults():
    from dataclasses import asdict
    config=json.loads((Path(__file__).resolve().parents[1]/'bias_config.json').read_text())
    for key,value in asdict(BiasConfig()).items():assert config[key]==value


def test_clean_runtime_is_exact_until_activation(bias_library,recording):
    plain=initialized(Library(bias_library.path),recording)
    adaptive=initialized(bias_library,recording)
    assert not adaptive.active
    np.testing.assert_array_equal(plain.snapshot().raw,adaptive.snapshot().raw[:358])
    plain.close();adaptive.close()


def test_hot_activation_preserves_physical_prior(bias_library,recording):
    nav=initialized(bias_library,recording);before=nav.snapshot()
    assert bias_library.lib.rf_activate_bias(nav.handle,90.,15.,3.,20.)==0
    nav.active=True;after=nav.snapshot()
    np.testing.assert_array_equal(before.raw[:358],after.raw[:358])
    np.testing.assert_array_equal(after.gnss_bias_cross,np.zeros((18,3)))
    np.testing.assert_array_equal(after.gnss_bias_covariance,np.eye(3)*400)
    assert bias_library.lib.rf_activate_bias(nav.handle,90.,15.,3.,20.)!=0
    np.testing.assert_array_equal(after.raw,nav.snapshot().raw)
    nav.close()


def test_fault_activates_without_using_injection_time_or_reference(bias_library,recording):
    imu,gnss,_=recording;gnss=corrupt(gnss,dict(fault='position',metres=20,start=35,end=55))
    nav=bias_library.create()
    for kind,row in events(imu,gnss):
        if row[0]>40:break
        getattr(nav,kind)(row)
    assert nav.active and 35<nav.activation_t<40
    before=nav.snapshot().raw.copy();copy=nav.clone()
    np.testing.assert_array_equal(copy.snapshot().raw,before)
    with pytest.raises(RuntimeError):copy.inject(np.zeros(18),copy.snapshot().P)
    np.testing.assert_array_equal(copy.snapshot().raw,before)
    copy.close();nav.close()


def test_rejected_duplicate_does_not_mutate_detector(bias_library,recording):
    nav=initialized(bias_library,recording);before=nav.snapshot().raw.copy()
    with pytest.raises(ValueError):nav.gnss(nav.history[-1])
    np.testing.assert_array_equal(nav.snapshot().raw,before)
    bad=nav.history[-1].copy();bad[0]+=1;bad[7]=-1
    with pytest.raises(ValueError):nav.gnss(bad)
    np.testing.assert_array_equal(nav.snapshot().raw,before);nav.close()


def test_input_gap_does_not_trigger_from_a_stale_receiver_fix(bias_library,recording):
    nav=initialized(bias_library,recording);row=nav.history[-1].copy();row[0]+=10;row[1]+=.0002
    nav.prepare_gnss(row)
    assert not nav.active and not nav.history
    nav.gnss(row);assert len(nav.history)==1
    nav.close()


def reset_matrix(dx):
    G=np.eye(21);x,y,z=dx[:3];skew=np.array([[0,-z,y],[z,0,-x],[-y,x,0]])
    G[:3,:3]-=.5*skew
    return G


def test_augmented_map_interval_matches_joint_moment_formula(bias_library,recording):
    nav=initialized(bias_library,recording)
    assert bias_library.lib.rf_activate_bias(nav.handle,90.,15.,3.,2.)==0
    nav.active=True
    # Consume original observations to create non-zero navigation/bias cross terms.
    imu,gnss,_=recording
    for kind,row in events(imu,gnss):
        if row[0]<=30:continue
        if row[0]>32:break
        getattr(nav,kind)(row)
    before=nav.snapshot();assert np.linalg.norm(before.gnss_bias_cross)>1e-6
    h=np.zeros(18);h[6]=.6;h[7]=.8
    u=before.augmented_covariance@np.r_[h,np.zeros(3)]
    S=float(h@before.P@h+2.25);alpha=.01;beta=-.25/S
    dx=alpha*u[:18];G=reset_matrix(dx)
    expected=G@(before.augmented_covariance+beta*np.outer(u,u))@G.T
    nav.condition_interval(h,alpha,beta);after=nav.snapshot()
    np.testing.assert_allclose(after.augmented_covariance,expected,atol=2e-5,rtol=2e-5)
    np.testing.assert_allclose(after.gnss_bias,before.gnss_bias+alpha*u[18:],atol=2e-6,rtol=2e-5)
    assert np.linalg.eigvalsh(after.augmented_covariance)[0]>-1e-6
    nav.close()


def test_interval_validation_does_not_mutate_state(bias_library,recording):
    nav=initialized(bias_library,recording);before=nav.snapshot().raw.copy();h=np.zeros(18);h[6]=1.
    with pytest.raises(ValueError):nav.condition_interval(h,0.,1.)
    with pytest.raises(ValueError):nav.condition_interval(h,0.,-100.)
    np.testing.assert_array_equal(nav.snapshot().raw,before);nav.close()


def test_bias_checkpoint_replay_includes_detector_and_joint_covariance(bias_library,recording):
    bank=native_bank(bias_library,recording);bank.step(30.)
    assert bank.window.nav is not None
    control=bank.base.clone();imu,gnss,_=recording
    for kind,row in events(imu,gnss):
        if row[0]<=30:continue
        if row[0]>31:break
        getattr(bank,kind)(row);getattr(control,kind)(row)
    bank.step(31.);before=bank.base.snapshot().raw.copy()
    np.testing.assert_array_equal(before,control.snapshot().raw)
    bank.close_branches();assert bank._recover(bank.base.snapshot())
    np.testing.assert_array_equal(before,bank.base.snapshot().raw)
    bank.reset();bank.base.close();control.close()
