"""Boundary invariants; dataset references never enter the navigation adapter."""
import gzip
import json
import os
from pathlib import Path
import sys
import numpy as np
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from native import Frame,Library
from datasets import read_csv,events,load_inputs
from evaluate_map import stress_gnss
from road_bank import RoadBank
from osm_graph import compile_osm

@pytest.fixture
def library():
    path=os.environ.get('ROUTEFUSION_NATIVE_LIBRARY')
    if not path:pytest.skip('Set ROUTEFUSION_NATIVE_LIBRARY for native integration tests')
    return Library(path)

@pytest.fixture
def recording():
    path=os.environ.get('ROUTEFUSION_DATASETS')
    if not path:pytest.skip('Set ROUTEFUSION_DATASETS for recorded replay tests')
    return load_inputs(Path(path)/'urban-short-turn-loop-nominal-002')

def initialized(library,recording):
    imu,gnss,_=recording
    nav=library.create()
    for kind,row in events(imu,gnss):
        if row[0]>30:break
        getattr(nav,kind)(row)
    assert nav.snapshot().ready
    return nav

def test_frame_orthogonality_and_position_covariance_transform():
    frame=Frame([43.47,-79.70,100.]);A=frame.from_ned(frame.origin)
    np.testing.assert_allclose(A,[[0,1,0],[1,0,0],[0,0,-1]],atol=1e-14)
    np.testing.assert_allclose(frame.project(frame.origin),0,atol=1e-8)
    np.testing.assert_allclose(A@A.T,np.eye(3),atol=1e-14)

def test_timestamp_duplicates_fail_closed(tmp_path):
    p=tmp_path/'input.csv.gz'
    with gzip.open(p,'wt') as f:f.write('t_s,x\n1,2\n1,3\n')
    with pytest.raises(ValueError):read_csv(p)

def test_epoch_order_is_imu_first_on_tie():
    assert [k for k,_ in events(np.array([[1.],[2.]]),np.array([[1.],[1.5]]))]==['imu','gnss','gnss','imu']

def test_stress_keeps_original_inputs_unmodified(recording):
    _,g,_=recording;copy=g.copy();z=stress_gnss(g,'north_bias20')
    np.testing.assert_array_equal(g,copy)
    np.testing.assert_array_equal(z[:,[0,2,3,4,5,6,7,8,9,10,11,12,13]],g[:,[0,2,3,4,5,6,7,8,9,10,11,12,13]])
    assert np.any(z[:,1]!=g[:,1])
    zz=stress_gnss(g,'outage30');assert not np.any((zz[:,0]>=90)&(zz[:,0]<=120))

def test_native_clone_isolation_and_zero_error(library,recording):
    nav=initialized(library,recording);copy=nav.clone();before=nav.snapshot()
    np.testing.assert_array_equal(copy.snapshot().raw,before.raw)
    copy.inject(np.zeros(18),before.P)
    np.testing.assert_allclose(copy.snapshot().raw,before.raw,atol=1e-7,rtol=1e-7)
    dx=np.zeros(18);dx[6]=1.
    copy.inject(dx,copy.snapshot().P)
    assert abs(copy.snapshot().position[0]-before.position[0]-1)<1e-5
    np.testing.assert_array_equal(nav.snapshot().raw,before.raw)
    nav.close();copy.close()

def test_bad_moment_update_fails_without_mutation(library,recording):
    nav=initialized(library,recording);before=nav.snapshot().raw.copy()
    with pytest.raises(ValueError):nav.inject(np.zeros(18),-np.eye(18))
    np.testing.assert_array_equal(nav.snapshot().raw,before);nav.close()

def test_missing_map_exact_fallback_and_repeated_epoch(library,recording):
    nav=initialized(library,recording);s=nav.snapshot()
    class AbsentMap:
        frame=Frame(s.anchor)
        def covers(self,lla):return False
    bank=RoadBank(nav,AbsentMap());out=bank.step(30.)
    assert out['status']=='MAP_UNAVAILABLE' and not bank.branches
    np.testing.assert_array_equal(out['base'].raw,s.raw)
    np.testing.assert_array_equal(out['conditional_position'],out['position'])
    with pytest.raises(ValueError):bank.step(30.)
    nav.close()

def test_input_loader_does_not_read_reference_files(tmp_path,recording):
    source=Path(os.environ['ROUTEFUSION_DATASETS'])/'urban-short-turn-loop-nominal-002'
    import shutil
    for name in ('manifest.json','imu.csv.gz','gnss.csv.gz'):shutil.copyfile(source/name,tmp_path/name)
    a,b,meta=load_inputs(tmp_path)
    assert len(a)==len(recording[0]) and len(b)==len(recording[1])
    assert meta['reference_available']

def test_map_checksum_fails_closed(tmp_path):
    p=tmp_path/'map.gz';p.write_bytes(gzip.compress(b'{}'))
    meta=tmp_path/'provenance.json';meta.write_text(json.dumps({'response_sha256':'wrong'}))
    with pytest.raises(ValueError):compile_osm(p,meta)

def test_original_gnss_evidence_finite(library,recording):
    nav=initialized(library,recording);_,g,_=recording
    z=g[np.searchsorted(g[:,0],30.)];assert np.isfinite(RoadBank.gnss_evidence(nav.snapshot(),z));nav.close()
