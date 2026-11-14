import numpy as np
import pytest
from routefusion import Edge,TileStore,PlanarFilter,RouteTracker,PVT
from routefusion.filter import N,PXY,VXY,YAW
from routefusion.tracker import Branch
from routefusion.simulate import Case,initial_filter,run_case

def test_tiles_roundtrip_and_shared_edge_identity(tmp_path):
    s=TileStore(tile_size=256)
    e=Edge('long',np.array([[10,10],[700,10]]))
    s.add(e); s.cover([0,0],[800,300]); s.save(tmp_path)
    t=TileStore.load(tmp_path)
    assert len(t.edges)==1
    assert len(t.candidates(np.array([256.,10.]),20))==1
    assert t.edges['long'].to_dict()==e.to_dict()
    assert t.is_loaded(np.array([600,100]))

def test_crossing_lines_are_not_connected():
    s=TileStore()
    s.add(Edge('bridge',np.array([[0,50],[100,50]]),layer=1))
    s.add(Edge('ground',np.array([[50,0],[50,100]]),layer=0))
    assert s.edges['bridge'].successors==()
    assert s.edges['ground'].successors==()

def test_conflicting_tile_geometry_rejected():
    s=TileStore(); s.add(Edge('x',np.array([[0,0],[10,0]])))
    with pytest.raises(ValueError): s.add(Edge('x',np.array([[0,0],[11,0]])))

def test_no_map_fallback_is_identical_to_map_free_filter():
    x=np.zeros(N); x[2]=8
    s=TileStore(); f=PlanarFilter(x,np.eye(N)); reference=f.clone()
    tr=RouteTracker(f,s)
    for i in range(1,6):
        a=np.array([.01,.02]); g=.002
        tr.predict(a,g,.2); reference.predict(a,g,.2)
    z=PVT(1,1,np.array([8.,0]),np.array([8.,0]),np.eye(4))
    out=tr.step(1,z); reference.nhc(); reference.update_pvt(z)
    assert out.status=='MAP_UNAVAILABLE'
    np.testing.assert_array_equal(tr.base.x,reference.x)
    np.testing.assert_array_equal(tr.base.P,reference.P)

def test_repeated_spatial_map_factor_is_not_reapplied():
    s=TileStore(); s.add(Edge('x',np.array([[0.,0],[200.,0]])))
    x=np.zeros(N); x[:2]=[50,4]; x[2]=8
    tr=RouteTracker(PlanarFilter(x,np.eye(N)*4),s)
    b=Branch('x',tr.base.clone(),0)
    assert np.isfinite(tr._map_update(b))
    xn,Pn=b.nav.x.copy(),b.nav.P.copy()
    assert tr._map_update(b)==0.0
    np.testing.assert_array_equal(xn,b.nav.x)
    np.testing.assert_array_equal(Pn,b.nav.P)

def test_out_of_order_map_epoch_rejected():
    tr=RouteTracker(PlanarFilter(np.zeros(N),np.eye(N)),TileStore())
    tr.step(1)
    with pytest.raises(ValueError): tr.step(1)

def test_symmetric_parallel_routes_remain_ambiguous():
    _,rows=run_case('ambiguous_parallel',0)
    assert all(r['status']=='AMBIGUOUS' for r in rows)
    assert max(abs(r['top_weight']-.5) for r in rows)<1e-8

def test_missing_tiles_do_not_create_offroad_evidence():
    _,rows=run_case('missing_tiles',0)
    for r in rows:
        if 45<=r['t']<=70:
            assert r['status']=='MAP_UNAVAILABLE'
            assert r['route_group'] is None
            assert r['output_e']==r['fallback_e'] and r['output_n']==r['fallback_n']

def test_turn_during_outage_keeps_correct_branch_in_development_case():
    metric,rows=run_case('turn_outage',0)
    assert metric['false_latch_rate']==0
    assert metric['latch_availability']>.75
    assert any(not r['gnss_available'] and r['status']=='LATCHED' and r['t']>50 for r in rows)

def test_offroad_departure_abstains_in_development_case():
    _,rows=run_case('offroad_leave',0)
    # A finite delay through the turning/road-width region is permitted.
    assert all(r['route_group'] is None for r in rows if r['t']>55)

def test_wgs84_to_local_east_north_and_graph_compiler(tmp_path):
    from routefusion.build_tiles import enu,compile_graph
    xy=enu([[0,0],[0.00001,0],[0,0.00001]],[0,0,0])
    np.testing.assert_allclose(xy[0],[0,0],atol=1e-9)
    assert 1.11<xy[1,0]<1.12 and abs(xy[1,1])<1e-8
    assert 1.10<xy[2,1]<1.11 and abs(xy[2,0])<1e-8
    doc={'type':'FeatureCollection','features':[{'type':'Feature',
        'geometry':{'type':'LineString','coordinates':[[0,0],[.001,0]]},
        'properties':{'edge_id':'test','successors':[],'half_width_m':6}}]}
    graph=compile_graph(doc,[0,0,0],'test-v1'); graph.save(tmp_path)
    loaded=TileStore.load(tmp_path)
    assert loaded.origin_wgs84==[0,0,0]
    assert loaded.edges['test'].successors==()

def test_compiler_requires_explicit_topology():
    from routefusion.build_tiles import compile_graph
    doc={'type':'FeatureCollection','features':[{'type':'Feature',
        'geometry':{'type':'LineString','coordinates':[[0,0],[.001,0]]},
        'properties':{'edge_id':'test','half_width_m':6}}]}
    with pytest.raises(ValueError): compile_graph(doc,[0,0,0],'v1')
