import math
import numpy as np
import pytest
from adapter import local_en,calibrate,planar_controls,predict_to,observation
from roads import compile_osm


def test_local_frame_east_north():
    xy=local_en(np.array([0,.00001,0]),np.array([0,0,.00001]),0,[0,0,0])
    np.testing.assert_allclose(xy[0],[0,0],atol=1e-9)
    assert 1.11<xy[1,0]<1.12 and 1.10<xy[2,1]<1.11


def test_pvt_ned_to_en_variances_and_no_heading_reuse():
    r={'t_s':1.,'ve_mps':2.,'vn_mps':3.,'pos_std_e_m':4.,'pos_std_n_m':5.,
       'vel_std_e_mps':.7,'vel_std_n_mps':.8,'heading_rad':100.}
    z=observation(r,np.array([10.,20.]),1)
    np.testing.assert_array_equal(z.velocity,[2,3])
    np.testing.assert_allclose(np.diag(z.covariance),[16,25,.49,.64])


def test_predict_consumes_only_past_samples_and_exact_duration():
    class Recorder:
        def __init__(self):self.elapsed=0.;self.dv=np.zeros(2);self.da=0.
        def predict(self,a,g,dt):
            assert 0<dt<=.05000001
            self.elapsed+=dt;self.dv+=a*dt;self.da+=g*dt
    c=np.array([[0,1,2,3],[.03,5,6,7],[.09,1000,1000,1000],[1.,0,0,0]])
    r=Recorder();predict_to(r,c,0,.08)
    np.testing.assert_allclose(r.dv,np.array([1,2])*.03+np.array([5,6])*.05)
    assert abs(r.elapsed-.08)<1e-12 and abs(r.da-(3*.03+7*.05))<1e-12


def make_imu(n=10):
    a=np.zeros(n,dtype=[(k,float) for k in ['t_s','gx_radps','gy_radps','gz_radps','ax_mps2','ay_mps2','az_mps2']])
    a['t_s']=np.arange(n)*.01;a['az_mps2']=9.81
    return a


def test_stationary_gravity_not_fed_as_horizontal_acceleration():
    a=make_imu();c={'up_body':[0,0,1],'forward_body':[1,0,0],'gyro_bias_radps':[0,0,0],'gravity_norm':9.81}
    result=planar_controls(a,c)
    np.testing.assert_allclose(result[:,1:],0,atol=1e-12)


def test_control_adapter_has_no_future_imu_dependency():
    a=make_imu();b=a.copy();b['ax_mps2'][5:]=4.
    c={'up_body':[0,0,1],'forward_body':[1,0,0],'gyro_bias_radps':[0,0,0],'gravity_norm':9.81}
    np.testing.assert_array_equal(planar_controls(a,c)[:5],planar_controls(b,c)[:5])


def test_imu_gap_is_not_silently_filled():
    a=make_imu();a['t_s'][5:]+=2
    c={'up_body':[0,0,1],'forward_body':[1,0,0],'gyro_bias_radps':[0,0,0],'gravity_norm':9.81}
    with pytest.raises(ValueError):planar_controls(a,c)


def way(i,nodes,points,**tags):
    return {'type':'way','id':i,'nodes':nodes,'geometry':[{'lon':x,'lat':y} for x,y in points],'tags':{'highway':'residential',**tags}}


def test_osm_crossings_do_not_invent_connectivity():
    doc={'elements':[way(1,[1,2],[(-.001,0),(.001,0)]),way(2,[3,4],[(0,-.001),(0,.001)])]}
    store,_=compile_osm(doc,[0,0,0],[-.003,-.003,.003,.003],'test')
    assert len(store.edges)==4 and all(not e.successors for e in store.edges.values())


def test_osm_shared_nodes_and_oneway_are_preserved():
    doc={'elements':[way(1,[1,2],[(-.001,0),(0,0)],oneway='yes'),way(2,[2,3],[(0,0),(0,.001)],oneway='yes')]}
    store,_=compile_osm(doc,[0,0,0],[-.003,-.003,.003,.003],'test')
    assert len(store.edges)==2
    assert store.edges['osm:1:1:2:f'].successors==('osm:2:2:3:f',)


def test_node_turn_restriction_is_enforced():
    relation={'type':'relation','id':9,'tags':{'restriction':'no_right_turn'},'members':[
        {'type':'way','role':'from','ref':1},{'type':'way','role':'to','ref':2},{'type':'node','role':'via','ref':2}]}
    doc={'elements':[way(1,[1,2],[(-.001,0),(0,0)],oneway='yes'),way(2,[2,3],[(0,0),(0,.001)],oneway='yes'),relation]}
    store,meta=compile_osm(doc,[0,0,0],[-.003,-.003,.003,.003],'test')
    assert not store.edges['osm:1:1:2:f'].successors and meta['simple_restrictions']==1
