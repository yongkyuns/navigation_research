"""Paired full-runtime map-free / road-conditioned replay on independent OSM.

The reference is evaluation-only and its independence remains unverified.
The published shadow output stays identical to a separate map-free replay.
"""
from pathlib import Path
import argparse,csv,json,time
import numpy as np
from native import Library
from datasets import load_inputs,load_reference,events
from osm_graph import compile_osm
from road_bank import RoadBank
from evaluate_baseline import stats

def stress_gnss(gnss, mode):
    """Deterministic input stress, not observed receiver errors or fresh truth."""
    z=gnss.copy()
    if mode=='none':return z
    t=z[:,0];active=(t>=90.)&(t<=120.)
    if mode=='outage30':return z[~active]
    if mode=='north_bias20':
        amplitude=20.*np.clip(np.minimum((t-90.)/5.,(120.-t)/5.),0.,1.)
        lat=np.deg2rad(z[:,1]);e2=6.6943799901413165e-3
        meridian=6378137.*(1-e2)/(1-e2*np.sin(lat)**2)**1.5
        z[:,1]+=np.rad2deg(amplitude/(meridian+z[:,3]))
        return z
    raise ValueError('Unknown stress mode')

def run(library,directory,roads,out,max_branches=24,stress='none'):
    imu,gnss,provenance=load_inputs(directory);gnss=stress_gnss(gnss,stress);bank=RoadBank(library.create(),roads,max_branches);control=library.create();rows=[];next_out=float(imu[0,0]);start=time.perf_counter();exact=True;peak=0;negative=0
    for kind,row in events(imu,gnss):
        if kind=='imu':bank.imu(row);control.imu(row)
        else:bank.gnss(row);control.gnss(row)
        if kind!='imu' or row[0]+1e-9<next_out:continue
        next_out=row[0]+1.
        t0=time.perf_counter();res=bank.step(row[0]);elapsed=(time.perf_counter()-t0)*1000;s=res['base'];q=control.snapshot();same=np.array_equal(s.raw,q.raw);exact&=same
        r={'t_s':float(row[0]),'ready':s.ready,'usable':s.usable,'nav_state':s.state,'status':res['status'],'latched':res['latched'],'road_group':res['group'],'edge':res['edge'],'top_weight':res['top_weight'],'pruned_mass':res['pruned_mass'],'branches':res['branches'],'map_step_ms':elapsed,'base_equal_independent':same, 'junction_phases':json.dumps(res.get('phases',{}),sort_keys=True), 'maneuver_modes':res.get('maneuver_modes',0), 'recovery_used':res.get('recovery_used',False), 'recovery_status':res.get('recovery_status','empty')}
        peak=max(peak,res['branches'])
        if s.ready:
            p,P,v=s.pose(roads.frame);pp=res.get('conditional_position',p);pc=res.get('conditional_covariance',P)
            r.update(base_e=float(p[0]),base_n=float(p[1]),candidate_e=float(pp[0]),candidate_n=float(pp[1]),correction_m=float(np.linalg.norm(pp-p)),p_ee=float(pc[0,0]),p_en=float(pc[0,1]),p_nn=float(pc[1,1]))
            if np.linalg.eigvalsh(pc)[0]<-1e-6:negative+=1
        rows.append(r)
    runtime=time.perf_counter()-start;instance_bytes=bank.base.snapshot().bytes;bank.reset();bank.base.close();control.close()
    ref=load_reference(directory);base_err=[];map_err=[]
    if ref is not None:
        rp=roads.frame.project(ref[:,1:4]);
        for r in rows:
            if not r['ready']:continue
            j=np.searchsorted(ref[:,0],r['t_s'])
            if j==0 or j==len(ref) or ref[j,0]-ref[j-1,0]>.2:continue
            f=(r['t_s']-ref[j-1,0])/(ref[j,0]-ref[j-1,0]);target=(1-f)*rp[j-1,:2]+f*rp[j,:2]
            be=float(np.linalg.norm(np.array([r['base_e'],r['base_n']])-target));me=float(np.linalg.norm(np.array([r['candidate_e'],r['candidate_n']])-target))
            r.update(base_reference_discrepancy_m=be,candidate_reference_discrepancy_m=me);base_err.append(be);map_err.append(me)
    available=[r for r in rows if r['ready'] and r['usable'] and r['status']!='MAP_UNAVAILABLE'];latched=[r for r in available if r['latched']]
    corrections=[r['correction_m'] for r in rows if r['ready']]
    metric={**provenance,'input_stress':stress,'source_runtime':'actual pinned Rust SensorFusion','mode':'shadow publication; experimental road-conditioned estimate evaluated separately','map_is_independent_of_reference':True,'map_sha256':roads.metadata['response_sha256'],'reference_independence':'unverified; discrepancies, NOT ground-truth accuracy','epochs':len(rows),'ready_epochs':sum(r['ready'] for r in rows),'map_available_epochs':len(available),'latched_epochs':len(latched),'latch_fraction_available':len(latched)/max(1,len(available)),'map_free_exactly_matches_independent_replay':exact,'base_reference_discrepancy':stats(base_err),'stress_window_base_discrepancy':stats([r['base_reference_discrepancy_m'] for r in rows if 90<=r['t_s']<=120 and 'base_reference_discrepancy_m' in r]),'stress_window_candidate_discrepancy':stats([r['candidate_reference_discrepancy_m'] for r in rows if 90<=r['t_s']<=120 and 'candidate_reference_discrepancy_m' in r]),'candidate_reference_discrepancy':stats(map_err),'correction_m':stats(corrections),'max_branches':peak,'configured_cap':max_branches,'max_pruned_mass':bank.max_pruned,'map_update_count':bank.map_updates,'recovery_count':bank.recoveries,'recovery_window_s':bank.config.recovery_s,'recovery_event_cap':bank.config.recovery_events,'status_counts':{status:sum(r['status']==status for r in rows) for status in sorted({r['status'] for r in rows})},'max_native_instances_bound_excludes_evaluation_control':2*max_branches+4,'native_peak_storage_bound_excludes_python_maps_and_control':instance_bytes*(2*max_branches+4),'negative_position_covariance_epochs':negative,'wall_runtime_s':runtime,'native_instance_bytes':instance_bytes,'native_branch_storage_bytes_excludes_python_and_maps':instance_bytes*(peak+1),'map_step_ms':{'p50':float(np.percentile([r['map_step_ms'] for r in rows],50)),'p95':float(np.percentile([r['map_step_ms'] for r in rows],95)),'max':float(max(r['map_step_ms'] for r in rows))}}
    out=Path(out);out.mkdir(parents=True,exist_ok=True);fields=list(dict.fromkeys(k for r in rows for k in r))
    with (out/(directory.name+'.csv')).open('w',newline='') as f:w=csv.DictWriter(f,fieldnames=fields);w.writeheader();w.writerows(rows)
    (out/(directory.name+'.json')).write_text(json.dumps(metric,indent=2,allow_nan=False)+'\n');return metric

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--library',type=Path,required=True);p.add_argument('--datasets',type=Path,required=True);p.add_argument('--map',type=Path,required=True);p.add_argument('--map-provenance',type=Path,required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--ids',nargs='+',required=True);p.add_argument('--stress',choices=['none','outage30','north_bias20'],default='none');a=p.parse_args();lib=Library(a.library);roads=compile_osm(a.map,a.map_provenance);a.output.mkdir(parents=True,exist_ok=True)
    roads.store.save(a.output/'tiles');(a.output/'map-provenance.json').write_text(json.dumps(roads.metadata,indent=2)+'\n');metrics=[]
    for name in a.ids:
        m=run(lib,a.datasets/name,roads,a.output,stress=a.stress);metrics.append(m);print(name,json.dumps({k:m[k] for k in ['latch_fraction_available','base_reference_discrepancy','candidate_reference_discrepancy','wall_runtime_s']}),flush=True)
    (a.output/'summary.json').write_text(json.dumps(metrics,indent=2,allow_nan=False)+'\n')