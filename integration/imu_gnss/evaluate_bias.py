"""Three-way recorded replay with frozen parameters and labelled injected faults.

Reference streams are loaded only after the entire estimator replay. No road
centerline, supplied reference, or known injected offset is used by the detector.
"""
from dataclasses import asdict
from pathlib import Path
import argparse,csv,gzip,json,hashlib,time
import numpy as np
from native import Library,Frame,ned_rotation,ecef
from native_bias import BiasLibrary,BiasConfig
from datasets import load_inputs,load_reference,events
from osm_graph import compile_osm
from road_bank import RoadBank
from evaluate_baseline import stats


def corrupt(gnss,case):
    out=gnss.copy();start=case.get('start',90.);end=case.get('end',120.)
    t=out[:,0];inside=(t>=start)&(t<=end);kind=case.get('fault','none')
    if kind=='none':return out
    if kind=='outage':return out[~inside]
    if kind not in ('position','position_velocity','cold_start'):raise ValueError('Unknown fault')
    amplitude=case.get('metres',20.)
    ramp=case.get('ramp_s',5.)
    if kind=='cold_start':factor=np.ones(len(t));derivative=np.zeros(len(t))
    else:
        factor=np.clip(np.minimum((t-start)/ramp,(end-t)/ramp),0,1)
        derivative=np.where((t>start)&(t<start+ramp),1/ramp,np.where((t>end-ramp)&(t<end),-1/ramp,0.))
    angle=np.deg2rad(case.get('bearing_deg',0.));north=amplitude*np.cos(angle);east=amplitude*np.sin(angle)
    lat=np.deg2rad(out[:,1]);e2=6.6943799901413165e-3;a=6378137.
    rn=a/np.sqrt(1-e2*np.sin(lat)**2);rm=a*(1-e2)/(1-e2*np.sin(lat)**2)**1.5
    out[:,1]+=np.rad2deg(north*factor/(rm+out[:,3]));out[:,2]+=np.rad2deg(east*factor/((rn+out[:,3])*np.cos(lat)))
    if kind=='position_velocity':out[:,4]+=north*derivative;out[:,5]+=east*derivative
    return out


def run_case(library_path,directory,roads,outdir,case,config=BiasConfig()):
    imu,gnss,provenance=load_inputs(directory);gnss=corrupt(gnss,case)
    raw=Library(library_path).create();biaslib=BiasLibrary(library_path,config)
    bias=biaslib.create();bank=RoadBank(biaslib.create(),roads)
    rows=[];next_out=float(imu[0,0]);exact=True;min_eigen=0.;start=time.perf_counter()
    try:
        for kind,row in events(imu,gnss):
            getattr(raw,kind)(row);getattr(bias,kind)(row);getattr(bank,kind)(row)
            if kind!='imu' or row[0]+1e-9<next_out:continue
            next_out=row[0]+1.
            t0=time.perf_counter();res=bank.step(row[0],available=not case.get('missing_map',False));map_ms=(time.perf_counter()-t0)*1000
            a,b=raw.snapshot(),bias.snapshot();s=res['base'];equal=np.array_equal(s.raw,b.raw);exact&=equal
            record=dict(t_s=float(row[0]),raw_ready=a.ready,bias_ready=b.ready,bias_usable=b.usable,bias_active=b.bias_active,activation_t=b.activation_t,
                status=res['status'],latched=res['latched'],top_weight=res['top_weight'],branches=res['branches'],pruned_mass=res['pruned_mass'],
                fallback_exact=equal,road_edge=res['edge'],map_step_ms=map_ms,recoveries=res.get('recoveries',0))
            peak_states=[b]+[branch.nav.snapshot() for branch in bank.branches]
            record['active_road_branches']=sum(snap.bias_active for snap in peak_states[1:])
            for prefix,snap in [('raw',a),('bias',b)]:
                if snap.ready:
                    p,P,v=snap.pose(roads.frame)
                    record.update({prefix+'_e':float(p[0]),prefix+'_n':float(p[1]),prefix+'_p_ee':float(P[0,0]),prefix+'_p_en':float(P[0,1]),prefix+'_p_nn':float(P[1,1])})
            if b.ready:
                p,P,_=b.pose(roads.frame);candidate=res.get('conditional_position',p);cp=res.get('conditional_covariance',P)
                record.update(after_e=float(candidate[0]),after_n=float(candidate[1]),after_p_ee=float(cp[0,0]),after_p_en=float(cp[0,1]),after_p_nn=float(cp[1,1]),
                              estimated_bias_n=float(b.gnss_bias[0]),estimated_bias_e=float(b.gnss_bias[1]),estimated_bias_d=float(b.gnss_bias[2]))
                for snap in peak_states:
                    eigen=float(np.linalg.eigvalsh((snap.augmented_covariance+snap.augmented_covariance.T)/2)[0])
                    min_eigen=min(min_eigen,eigen)
                    if not np.isfinite(snap.raw).all() or eigen < -1e-5*max(1,np.diag(snap.augmented_covariance).max()):
                        raise ValueError('Invalid augmented navigation state or covariance')
            rows.append(record)
    finally:
        bank.reset();bank.base.close();bias.close();raw.close()
    elapsed=time.perf_counter()-start
    # Evaluation boundary: no reference sample exists in the runtime loop above.
    reference=load_reference(directory)
    if reference is not None:
        refpos=roads.frame.project(reference[:,1:4])
        for r in rows:
            j=np.searchsorted(reference[:,0],r['t_s'])
            if j==0 or j==len(reference) or reference[j,0]-reference[j-1,0]>.2:continue
            f=(r['t_s']-reference[j-1,0])/(reference[j,0]-reference[j-1,0]);target=(1-f)*refpos[j-1,:2]+f*refpos[j,:2]
            for prefix in ('raw','bias','after'):
                if prefix+'_e' in r:r[prefix+'_reference_discrepancy_m']=float(np.linalg.norm(np.array([r[prefix+'_e'],r[prefix+'_n']])-target))
    def summarize(selected):
        paired=[r for r in selected if all(p+'_reference_discrepancy_m' in r for p in ('raw','bias','after'))]
        return {p:stats([r[p+'_reference_discrepancy_m'] for r in paired]) for p in ('raw','bias','after')}
    begin,end=case.get('start',90.),case.get('end',120.)
    metric=dict(dataset=directory.name,case=case,config=asdict(config),input=provenance,epochs=len(rows),
                overall=summarize(rows),fault_window=summarize([r for r in rows if begin<=r['t_s']<=end]),
                recovery_window=summarize([r for r in rows if end<r['t_s']<=end+30]),
                initialized_epochs=sum(r['bias_ready'] for r in rows),raw_initialized_epochs=sum(r['raw_ready'] for r in rows),
                usable_epochs=sum(r['bias_usable'] for r in rows),active_epochs=sum(r['bias_active'] for r in rows),
                road_active_epochs=sum(r['active_road_branches']>0 for r in rows),
                activation_t=next((r['activation_t'] for r in rows if r['bias_active']),None),
                map_latch_fraction=sum(r['latched'] for r in rows)/max(1,sum(r['bias_usable'] and r['status']!='MAP_UNAVAILABLE' for r in rows)),
                map_free_matches_independent=exact,min_augmented_covariance_eigenvalue=min_eigen,wall_runtime_s=elapsed,
                map_step_p95_ms=float(np.percentile([r['map_step_ms'] for r in rows],95)),
                reference_independence='unverified; discrepancy, not established accuracy',
                injected_fault_is_synthetic=case.get('fault','none')!='none',map_sha256=roads.metadata['response_sha256'])
    target=Path(outdir);target.mkdir(parents=True,exist_ok=True);stem=case['name']
    fields=list(dict.fromkeys(key for row in rows for key in row))
    with gzip.open(target/(stem+'.csv.gz'),'wt',newline='') as stream:
        writer=csv.DictWriter(stream,fieldnames=fields);writer.writeheader();writer.writerows(rows)
    (target/(stem+'.json')).write_text(json.dumps(metric,indent=2,allow_nan=False)+'\n')
    print(stem,'active',metric['activation_t'],'RMSE', {p:(metric['fault_window'][p] or {}).get('rmse_m') for p in ('raw','bias','after')},flush=True)
    return metric

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--library',type=Path,required=True);p.add_argument('--datasets',type=Path,required=True);p.add_argument('--map-cache',type=Path,required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--cases',type=Path,required=True)
    a=p.parse_args()
    if a.output.exists():raise ValueError('Refuse to overwrite a previous evaluation')
    roads=compile_osm(a.map_cache/'osm-roads.json.gz',a.map_cache/'provenance.json');cases=json.loads(a.cases.read_text())
    result=[run_case(a.library,a.datasets/c['dataset'],roads,a.output,c) for c in cases]
    (a.output/'summary.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
