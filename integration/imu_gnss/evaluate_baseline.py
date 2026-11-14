"""Evaluate real upstream runtime on checked-in recordings; never use references as filter inputs."""
from pathlib import Path
import argparse,csv,json,time,platform,hashlib
import numpy as np
from native import Library,Frame
from datasets import load_inputs,load_reference,events

def stats(x):
    x=np.asarray(x);return {'n':len(x),'rmse_m':float(np.sqrt(np.mean(x*x))),'median_m':float(np.median(x)),'p95_m':float(np.percentile(x,95)),'max_m':float(x.max())} if len(x) else None

def run(library,directory,out):
    imu,gnss,provenance=load_inputs(directory);nav=library.create();frame=Frame(gnss[0,1:4]);rows=[];next_out=float(imu[0,0]);first=None;gnss_last=None;mask=0;start=time.perf_counter();n_numeric=0
    for kind,row in events(imu,gnss):
        if kind=='imu':nav.imu(row)
        else:nav.gnss(row);gnss_last=row
        if kind!='imu':continue
        if row[0]+1e-9<next_out:continue
        next_out=row[0]+.5
        s=nav.snapshot();r={'t_s':float(row[0]),'ready':s.ready,'usable':s.usable,'state':s.state,'reason':s.reason,'reanchors':s.reanchors,'native_instance_bytes':s.bytes}
        if s.ready:
            p,P,v=s.pose(frame);numeric=bool(np.isfinite(s.raw).all())
            if not numeric:n_numeric+=1
            if first is None:first=float(row[0])
            r.update(e=float(p[0]),n=float(p[1]),ve=float(v[0]),vn=float(v[1]),p_ee=float(P[0,0]),p_en=float(P[0,1]),p_nn=float(P[1,1]),numeric=numeric)
            j=np.searchsorted(gnss[:,0],row[0])
            if 0<j<len(gnss) and gnss[j,0]-gnss[j-1,0]<=2.:
                alpha=(row[0]-gnss[j-1,0])/(gnss[j,0]-gnss[j-1,0])
                g=(1-alpha)*frame.project(gnss[j-1,1:4])+alpha*frame.project(gnss[j,1:4])
                r['gnss_agreement_m']=float(np.linalg.norm(p-g[:2]))
        rows.append(r)
    elapsed=time.perf_counter()-start;nav.close()
    ref=load_reference(directory);ref_metrics=None;ref_count=0;repeated=None
    if ref is not None:
        pos=frame.project(ref[:,1:4]);errors=[];coverage=[]
        for r in rows:
            if not r.get('ready'):continue
            j=np.searchsorted(ref[:,0],r['t_s'])
            if j==0 or j==len(ref) or ref[j,0]-ref[j-1,0]>.2:continue
            f=(r['t_s']-ref[j-1,0])/(ref[j,0]-ref[j-1,0]);truth=(1-f)*pos[j-1,:2]+f*pos[j,:2]
            d=np.array([r['e'],r['n']])-truth;err=float(np.linalg.norm(d));r['reference_discrepancy_m']=err;errors.append(err)
            P=np.array([[r['p_ee'],r['p_en']],[r['p_en'],r['p_nn']]])
            try:r['reference_nees_2d']=float(d@np.linalg.solve(P,d));coverage.append(r['reference_nees_2d']<=5.991464547)
            except np.linalg.LinAlgError:pass
        ref_metrics=stats(errors);ref_count=len(ref)
        repeated=float(np.mean(np.all(np.diff(ref[:,1:4],axis=0)==0,axis=1)))
    metric={**provenance,'duration_s':float(max(imu[-1,0],gnss[-1,0])-min(imu[0,0],gnss[0,0])),'wall_runtime_s':elapsed,'initialization_t_s':first,'sampled_epochs':len(rows),'ready_fraction':sum(r['ready'] for r in rows)/len(rows),'usable_fraction':sum(r['usable'] for r in rows)/len(rows),'nonfinite_outputs':n_numeric,'reference_rows':ref_count,'reference_discrepancy':ref_metrics,'reference_repeated_position_fraction':repeated,'reference_independence':'unverified; not ground-truth accuracy','gnss_agreement_not_accuracy':stats([r['gnss_agreement_m'] for r in rows if 'gnss_agreement_m' in r]),'reanchors':max(r['reanchors'] for r in rows),'native_instance_bytes':rows[-1]['native_instance_bytes']}
    out=Path(out);out.mkdir(parents=True,exist_ok=True)
    fields=list(dict.fromkeys(k for r in rows for k in r))
    with (out/(directory.name+'.csv')).open('w',newline='') as f:w=csv.DictWriter(f,fieldnames=fields);w.writeheader();w.writerows(rows)
    (out/(directory.name+'.json')).write_text(json.dumps(metric,indent=2,allow_nan=False)+'\n')
    return metric

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--library',type=Path,required=True);p.add_argument('--datasets',type=Path,required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--ids',nargs='*');a=p.parse_args()
    a.output.mkdir(parents=True,exist_ok=True);lib=Library(a.library);metrics=[]
    selected=sorted(a.ids or [d.name for d in a.datasets.iterdir() if d.is_dir() and (d/'manifest.json').exists()])
    for name in selected:
        try:
            load_inputs(a.datasets/name)
        except ValueError as exc:
            result={'id':name,'status':'INPUT_REJECTED','reason':str(exc)}
            (a.output/(name+'.json')).write_text(json.dumps(result,indent=2)+'\n')
            metrics.append(result);print(json.dumps(result),flush=True);continue
        result=run(lib,a.datasets/name,a.output);result['status']='REPLAYED';metrics.append(result);print(name,json.dumps({k:result[k] for k in ['duration_s','initialization_t_s','ready_fraction','reference_discrepancy','wall_runtime_s']}),flush=True)
    (a.output/'summary.json').write_text(json.dumps(metrics,indent=2,allow_nan=False)+'\n')
