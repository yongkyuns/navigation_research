"""Audit the frozen fault-mode policy on every unmodified accepted recording.

This compares map-free native variants only, to isolate receiver error modelling.
References are loaded after runtime replay; no labels or map are detector inputs.
"""
from pathlib import Path
from dataclasses import asdict
import argparse, csv, gzip, json, time
import numpy as np
from native import Library, Frame
from native_bias import BiasLibrary, BiasConfig
from datasets import load_inputs, load_reference, events
from evaluate_baseline import stats


def run(library, directory, output, config=BiasConfig()):
    imu, gnss, provenance = load_inputs(directory)
    raw, candidate = Library(library).create(), BiasLibrary(library, config).create()
    rows=[]; next_output=float(imu[0,0]); start=time.perf_counter(); equal=True; finite=True; minimum=0.
    frame=Frame(gnss[0,1:4])
    try:
        for kind,row in events(imu,gnss):
            getattr(raw,kind)(row); getattr(candidate,kind)(row)
            if kind!='imu' or row[0]+1e-9<next_output: continue
            next_output=row[0]+.5
            a,b=raw.snapshot(),candidate.snapshot()
            before_active=np.array_equal(a.raw,b.raw[:358])
            if not b.bias_active: equal &= before_active
            item=dict(t_s=float(row[0]),raw_ready=a.ready,bias_ready=b.ready,bias_usable=b.usable,
                      active=b.bias_active,activation_t=b.activation_t,physical_equal=before_active)
            for prefix,s in [('raw',a),('bias',b)]:
                if s.ready:
                    p,P,v=s.pose(frame)
                    item.update({prefix+'_e':float(p[0]),prefix+'_n':float(p[1])})
            if b.ready:
                finite &= bool(np.isfinite(b.raw).all())
                ev=float(np.linalg.eigvalsh((b.augmented_covariance+b.augmented_covariance.T)/2)[0]); minimum=min(ev,minimum)
                if not finite or ev < -1e-5*max(1.,np.diag(b.augmented_covariance).max()):
                    raise ValueError('Invalid augmented navigation state')
            rows.append(item)
        instance_bytes=candidate.snapshot().bytes
    finally:
        raw.close();candidate.close()
    elapsed=time.perf_counter()-start
    reference=load_reference(directory)
    raw_errors=[];bias_errors=[]
    if reference is not None:
        positions=frame.project(reference[:,1:4])
        for r in rows:
            if not (r['raw_ready'] and r['bias_ready']): continue
            j=np.searchsorted(reference[:,0],r['t_s'])
            if j==0 or j==len(reference) or reference[j,0]-reference[j-1,0]>.2: continue
            f=(r['t_s']-reference[j-1,0])/(reference[j,0]-reference[j-1,0]);target=(1-f)*positions[j-1,:2]+f*positions[j,:2]
            for prefix,errors in [('raw',raw_errors),('bias',bias_errors)]:
                error=float(np.linalg.norm(np.array([r[prefix+'_e'],r[prefix+'_n']])-target));errors.append(error);r[prefix+'_reference_discrepancy_m']=error
    result=dict(dataset=directory.name,input=provenance,config=asdict(config),epochs=len(rows),
                activation_t=next((r['activation_t'] for r in rows if r['active']),None),
                equal_until_activation=bool(equal),finite=bool(finite),min_augmented_eigenvalue=minimum,
                raw_ready_epochs=sum(r['raw_ready'] for r in rows),bias_ready_epochs=sum(r['bias_ready'] for r in rows),
                usable_epochs=sum(r['bias_usable'] for r in rows),active_epochs=sum(r['active'] for r in rows),
                raw_reference_discrepancy=stats(raw_errors),bias_reference_discrepancy=stats(bias_errors),
                native_instance_bytes=instance_bytes,wall_runtime_s=elapsed,
                reference_independence='unverified; no surveyed accuracy or detector false-alarm labels')
    output=Path(output);output.mkdir(parents=True,exist_ok=True)
    fields=list(dict.fromkeys(k for r in rows for k in r))
    with gzip.open(output/(directory.name+'.csv.gz'),'wt',newline='') as stream:
        writer=csv.DictWriter(stream,fieldnames=fields);writer.writeheader();writer.writerows(rows)
    (output/(directory.name+'.json')).write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    print(directory.name, 'activation',result['activation_t'], 'reference RMSE',
          None if not raw_errors else (result['raw_reference_discrepancy']['rmse_m'], result['bias_reference_discrepancy']['rmse_m']), flush=True)
    return result


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--library',type=Path,required=True);p.add_argument('--datasets',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    if a.output.exists():raise ValueError('Refuse to overwrite a previous evaluation')
    a.output.mkdir(parents=True);results=[];rejected=[]
    for manifest in sorted(a.datasets.glob('*/manifest.json')):
        directory=manifest.parent
        try: load_inputs(directory)
        except ValueError as exc:
            rejected.append(dict(dataset=directory.name,reason=str(exc)));continue
        results.append(run(a.library,directory,a.output))
    summary=dict(config=asdict(BiasConfig()),accepted=len(results),rejected=rejected,
                 recordings_with_activation=sum(r['activation_t'] is not None for r in results),
                 all_equal_until_activation=all(r['equal_until_activation'] for r in results),
                 results=results)
    (a.output/'summary.json').write_text(json.dumps(summary,indent=2,allow_nan=False)+'\n')

if __name__=='__main__':main()
