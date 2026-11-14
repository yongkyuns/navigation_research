"""Execute recorded-data replay, without treating receiver/reference output as truth."""
from __future__ import annotations
import argparse, collections, csv, hashlib, json, math, platform, sys, time, traceback, urllib.request
from pathlib import Path
import numpy as np
from routefusion import RouteTracker,TileStore
from routefusion.tracker import TrackerConfig
from adapter import SOURCE_SHA, load_stream, local_en, calibrate, planar_controls, predict_to, initialize, observation
from roads import fetch_map,compile_osm,ATTRIBUTION

DATASETS=['urban-short-turn-loop-nominal-001','ios-mixed-drive-reverse-parking-001',
          'ios-drive-20260521-130835','ios-drive-20260521-131420']

def dump(path,data):
    path.write_text(json.dumps(data,indent=2,allow_nan=False)+'\n')

def fetch_inputs(root):
    base=f'https://raw.githubusercontent.com/yongkyuns/imu_gnss_fusion/{SOURCE_SHA}/web/datasets/'
    for name in DATASETS:
        folder=root/name;folder.mkdir(parents=True,exist_ok=True)
        with urllib.request.urlopen(base+name+'/manifest.json',timeout=90) as r:raw=r.read()
        meta=json.loads(raw);(folder/'manifest.json').write_bytes(raw)
        for kind in ['imu','gnss','reference_position']:
            if kind not in meta['files']:continue
            item=meta['files'][kind];path=folder/item['path']
            if path.exists() and hashlib.sha256(path.read_bytes()).hexdigest()==item['sha256']:continue
            with urllib.request.urlopen(base+name+'/'+item['path'],timeout=120) as r:data=r.read()
            if hashlib.sha256(data).hexdigest()!=item['sha256']:raise ValueError('Download checksum mismatch')
            path.write_bytes(data)

def rms(values):
    return float(np.sqrt(np.mean(np.square(values)))) if len(values) else None

def percentiles(values):
    return {key:float(np.quantile(values,q)) for key,q in [('median',.5),('p95',.95),('max',1.)]} if len(values) else None

def replay(imu,gnss,controls,calibration,store,origin,mode):
    xy=local_en(gnss['lon_deg'],gnss['lat_deg'],gnss['height_m'],origin)
    speed=np.hypot(gnss['ve_mps'],gnss['vn_mps'])
    candidates=np.flatnonzero((gnss['t_s']>calibration['cutoff_s'])&(speed>=2)&(gnss['t_s']<=imu['t_s'][-1]))
    if not len(candidates):raise ValueError('No moving GNSS seed after calibration')
    seed=int(candidates[0]);start=float(gnss['t_s'][seed]);previous=start
    nav=initialize(gnss[seed],xy[seed],calibration,seed)
    tracker=RouteTracker(nav,store,TrackerConfig(boundary_sigma_m=1.5))
    coverage=store.loaded.copy();rows=[];min_eigen=math.inf;started=time.perf_counter()
    hold_start,hold_end=start+30,start+60
    for k in range(seed+1,len(gnss)):
        row=gnss[k];t=float(row['t_s'])
        if t>imu['t_s'][-1]:break
        predict_to(tracker,controls,previous,t)
        withheld=(mode=='withheld_gnss_30s' and hold_start<=t<hold_end)
        absent=(mode=='missing_map_30s' and hold_start<=t<hold_end)
        store.loaded=set() if absent else coverage.copy()
        result=tracker.step(t,None if withheld else observation(row,xy[k],k))
        for state in [tracker.base]+[b.nav for b in tracker.branches]:
            if not np.isfinite(state.x).all() or not np.isfinite(state.P).all():raise ValueError('Nonfinite filter state')
            eigen=float(np.linalg.eigvalsh(state.P).min());min_eigen=min(min_eigen,eigen)
            if eigen < -1e-7:raise ValueError('Non-PSD covariance')
        if absent:
            if result.status!='MAP_UNAVAILABLE':raise AssertionError('Missing map did not abstain')
            np.testing.assert_array_equal(result.position,result.fallback_position)
            np.testing.assert_array_equal(result.covariance,result.fallback_covariance)
        correction=result.position-result.fallback_position
        rows.append({'t_s':t,'raw_e':float(xy[k,0]),'raw_n':float(xy[k,1]),
            'baseline_e':float(result.fallback_position[0]),'baseline_n':float(result.fallback_position[1]),
            'output_e':float(result.position[0]),'output_n':float(result.position[1]),
            'speed_mps':float(speed[k]),'status':result.status,'edge':result.route_edge or '',
            'top_conditional_weight':result.top_weight,'branches':result.branch_count,
            'pruned_mass':result.pruned_mass,'map_correction_m':float(np.linalg.norm(correction)),
            'raw_baseline_distance_m':float(np.linalg.norm(result.fallback_position-xy[k])),
            'raw_output_distance_m':float(np.linalg.norm(result.position-xy[k])),
            'withheld_gnss':withheld,'missing_map_injected':absent,
            'position_cov_ee':float(result.covariance[0,0]),'position_cov_en':float(result.covariance[0,1]),
            'position_cov_nn':float(result.covariance[1,1])})
        previous=t
    store.loaded=coverage
    if not rows:raise ValueError('No replay outputs')
    moving=[r for r in rows if r['speed_mps']>=2]
    kept=[r for r in rows if r['status']=='LATCHED'];hidden=[r for r in rows if r['withheld_gnss']]
    delta=np.array([[r['output_e']-r['baseline_e'],r['output_n']-r['baseline_n']] for r in rows])
    metrics={'mode':mode,'seed_time_s':start,'end_time_s':rows[-1]['t_s'],
        'evaluated_duration_s':rows[-1]['t_s']-start,'output_epochs':len(rows),
        'input_imu_rows_total':len(imu),'input_gnss_rows_total':len(gnss),
        'evaluated_imu_rows':int(np.sum((imu['t_s']>=start)&(imu['t_s']<=rows[-1]['t_s']))),
        'status_counts':dict(collections.Counter(r['status'] for r in rows)),
        'latched_fraction':len(kept)/len(rows),
        'latched_moving_fraction':sum(r['status']=='LATCHED' for r in moving)/len(moving) if moving else None,
        'map_correction_m':percentiles([r['map_correction_m'] for r in rows]),
        'max_map_correction_change_m':float(np.linalg.norm(np.diff(delta,axis=0),axis=1).max()) if len(delta)>1 else 0.,
        'raw_receiver_agreement_baseline_rms_m':rms([r['raw_baseline_distance_m'] for r in rows]),
        'raw_receiver_agreement_output_rms_m':rms([r['raw_output_distance_m'] for r in rows]),
        'withheld_fix_comparison_baseline_rms_m':rms([r['raw_baseline_distance_m'] for r in hidden]),
        'withheld_fix_comparison_output_rms_m':rms([r['raw_output_distance_m'] for r in hidden]),
        'withheld_epochs':len(hidden),'missing_map_epochs':sum(r['missing_map_injected'] for r in rows),
        'max_branches':max(r['branches'] for r in rows),'max_pruned_mass':max(r['pruned_mass'] for r in rows),
        'min_state_covariance_eigenvalue':min_eigen,'runtime_s':time.perf_counter()-started,
        'absolute_accuracy_measured':False,'wrong_road_rate_measured':False,
        'receiver_agreement_is_not_accuracy':True}
    return rows,metrics

def reference_comparison(folder,rows,origin):
    meta=json.loads((folder/'manifest.json').read_text())
    if 'reference_position' not in meta['files']:return None
    ref=load_stream(folder,'reference_position')
    xy=local_en(ref['lon_deg'],ref['lat_deg'],ref['height_m'],origin)
    values={'raw':[],'baseline':[],'output':[]}
    for r in rows:
        t=r['t_s'];j=int(np.searchsorted(ref['t_s'],t))
        if not 0<j<len(ref) or ref['t_s'][j]-ref['t_s'][j-1]>.2:continue
        target=np.array([np.interp(t,ref['t_s'],xy[:,k]) for k in range(2)])
        for key in values:
            values[key].append(float(np.linalg.norm(np.array([r[key+'_e'],r[key+'_n']])-target)))
    return {'meaning':'Agreement with supplied fused reference, NOT independently surveyed truth',
            'evaluated_epochs':len(values['raw']),
            'rms_m':{k:rms(v) for k,v in values.items()}}

def plot_trace(rows,store,out,title):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.collections import LineCollection
    fig,ax=plt.subplots(figsize=(9,7))
    if store.edges:
        ax.add_collection(LineCollection([e.points for e in store.edges.values()],linewidths=.5,alpha=.2))
    for key,label in [('raw','Receiver PVT'),('baseline','Map-free planar filter'),('output','RouteFusion output / fallback')]:
        ax.plot([r[key+'_e'] for r in rows],[r[key+'_n'] for r in rows],label=label,linewidth=1)
    xx=[r['raw_e'] for r in rows]; yy=[r['raw_n'] for r in rows]
    ax.set_xlim(min(xx)-100,max(xx)+100);ax.set_ylim(min(yy)-100,max(yy)+100)
    ax.set_aspect('equal',adjustable='box');ax.set_xlabel('Local east (m)');ax.set_ylabel('Local north (m)')
    ax.set_title(title+'\nRecorded inputs; no independent ground truth')
    ax.legend();fig.text(.01,.01,ATTRIBUTION,fontsize=7);fig.tight_layout(rect=[0,.03,1,1])
    fig.savefig(out,dpi=150);plt.close(fig)

def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--input',type=Path,default=Path('runs/recorded-inputs'))
    ap.add_argument('--output',type=Path,default=Path('experiments/recorded/results'))
    ap.add_argument('--fetch',action='store_true')
    args=ap.parse_args();args.output.mkdir(parents=True,exist_ok=True)
    if args.fetch:fetch_inputs(args.input)
    report={'source_repo':'yongkyuns/imu_gnss_fusion','source_sha':SOURCE_SHA,
            'routefusion_base_commit':'52b4a83a37493d9b1486c519b693fded9e94131b',
            'independent_ground_truth':False,'dataset_count':len(DATASETS),'datasets':[],
            'environment':{'python':sys.version,'numpy':np.__version__,'platform':platform.platform()},
            'experiment_source_hashes':{p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in Path(__file__).parent.glob('*.py')},
            'constraints':['Not a full 3D INS integration','No reference stream enters calibration, propagation or road compilation',
                           'First min(90 s,25% of recording) is calibration only',
                           'Receiver position/velocity cross-correlation is unavailable; diagonal covariance plus floors is approximate',
                           'Phone location is not established to be raw independent GNSS',
                           'Road widths and map alignment are not surveyed','No ground-truth road labels',
                           'GNSS outage and cache loss are injected tests, not claimed recording conditions']}
    failures=0
    for name in DATASETS:
        entry={'dataset':name,'status':'not_started'};report['datasets'].append(entry)
        folder=args.input/name;dest=args.output/name;dest.mkdir(exist_ok=True)
        try:
            imu=load_stream(folder,'imu');gnss=load_stream(folder,'gnss')
            meta=json.loads((folder/'manifest.json').read_text())
            entry['input_manifest']=meta
            start=max(float(imu['t_s'][0]),float(gnss['t_s'][0]))
            end=min(float(imu['t_s'][-1]),float(gnss['t_s'][-1]))
            cutoff=start+min(90.,.25*(end-start))
            calibration=calibrate(imu,gnss,cutoff);entry['calibration']=calibration
            controls=planar_controls(imu,calibration)
            origin=[float(gnss['lon_deg'][0]),float(gnss['lat_deg'][0]),float(gnss['height_m'][0])]
            entry['origin_wgs84']=origin
            try:
                doc,provenance=fetch_map(gnss,dest/'map')
                store,mapinfo=compile_osm(doc,origin,provenance['bounds_s_w_n_e'],provenance['response_sha256'][:16])
                entry['map']={'download':'success',**mapinfo,**provenance}
                store.save(dest/'tiles')
            except Exception as exc:
                entry['map']={'download':'failed','error':str(exc)}
                store=TileStore();failures+=1
            modes=['original']
            if name in ('urban-short-turn-loop-nominal-001','ios-drive-20260521-130835'):
                modes+=['withheld_gnss_30s','missing_map_30s']
            entry['runs']=[]
            for mode in modes:
                rows,metrics=replay(imu,gnss,controls,calibration,store,origin,mode)
                metrics['supplied_fused_reference_comparison']=reference_comparison(folder,rows,origin)
                entry['runs'].append(metrics)
                with (dest/(mode+'.csv')).open('w',newline='') as f:
                    writer=csv.DictWriter(f,fieldnames=list(rows[0]));writer.writeheader();writer.writerows(rows)
                plot_trace(rows,store,dest/(mode+'.png'),name+' / '+mode)
                print(name,mode,json.dumps(metrics),flush=True)
                dump(args.output/'summary.json',report)
            entry['status']='completed' if entry['map']['download']=='success' else 'completed_without_map'
        except Exception as exc:
            entry['status']='failed';entry['error']=str(exc);entry['traceback']=traceback.format_exc();failures+=1
            print(entry['traceback'],flush=True)
        dump(args.output/'summary.json',report)
    report['execution_failures']=failures
    dump(args.output/'summary.json',report)
    lines=['# Recorded-data experiment results','',
           'These are executed replay diagnostics, **not absolute-accuracy or correct-road measurements**. See ../README.md.',
           '', '| Dataset | Mode | Epochs | Latched | P95 / max map correction (m) | Receiver agreement baseline / output RMS (m) |',
           '|---|---|---:|---:|---:|---:|']
    for d in report['datasets']:
        for r in d.get('runs',[]):
            c=r['map_correction_m']
            lines.append(f'| {d["dataset"]} | {r["mode"]} | {r["output_epochs"]} | {100*r["latched_fraction"]:.1f}% | {c["p95"]:.2f} / {c["max"]:.2f} | {r["raw_receiver_agreement_baseline_rms_m"]:.2f} / {r["raw_receiver_agreement_output_rms_m"]:.2f} |')
        if d['status']!='completed':lines.append('\n'+d['dataset']+': '+d['status']+' '+d.get('error',''))
    lines+=['','A higher latch rate or smaller distance to a map is not proof of better positioning. Reference-track comparison is agreement with another fused estimate, not surveyed truth.',
            '', 'Maps: '+ATTRIBUTION, '', 'Input source commit: `'+SOURCE_SHA+'`. Full diagnostics and checksums: `summary.json`.']
    (args.output/'REPORT.md').write_text('\n'.join(lines)+'\n')
    if failures:raise SystemExit(f'{failures} dataset/map failures; preserved in summary.json')

if __name__=='__main__':main()
