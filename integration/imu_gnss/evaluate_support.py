"""Matched native replay of bounded path support and soft corridor measurements.

Input is the retained before/after playback data, not surveyed truth. Accepted
road seeds retain their original causal times. Continuation is computed online
using only the independent raw EKF and known map geometry; no corrected state is
used as fresh road-selection evidence. Every original sensor event is processed.
"""
import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path
import time
import numpy as np
from native import Library, Frame
from datasets import load_inputs, events
from osm_graph import RoadMap
from routefusion.tiles import Edge, TileStore
from path_support import PathIndex, CausalSupport
from supported_map import SupportedMapEKF, MeasurementConfig


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def load_roads(doc):
    datasets=[d for d in doc['datasets'] if d['kind']=='map']
    if not datasets or any(d['origin']!=datasets[0]['origin'] for d in datasets):
        raise ValueError('Expected a shared local map frame')
    origin=datasets[0]['origin']
    store=TileStore(256,'frozen-playback-map',origin)
    for r in doc['roads']:
        store.add(Edge(r[0],r[3],tuple(r[4]),r[2],r[5],r[1]))
    store.loaded={tuple(k) for k in doc['loaded_tiles']}
    return datasets,RoadMap(store,Frame(origin),doc['provenance']['map_bbox'],
                           {'response_sha256':doc['provenance']['map_response_sha256']})


def run(playback, datasets, library, output, sigmas=(.25,), source_root=None):
    doc=json.loads(Path(playback).read_text())
    ds,roads=load_roads(doc)
    sigmas=sorted(float(s) for s in sigmas)
    if not sigmas or len(set(sigmas))!=len(sigmas):
        raise ValueError('Nonempty distinct sigma settings required')
    for sigma in sigmas: MeasurementConfig(sigma_m=sigma)
    index=PathIndex(roads)
    library=Path(library)
    lib=Library(library)
    proof=dict(schema=1,implementation='bounded causal path support and persistent map EKF',
               original_playback_sha256=digest(playback),library_sha256=digest(library),
               sigma_values_m=sigmas, fresh_native_sensor_replays=True, fresh_native_build=False,
               source_hashes={}, datasets=[], raw_ekf_substitutions=0, wrapper_resets=0,
               map_edges=len(roads.store.edges),canonical_paths=len(index.paths),
               limits=['Not a newly qualified HMM: original accepted seeds, online bounded continuation.',
                       'No surveyed truth; availability and jump magnitude are not accuracy.',
                       'One factor per geometric 10 m cell remains a correlation heuristic.',
                       'No increased update rate or exact persistent map-error state model.',
                       'No GNSS-bias modeling, default-output change or publication change.'])
    for name in ['path_support.py','supported_map.py','evaluate_support.py']:
        proof['source_hashes'][name]=digest(Path(__file__).with_name(name))
    if source_root is not None:
        expected=json.loads(Path(__file__).with_name('upstream-source-sha256.json').read_text())
        for name,value in expected.items():
            if digest(Path(source_root)/name)!=value:
                raise AssertionError('Native source mismatch: '+name)
        proof['pinned_native_source_hashes_verified']=len(expected)
    output=Path(output);output.mkdir(parents=True,exist_ok=False)
    for d in ds:
        wall=time.perf_counter()
        imu,gnss,provenance=load_inputs(Path(datasets)/d['id'])
        if provenance['input_sha256']!=d['input_sha256']:
            raise AssertionError('Input package changed')
        arrays=[hashlib.sha256(x.tobytes()).hexdigest() for x in (imu,gnss)]
        raw,control=lib.create(),lib.create()
        support=CausalSupport(index)
        arms={}
        variants={}
        for model in ['centerline','corridor']:
            for sigma in sigmas:
                key=f'{model}:{sigma:g}'
                cfg=MeasurementConfig(sigma_m=sigma,map_sigma_m=1.5 if model=='corridor' else 0.,
                                      corridor=model=='corridor',taper=model=='corridor')
                arms[key]=SupportedMapEKF(lib.create(),roads,cfg)
                variants[key]=dict(model=model,sigma_m=sigma,records=[],diagnostics=[])
        disabled=SupportedMapEKF(lib.create(),roads,enabled=False)
        handles={key:arm.nav.handle for key,arm in arms.items()}
        supports=[];epoch_index=0;minimum_eigenvalue=float('inf');skips=Counter()
        try:
            for kind,value in events(imu,gnss):
                getattr(raw,kind)(value);getattr(control,kind)(value)
                disabled.feed(kind,value)
                for arm in arms.values():arm.feed(kind,value)
                if kind!='imu' or epoch_index>=len(d['records']):continue
                r=d['records'][epoch_index];epoch=r[0]+d['original_start_t_s']
                if value[0]<epoch-1e-9:continue
                if abs(value[0]-epoch)>1e-8:raise AssertionError('Sample epoch changed')
                b=raw.snapshot()
                if not np.array_equal(b.raw,control.snapshot().raw):raise AssertionError('Raw control differs')
                if b.ready!=r[12] or b.usable!=r[13]:raise AssertionError('Initialization changed')
                bp=None
                if b.ready:
                    bp,_,_=b.pose(roads.frame)
                    if not np.array_equal(bp,np.array(r[1:3])):raise AssertionError('Retained baseline differs')
                path,note=support.select(epoch,b,r[7] if r[6] else None,r[5])
                note.update(elapsed_s=r[0],original_accepted=bool(r[6]))
                supports.append(note)
                dsnap,_=disabled.step(epoch,path,note)
                if not np.array_equal(b.raw,dsnap.raw):raise AssertionError('Map-disabled baseline differs')
                for key,arm in arms.items():
                    before=arm.nav.snapshot()
                    if arm.update_count==0 and not np.array_equal(before.raw,b.raw):
                        raise AssertionError('Changed state before first map factor')
                    s,n=arm.step(epoch,path,note)
                    if arm.nav.handle!=handles[key]:raise AssertionError('State instance replaced')
                    if not n['updated']:
                        if not np.array_equal(before.raw,s.raw):raise AssertionError('Skip mutated native state')
                        skips[key]+=1
                    p=P=None
                    if s.ready:
                        if not np.isfinite(s.raw).all():raise AssertionError('Nonfinite initialized state')
                        p,P,_=s.pose(roads.frame)
                        eig=float(np.linalg.eigvalsh((s.P+s.P.T)/2)[0])
                        if eig < -1e-6*max(1.,float(np.diag(s.P).max())):raise AssertionError('Indefinite covariance')
                        minimum_eigenvalue=min(minimum_eigenvalue,eig)
                    if s.ready!=b.ready:raise AssertionError('Lost continuous initialized output')
                    age=None if arm.last_update_t is None else epoch-arm.last_update_t
                    rec=[r[0],None if p is None else float(p[0]),None if p is None else float(p[1]),
                         bool(s.ready),bool(s.usable),bool(n['updated']),bool(arm.update_count),
                         None if P is None else float(P[0,0]),None if P is None else float(P[0,1]),
                         None if P is None else float(P[1,1]),
                         None if p is None else float(np.linalg.norm(p-bp)),age,int(s.state)]
                    n['elapsed_s']=r[0]
                    variants[key]['records'].append(rec)
                    variants[key]['diagnostics'].append(n)
                if not np.array_equal(b.raw,raw.snapshot().raw):raise AssertionError('Comparator mutated')
                epoch_index+=1
            if epoch_index!=len(d['records']):raise AssertionError('Incomplete replay')
            for arm in [*arms.values(),disabled]:
                if arm.sensor_counts!={'imu':len(imu),'gnss':len(gnss)}:raise AssertionError('Skipped sensors')
            final=raw.snapshot()
            if not np.array_equal(final.raw,control.snapshot().raw) or not np.array_equal(final.raw,disabled.nav.snapshot().raw):
                raise AssertionError('Final baseline mismatch')
        finally:
            raw.close();control.close();disabled.close()
            for arm in arms.values():arm.close()
        if arrays!=[hashlib.sha256(x.tobytes()).hexdigest() for x in (imu,gnss)]:raise AssertionError('Inputs mutated')
        for key,v in variants.items():
            ready=[r for r in v['records'] if r[3]]
            applied=[n for n in v['diagnostics'] if n['updated']]
            steps=[n['native_position_step_m'] for n in applied]
            v['stats']=dict(initialized_outputs=len(ready),usable_outputs=sum(r[4] for r in ready),
                supported_epochs=sum(n['supported'] for n in supports),
                additional_supported_epochs=sum(n['supported'] and not n['original_accepted'] for n in supports),
                original_accepted_epochs=sum(n['original_accepted'] for n in supports),
                map_updates=len(applied),max_map_step_m=max(steps,default=0.),
                p95_map_step_m=float(np.percentile(steps,95)) if steps else 0.,
                max_displacement_from_raw_m=max((r[10] for r in ready),default=0.),
                skip_bit_identity_checks=skips[key],reasons=dict(Counter(n['reason'] for n in v['diagnostics'])))
        result=dict(schema=1,id=d['id'],variants=variants,support=supports)
        (output/(d['id']+'.json')).write_text(json.dumps(result,separators=(',',':'),allow_nan=False)+'\n')
        summary=dict(id=d['id'],sensor_events=dict(imu=len(imu),gnss=len(gnss)),epochs=epoch_index,
            input_provenance=provenance,inputs_unmodified=True,baseline_bit_identical=True,
            skipped_factors_bit_identical=True,native_instances_preserved=True,
            minimum_covariance_eigenvalue=minimum_eigenvalue,wall_seconds=time.perf_counter()-wall,
            support_reasons=dict(Counter(n['reason'] for n in supports)),
            variants={k:v['stats'] for k,v in variants.items()})
        proof['datasets'].append(summary)
        (output/'verification.json').write_text(json.dumps(proof,indent=2,allow_nan=False)+'\n')
        print(json.dumps(summary),flush=True)
    return proof


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--playback',type=Path,required=True)
    p.add_argument('--datasets',type=Path,required=True)
    p.add_argument('--library',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--sigma-m',type=float,nargs='+',default=[.25])
    p.add_argument('--source-root',type=Path)
    a=p.parse_args();run(a.playback,a.datasets,a.library,a.output,a.sigma_m,a.source_root)
