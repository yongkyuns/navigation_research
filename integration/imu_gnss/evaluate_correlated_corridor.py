"""Matched real-road native replay: independent versus shared map uncertainty.

All arms receive all original sensors. Both proposed corridor factors use the
same causal raw-EKF path support, geometry normal and spatial key. Only the
map covariance treatment differs. No surveyed truth or new matcher is used.
"""
from __future__ import annotations
import argparse
from collections import Counter
import hashlib
import json
import math
import os
from pathlib import Path
import numpy as np
from correlated_corridor import NativeCorridorSchmidt, map_patch_id
from datasets import load_inputs, events
from evaluate_support import load_roads
from native import Library
from native_schmidt import compose_joint
from path_support import PathIndex, CausalSupport
from verify_native_transition import compare_navigation_snapshots

IDS=('ios-mixed-road-drive-001','urban-short-turn-loop-nominal-001',
     'urban-short-turn-loop-nominal-002','urban-stop-go-tight-turns-nominal-001')


def digest(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def observation(path,support,raw_state,roads,sigma_m,cell_m):
    """Same world-frame road line and information key for every comparison arm."""
    if path is None or not support['supported']:
        return None
    p,_,_=raw_state.pose(roads.frame)
    q=path.project(p)
    quality=min(support['quality'],path.quality(q.s,support['taper_m']))
    if not 0 < quality <= 1.:
        return None
    return dict(path=path,point=q.point,normal=q.normal,
                cell=f'{roads.store.version}:{path.key}:{math.floor(q.s/cell_m)}',
                local_variance=(sigma_m**2+path.deviation_m**2)/quality,
                half_width=path.width,quality=float(quality))


def apply_observation(arm,obs,roads,shared_covariance,independent):
    before=arm.nav.snapshot()
    note=dict(applied=False,reason='NO_SUPPORT',native_position_step_m=0.,
              pre_cross_m=None,post_cross_m=None,information_fraction=0.)
    if obs is None:
        return note
    note['cell']=obs['cell']
    if not before.ready or not before.usable:
        return {**note,'reason':'NATIVE_UNAVAILABLE'}
    p,_,v=before.pose(roads.frame);path=obs['path']
    if (not roads.covers(before.lla) or not roads.store.is_loaded(p)
            or any(not roads.store.available(e) for e in path.edges)):
        return {**note,'reason':'MISSING_GEOMETRY'}
    try:q=path.project(p)
    except ValueError:return {**note,'reason':'AMBIGUOUS_PROJECTION'}
    if not 0 <= q.s <= path.length:
        return {**note,'reason':'OUTSIDE_PATH'}
    speed=float(np.linalg.norm(v))
    if speed>2 and q.tangent@v < .2*speed:
        return {**note,'reason':'HEADING_INCOMPATIBLE'}
    if q.distance>30.:
        return {**note,'reason':'TOO_DISTANT'}
    n=obs['normal'];h=np.zeros(18);h[6:9]=n@roads.frame.from_ned(before.anchor)[:2]
    j=-n
    d=float(n@(p-obs['point']))
    local_var=float(obs['local_variance'])
    if independent:
        local_var+=float(j@shared_covariance@j)
    result=arm.interval_factor(patch_id=arm.patch_id,cell=obs['cell'],cross_track_m=d,
            half_width_m=obs['half_width'],h=h,j=j,local_sigma_m=math.sqrt(local_var))
    note.update(result,pre_cross_m=d,local_sigma_m=math.sqrt(local_var),quality=obs['quality'])
    if result['applied']:
        after,_,_=arm.nav.snapshot().pose(roads.frame)
        note.update(native_position_step_m=float(np.linalg.norm(after-p)),
                    post_cross_m=float(n@(after-obs['point'])))
    return note


def run(playback,datasets,control_library,traced_library,output,*,sigma_m=.25,map_sigma_m=1.5,source_root):
    if not all(math.isfinite(v) and v>0 for v in (sigma_m,map_sigma_m)):
        raise ValueError('Positive finite observation and shared-map sigmas required')
    doc=json.loads(Path(playback).read_text());recordings,roads=load_roads(doc)
    if {d['id'] for d in recordings}!=set(IDS) or len(recordings)!=4:
        raise ValueError('All four unmodified mapped recordings are required')
    source_hashes=json.loads(Path(__file__).with_name('upstream-source-sha256.json').read_text())
    if len(source_hashes)!=37:
        raise ValueError('Expected the existing 37-file pinned native manifest')
    for name,wanted in source_hashes.items():
        if digest(Path(source_root)/name)!=wanted:raise ValueError('Changed pinned source: '+name)
    index=PathIndex(roads);map_version=roads.store.version
    patch=map_patch_id(roads.metadata['response_sha256'],roads.frame.origin)
    U=np.eye(2)*map_sigma_m**2
    output=Path(output);output.mkdir(parents=True,exist_ok=False)
    baseline,traced=Library(control_library),Library(traced_library)
    proof=dict(schema=1,execution_commit=os.environ.get('GITHUB_SHA','local-uncommitted'),
        recordings=[],sigma_m=sigma_m,map_sigma_m=map_sigma_m,cell_m=10.,
        map_patch=patch,map_response_sha256=roads.metadata['response_sha256'],
        playback_sha256=digest(playback),pinned_sources_verified=37,
        control_library_sha256=digest(control_library),traced_library_sha256=digest(traced_library),
        no_truth_used=True,road_selection='unchanged causal seeds plus bounded raw-state path continuation',
        approximation='Gaussian interval navigation moments with fixed-map Schmidt moment projection',
        complete=False,source_sha256={})
    for name in ['correlated_corridor.py','evaluate_correlated_corridor.py','path_support.py',
                 'native_schmidt.py','native.py','prepare_native_transition.py']:
        proof['source_sha256'][name]=digest(Path(__file__).with_name(name))
    for d in sorted(recordings,key=lambda x:x['id']):
        imu,gnss,inputs=load_inputs(Path(datasets)/d['id'])
        if inputs['input_sha256']!=d['input_sha256']:raise AssertionError('Input archive changed')
        input_arrays=[hashlib.sha256(a.tobytes()).hexdigest() for a in (imu,gnss)]
        raw=baseline.create()
        arms={name:NativeCorridorSchmidt(traced.create(),patch,U if name=='correlated' else np.zeros((2,2)))
              for name in ('correlated','independent','disabled')}
        handles={name:f.nav.handle for name,f in arms.items()}
        selector=CausalSupport(index);rows=[];diagnostics=[];sizes=None;epoch_index=0;count=0
        stats={name:Counter() for name in arms};steps={name:[] for name in arms}
        min_eigen={name:math.inf for name in arms};last_kind='none';last_t=None
        try:
            for kind,value in events(imu,gnss):
                last_kind,last_t=kind,float(value[0]);count+=1
                getattr(raw,kind)(value)
                for arm in arms.values():arm.feed(kind,value)
                b=raw.snapshot()
                new_sizes=compare_navigation_snapshots(b,arms['disabled'].nav.snapshot())
                if sizes is None:sizes=new_sizes
                elif sizes!=new_sizes:raise AssertionError('Allocation-size metadata changed')
                for name,arm in arms.items():
                    if arm.nav.handle!=handles[name]:raise AssertionError('Native instance replaced')
                    if not arm.engaged:compare_navigation_snapshots(b,arm.nav.snapshot())
                if kind!='imu' or epoch_index==len(d['records']):continue
                r=d['records'][epoch_index];t=r[0]+d['original_start_t_s']
                if value[0]<t-1e-9:continue
                if abs(value[0]-t)>1e-8:raise AssertionError('Map epoch changed')
                if b.ready!=r[12] or b.usable!=r[13]:raise AssertionError('Initialization changed')
                raw_p=None
                if b.ready:
                    raw_p,_,_=b.pose(roads.frame)
                    if not np.array_equal(raw_p,np.array(r[1:3])):raise AssertionError('Raw baseline changed')
                if roads.store.version!=map_version:raise RuntimeError('Map version changed')
                path,support=selector.select(t,b,r[7] if r[6] else None,r[5])
                obs=observation(path,support,b,roads,sigma_m,10.)
                row=dict(elapsed_s=r[0],raw_position=None if raw_p is None else raw_p.tolist(),
                         original_edge=r[7] if r[6] else None,support_reason=support['reason'],
                         proposed_cell=None if obs is None else obs['cell'],outputs={})
                notes={}
                for name,arm in arms.items():
                    old=arm.nav.snapshot();old_c=arm.C.tobytes();old_u=arm.U.tobytes()
                    note=(dict(applied=False,reason='MAP_DISABLED',native_position_step_m=0.) if name=='disabled'
                          else apply_observation(arm,obs,roads,U,name=='independent'))
                    after=arm.nav.snapshot()
                    if not note['applied']:
                        if after.raw.tobytes()!=old.raw.tobytes() or arm.C.tobytes()!=old_c:
                            raise AssertionError('Rejected map factor mutated moments')
                        stats[name]['skip_identity_checks']+=1
                    if arm.U.tobytes()!=old_u:raise AssertionError('Map prior changed')
                    if after.ready!=b.ready:raise AssertionError('Lost continuous initialized output')
                    p=P=None
                    if after.ready:
                        p,P,_=after.pose(roads.frame)
                        if not np.isfinite(after.raw).all():raise AssertionError('Nonfinite native state')
                        joint=compose_joint(after.P,arm.C,arm.U)
                        mineig=float(np.linalg.eigvalsh((joint+joint.T)*.5)[0])
                        min_eigen[name]=min(min_eigen[name],mineig)
                        if mineig < -2e-5*max(1.,float(np.diag(joint).max())):
                            raise AssertionError('Indefinite navigation/map covariance')
                        stats[name]['initialized_outputs']+=1
                        stats[name]['usable_outputs']+=int(after.usable)
                    row['outputs'][name]=dict(position=None if p is None else p.tolist(),
                        covariance=None if P is None else P.tolist(),ready=bool(after.ready),usable=bool(after.usable),
                        cross_norm=float(np.linalg.norm(arm.C)),map_updates=arm.map_updates,
                        displacement_from_raw_m=None if p is None else float(np.linalg.norm(p-raw_p)))
                    stats[name][note['reason']]+=1
                    if note['applied']:steps[name].append(note['native_position_step_m'])
                    notes[name]=note
                rows.append(row);diagnostics.append(notes);epoch_index+=1
            if count!=len(imu)+len(gnss) or epoch_index!=len(d['records']):
                raise AssertionError('Incomplete sensor or output replay')
            for name,arm in arms.items():
                if arm.imu_events!=len(imu) or arm.gnss_events!=len(gnss):raise AssertionError('Missed sensor events')
                if name!='disabled' and not arm.map_updates:raise AssertionError('No road factors were exercised')
            if input_arrays!=[hashlib.sha256(a.tobytes()).hexdigest() for a in (imu,gnss)]:
                raise AssertionError('Sensor inputs mutated')
            result=dict(dataset=d['id'],source=inputs,map_patch=patch,rows=rows,diagnostics=diagnostics)
            (output/(d['id']+'.json')).write_text(json.dumps(result,separators=(',',':'),allow_nan=False)+'\n')
            summary=dict(dataset=d['id'],sensor_events=count,imu=len(imu),gnss=len(gnss),exported_epochs=epoch_index,
                original_selected_epochs=sum(bool(r[6]) for r in d['records']),
                supported_epochs=sum(r['proposed_cell'] is not None for r in rows),
                control_allocation_bytes=sizes[0],traced_allocation_bytes=sizes[1],
                raw_navigation_bit_identical=True,compared_fields=357,excluded_metadata_slot=7,
                inputs_unmodified=True,native_instances_preserved=True,skips_bit_identical=True,models={})
            for name,arm in arms.items():
                summary['models'][name]=dict(counts=dict(stats[name]),map_updates=arm.map_updates,
                    max_map_step_m=max(steps[name],default=0.),
                    p95_map_step_m=float(np.percentile(steps[name],95)) if steps[name] else 0.,
                    max_displacement_from_raw_m=max((r['outputs'][name]['displacement_from_raw_m'] or 0. for r in rows)),
                    minimum_joint_eigenvalue=min_eigen[name],map_covariance=arm.U.tolist())
            proof['recordings'].append(summary)
            print(json.dumps(summary,allow_nan=False),flush=True)
        except Exception as exc:
            (output/'failure.json').write_text(json.dumps(dict(dataset=d['id'],event=count,kind=last_kind,t_s=last_t,
                error=type(exc).__name__+': '+str(exc),completed_recordings=len(proof['recordings']),
                complete=False,raw_ekf_fallback_used=False),indent=2)+'\n')
            raise
        finally:
            raw.close()
            for arm in arms.values():arm.close()
    proof['complete']=True
    proof['output_sha256']={p.name:digest(p) for p in sorted(output.glob('*.json'))}
    (output/'verification.json').write_text(json.dumps(proof,indent=2,allow_nan=False)+'\n')
    return proof


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    for name in ('playback','datasets','control-library','traced-library','output','source-root'):
        p.add_argument('--'+name,type=Path,required=True)
    p.add_argument('--sigma-m',type=float,default=.25)
    p.add_argument('--map-sigma-m',type=float,default=1.5)
    a=p.parse_args()
    run(a.playback,a.datasets,a.control_library,a.traced_library,a.output,
        sigma_m=a.sigma_m,map_sigma_m=a.map_sigma_m,source_root=a.source_root)
