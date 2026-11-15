"""Paired internal-EKF route convergence using the unmodified native filter.

No bias state/detector, snapped marker, reference trajectory, or new EKF tuning.
The optional prior diagnostic adds the SAME stated uncertainty to every arm.
Known-route conditioning calls RoadBank.condition directly. Automatic association
runs the actual full RoadBank, and its selected native state is logged separately
from the published mixture/fallback. Recordings and road geometry are immutable.
"""
from __future__ import annotations
import argparse
import csv
from collections import Counter
import hashlib
import json
import math
from pathlib import Path
import sys
import time
import numpy as np

HERE = Path(__file__).resolve().parent
INTEGRATION = HERE.parent
ROOT = INTEGRATION.parents[1]
sys.path[:0] = [str(INTEGRATION), str(ROOT / 'routefusion_prototype')]
from native import Library
from datasets import load_inputs, events
from osm_graph import compile_osm
from road_bank import RoadBank, Branch
from routefusion.filter import interval_moments


def sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def supplied_edge(route: dict, t: float) -> str:
    return next(edge for when, edge in reversed(route['segments']) if when <= t + 1e-8)


def geometry_metrics(p: np.ndarray, roads, route: dict, edge_id: str) -> dict:
    projections = [(roads.store.edges[e], roads.store.edges[e].project(p))
                   for _, e in route['segments']]
    distance_to_corridor = min(max(0.0, q.distance - e.half_width) for e, q in projections)
    edge = roads.store.edges[edge_id]
    q = edge.project(p)
    return {'east_m': float(p[0]), 'north_m': float(p[1]),
            'cross_track_m': float(q.cross), 'half_width_m': edge.half_width,
            'route_excess_m': float(distance_to_corridor), 'along_m': float(q.s)}


def perturb(nav, roads, edge_id: str, target: float | None, prior: str) -> dict:
    """Move only initial position; preserve physical covariance unless declared.

    Apply once BEFORE cloning all comparison arms. The delta is not a map update.
    Normal/tangent are frozen at the clean checkpoint, not recomputed from truth.
    """
    s = nav.snapshot()
    assert s.ready and s.usable
    assert s.P.shape == (18, 18) and not hasattr(s, 'gnss_bias')
    p, P, _ = s.pose(roads.frame)
    q = roads.store.edges[edge_id].project(p)
    normal_ned = roads.frame.from_ned(s.anchor)[:2].T @ q.normal
    dx = np.zeros(18)
    if target is not None:
        dx[6:9] = normal_ned * (target - q.cross)
    covariance = s.P.copy()
    if prior == 'additional_10m_cross_track_sigma':
        covariance[6:9, 6:9] += 100.0 * np.outer(normal_ned, normal_ned)
    elif prior != 'native':
        raise ValueError('Unknown initial covariance policy')
    if target is not None or prior != 'native':
        nav.inject(dx, covariance)
    after = nav.snapshot()
    pa, Pa, _ = after.pose(roads.frame)
    return {'unperturbed_cross_m': float(q.cross),
            'requested_target_cross_m': target,
            'actual_initial_cross_m': float(roads.store.edges[edge_id].project(pa).cross),
            'position_shift_m': float(np.linalg.norm(dx[6:9])),
            'native_cross_sigma_m': float(np.sqrt(q.normal @ P @ q.normal)),
            'initial_cross_sigma_m': float(np.sqrt(q.normal @ Pa @ q.normal)),
            'covariance_policy': prior}


def factor_diagnostic(branch, bank) -> dict:
    """Read-only explanation of the original condition() gates, not a new update."""
    s = branch.nav.snapshot()
    if not s.ready:
        return {'reason': 'NOT_READY'}
    p, _, v = s.pose(bank.roads.frame)
    edge = bank.roads.store.edges[branch.edge]
    q = edge.project(p - branch.offset)
    h = np.zeros(18)
    h[6:9] = q.normal @ bank.roads.frame.from_ned(s.anchor)[:2]
    c = s.P @ h
    S = float(h @ c + 1.5**2)
    if q.s < -12 or q.s > edge.length + 15:
        return {'reason': 'ENDPOINT_GATE'}
    if np.linalg.norm(v) > 2 and q.tangent @ v < .2 * np.linalg.norm(v):
        return {'reason': 'DIRECTION_GATE'}
    if S <= 0:
        return {'reason': 'NONPOSITIVE_VARIANCE'}
    ll, mean, variance = interval_moments(q.cross, S, -edge.half_width, edge.half_width)
    result = {'log_interval_probability': float(ll), 'pre_cross_m': float(q.cross),
              'cross_sigma_m': float(np.sqrt(max(0., h @ c)))}
    if ll < -12:
        return {**result, 'reason': 'INTERVAL_REJECTED'}
    cell = (bank._version, edge.id, int(math.floor(q.s / 10.)))
    if cell in branch.cells:
        return {**result, 'reason': 'REUSED_CELL'}
    dx = c * ((mean-q.cross)/S)
    if np.linalg.norm(dx[:3]) > .25 or np.linalg.norm(dx[15:18]) > .15:
        return {**result, 'reason': 'ATTITUDE_OR_MOUNT_GATE'}
    return {**result, 'reason': 'APPLIED', 'expected_cross_shift_m': float(h @ dx)}


def metrics(rows: list[dict], prefix: str, cfg: dict) -> dict:
    """Time-weighted residence; post-update state owns a duplicate map epoch.

    Pre-map t=0 remains in the CSV and initial-error diagnostic, but has zero
    duration. Do not dilute residence with a duplicated timestamp or condition
    the denominator on successful road latches.
    """
    initial_error = float(rows[0][prefix+'route_excess_m'])
    unique = dict((r['elapsed_s'],r) for r in rows)
    samples = list(unique.values())
    times = np.array([r['elapsed_s'] for r in samples])
    excess = np.array([r[prefix+'route_excess_m'] for r in samples])
    inside = excess <= cfg['inside_tolerance_m']
    duration = np.diff(times)
    if np.any(duration <= 0): raise ValueError('Nonchronological metric rows')
    hold = cfg['hold_duration_s']
    first_hold = None
    for i in range(len(samples)):
        j = np.searchsorted(times, times[i]+hold, side='left')
        if j < len(samples) and np.all(inside[i:j+1]):
            first_hold = float(times[i]); break
    outside = np.flatnonzero(~inside)
    last_entry = int(outside[-1])+1 if len(outside) else 0
    lock_until_end = (float(times[last_entry]) if last_entry < len(times)
                      and times[-1]-times[last_entry] >= hold else None)
    first_duration = np.maximum(0.,np.minimum(times[1:],cfg['withheld_gnss_duration_s'])-times[:-1])
    followup = np.maximum(0.,times[1:]-np.maximum(times[:-1],first_hold or 0.))
    return {'output_epochs':len(samples),'initial_route_excess_m':initial_error,
            'after_initial_map_route_excess_m':float(excess[0]),
            'final_route_excess_m':float(excess[-1]),
            'inside_fraction':float(inside[:-1]@duration/duration.sum()),
            'first_10s_inside_fraction':float(inside[:-1]@first_duration/first_duration.sum()),
            'first_5s_hold_entry_s':first_hold,
            'entry_then_stays_until_end_s':lock_until_end,
            'route_excess_rms_m':float(np.sqrt((excess[:-1]**2)@duration/duration.sum())),
            'route_excess_p95_m':float(np.percentile(excess,95)),
            'post_first_hold_outside_fraction':(None if first_hold is None else
                float((~inside[:-1])@followup/followup.sum()))}


def accepted_native_latch(rows: list[dict], cfg: dict) -> dict:
    """Require a selected native branch ON the supplied route AND inside it.

    A good fallback coordinate or a road ID alone cannot satisfy this metric.
    """
    adapted = []
    for row in rows:
        correct = bool(row.get('auto_latched') and row.get('auto_selected_on_supplied_route'))
        inside = row.get('auto_route_excess_m',float('inf')) <= cfg['inside_tolerance_m']
        adapted.append({'elapsed_s':row['elapsed_s'],
                        'accepted_route_excess_m':0. if correct and inside else 1.})
    m = metrics(adapted,'accepted_',cfg)
    return {k:m[k] for k in ('inside_fraction','first_10s_inside_fraction',
                            'first_5s_hold_entry_s','entry_then_stays_until_end_s')}


def checkpoint(lib, imu, gnss, route):
    nav = lib.create()
    for kind, row in events(imu, gnss):
        getattr(nav, kind)(row)
        if kind == 'imu' and row[0] >= route['start_s']:
            if not nav.snapshot().usable:
                nav.close(); raise ValueError('Chosen checkpoint is not usable')
            return nav, float(row[0])
    raise ValueError('No checkpoint')


def run_case(lib, imu, gnss, provenance, roads, route, target, prior, feed, cfg,
             output: Path, *, automatic: bool = True):
    nav, t0 = checkpoint(lib, imu, gnss, route)
    original_hashes = (hashlib.sha256(imu.tobytes()).hexdigest(), hashlib.sha256(gnss.tobytes()).hexdigest())
    initial = perturb(nav, roads, supplied_edge(route,t0), target, prior)
    base, control, known = nav.clone(), nav.clone(), nav.clone()
    bank = RoadBank(nav.clone(), roads) if automatic else None
    assert np.array_equal(base.snapshot().raw, known.snapshot().raw)
    helper = RoadBank(nav, roads, recovery=False)
    helper._version = roads.store.version
    branch = Branch(supplied_edge(route,t0), known, np.zeros(2), 0.)
    rows, factors = [], []
    next_map, next_output = t0, t0
    last_out = None
    accepted, rejected = 0, 0
    counters = Counter()
    t_start = time.perf_counter()
    min_eigenvalue = float('inf')
    imu_count = gnss_count = withheld = 0
    exact = True

    def sample(t, map_happened=False):
        nonlocal min_eigenvalue, exact
        sb, sk = base.snapshot(), known.snapshot()
        exact &= np.array_equal(sb.raw, control.snapshot().raw)
        if bank is not None:
            exact &= np.array_equal(sb.raw, bank.base.snapshot().raw)
        if not sb.ready or not sk.ready:
            raise AssertionError('Paired initialization availability changed')
        edge = supplied_edge(route,t)
        pb, _, _ = sb.pose(roads.frame); pk, _, _ = sk.pose(roads.frame)
        row = {'t_s':float(t),'elapsed_s':float(t-t0),'assigned_edge':edge,
               'map_epoch':bool(map_happened),'base_usable':sb.usable,'known_usable':sk.usable}
        for prefix,p in [('before_',pb),('known_',pk)]:
            row.update({prefix+k:v for k,v in geometry_metrics(p,roads,route,edge).items()})
        row['known_displacement_m'] = float(np.linalg.norm(pk-pb))
        min_eigenvalue = min(min_eigenvalue,float(np.linalg.eigvalsh(sk.P)[0]))
        if not np.isfinite(sb.raw).all() or not np.isfinite(sk.raw).all():
            raise AssertionError('Nonfinite native state')
        # Default public position is intentionally still map-free. For this
        # test expose one selected REAL native EKF, never the mixture mean.
        if bank is not None:
            selected = None
            if last_out and last_out.get('latched'):
                eid = last_out['edge']
                choices = [b for b in bank.branches if b.edge == eid and
                           (b.maneuver is None or b.maneuver.phase == 'APPROACH')]
                if choices:
                    selected = max(choices,key=lambda b:b.logw)
            ps = selected.nav.snapshot().pose(roads.frame)[0] if selected else pb
            row.update({ 'auto_status':last_out['status'] if last_out else 'NOT_EVALUATED',
                'auto_latched':bool(selected),'auto_edge':selected.edge if selected else None,
                'auto_selected_on_supplied_route':bool(selected and selected.edge in {e for _,e in route['segments']}),
                'auto_branch_count':len(bank.branches)})
            row.update({'auto_'+k:v for k,v in geometry_metrics(ps,roads,route,edge).items()})
            row['auto_position_is_native_or_fallback'] = True
        rows.append(row)

    def map_step(t):
        nonlocal last_out, accepted, rejected
        branch.edge = supplied_edge(route,t)
        diag = factor_diagnostic(branch,helper)
        pre = known.snapshot()
        ll = helper.condition(branch)  # THE ORIGINAL IMPLEMENTATION, unmodified.
        post = known.snapshot()
        applied = diag['reason']=='APPLIED'
        assert np.isfinite(ll) == (diag['reason'] in ('APPLIED','REUSED_CELL'))
        if not applied:
            assert np.array_equal(pre.raw,post.raw), 'Rejected/reused factor mutated state'
        else:
            accepted += 1
        if not np.isfinite(ll): rejected += 1
        counters[diag['reason']] += 1
        pp=pre.pose(roads.frame)[0];pn=post.pose(roads.frame)[0]
        factors.append({'t_s':float(t),'elapsed_s':float(t-t0),'edge':branch.edge,
                        **{k:(None if isinstance(v,float) and not math.isfinite(v) else v) for k,v in diag.items()},
                        'state_displacement_m':float(np.linalg.norm(pn-pp)),
                        'post_cross_m':float(roads.store.edges[branch.edge].project(pn).cross)})
        if bank is not None: last_out=bank.step(t)

    try:
        sample(t0)  # initial pre-map state, paired exactly
        map_step(t0)
        sample(t0,True)  # immediate native mean/covariance change
        next_map=t0+cfg['map_period_s'];next_output=t0+cfg['output_period_s']
        # The checkpoint already consumed the first IMU at t0; same-time GNSS
        # follows it in the upstream ordering and must be delivered once.
        for kind,row in events(imu,gnss):
            t=float(row[0])
            if t<t0 or (kind=='imu' and t<=t0):continue
            if t>route['end_s']:break
            if kind=='gnss' and feed=='withhold_first_10s' and t<t0+cfg['withheld_gnss_duration_s']:
                withheld+=1;continue
            for n in (base,control,known):getattr(n,kind)(row)
            if bank is not None:getattr(bank,kind)(row)
            if kind=='imu':imu_count+=1
            else:gnss_count+=1
            if kind!='imu':continue
            map_now=t+1e-8>=next_map
            if map_now:
                map_step(t);next_map=t+cfg['map_period_s']
            if t+1e-8>=next_output or map_now:
                sample(t,map_now);next_output=t+cfg['output_period_s']
        assert exact, 'Baseline contaminated by map/replay'
        assert min_eigenvalue>=-1e-6
        assert original_hashes == (hashlib.sha256(imu.tobytes()).hexdigest(),hashlib.sha256(gnss.tobytes()).hexdigest())
        id=f"{route['name']}__{'clean' if target is None else 'cross'+str(int(target))}__{prior}__{feed}"
        summary={'id':id,'route':route,'target_cross_m':target,'prior':prior,'gnss_mode':feed,
            'bias_model_enabled':False,'native_error_states':18,'initial':initial,
            'start_s':t0,'end_s':float(rows[-1]['t_s']),'input_provenance':provenance,
            'delivered_after_checkpoint':{'imu':imu_count,'gnss':gnss_count,'withheld':withheld},
            'input_arrays_unchanged':True,'fallback_bit_identical':exact,
            'known_map_updates':accepted,'known_map_rejections':rejected,'factor_reasons':dict(counters),
            'minimum_known_covariance_eigenvalue':min_eigenvalue,
            'before':metrics(rows,'before_',cfg),'known_route':metrics(rows,'known_',cfg),
            'automatic':metrics(rows,'auto_',cfg) if bank is not None else None,
            'automatic_accepted_native_latch':accepted_native_latch(rows,cfg) if bank is not None else None,
            'automatic_accepted_fraction':float(np.mean([r.get('auto_latched',False) for r in rows])),
            'automatic_on_supplied_route_fraction':float(np.mean([r.get('auto_selected_on_supplied_route',False) for r in rows])),
            'automatic_status_counts':dict(Counter(r.get('auto_status') for r in rows)) if bank else {},
            'max_known_before_displacement_m':max(r['known_displacement_m'] for r in rows),
            'wall_seconds':time.perf_counter()-t_start}
        for name, records in [(id+'.csv',rows),(id+'__factors.csv',factors)]:
            fields=list(dict.fromkeys(k for r in records for k in r))
            with (output/name).open('w',newline='') as f:
                w=csv.DictWriter(f,fieldnames=fields);w.writeheader();w.writerows(records)
        (output/(id+'.json')).write_text(json.dumps(summary,indent=2,allow_nan=False)+'\n')
        return summary
    finally:
        base.close();control.close();known.close();nav.close()
        if bank is not None:bank.reset();bank.base.close()


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--library',type=Path,required=True)
    ap.add_argument('--datasets',type=Path,required=True)
    ap.add_argument('--map-cache',type=Path,required=True)
    ap.add_argument('--output',type=Path,required=True)
    ap.add_argument('--config',type=Path,default=HERE/'cases.json')
    ap.add_argument('--route',choices=['short2-turn','short1-route','medium-route'])
    ap.add_argument('--known-only',action='store_true')
    args=ap.parse_args();args.output.mkdir(parents=True,exist_ok=False)
    cfg=json.loads(args.config.read_text())
    lib=Library(args.library)
    if hasattr(lib.lib,'rf_new_bias'):raise ValueError('Use unaugmented 18-state native build, not bias-capable library')
    roads=compile_osm(args.map_cache/'osm-roads.json.gz',args.map_cache/'provenance.json')
    summaries=[]
    for route in cfg['routes']:
        if args.route and route['name']!=args.route:continue
        # Supplied itinerary must be connected in the independent graph.
        ids=[e for _,e in route['segments']]
        for a,b in zip(ids,ids[1:]):
            if b not in roads.store.edges[a].successors:raise ValueError('Supplied route disconnected: '+a+' -> '+b)
        imu,gnss,provenance=load_inputs(args.datasets/route['dataset'])
        cases=[(None,'native','original_gnss')]
        cases += [(target,prior,feed) for target in cfg['cross_track_targets_m']
                  for prior in cfg['prior_modes'] for feed in ['original_gnss','withhold_first_10s']]
        for target,prior,feed in cases:
            result=run_case(lib,imu,gnss,provenance,roads,route,target,prior,feed,cfg,args.output,automatic=not args.known_only)
            summaries.append(result)
            print(result['id'], json.dumps({k:result[k] for k in ['known_map_updates','known_map_rejections']}),
                  'first10 before/known:',result['before']['first_10s_inside_fraction'],result['known_route']['first_10s_inside_fraction'],flush=True)
    record={'config':cfg,'config_sha256':sha(args.config),'library_sha256':sha(args.library),
            'source_git_revision':'4ff67b909fd2ac025f06c550eedbdc6ef21c52a5',
            'native_upstream_revision':'8a34212c7befe30f8d3eae4ad708cc0dcd4aa0d6',
            'source_sha256':{str(p.relative_to(ROOT)):sha(p) for p in [INTEGRATION/'road_bank.py',INTEGRATION/'intersections.py',INTEGRATION/'native.py',INTEGRATION/'recovery.py',INTEGRATION/'osm_graph.py',Path(__file__)]},
            'map_response_sha256':roads.metadata['response_sha256'],'results':summaries}
    (args.output/'summary.json').write_text(json.dumps(record,indent=2,allow_nan=False)+'\n')

if __name__=='__main__':main()
