"""Reproduce paired map-only recovery, including failures and clean regressions.

Uses the ORIGINAL 18-state library; no bias-capable library is accepted. The
existing supplied-route test fixture is reused, not advertised as held-out truth.
Results are internal native EKF coordinates, not a snapped display or mixture.
"""
from pathlib import Path
import argparse
import csv
import hashlib
import json
import os
import sys
import xml.etree.ElementTree as ET
from dataclasses import asdict

HERE=Path(__file__).resolve().parent
sys.path[:0]=[str(HERE/'route_latch'),str(HERE),str(HERE.parents[1]/'routefusion_prototype')]
from evaluate import Library, load_inputs, compile_osm, run_case
from evaluate_map import run as map_run
from road_bank import RoadBank
from route_recovery import RelocalizingRoadBank, RelocalizationConfig


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def compare_native_csv(a,b):
    left=list(csv.DictReader(Path(a).open()))
    right=list(csv.DictReader(Path(b).open()))
    if len(left)!=len(right):raise AssertionError('Changed clean coverage')
    # Do not compare elapsed wall-clock time or diagnostic recovery-buffer state.
    fields=('t_s','status','latched','edge','base_e','base_n','candidate_e','candidate_n','p_ee','p_en','p_nn')
    for x,y in zip(left,right):
        if any(x.get(k)!=y.get(k) for k in fields):
            raise AssertionError('Changed clean native/map output: '+str(x['t_s']))
    return len(left)


def run(args):
    output=args.output
    output.mkdir(parents=True,exist_ok=False)
    library=Library(args.library)
    if hasattr(library.lib,'rf_new_bias'):
        raise ValueError('Bias-capable build is not the original 18-state runtime')
    cfg=json.loads((HERE/'route_latch/cases.json').read_text())
    roads=compile_osm(args.map_cache/'osm-roads.json.gz',args.map_cache/'provenance.json')
    parameters=asdict(RelocalizationConfig())
    # Capture parameters before evaluating, never change them in response to results.
    (output/'policy.json').write_text(json.dumps(parameters,indent=2)+'\n')
    records=[]
    for route in cfg['routes']:
        imu,gnss,provenance=load_inputs(args.datasets/route['dataset'])
        cases=[(None,'original_gnss')]+[(t,f) for t in cfg['cross_track_targets_m']
                                      for f in ('original_gnss','withhold_first_10s')]
        for target,feed in cases:
            pair={}
            for variant,enabled in [('original',False),('recovery',True)]:
                dest=output/variant;dest.mkdir(exist_ok=True)
                m=run_case(library,imu,gnss,provenance,roads,route,target,'native',feed,
                           cfg,dest,relocalize=enabled)
                if not m['fallback_bit_identical'] or not m['input_arrays_unchanged']:
                    raise AssertionError('Map affected control/input ownership')
                if m['minimum_known_covariance_eigenvalue'] < -1e-6:
                    raise AssertionError('Invalid covariance')
                pair[variant]=m
            if pair['original']['before']!=pair['recovery']['before']:
                raise AssertionError('Changed independent baseline')
            if target is None:
                for key in ('known_route','automatic','automatic_accepted_native_latch'):
                    if pair['original'][key]!=pair['recovery'][key]:raise AssertionError('Changed clean fixture')
            records.append(pair)
            (output/'paired-summary.json').write_text(json.dumps(records,indent=2,allow_nan=False)+'\n')
            print(pair['recovery']['id'], 'known entry',pair['recovery']['known_route']['first_5s_hold_entry_s'],
                  'accepted automatic entry',pair['recovery']['automatic_accepted_native_latch']['first_5s_hold_entry_s'],flush=True)
    clean=[]
    for name in ('urban-short-turn-loop-nominal-001','urban-short-turn-loop-nominal-002',
                 'urban-stop-go-tight-turns-nominal-001','ios-mixed-road-drive-001'):
        paths={}
        for variant,cls in [('original',RoadBank),('recovery',RelocalizingRoadBank)]:
            dest=output/'clean'/variant
            paths[variant]=dest/(name+'.csv')
            metric=map_run(library,args.datasets/name,roads,dest,bank_class=cls)
            if not metric['map_free_exactly_matches_independent_replay']:
                raise AssertionError('Clean fallback contaminated')
        epochs=compare_native_csv(paths['original'],paths['recovery'])
        clean.append({'dataset':name,'sampled_epochs':epochs,'native_map_outputs_unchanged':True})
    if len(records)!=27:raise AssertionError('Unexpected paired coverage')
    outcomes=[]
    for magnitude in (10.,20.):
        subset=[r for r in records if r['original']['target_cross_m'] is not None and
                abs(r['original']['target_cross_m'])==magnitude and
                r['original']['gnss_mode']=='withhold_first_10s']
        if len(subset)!=6:raise AssertionError('Expected six equal disturbance cases')
        group={'initial_cross_magnitude_m':magnitude,'cases':6}
        for variant in ('original','recovery'):
            for key in ('known_route','automatic_accepted_native_latch'):
                times=[r[variant][key]['first_5s_hold_entry_s'] for r in subset]
                group[variant+'_'+key]={'entry_before_gnss_return':sum(t is not None and t<10 for t in times),
                    'five_second_hold_completed_before_gnss_return':sum(t is not None and t+5<10 for t in times),
                    'first_5s_hold_entry_s':times}
        outcomes.append(group)
    # Test execution integrity explicitly; do not turn failed road associations
    # into CI passes by redefining the denominator or reporting only successful tails.
    suites=list(ET.parse(args.tests).getroot().iter('testsuite'))
    if not suites or any(int(s.get(k,0)) for s in suites for k in ('errors','failures','skipped')):
        raise AssertionError('Tests missing/skipped/failing')
    tests=sum(int(s.get('tests',0)) for s in suites)
    if tests<70:raise AssertionError('Missing old or new regression checks')
    result={'experiment_integrity_passed':True,'tests':tests,'paired_cases':27,'clean_full_runs':clean,
            'bias_model_enabled':False,'initial_covariance_changed_before_comparison':False,
            'candidate_recovery_prior_explicitly_changes_covariance':True,'policy':parameters,
            'outcomes':outcomes,'execution_commit':os.environ.get('GITHUB_SHA','local'),
            'native_library_sha256':digest(args.library),'map_response_sha256':roads.metadata['response_sha256'],
            'source_sha256':{str(p.relative_to(HERE)):digest(p) for p in
                [HERE/'route_recovery.py',HERE/'route_latch/evaluate.py',HERE/'evaluate_map.py',Path(__file__)]}}
    (output/'qualification.json').write_text(json.dumps(result,indent=2)+'\n')
    lines=['# Map-only recovery: executed comparison','',
           'Original 18-state runtime, GNSS-bias model disabled. Initial means/covariances and original inputs are identical across arms. Recovery adds uncertainty to a separate candidate only after its confirmation policy fires. These are reused controlled route fixtures, not independent surveyed truth.','',
           '| Initial cross-track | Cases | Original supplied route | Recovery supplied route | Original automatic | Recovery automatic |',
           '|---|---:|---:|---:|---:|---:|']
    for g in outcomes:
        counts=[g[v+'_'+k]['entry_before_gnss_return'] for v,k in
                [('original','known_route'),('recovery','known_route'),('original','automatic_accepted_native_latch'),('recovery','automatic_accepted_native_latch')]]
        lines.append('| +/-'+str(int(g['initial_cross_magnitude_m']))+' m | 6 | '+' | '.join(str(n)+'/6' for n in counts)+' |')
    lines+=['','Entry is followed by five sampled seconds of retention; for automatic latches this can extend past GNSS resumption at ten seconds. The JSON separately counts holds completed before fixes resume. No good fallback position counts as accepted automatic latching.',
            '', 'Automatic association may abstain at intersections; not all cases satisfy a continuous five-second accepted latch. An unknown, coherent parallel road and an explicitly supplied wrong route cannot be resolved from these inputs alone. Map-free production/default outputs remain unchanged.',
            '',f'{tests} tests passed. Four full unmodified mapped recordings retain the same sampled native map outputs. All paired fallback controls remain identical. Per-case trajectories and diagnostics include failures, not only recovered intervals.']
    (output/'REPORT.md').write_text('\n'.join(lines)+'\n')
    return result

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    for name in ('library','datasets','map-cache','output','tests'):p.add_argument('--'+name,type=Path,required=True)
    args=p.parse_args();print(json.dumps(run(args),indent=2))
