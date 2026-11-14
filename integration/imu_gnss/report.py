"""Build an auditable report; never convert reference agreement into accuracy."""
from pathlib import Path
import argparse
import gzip
import hashlib
import json
import platform
import sys
import numpy as np
from scipy.spatial import cKDTree
from native import Frame


def audit(dataset_root):
    rows=[]
    for manifest in sorted(dataset_root.glob('*/manifest.json')):
        meta=json.loads(manifest.read_text())
        row={'id':meta['dataset_id'],'timestamp_findings':{},'source':meta.get('source',{}),'reference_provenance':'not independently established'}
        arrays={}
        for key in ('imu','gnss','reference_position'):
            entry=meta['files'].get(key)
            if entry is None:continue
            path=manifest.parent/entry['path'];raw=path.read_bytes()
            if hashlib.sha256(raw).hexdigest()!=entry['sha256']:raise ValueError(f'Checksum failure: {path}')
            with gzip.open(path,'rt') as stream:values=np.loadtxt(stream,delimiter=',',skiprows=1,ndmin=2)
            arrays[key]=values;dt=np.diff(values[:,0])
            row['timestamp_findings'][key]={'rows':len(values),'duplicate_intervals':int(np.sum(dt==0)),'backward_intervals':int(np.sum(dt<0)),'gaps_above_50ms':int(np.sum(dt>.05)),'largest_gap_s':float(dt.max()) if len(dt) else None}
        ref=arrays.get('reference_position')
        if ref is not None:
            g=arrays['gnss'];frame=Frame(g[0,1:4])
            distances,_=cKDTree(frame.project(g[:,1:4])[:,:2]).query(frame.project(ref[:,1:4])[:,:2])
            row['reference_repeated_position_fraction']=float(np.mean(np.all(np.diff(ref[:,1:4],axis=0)==0,axis=1)))
            row['reference_within_1mm_of_any_input_gnss_fraction']=float(np.mean(distances<.001))
            row['matching_note']='Diagnostic, not proof of source dependence; stationary repeats are possible. No surveyed reference-device provenance is established by inspected package metadata.'
        rows.append(row)
    return rows


def number(value):
    return '—' if value is None else f'{value:.3f}'


def write_report(root,datasets,destination):
    audits=audit(datasets)
    (root/'dataset-audit.json').write_text(json.dumps(audits,indent=2)+'\n')
    baseline=json.loads((root/'baseline/summary.json').read_text())
    good=[x for x in baseline if 'rows' in x];rejected=[x for x in baseline if 'rows' not in x]
    maps=json.loads((root/'map/summary.json').read_text())
    stress=[json.loads((root/name/'summary.json').read_text())[0] for name in ('stress-outage30','stress-bias20')]
    epochs=sum(x['sampled_epochs'] for x in good)
    env={'python':sys.version,'platform':platform.platform(),'numpy':np.__version__,'upstream_revision':'8a34212c7befe30f8d3eae4ad708cc0dcd4aa0d6'}
    (root/'evaluation-environment.json').write_text(json.dumps(env,indent=2)+'\n')
    text=[
        '# Native IMU/GNSS integration and recorded-data evaluation','',
        '## What was integrated','',
        'The actual `sensor_fusion` Rust crate from `yongkyuns/imu_gnss_fusion`, pinned to `8a34212c7befe30f8d3eae4ad708cc0dcd4aa0d6`, runs through a small C ABI. Its 18-error-state 3-D propagation, auto mount alignment, GNSS/NHC updates, gating, timing and health policies are retained. No reference attitude, mount, position or motion is fed into the estimator.','',
        'An isolated build adds only clone support, an f64 anchor accessor, and an error-moment injection hook using the existing nominal injection and covariance reset. The source repository is unchanged. Road branches clone the entire runtime and ingest original IMU/GNSS once per alternative. The original planar prototype and frozen evidence are unchanged.','',
        '## Evidence boundary','',
        '**This is a recorded-data replay and integration evaluation, not demonstrated urban-canyon accuracy.** Package references do not establish an independent surveyed reference device. Discrepancies are agreement, not ground-truth error. iOS recordings have no reference trajectory. No independent road labels exist, so latch fraction is not road-association accuracy. These recordings are not established here as the target M9/LSM6DSO hardware.','',
        f"The package directory contains {len(baseline)} datasets. {len(good)} passed strict input checks and replayed: {sum(x['rows']['imu'] for x in good):,} IMU samples and {sum(x['rows']['gnss'] for x in good):,} GNSS samples over {sum(x['duration_s'] for x in good)/3600:.3f} hours. {len(rejected)} dataset was rejected rather than silently retimed. {sum(x['nonfinite_outputs'] for x in good)} non-finite outputs were found at the {epochs:,} inspected output epochs.",'',
        f"Initialization succeeded in {sum(x['initialization_t_s'] is not None for x in good)} replays; the two short stationary smoke recordings did not initialize automatic vehicle alignment. Output was available at {100*sum(x['ready_fraction']*x['sampled_epochs'] for x in good)/epochs:.2f}% of sampled epochs, including initialization time rather than selecting only successful tails.",'',
        '### Per-recording baseline','',
        '| Dataset | Duration (s) | First initialized epoch (s) | Output availability | Reference discrepancy RMSE (m) |',
        '|---|---:|---:|---:|---:|',
    ]
    for x in good:
        r=x['reference_discrepancy']
        text.append(f"| {x['id']} | {x['duration_s']:.1f} | {number(x['initialization_t_s'])} | {100*x['ready_fraction']:.1f}% | {number(r['rmse_m'] if r else None)} |")
    text += ['',
        'Availability is sampled at approximately 2 Hz. The first initialized epoch uses original log time. Reference comparisons interpolate only within 0.2-second reference gaps. GNSS agreement uses time-aligned fixes, not stale held positions.','',
        '### Rejected input','',
        '`covered-parking-urban-drive-data-gap-001` contains 87,003 duplicate adjacent IMU timestamps, including differing measurements at the same timestamp. It needs a documented timestamp-repair or aggregation policy. This evaluator does not sort, discard, or invent sub-sample times to make it pass.','',
        '## Road-map experiment','',
        'An independent cached OpenStreetMap extract is compiled to canonical directed graph tiles using shared node connectivity and one-way tags. It is not generated from the reference track. The current map may differ from the recording date; default widths are unsurveyed. Turn restriction relations, conditional access, lane geometry and surveyed elevation are not implemented.','',
        'The bounded bank applies finite-width corridor moments in the full 18-state covariance and five persistent map-translation alternatives. No snapped coordinate is returned as a fresh GNSS observation. The published shadow output remains map-free; only the separately named experimental candidate is corrected.','',
        '| Recording | Latch fraction of usable/map-covered epochs | Map-free discrepancy RMSE (m) | Candidate discrepancy RMSE (m) |',
        '|---|---:|---:|---:|',
    ]
    for x in maps:
        b,c=x['base_reference_discrepancy'],x['candidate_reference_discrepancy']
        text.append(f"| {x['id']} | {100*x['latch_fraction_available']:.1f}% | {number(b['rmse_m'] if b else None)} | {number(c['rmse_m'] if c else None)} |")
    text += ['',
        'The clean reference-bearing runs show essentially no coordinate benefit. This null result is retained, not used to tune widths until plots improve. The iOS candidate can move by metres, but no truth stream establishes whether that helps.','',
        f"At every sampled map epoch the map-free filter matched a separate native replay exactly: {all(x['map_free_exactly_matches_independent_replay'] for x in maps)}. No negative horizontal candidate covariance eigenvalues occurred: {all(x['negative_position_covariance_epochs']==0 for x in maps)}.",'',
        '### Controlled perturbations','',
        'The second short-loop recording was replayed with either all GNSS fixes removed at seconds 90–120 or a 20 m north position bias ramped in/out over five seconds in that interval. IMU, velocity and stated GNSS uncertainty were unchanged. These are injected stresses, not natural receiver errors, independent trials or held-out hardware tests. The original reference remains evaluation-only.','',
        '| Stress (90–120 s) | Map-free reference discrepancy RMSE (m) | Candidate reference discrepancy RMSE (m) |',
        '|---|---:|---:|',
    ]
    for x in stress:
        text.append(f"| {x['input_stress']} | {x['stress_window_base_discrepancy']['rmse_m']:.3f} | {x['stress_window_candidate_discrepancy']['rmse_m']:.3f} |")
    text += ['',
        'Geometry reduced discrepancy in this outage example, but barely helped the position-bias case. The pinned upstream EKF does not have the planar prototype’s augmented colored GNSS position/velocity bias states. Its original covariance and rejection behavior remain intact; persistent urban multipath is not solved by this integration.','',
        '## Computation and memory','',
        f"A complete native clone occupies {good[0]['native_instance_bytes']:,} bytes (`size_of::<Bridge>()`). At 24 alternatives plus fallback, native instances alone use {25*good[0]['native_instance_bytes']:,} bytes, excluding graph tiles, Python state, histories, allocation and stack. The evaluation also runs an independent control. This is exact runtime reuse for research, not a RAM-optimized firmware design.",'',
        f"Baseline replay loops took {sum(x['wall_runtime_s'] for x in good):.2f} s on this host, excluding decompression/audit. Four map runs took {min(x['wall_runtime_s'] for x in maps):.2f}–{max(x['wall_runtime_s'] for x in maps):.2f} s each. One-Hz map-step p95 ranged from {min(x['map_step_ms']['p95'] for x in maps):.2f} to {max(x['map_step_ms']['p95'] for x in maps):.2f} ms. These are desktop measurements, not MCU latency, stack, RAM or power qualification.",'',
        '## Remaining limits','',
        'Inference is approximate: sequential Gaussian interval moments, locally linearized polylines, bounded hypotheses, finite static map-offset prior, and broad-mixture branch scoring. The score is not the exact marginal likelihood of upstream’s gated GNSS/NHC update. No calibrated free-motion mode is implemented. Map histories grow with travel and lookup is not MCU optimized.','',
        f"Peak discarded conditional branch mass was {max(x['max_pruned_mass'] for x in maps):.1%}. The policy abstains at that epoch, but does not bound cumulative loss of a correct alternative. A 97% retained-group weight is not a calibrated 97% probability of the correct road. OSM-way groups do not certify a unique directed segment or lane.",'',
        'The integration inherits the pinned runtime’s reset convention, including its attitude-only reset implementation; it does not independently requalify every upstream covariance approximation. Automatic alignment delays, persistent GNSS error modeling, branch recovery, independently labelled routes, surveyed reference data, map validity and MCU qualification remain explicit work.','',
        '## Recommendation','',
        'Keep this integration in shadow/output-only mode. Prioritize a calibrated or warm-start mount path for a permanently installed device, then evaluate persistent GNSS error modeling and safe hypothesis recovery on independently labelled urban-canyon recordings. Do not promote geometric feedback based on the clean-run null result or one injected-outage improvement.','',
        'See `README.md` for reproduction, `evidence/dataset-audit.json` for input findings, per-run CSV/JSON for every result, and `map_cache/provenance.json` for the exact OSM snapshot. Road data © OpenStreetMap contributors, ODbL 1.0; map terms are separate from code licensing.'
    ]
    destination.write_text('\n'.join(text)+'\n')


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--results',type=Path,required=True);p.add_argument('--datasets',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();write_report(a.results,a.datasets,a.output)
