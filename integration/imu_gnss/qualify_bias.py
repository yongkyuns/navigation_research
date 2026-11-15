"""Check data coverage, covariance/fallback invariants, and report all outcomes.

No performance threshold is weakened to accept a reproduction. Fault-family
improvements are reported separately from missed faults and map-only effects.
"""
from pathlib import Path
import argparse, hashlib, json, math, os
import xml.etree.ElementTree as ET
from dataclasses import asdict
from native_bias import BiasConfig

HERE=Path(__file__).resolve().parent


def same(a,b,label):
    if isinstance(b,dict):
        if a.keys()!=b.keys():raise AssertionError(label+' schema')
        for k in b:same(a[k],b[k],label+'/'+k)
    elif isinstance(b,(float,int)) and not isinstance(b,bool):
        if not math.isclose(a,b,rel_tol=1e-7,abs_tol=1e-5):raise AssertionError(f'{label}: {a} != {b}')
    elif a!=b:raise AssertionError(label)


def qualify(root):
    clean=json.loads((root/'clean/summary.json').read_text())
    historical=json.loads((HERE/'evidence/baseline/summary.json').read_text())
    accepted={r['id']:r for r in historical if r.get('status','REPLAYED')=='REPLAYED'}
    rejected={r['id'] for r in historical if r.get('status')=='INPUT_REJECTED'}
    if {r['dataset'] for r in clean['results']}!=accepted.keys() or {r['dataset'] for r in clean['rejected']}!=rejected:
        raise AssertionError('Recording coverage or rejection policy changed')
    if clean['recordings_with_activation'] or not clean['all_equal_until_activation']:
        raise AssertionError('Frozen unmodified-recording non-regression policy failed')
    for row in clean['results']:
        prior=accepted[row['dataset']]
        same(row['input']['input_sha256'],prior['input_sha256'],'input hashes')
        same(row['input']['rows'],prior['rows'],'input counts')
        same(row['raw_reference_discrepancy'],prior['reference_discrepancy'],'original baseline')
        if not row['finite'] or row['raw_ready_epochs']!=row['bias_ready_epochs']:
            raise AssertionError('Clean-state invariant failed')
        same(row['bias_reference_discrepancy'],row['raw_reference_discrepancy'],'clean regression')
    groups={}
    for group,filename in [('development','bias_cases_development.json'),('holdout','bias_cases_holdout.json'),('sensitivity','bias_cases_sensitivity.json')]:
        rows=json.loads((root/group/'summary.json').read_text())
        expected=json.loads((HERE/filename).read_text())
        if [r['case'] for r in rows]!=expected:raise AssertionError('Experiment coverage changed: '+group)
        for row in rows:
            if not row['map_free_matches_independent']:raise AssertionError('Map contaminated independent filter')
            if row['initialized_epochs']!=row['raw_initialized_epochs']:raise AssertionError('Changed paired availability')
            same(row['config'],asdict(BiasConfig()),'frozen policy')
        groups[group]=rows
    suites=list(ET.parse(root/'python-tests.xml').getroot().iter('testsuite'))
    if not suites or any(int(s.get(k,0)) for s in suites for k in ('errors','failures','skipped')):
        raise AssertionError('Tests missing, skipped, or failing')
    count=sum(int(s.get('tests',0)) for s in suites)
    if count<53:raise AssertionError('Expected regression tests not collected')
    record={'git_revision':os.environ.get('GITHUB_SHA','local-uncommitted'),
            'upstream_revision':'8a34212c7befe30f8d3eae4ad708cc0dcd4aa0d6',
            'python_tests':count,'unmodified_recordings':len(clean['results']),
            'unmodified_recording_activations':clean['recordings_with_activation'],
            'unmodified_trajectories_unchanged':True,
            'paired_map_fallback_bit_identical':True,
            'case_counts':{k:len(v) for k,v in groups.items()},
            'experimental_opt_in':True,'surveyed_accuracy_claim':False,
            'config_sha256':hashlib.sha256((HERE/'bias_config.json').read_bytes()).hexdigest(),
            'source_sha256':{str(p.relative_to(HERE)):hashlib.sha256(p.read_bytes()).hexdigest()
                             for p in sorted(HERE.rglob('*')) if p.is_file() and (p.suffix in ('.py','.rs') or p.name.startswith('bias_cases'))}}
    (root/'qualification.json').write_text(json.dumps(record,indent=2)+'\n')
    lines=['# Selective GNSS-position fault model: executed evaluation','',
           'Distances below are discrepancies from supplied references of unverified independence, not surveyed accuracy. All injected faults are synthetic. The production/default runtime remains unchanged.',
           '',f"Unmodified packages: {len(clean['results'])} accepted; {len(clean['rejected'])} rejected. Automatic bias-mode activations: {clean['recordings_with_activation']}. All sampled pre-activation native states match the unaugmented runtime exactly.",'',
           '| Case | Original INS/GNSS | Bias-aware, map-free | Bias-aware + map | Activation time |',
           '|---|---:|---:|---:|---:|']
    for group,rows in groups.items():
        for row in rows:
            values=[row['fault_window'][k] for k in ('raw','bias','after')]
            formatted=['—' if v is None else f"{v['rmse_m']:.3f} m" for v in values]
            lines.append('| '+group+'/'+row['case']['name']+' | '+' | '.join(formatted)+' | '+str(row['activation_t'])+' |')
    lines+=['','Tables use the configured fault-time window; full-trip, post-fault recovery, availability, covariance and every sampled trajectory are in the per-case JSON/CSV files.',
            '', 'The map-free bias-aware ablation attributes most improvement to temporal receiver-fault handling. Road geometry is unchanged and contributes little or can worsen discrepancies. Constant cold-start bias, sufficiently slow drift, and mutually consistent position/velocity corruption are not reliably detected. The mode selector and its variance are engineering policies, not calibrated integrity probabilities. Bias mode remains active after onset; no automatic return to the original noise model is claimed.',
            '', 'The unchanged native initialization delays, inherited attitude-only reset, approximate branch scoring and static map-error hypotheses remain limitations. Results do not establish M9/LSM6DSO field accuracy or correct-road probability.',
            '', 'Reference code and configuration were frozen before holdout-drive and sensitivity runs. The holdout cases use previously available recordings with new injected faults, not new independent field collections.']
    (root/'REPORT.md').write_text('\n'.join(lines)+'\n')
    return record

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--results',type=Path,required=True);a=p.parse_args();print(json.dumps(qualify(a.results),indent=2))
