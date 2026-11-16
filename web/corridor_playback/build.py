"""Build an offline comparison viewer from the exact reviewed PR #13 CI evidence.

This reader runs no filter. It preserves numerical values and separates sampled
positions, road association, geometric support and actual measurement updates.
"""
from __future__ import annotations
import argparse
import hashlib
import html
import json
import math
from pathlib import Path
import zipfile

ARCHIVE_SHA256 = '21dbc3199646e78438af1a720bf759fb19a690250ab2494e0333348989670ad6'
HEAD = 'b3ee9b3b3a704a9b12b51e4d394af99fe1b42399'
MERGE = 'dd3b08d32958c43825e6ce4a6382426f77a9b9d5'
RUN = 43938102885
ARTIFACT = 14264310454
LABELS = {
 'ios-mixed-road-drive-001': 'Mixed-road drive',
 'urban-short-turn-loop-nominal-001': 'Urban loop 001',
 'urban-short-turn-loop-nominal-002': 'Urban loop 002',
 'urban-stop-go-tight-turns-nominal-001': 'Stop / go & tight turns',
}
MODELS = ('disabled', 'independent', 'correlated')


def digest(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def load_evidence(archive: Path) -> dict:
    raw = archive.read_bytes()
    if digest(raw) != ARCHIVE_SHA256:
        raise ValueError('Archive is not the reviewed current-head CI artifact')
    with zipfile.ZipFile(archive) as z:
        names = z.namelist()
        if len(names) != len(set(names)):
            raise ValueError('Duplicate archive members')
        manifest = json.loads(z.read('SHA256SUMS.json'))
        if len(manifest) != 27 or set(names) != set(manifest) | {'SHA256SUMS.json'}:
            raise ValueError('Incomplete artifact manifest')
        for name, expected in manifest.items():
            if digest(z.read(name)) != expected:
                raise ValueError('Artifact member changed: ' + name)
        proof = json.loads(z.read('first/verification.json'))
        if (proof['execution_commit'] != HEAD or proof['complete'] is not True
                or proof['pinned_sources_verified'] != 37
                or proof['sigma_m'] != .25 or proof['map_sigma_m'] != 1.5):
            raise ValueError('Unexpected qualification provenance')
        if z.read('first/verification.json') != z.read('repeat/verification.json'):
            raise ValueError('Qualification repeat differs')
        seeds = json.loads(z.read('seeds/playback-data.json'))
        if digest(z.read('seeds/playback-data.json')) != proof['playback_sha256']:
            raise ValueError('Road support input changed')
        source = {d['id']: d for d in seeds['datasets']}
        if len(source) != 4 or set(source) != set(LABELS):
            raise ValueError('Expected four unique mapped recordings')
        summaries = {r['dataset']: r for r in proof['recordings']}
        if set(summaries) != set(LABELS):
            raise ValueError('Recording qualification is incomplete')
        road_ids = {r[0] for r in seeds['roads']}
        if len(road_ids) != len(seeds['roads']):
            raise ValueError('Duplicate road IDs')
        recordings = []
        for name, label in LABELS.items():
            content = z.read('first/' + name + '.json')
            if content != z.read('repeat/' + name + '.json'):
                raise ValueError('Trajectory repeat differs: ' + name)
            result = json.loads(content)
            seed, summary = source[name], summaries[name]
            rows, notes = result['rows'], result['diagnostics']
            if len(rows) != len(seed['records']) or len(notes) != len(rows):
                raise ValueError('Diagnostic/sample alignment changed')
            initialized = 0
            for i, (row, note, original) in enumerate(zip(rows, notes, seed['records'])):
                t = row['elapsed_s']
                if not math.isfinite(t) or (i and t <= rows[i-1]['elapsed_s']):
                    raise ValueError('Nonchronological output')
                if t != original[0] or row['original_edge'] not in road_ids | {None}:
                    raise ValueError('Wrong sample time or road identity')
                original_position = original[1:3] if original[12] else None
                if row['raw_position'] != original_position:
                    raise ValueError('Raw position differs from original association input')
                if row['outputs']['disabled']['position'] != row['raw_position']:
                    raise ValueError('Map-disabled trajectory differs from raw EKF')
                if any(row['outputs'][m]['ready'] != bool(original[12]) for m in MODELS):
                    raise ValueError('Lost continuous output')
                for model in MODELS:
                    out = row['outputs'][model]
                    if out['ready']:
                        if (out['position'] is None or len(out['position']) != 2
                                or not all(math.isfinite(v) for v in out['position'])):
                            raise ValueError('Invalid initialized output')
                    elif out['position'] is not None:
                        raise ValueError('Fabricated pre-initialization position')
                initialized += bool(original[12])
            for model in MODELS:
                if initialized != summary['models'][model]['counts']['initialized_outputs']:
                    raise ValueError('Output coverage differs from qualification')
                if sum(bool(n[model]['applied']) for n in notes) != summary['models'][model]['map_updates']:
                    raise ValueError('Update coverage differs from qualification')
            if not all(summary[k] is True for k in ('raw_navigation_bit_identical',
                    'inputs_unmodified', 'native_instances_preserved', 'skips_bit_identical')):
                raise ValueError('Native qualification failed')
            # Keep full numbers, diagnostics and provenance. No rounding, snaps,
            # interpolation, trajectory blending, or extra observation is used.
            recordings.append(dict(id=name, label=label, origin=seed['origin'],
                gnss=seed['gnss'], rows=rows, diagnostics=notes, summary=summary,
                input_sha256=seed['input_sha256'], source=result['source']))
        totals = dict(sensor_events=sum(r['sensor_events'] for r in proof['recordings']),
                      exported_epochs=sum(len(d['rows']) for d in recordings),
                      initialized_outputs=sum(d['summary']['models']['correlated']['counts']['initialized_outputs'] for d in recordings))
        if totals != dict(sensor_events=159075, exported_epochs=1569, initialized_outputs=1402):
            raise ValueError('Unexpected replay coverage')
        return dict(schema=1, datasets=recordings, roads=seeds['roads'],
            provenance=dict(archive_sha256=ARCHIVE_SHA256, ci_head=HEAD, merge_commit=MERGE,
                run_id=RUN, artifact_id=ARTIFACT, member_sha256=manifest,
                map=seeds['provenance'], sigma_m=proof['sigma_m'], map_sigma_m=proof['map_sigma_m'],
                totals=totals, source_sha256=proof['source_sha256'],
                approximation=proof['approximation'], no_new_estimator_execution=True))


def safe_json(data) -> str:
    return json.dumps(data, separators=(',', ':'), allow_nan=False).replace('<', '\\u003c').replace('&', '\\u0026')


def fallback(data) -> str:
    """SVG overview still works when an embedded preview disables JavaScript."""
    rows = data['datasets'][0]['rows']
    points = [r['raw_position'] for r in rows if r['raw_position'] is not None]
    x0,x1 = min(p[0] for p in points), max(p[0] for p in points)
    y0,y1 = min(p[1] for p in points), max(p[1] for p in points)
    s = min(940/max(1.,x1-x0), 360/max(1.,y1-y0))
    def xy(p): return f'{500+(p[0]-(x0+x1)/2)*s:.3f},{210-(p[1]-(y0+y1)/2)*s:.3f}'
    svg = ['<svg viewBox="0 0 1000 420" role="img" aria-label="Static mixed-road comparison overview"><rect width="1000" height="420" fill="#0b1621"/>']
    for road in data['roads']:
        ps=road[3]
        if any(x0-80<=p[0]<=x1+80 and y0-80<=p[1]<=y1+80 for p in ps):
            svg.append('<polyline points="'+' '.join(xy(p) for p in ps)+'" fill="none" stroke="#233c50" stroke-width="1"/>')
    for model, color in [('disabled','#6ba9ff'),('independent','#70d5ed'),('correlated','#61e5b3')]:
        ps=[r['outputs'][model]['position'] for r in rows if r['outputs'][model]['ready']]
        svg.append('<polyline points="'+' '.join(xy(p) for p in ps)+'" fill="none" stroke="'+color+'" stroke-width="2"/>')
    svg.append('</svg>')
    return ''.join(svg)


def build(archive: Path, destination: Path) -> dict:
    data = load_evidence(archive)
    here = Path(__file__).parent
    template = (here/'viewer.html').read_text()
    substitutions = {'STYLE':(here/'viewer.css').read_text(), 'SCRIPT':(here/'viewer.js').read_text(),
                     'DATA':safe_json(data), 'FALLBACK':fallback(data)}
    for key,value in substitutions.items():
        token='@@'+key+'@@'
        if template.count(token)!=1: raise ValueError('Missing template slot: '+key)
        template=template.replace(token,value)
    destination.parent.mkdir(parents=True,exist_ok=True)
    destination.write_text(template)
    return dict(html_sha256=digest(destination.read_bytes()), size_bytes=destination.stat().st_size,
                artifact_sha256=ARCHIVE_SHA256, totals=data['provenance']['totals'],
                no_estimator_run=True, no_numerical_transform=True)


if __name__=='__main__':
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--artifact',type=Path,required=True)
    ap.add_argument('--output',type=Path,required=True)
    args=ap.parse_args()
    print(json.dumps(build(args.artifact,args.output),indent=2))
