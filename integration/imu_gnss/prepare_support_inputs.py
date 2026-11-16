"""Rebuild causal seed associations with the unchanged original RoadBank.

Produces the compact playback-data schema consumed by evaluate_support.py. This
is an independent before arm, never a matcher fed the newly corrected positions.
Raw recorded GNSS is included for visualization; reference trajectories are not.
"""
import argparse
import csv
import hashlib
import json
from pathlib import Path
from datasets import load_inputs
from evaluate_map import run as run_original
from native import Library
from osm_graph import compile_osm

IDS=('ios-mixed-road-drive-001','urban-short-turn-loop-nominal-001',
     'urban-short-turn-loop-nominal-002','urban-stop-go-tight-turns-nominal-001')


def prepare(library,datasets,map_cache,output):
    output=Path(output);output.mkdir(parents=True,exist_ok=False)
    roads=compile_osm(Path(map_cache)/'osm-roads.json.gz',Path(map_cache)/'provenance.json')
    lib=Library(library)
    doc=dict(schema=1,roads=[[e.id,e.group,e.half_width,e.points.tolist(),list(e.successors),
                             e.layer,'',''] for e in roads.store.edges.values()],
             loaded_tiles=[list(k) for k in sorted(roads.store.loaded)],datasets=[],
             provenance=dict(map_bbox=roads.bbox.tolist(),map_response_sha256=roads.metadata['response_sha256'],
                 library_sha256=hashlib.sha256(Path(library).read_bytes()).hexdigest(),
                 association='Fresh unmodified original RoadBank; no corrected-state feedback'))
    for name in IDS:
        directory=Path(datasets)/name
        imu,gnss,provenance=load_inputs(directory)
        run_original(lib,directory,roads,output/'original-association')
        with (output/'original-association'/(name+'.csv')).open() as f:
            rows=list(csv.DictReader(f))
        records=[]
        start=min(float(imu[0,0]), float(gnss[0,0]))
        for r in rows:
            ready=r['ready']=='True';latched=r['latched']=='True'
            records.append([float(r['t_s'])-start,float(r['base_e']) if ready else None,
                float(r['base_n']) if ready else None,float(r['candidate_e']) if ready else None,
                float(r['candidate_n']) if ready else None,r['status'],latched,
                r['edge'] if latched else None,float(r['top_weight']),int(r['branches']),
                float(r['pruned_mass']),json.loads(r['junction_phases']),ready,r['usable']=='True',
                float(r['correction_m']) if ready else None])
        gps=roads.frame.project(gnss[:,1:4]);grows=[]
        for raw,p in zip(gnss,gps):
            grows.append([float(raw[0])-start,float(p[0]),float(p[1]),float(raw[1]),float(raw[2])])
        doc['datasets'].append(dict(id=name,label=name,kind='map',origin=roads.frame.origin.tolist(),
            original_start_t_s=start,duration=float(imu[-1,0])-start,sample_period_s=1.,records=records,
            gnss=grows,input_sha256=provenance['input_sha256']))
    target=output/'playback-data.json'
    target.write_text(json.dumps(doc,separators=(',',':'),allow_nan=False)+'\n')
    return target


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    for name in ('library','datasets','map-cache','output'):
        p.add_argument('--'+name,type=Path,required=True)
    a=p.parse_args();print(prepare(a.library,a.datasets,a.map_cache,a.output))
