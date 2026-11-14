"""Replay the explicit adapter contract (not a UBX binary or raw-IMU decoder).

python -m routefusion.replay results/parallel_bias_example --output replay.jsonl
One local EN frame, aligned timestamps, gravity-compensated body horizontal
acceleration, and valid PVT covariance are required. See docs/INTEGRATION.md.
"""
import argparse,json
from pathlib import Path
import numpy as np
from .filter import PlanarFilter,PVT
from .tiles import TileStore
from .tracker import RouteTracker

def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('directory'); ap.add_argument('--output',default='replay.jsonl')
    args=ap.parse_args(); root=Path(args.directory)
    initial=json.loads((root/'initial.json').read_text())
    tiles=TileStore.load(root/'tiles'); loaded=tiles.loaded.copy()
    tracker=RouteTracker(PlanarFilter(np.array(initial['x']),np.array(initial['P'])),tiles)
    with open(root/'events.jsonl') as f,open(args.output,'w') as out:
        for line in f:
            e=json.loads(line)
            if e['kind']=='imu':
                tracker.predict(np.array(e['accel_body']),e['gyro_z'],e['dt'])
            elif e['kind']=='epoch':
                tiles.loaded=loaded.copy() if e.get('map_available',True) else set()
                z=e['pvt']
                pvt=None if z is None else PVT(z['epoch'],z['t'],np.array(z['position']),
                    np.array(z['velocity']),np.array(z['covariance']))
                result=tracker.step(e['t'],pvt)
                out.write(json.dumps(dict(t=e['t'],status=result.status,
                    position=result.position.tolist(),covariance=result.covariance.tolist(),
                    route_group=result.route_group,route_edge=result.route_edge,
                    conditional_route_weights=result.conditional_route_weights,
                    fallback_position=result.fallback_position.tolist(),
                    pruned_mass=result.pruned_mass))+'\n')
            else: raise ValueError('unknown event kind: '+str(e['kind']))

if __name__=='__main__': main()
