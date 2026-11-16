"""Reproducible *synthetic* comparison of repeated biased road pseudomeasurements.

This is not a native INS/GNSS replay, road association evaluation, or surveyed
accuracy study. Truth and map offset are sampled and visible to the evaluator
ONLY. Both naive and Schmidt methods receive exactly the same sensors and map.
"""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np
from map_correlation import SchmidtMapReference

NAMES=('GNSS only','naive independent centerline','independent inflated noise',
       'correlated Schmidt map offset')


def evaluate(*, trials=180, seconds=65, seed=20261116):
    if not 1<=trials<=100000 or not 3<=seconds<=3600:
        raise ValueError('Unreasonable simulation size')
    rng=np.random.default_rng(seed)
    F=np.array([[1.,1.],[0.,1.]])
    Q=np.array([[.0125/3,.0125/2],[.0125/2,.0125]])
    H=np.array([[1.,0.]])
    J=np.array([[1.]])
    init_P=np.diag([100.,1.])
    map_sigma=4.
    geom_noise=.3
    gps_sigma=2.
    accum={name:dict(errors=[],nees=[],map_steps=[],post_var=[],map_update_count=[]) for name in NAMES}
    for trial in range(trials):
        map_offset=float(rng.normal(scale=map_sigma))
        truth=rng.multivariate_normal([0.,0.],init_P)
        filters={name:SchmidtMapReference([0.,0.],init_P,
            [[map_sigma**2]] if name==NAMES[-1] else [[0.]],map_id='synthetic-single-road')
            for name in NAMES}
        for t in range(seconds):
            if t:
                truth=F@truth+rng.multivariate_normal(np.zeros(2),Q)
                for f in filters.values(): f.predict(F,Q)
            z_gps=float(truth[0]+rng.normal(scale=gps_sigma))
            z_map=float(truth[0]+map_offset+rng.normal(scale=geom_noise))
            for name,f in filters.items():
                f.sensor([z_gps],H,[[gps_sigma**2]])
                if name==NAMES[1]:
                    update=f.road([z_map],H,np.zeros((1,1)),[[geom_noise**2]],
                                  observation_key=f'cell-{t}')
                elif name==NAMES[2]:
                    update=f.road([z_map],H,np.zeros((1,1)),[[geom_noise**2+map_sigma**2]],
                                  observation_key=f'cell-{t}')
                elif name==NAMES[3]:
                    update=f.road([z_map],H,J,[[geom_noise**2]],
                                  observation_key=f'cell-{t}')
                else:
                    update=None
                if update is not None and update.applied:
                    accum[name]['map_steps'].append(abs(update.delta_state[0]))
        for name,f in filters.items():
            pos=float(f.x[0]-truth[0]);var=float(f.P[0,0])
            accum[name]['errors'].append(pos)
            accum[name]['nees'].append(pos*pos/var)
            accum[name]['post_var'].append(var)
            accum[name]['map_update_count'].append(f.map_updates)
    result=dict(experiment='Synthetic linear lateral position with shared, fixed 4 m map offset and independent .3 m local noise',
        source='original controlled simulation, not an actual dataset',
        seed=seed,trials=trials,seconds=seconds,
        gaussian_assumptions=dict(map_shift_sigma_m=map_sigma,local_geometry_sigma_m=geom_noise,
                                  independent_gnss_sigma_m=gps_sigma),
        results={})
    for name,s in accum.items():
        errors=np.array(s['errors']); variances=np.array(s['post_var'])
        steps=np.array(s['map_steps'])
        result['results'][name]=dict(
            position_rmse_m=float(np.sqrt(np.mean(errors**2))),
            mean_absolute_error_m=float(np.mean(np.abs(errors))),
            mean_predicted_sigma_m=float(np.mean(np.sqrt(variances))),
            mean_position_nees=float(np.mean(s['nees'])),
            map_updates_per_trial=int(np.mean(s['map_update_count'])),
            p95_road_update_step_m=float(np.percentile(steps,95)) if len(steps) else 0.,
            largest_road_update_step_m=float(np.max(steps)) if len(steps) else 0.)
    return result


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',required=True,type=Path)
    p.add_argument('--trials',type=int,default=180)
    p.add_argument('--seconds',type=int,default=65)
    a=p.parse_args()
    res=evaluate(trials=a.trials,seconds=a.seconds)
    a.output.parent.mkdir(parents=True,exist_ok=True)
    a.output.write_text(json.dumps(res,indent=2,sort_keys=True)+'\n')
    print(json.dumps(res,indent=2,sort_keys=True))

if __name__=='__main__':main()
