"""Deterministic synthetic stress tests. NOT measured M9/LSM6DSO performance.

Development seeds are 0..4; the default evaluation uses fresh seeds 100..109.
Measurements have deliberately mismatched piecewise GNSS bias/outliers rather
than being sampled entirely from the estimator's own Gaussian model.
"""
from __future__ import annotations
from dataclasses import dataclass
from pathlib import Path
import argparse,csv,json,math,time
import numpy as np
from .filter import N,PXY,VXY,YAW,BA,BG,GB,MB,PlanarFilter,FilterConfig,PVT
from .tiles import Edge,TileStore
from .tracker import RouteTracker

SCENARIOS=['parallel_bias','turn_outage','offroad_leave','ambiguous_parallel',
           'map_offset','missing_tiles']

@dataclass
class Case:
    name: str
    duration: float=120.0
    dt: float=0.2
    speed: float=8.0

    def truth(self,t):
        s=self.speed*t
        turn=self.name in ('turn_outage','offroad_leave')
        start=300.0 if self.name=='turn_outage' else 320.0
        radius=30.0
        if not turn or s<=start:
            p=np.array([s,0.0]); psi=0.; omega=0.
        elif s<=start+radius*math.pi/2:
            psi=(s-start)/radius
            p=np.array([start+radius*math.sin(psi),radius*(1-math.cos(psi))])
            omega=self.speed/radius
        else:
            psi=math.pi/2
            p=np.array([start+radius,radius+s-start-radius*math.pi/2])
            omega=0.
        v=self.speed*np.array([math.cos(psi),math.sin(psi)])
        a=np.array([0.,self.speed*omega])
        group=None if self.name=='offroad_leave' and t>40 else 'main'
        return p,v,psi,a,omega,group

    def graph(self):
        store=TileStore()
        offset=6.0 if self.name=='map_offset' else 0.0
        if self.name=='turn_outage':
            r=30.; ang=np.linspace(0,np.pi/2,21)
            arc=np.c_[300+r*np.sin(ang),r*(1-np.cos(ang))]
            store.add(Edge('main_0',np.array([[0,0],[300,0]]),('main_curve','east_branch'),group='main'))
            store.add(Edge('main_curve',arc,('main_north',),group='main'))
            store.add(Edge('main_north',np.array([[330,30],[330,1000]]),group='main'))
            store.add(Edge('east_branch',np.array([[300,0],[1400,0]]),group='east_branch'))
        else:
            # A true topological transition at x=512 also exercises tile seams.
            store.add(Edge('main_0',np.array([[0,offset],[512,offset]]),('main_1',),group='main'))
            store.add(Edge('main_1',np.array([[512,offset],[1400,offset]]),group='main'))
        store.add(Edge('parallel_0',np.array([[0,26+offset],[512,26+offset]]),('parallel_1',),group='parallel'))
        store.add(Edge('parallel_1',np.array([[512,26+offset],[1400,26+offset]]),group='parallel'))
        store.cover([-256,-256],[1536,1280])
        return store

    def gnss_bias(self,t):
        if self.name=='ambiguous_parallel': return np.array([0.,13.])
        if self.name == 'parallel_bias':
            rise=np.clip((t-15)/10,0,1); fall=np.clip((110-t)/15,0,1)
            return np.array([4.0,26.0])*min(rise,fall)
        if self.name=='turn_outage' and 75<t<105: return np.array([20.,4.])
        return np.zeros(2)

    def gnss_available(self,t):
        return not(self.name=='turn_outage' and 25<=t<=70)


def initial_filter(case,rng):
    p,v,psi,*_=case.truth(0)
    x=np.zeros(N)
    x[PXY]=p+rng.normal(0,1.2,2)
    x[VXY]=v+rng.normal(0,0.15,2)
    x[YAW]=psi+rng.normal(0,math.radians(1.0))
    std=np.array([3.,3.,0.5,0.5,math.radians(5),0.06,0.06,
                  math.radians(0.2),15.,15.,3.,3.,0.5,0.5])
    if case.name=='ambiguous_parallel':
        x[:]=0; x[PXY]=[0,13]; x[VXY]=[case.speed,0]
        std[PXY]=20
    return PlanarFilter(x,np.diag(std**2))


def run_case(name,seed,output_dir=None,write_inputs=False):
    case=Case(name)
    rng=np.random.default_rng(seed)
    store=case.graph()
    nav=initial_filter(case,rng)
    initial=dict(x=nav.x.tolist(),P=nav.P.tolist())
    tracker=RouteTracker(nav,store)
    ba=np.array([0.008,-0.012])+rng.normal(0,0.002,2)
    bg=math.radians(0.03)+rng.normal(0,math.radians(0.005))
    rows=[]; inputs=[]; saved_loaded=store.loaded.copy()
    start=time.perf_counter()
    steps=int(round(case.duration/case.dt))
    for k in range(1,steps+1):
        t=k*case.dt
        _,_,_,acc,gyro,_=case.truth(t-case.dt/2)
        if name=='ambiguous_parallel':
            am=np.zeros(2); wm=0.
        else:
            am=acc+ba+rng.normal(0,0.04/math.sqrt(case.dt),2)
            wm=gyro+bg+rng.normal(0,math.radians(0.06)/math.sqrt(case.dt))
        tracker.predict(am,wm,case.dt)
        if write_inputs:
            inputs.append(dict(kind='imu',t=t,dt=case.dt,accel_body=am.tolist(),gyro_z=float(wm)))
        if k%5: continue  # 1 Hz map/PVT events; inertial packets at 5 Hz
        p,v,psi,_,_,true_group=case.truth(t)
        z=None
        if case.gnss_available(t):
            pos=p+case.gnss_bias(t)+rng.normal(0,2.5,2)
            vel=v+rng.normal(0,0.15,2)
            # Adversarial PVT inconsistency: a persistent north-velocity bias
            # accompanies the urban position bias. This is not a hardware spec.
            if name == 'parallel_bias':
                vel[1] += 0.35*case.gnss_bias(t)[1]/26.0
            if name=='ambiguous_parallel': pos[1]=13.; vel[1]=0.
            elif rng.random()<0.025:
                pos+=rng.normal(0,40,2)
            z=PVT(int(round(t)),t,pos,vel,np.diag([2.5**2]*2+[0.15**2]*2))
        if name=='missing_tiles' and 45<=t<=70: store.loaded.clear()
        else: store.loaded=saved_loaded.copy()
        out=tracker.step(t,z)
        if write_inputs:
            inputs.append(dict(kind='epoch',t=t,map_available=bool(store.loaded),
                pvt=None if z is None else dict(epoch=z.epoch,t=z.t,position=z.position.tolist(),
                velocity=z.velocity.tolist(),covariance=z.covariance.tolist())))
        nearest=store.candidates(out.fallback_position,200)
        naive=min(nearest,key=lambda e:e.project(out.fallback_position).distance).group if nearest else None
        raw_nearest=store.candidates(z.position,200) if z is not None else []
        raw_group=min(raw_nearest,key=lambda e:e.project(z.position).distance).group if raw_nearest else None
        d=out.position-p; db=out.fallback_position-p
        normal=np.array([-math.sin(psi),math.cos(psi)])
        try: nees=float(d@np.linalg.solve(out.covariance,d))
        except np.linalg.LinAlgError: nees=float('nan')
        rows.append(dict(scenario=name,seed=seed,t=t,true_e=float(p[0]),true_n=float(p[1]),
            raw_gnss_e=None if z is None else float(z.position[0]),raw_gnss_n=None if z is None else float(z.position[1]),
            fallback_e=float(out.fallback_position[0]),fallback_n=float(out.fallback_position[1]),
            output_e=float(out.position[0]),output_n=float(out.position[1]),
            status=out.status,true_group=true_group,route_group=out.route_group,naive_group=naive,raw_nearest_group=raw_group,
            top_weight=out.top_weight,second_weight=out.second_weight,
            error_m=float(np.linalg.norm(d)),fallback_error_m=float(np.linalg.norm(db)),
            cross_error_m=abs(float(normal@d)),fallback_cross_error_m=abs(float(normal@db)),
            nees_2d=nees,covered95=bool(nees<=5.991464547),
            branch_count=out.branch_count,pruned_mass=out.pruned_mass,
            map_available=bool(store.loaded),gnss_available=z is not None))
    runtime=time.perf_counter()-start
    mapped=[r for r in rows if r['true_group'] is not None and r['map_available']]
    latched=[r for r in mapped if r['status']=='LATCHED']
    offmap=[r for r in rows if r['true_group'] is None]
    metric=dict(scenario=name,seed=seed,epochs=len(rows),runtime_s=runtime,
        output_rmse_m=float(np.sqrt(np.mean([r['error_m']**2 for r in rows]))),
        fallback_rmse_m=float(np.sqrt(np.mean([r['fallback_error_m']**2 for r in rows]))),
        output_p95_m=float(np.percentile([r['error_m'] for r in rows],95)),
        fallback_p95_m=float(np.percentile([r['fallback_error_m'] for r in rows],95)),
        output_cross_p95_m=float(np.percentile([r['cross_error_m'] for r in rows],95)),
        fallback_cross_p95_m=float(np.percentile([r['fallback_cross_error_m'] for r in rows],95)),
        latch_availability=len(latched)/max(len(mapped),1),
        correct_when_latched=sum(r['route_group']==r['true_group'] for r in latched)/len(latched) if latched else None,
        false_latch_rate=sum(r['route_group']!=r['true_group'] for r in latched)/max(len(mapped),1),
        naive_correct_rate=sum(r['naive_group']==r['true_group'] for r in mapped)/max(len(mapped),1),
        raw_nearest_correct_rate=sum(r['raw_nearest_group']==r['true_group'] for r in mapped if r['gnss_available'])/max(sum(r['gnss_available'] for r in mapped),1),
        offmap_latch_rate=sum(r['status']=='LATCHED' for r in offmap)/len(offmap) if offmap else None,
        covariance_95_coverage=sum(r['covered95'] for r in rows)/len(rows),
        max_branches=max(r['branch_count'] for r in rows),
        max_pruned_mass=max(r['pruned_mass'] for r in rows))
    if output_dir:
        dest=Path(output_dir); dest.mkdir(parents=True,exist_ok=True)
        with open(dest/f'{name}_seed{seed}.csv','w',newline='') as f:
            w=csv.DictWriter(f,fieldnames=list(rows[0])); w.writeheader(); w.writerows(rows)
        if write_inputs:
            folder=dest/f'{name}_example'; folder.mkdir(exist_ok=True)
            store.loaded=saved_loaded
            store.save(folder/'tiles')
            (folder/'initial.json').write_text(json.dumps(initial,indent=2))
            with open(folder/'events.jsonl','w') as f:
                for event in inputs: f.write(json.dumps(event,separators=(',',':'))+'\n')
    return metric,rows


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--seeds',type=int,default=10)
    ap.add_argument('--start-seed',type=int,default=100)
    ap.add_argument('--scenarios',nargs='+',choices=SCENARIOS,default=SCENARIOS)
    ap.add_argument('--output',default='results')
    ap.add_argument('--save-traces',action='store_true')
    args=ap.parse_args()
    dest=Path(args.output); dest.mkdir(parents=True,exist_ok=True)
    metrics=[]
    for name in args.scenarios:
        for seed in range(args.start_seed,args.start_seed+args.seeds):
            m,_=run_case(name,seed,dest if args.save_traces or seed==args.start_seed else None,
                         write_inputs=seed==args.start_seed)
            metrics.append(m)
        subset=[m for m in metrics if m['scenario']==name]
        print(name,json.dumps({k:round(float(np.mean([m[k] for m in subset])),4)
            for k in ['output_rmse_m','fallback_rmse_m','latch_availability','false_latch_rate',
                      'covariance_95_coverage']}),flush=True)
    (dest/'metrics.json').write_text(json.dumps(metrics,indent=2,allow_nan=False))
    summary={}
    for name in args.scenarios:
        group=[m for m in metrics if m['scenario']==name]
        summary[name]={'runs':len(group)}
        for key in group[0]:
            vals=[m[key] for m in group if isinstance(m[key],(int,float)) and key not in ('seed',)]
            if vals:
                summary[name][key]={'mean':float(np.mean(vals)),'min':float(np.min(vals)),
                                     'max':float(np.max(vals))}
    (dest/'summary.json').write_text(json.dumps(summary,indent=2,allow_nan=False))

if __name__=='__main__': main()
