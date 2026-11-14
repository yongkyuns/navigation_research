"""Bounded Gaussian-sum road-hypothesis tracker around a navigation filter.

Important statistical contract:
* each branch is an ALTERNATIVE conditional navigation solution;
* the map-free filter is a fallback, never fused with the bank as another sensor;
* branch weights are conditional on the enumerated roads and map assumptions;
* map validity/abstention is an explicit policy, NOT a calibrated null-mode Bayes
  probability or integrity bound;
* local moment matching, geometry projection, topology gates and beam pruning
  are approximations. See docs/DESIGN.md before claiming calibrated accuracy.
"""
from __future__ import annotations
from dataclasses import dataclass,field
import math
import numpy as np
from scipy.special import ndtr,logsumexp
from .filter import (N,PXY,VXY,YAW,MB,PlanarFilter,PVT,wrap,condition_interval,
                     interval_moments,mix_gaussians)
from .tiles import TileStore,Edge

@dataclass(frozen=True)
class TrackerConfig:
    max_branches: int = 12
    max_per_edge: int = 3
    search_radius_min_m: float = 35.0
    search_radius_max_m: float = 100.0
    map_cell_m: float = 8.0
    boundary_sigma_m: float = 0.75
    heading_bound_rad: float = math.radians(30)
    heading_noise_rad: float = math.radians(4)
    endpoint_padding_m: float = 12.0
    latch_weight: float = 0.97
    latch_odds: float = 30.0
    latch_dwell_epochs: int = 3
    invalid_log_event: float = -12.0
    max_event_memory: int = 256

@dataclass
class Branch:
    edge_id: str
    nav: PlanarFilter
    log_weight: float
    used_cells: tuple[tuple,...] = ()
    history: tuple[str,...] = ()

    def child(self,edge_id,log_prior):
        history=self.history
        if not history or history[-1]!=edge_id: history=(history+(edge_id,))[-32:]
        return Branch(edge_id,self.nav.clone(),self.log_weight+log_prior,
                      self.used_cells,history)

@dataclass
class Estimate:
    t: float
    status: str
    position: np.ndarray
    covariance: np.ndarray
    route_group: str|None
    route_edge: str|None
    top_weight: float
    second_weight: float
    conditional_route_weights: dict[str,float]
    fallback_position: np.ndarray
    fallback_covariance: np.ndarray
    mixture_position: np.ndarray|None
    mixture_covariance: np.ndarray|None
    branch_count: int
    pruned_mass: float
    note: str=''

class RouteTracker:
    def __init__(self,nav: PlanarFilter,tiles: TileStore,config=TrackerConfig()):
        self.base=nav
        self.tiles=tiles
        self.config=config
        self.branches: list[Branch]=[]
        self.dwell=0; self.last_group=None; self.last_epoch=-1
        self.last_t=-np.inf
        self.last_pruned_mass=0.0
        self.last_note=''

    def predict(self,accel_body,gyro_z,dt):
        self.base.predict(accel_body,gyro_z,dt)
        for b in self.branches: b.nav.predict(accel_body,gyro_z,dt)

    def _seed(self):
        P=self.base.P[PXY,PXY]
        radius=min(self.config.search_radius_max_m,
                   max(self.config.search_radius_min_m,4*math.sqrt(np.linalg.eigvalsh(P)[-1])))
        candidates=self.tiles.candidates(self.base.x[PXY],radius)
        n=max(1,len(candidates))
        return [Branch(e.id,self.base.clone(),-math.log(n),(),(e.id,)) for e in candidates]

    def _expand(self,b: Branch):
        e=self.tiles.edges[b.edge_id]
        proj=e.project(b.nav.x[PXY]-b.nav.x[MB])
        H=np.zeros(N); H[PXY]=proj.tangent; H[MB]=-proj.tangent
        sigma=math.sqrt(max(float(H@b.nav.P@H),1.0))
        remaining=e.length-proj.s
        # No geometric connection is invented. Missing successor tile is not a
        # negative observation; step() checks it before filtering candidates.
        successors=[i for i in e.successors if self.tiles.available(i)]
        if not successors or remaining > max(18.0,3*sigma):
            return [b.child(b.edge_id,0.0)]
        p=float(np.clip(ndtr((proj.s-e.length)/max(sigma,3.0)),1e-6,1-1e-6))
        result=[b.child(b.edge_id,math.log1p(-p))]
        result += [b.child(s,math.log(p)-math.log(len(successors))) for s in successors]
        return result

    def _map_update(self,b: Branch):
        """Return log geometry evidence, or -inf for an infeasible branch.

        Only a NEW road spatial cell contributes a map factor; repeated stationary
        application of the same Gaussianized truncation would manufacture
        information. Used-cell ownership is inherited through branch splits.
        """
        cfg=self.config; e=self.tiles.edges[b.edge_id]; nav=b.nav
        p=nav.x[PXY]-nav.x[MB]; proj=e.project(p)
        H=np.zeros(N); H[PXY]=proj.normal; H[MB]=-proj.normal
        S=float(H@nav.P@H+cfg.boundary_sigma_m**2)
        logcross,_,_=interval_moments(proj.cross,S,-e.half_width,e.half_width)
        Ha=np.zeros(N); Ha[PXY]=proj.tangent; Ha[MB]=-proj.tangent
        Sa=float(Ha@nav.P@Ha+cfg.boundary_sigma_m**2)
        logalong,_,_=interval_moments(proj.s,Sa,-cfg.endpoint_padding_m,
                                      e.length+cfg.endpoint_padding_m)
        speed=float(np.linalg.norm(nav.x[VXY]))
        angle=wrap(nav.x[YAW]-math.atan2(proj.tangent[1],proj.tangent[0]))
        logheading=0.0
        # Road heading is a broad corridor, not a lane-level exact yaw sensor.
        # Disable it close to a graph node to allow finite-radius intersection turns.
        use_heading=(speed>2 and 15<proj.s<e.length-15)
        if use_heading:
            logheading,_,_=interval_moments(angle,float(nav.P[YAW,YAW]+cfg.heading_noise_rad**2),
                                           -cfg.heading_bound_rad,cfg.heading_bound_rad)
        if min(logcross,logalong,logheading) < cfg.invalid_log_event:
            return -np.inf
        cell=(self.tiles.version,e.id,int(math.floor(proj.s/cfg.map_cell_m)))
        if cell in b.used_cells:
            return 0.0
        b.used_cells=(b.used_cells+(cell,))[-cfg.max_event_memory:]
        ll=0.0
        nav.x,nav.P,z=condition_interval(nav.x,nav.P,H,proj.cross,-e.half_width,e.half_width,
                                        cfg.boundary_sigma_m**2)
        ll+=z
        # Finite edge extent matters at dead ends. Re-evaluate at updated mean.
        proj=e.project(nav.x[PXY]-nav.x[MB])
        Ha=np.zeros(N); Ha[PXY]=proj.tangent; Ha[MB]=-proj.tangent
        nav.x,nav.P,z=condition_interval(nav.x,nav.P,Ha,proj.s,
                     -cfg.endpoint_padding_m,e.length+cfg.endpoint_padding_m,
                     cfg.boundary_sigma_m**2)
        ll+=z
        if use_heading:
            Hh=np.zeros(N); Hh[YAW]=1
            angle=wrap(nav.x[YAW]-math.atan2(proj.tangent[1],proj.tangent[0]))
            nav.x,nav.P,z=condition_interval(nav.x,nav.P,Hh,angle,
                -cfg.heading_bound_rad,cfg.heading_bound_rad,cfg.heading_noise_rad**2)
            ll+=z
        nav.x[YAW]=wrap(nav.x[YAW])
        return float(ll)

    def _fallback(self,t,status,note):
        self.dwell=0; self.last_group=None
        return Estimate(t,status,self.base.x[PXY].copy(),self.base.P[PXY,PXY].copy(),
            None,None,0.0,0.0,{},self.base.x[PXY].copy(),self.base.P[PXY,PXY].copy(),
            None,None,len(self.branches),self.last_pruned_mass,note)

    def step(self,t: float,pvt: PVT|None=None) -> Estimate:
        """Call at the map epoch AFTER predict() has propagated to time t.

        PVT may be absent during outages. Duplicate time/PVT IDs are rejected.
        Map-free and branch states both ingest NHC and PVT, each once. On reseed,
        clone the already-updated fallback and do NOT update it again with PVT.
        """
        if t<=self.last_t: raise ValueError('map epochs must increase')
        if pvt is not None and (abs(pvt.t-t)>1e-6 or pvt.epoch<=self.last_epoch):
            raise ValueError('misaligned, duplicate, or out-of-order PVT')
        self.last_t=t
        self.base.nhc()
        for b in self.branches: b.nav.nhc()
        if pvt is not None:
            self.last_epoch=pvt.epoch
            self.base.update_pvt(pvt)
            for b in self.branches: b.log_weight+=b.nav.update_pvt(pvt)
        if not self.tiles.is_loaded(self.base.x[PXY]):
            self.branches=[]
            return self._fallback(t,'MAP_UNAVAILABLE','No loaded tile at fallback position')
        # Unknown connectivity ahead must not turn into negative map evidence.
        for b in self.branches:
            e=self.tiles.edges[b.edge_id]
            proj=e.project(b.nav.x[PXY]-b.nav.x[MB])
            if e.length-proj.s<30 and any(not self.tiles.available(s) for s in e.successors):
                self.branches=[]
                return self._fallback(t,'MAP_UNAVAILABLE','Required successor tile is absent')
        seeded=not self.branches
        pool=self._seed() if seeded else [c for b in self.branches for c in self._expand(b)]
        valid=[]
        for b in pool:
            ll=self._map_update(b)
            if np.isfinite(ll):
                b.log_weight+=ll
                valid.append(b)
        if not valid:
            self.branches=[]
            return self._fallback(t,'OFF_MAP','No enumerated route remains consistent; not a proof of off-road motion')
        normalizer=float(logsumexp([b.log_weight for b in valid]))
        for b in valid: b.log_weight-=normalizer
        valid.sort(key=lambda b:b.log_weight,reverse=True)
        selected=[]; counts={}
        for b in valid:
            count=counts.get(b.edge_id,0)
            if count<self.config.max_per_edge and len(selected)<self.config.max_branches:
                selected.append(b); counts[b.edge_id]=count+1
        retained=sum(math.exp(b.log_weight) for b in selected)
        self.last_pruned_mass=max(0.0,1.0-retained)
        for b in selected: b.log_weight-=math.log(retained)
        self.branches=selected
        groups={}
        for b in selected:
            g=self.tiles.edges[b.edge_id].group
            groups[g]=groups.get(g,0)+math.exp(b.log_weight)
        ranked=sorted(groups.items(),key=lambda p:p[1],reverse=True)
        group,w=ranked[0]; w2=ranked[1][1] if len(ranked)>1 else 0.0
        eligible=(w>=self.config.latch_weight and w/max(w2,1e-12)>=self.config.latch_odds
                  and self.last_pruned_mass<0.05)
        if eligible:
            self.dwell=self.dwell+1 if group==self.last_group else 1
            self.last_group=group
        else:
            self.dwell=0; self.last_group=None
        best=max((b for b in selected if self.tiles.edges[b.edge_id].group==group),
                 key=lambda b:b.log_weight)
        weights=[math.exp(b.log_weight) for b in selected]
        mm,mc=mix_gaussians([b.nav.x[PXY] for b in selected],
                            [b.nav.P[PXY,PXY] for b in selected],weights)
        latched=eligible and self.dwell>=self.config.latch_dwell_epochs
        # The mixture moments are exposed for diagnostics only. Its mean may lie
        # between roads; do not publish it as a routable coordinate when ambiguous.
        pos=best.nav.x[PXY].copy() if latched else self.base.x[PXY].copy()
        cov=best.nav.P[PXY,PXY].copy() if latched else self.base.P[PXY,PXY].copy()
        return Estimate(t,'LATCHED' if latched else 'AMBIGUOUS',pos,cov,
            group if latched else None,best.edge_id if latched else None,w,w2,groups,
            self.base.x[PXY].copy(),self.base.P[PXY,PXY].copy(),mm,mc,len(selected),
            self.last_pruned_mass,'Weights conditional on retained mapped hypotheses; not integrity probabilities')
