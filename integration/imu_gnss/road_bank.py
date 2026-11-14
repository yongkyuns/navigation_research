"""Road-conditioned alternatives of the full upstream 18-error-state filter.

This is approximate inference, not calibrated integrity. Each branch consumes
original IMU/GNSS exactly once. Static map-translation hypotheses persist within
branches; they are not reintroduced as independent white noise each epoch.
No selected branch is ever fed into the map-free filter. Publication is shadow
(output-only) by default; experimental corrected output is returned separately.
"""
from dataclasses import dataclass
import math
import numpy as np
from scipy.special import logsumexp,ndtr
from native import ecef,ned_rotation
from routefusion.filter import interval_moments,mix_gaussians

@dataclass
class Branch:
    edge:str
    nav:object
    offset:np.ndarray
    logw:float
    cells:frozenset=frozenset()
    def fork(self,edge,weight=0.):return Branch(edge,self.nav.clone(),self.offset.copy(),self.logw+weight,self.cells)

class RoadBank:
    def __init__(self,base,roads,max_branches=24):
        self.base=base;self.roads=roads;self.branches=[];self.cap=max_branches;self.dwell=0;self.previous=None;self.last_score=-np.inf;self.max_pruned=0.;self.map_updates=0;self.last_t=-np.inf
    def imu(self,row):
        self.base.imu(row)
        for b in self.branches:b.nav.imu(row)
    def gnss(self,row):
        score=row[0]-self.last_score>=.9
        for b in self.branches:
            if score:b.logw+=self.gnss_evidence(b.nav.snapshot(),row)
            b.nav.gnss(row)
        self.base.gnss(row)
        if score:self.last_score=row[0]
    @staticmethod
    def gnss_evidence(s,row):
        if not s.ready:return 0.
        R=ned_rotation(s.anchor);pos=R@(ecef(row[1:4])-ecef(s.anchor));vel=R@ned_rotation(row[1:4]).T@row[4:7]
        resid=np.r_[pos-s.position,vel-s.velocity];idx=[6,7,8,3,4,5]
        ps=max(float(np.mean(row[7:10])),.1);vs=max(float(np.mean(row[10:13])),.01)
        # Upstream averages per-axis sigmas and scales vertical position by 2.5.
        sig=np.array([ps,ps,2.5*ps,vs,vs,vs]);S=s.P[np.ix_(idx,idx)]+np.diag(sig**2)
        evidence=[]
        for prior,extra in [(.95,np.zeros(6)),(.05,np.array([45.**2]*3+[3.**2]*3))]:
            C=S+np.diag(extra);sign,ld=np.linalg.slogdet(C)
            if sign<=0:return -np.inf
            evidence.append(math.log(prior)-.5*(6*math.log(2*math.pi)+ld+resid@np.linalg.solve(C,resid)))
        return float(logsumexp(evidence))
    def close_branches(self):
        for b in self.branches:b.nav.close()
        self.branches=[]
    def reset(self):self.close_branches();self.dwell=0;self.previous=None
    def seed(self,s):
        p,P,v=s.pose(self.roads.frame);radius=min(100.,max(25.,4*np.sqrt(max(np.linalg.eigvalsh(P)))))
        candidates=self.roads.store.candidates(p,radius)
        candidates=[e for e in candidates if np.linalg.norm(v)<2 or e.project(p).tangent@v>.25*np.linalg.norm(v)]
        offsets=[([0,0],.4),([-3,0],.15),([3,0],.15),([0,-3],.15),([0,3],.15)]
        return [Branch(e.id,self.base.clone(),np.array(o,dtype=float),math.log(w/max(1,len(candidates)))) for e in candidates for o,w in offsets]
    def expand(self,b):
        e=self.roads.store.edges[b.edge];s=b.nav.snapshot()
        if not s.ready:return []
        p,P,v=s.pose(self.roads.frame);pr=e.project(p-b.offset)
        if e.length-pr.s>max(15.,2*np.linalg.norm(v)):return [b.fork(b.edge)]
        candidates={b.edge};front=list(e.successors);depth=0
        while front and depth<3:
            nxt=[]
            for id in front:
                if id in candidates:continue
                ee=self.roads.store.edges.get(id)
                if ee is None:continue
                candidates.add(id)
                if ee.length<max(10.,1.5*np.linalg.norm(v)):nxt.extend(ee.successors)
            front=nxt;depth+=1
        w=-math.log(len(candidates));return [b.fork(id,w) for id in sorted(candidates)]
    def condition(self,b):
        s=b.nav.snapshot()
        if not s.ready:return -np.inf
        e=self.roads.store.edges[b.edge];p,P,v=s.pose(self.roads.frame);pr=e.project(p-b.offset)
        if pr.s < -12 or pr.s>e.length+15:return -np.inf
        if np.linalg.norm(v)>2 and pr.tangent@v<.2*np.linalg.norm(v):return -np.inf
        H=np.zeros(18);H[6:9]=pr.normal@self.roads.frame.from_ned(s.anchor)[:2]
        cov=s.P@H;S=float(H@cov+1.5**2)
        if S<=0:return -np.inf
        ll,mean,var=interval_moments(pr.cross,S,-e.half_width,e.half_width)
        if ll < -12:return -np.inf
        cell=(e.id,int(math.floor(pr.s/10.)))
        if cell in b.cells:return 0.
        dx=cov*((mean-pr.cross)/S);Pn=s.P+np.outer(cov,cov)*(var-S)/S**2
        if np.linalg.norm(dx[:3])>.25 or np.linalg.norm(dx[15:18])>.15:return -np.inf
        b.nav.inject(dx,Pn);b.cells=b.cells|{cell};self.map_updates+=1
        return ll
    def step(self,t,available=True):
        if t<=self.last_t:raise ValueError('Repeated/out-of-order map epoch')
        self.last_t=t;s=self.base.snapshot();out={'status':'NOT_READY','latched':False,'edge':None,'group':None,'top_weight':0.,'pruned_mass':0.,'branches':0,'base':s}
        if not s.ready or not s.usable:self.reset();return out
        p,P,v=s.pose(self.roads.frame);out.update(position=p,covariance=P,conditional_position=p,conditional_covariance=P)
        if not available or not self.roads.covers(s.lla):self.reset();out['status']='MAP_UNAVAILABLE';return out
        pool=self.seed(s) if not self.branches else [c for b in self.branches for c in self.expand(b)]
        self.close_branches();valid=[]
        for b in pool:
            ll=self.condition(b)
            if np.isfinite(ll):b.logw+=ll;valid.append(b)
            else:b.nav.close()
        if not valid:self.reset();out['status']='NO_CONSISTENT_ROUTE';return out
        norm=logsumexp([b.logw for b in valid])
        for b in valid:b.logw-=norm
        valid.sort(key=lambda b:b.logw,reverse=True);retained=valid[:self.cap]
        mass=sum(math.exp(b.logw) for b in retained);pruned=max(0.,1-mass);self.max_pruned=max(self.max_pruned,pruned)
        for b in valid[self.cap:]:b.nav.close()
        for b in retained:b.logw-=math.log(mass)
        self.branches=retained;groups={}
        for b in retained:
            g=self.roads.store.edges[b.edge].group;groups[g]=groups.get(g,0.)+math.exp(b.logw)
        group,w=max(groups.items(),key=lambda kv:kv[1]);eligible=w>.97 and pruned<.05
        self.dwell=self.dwell+1 if eligible and group==self.previous else 1 if eligible else 0;self.previous=group if eligible else None
        chosen=[b for b in retained if self.roads.store.edges[b.edge].group==group];poses=[b.nav.snapshot().pose(self.roads.frame) for b in chosen]
        weights=np.array([math.exp(b.logw) for b in chosen]);weights/=weights.sum()
        mp,mc=mix_gaussians([q[0] for q in poses],[q[1] for q in poses],weights)
        latched=eligible and self.dwell>=3
        best=max(chosen,key=lambda b:b.logw)
        out.update(status='LATCHED' if latched else 'AMBIGUOUS',latched=latched,edge=best.edge if latched else None,group=group if latched else None,top_weight=w,pruned_mass=pruned,branches=len(retained),conditional_position=mp if latched else p,conditional_covariance=mc if latched else P)
        return out
