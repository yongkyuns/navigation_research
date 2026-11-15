"""Opt-in, fault-activated receiver-position nuisance states.

The default research policy stays numerically identical to the original native
filter until BOTH receiver position/velocity inconsistency and a navigation
innovation indicate a possible new position fault. This is an engineering mode
selector, not a calibrated integrity probability. Reference/map coordinates are
not detector inputs. Once active, the three-state correlated-error model keeps
its full joint covariance; it is not reset merely because the bias stops moving.
"""
from collections import deque
from dataclasses import dataclass, asdict
import ctypes as C
import math
import numpy as np
from native import Library, NativeFusion, POINTER, ecef, ned_rotation

@dataclass(frozen=True)
class BiasConfig:
    tau_s: float = 90.0
    stationary_sigma_m: float = 15.0
    initial_sigma_m: float = 3.0
    shock_sigma_m: float = 20.0
    activation: str = 'innovation'
    window_s: float = 2.0
    max_gap_s: float = 1.1
    consistency_floor_m: float = 1.5
    consistency_nis: float = 16.0
    prediction_nis: float = 9.0
    velocity_nis: float = 16.0
    def __post_init__(self):
        nums=[v for k,v in asdict(self).items() if k!='activation']
        if not all(math.isfinite(x) and x>=0 for x in nums) or self.tau_s<=0 or self.window_s<=0 or self.max_gap_s<=0:
            raise ValueError('Invalid correlated-error prior or activation policy')
        if self.activation not in ('innovation','always'):raise ValueError('Unknown activation policy')

class BiasLibrary(Library):
    def __init__(self,path,config=BiasConfig()):
        super().__init__(path);self.config=config
        self.lib.rf_new_bias.argtypes=[C.c_double]*3;self.lib.rf_new_bias.restype=C.c_void_p
        self.lib.rf_activate_bias.argtypes=[C.c_void_p]+[C.c_double]*4;self.lib.rf_activate_bias.restype=C.c_int
        self.lib.rf_bias_snapshot.argtypes=[C.c_void_p,POINTER,C.c_size_t];self.lib.rf_bias_snapshot.restype=C.c_int
        self.lib.rf_bias_interval.argtypes=[C.c_void_p,POINTER,C.c_size_t,C.c_double,C.c_double];self.lib.rf_bias_interval.restype=C.c_int
    def create(self):
        q=self.config
        h=self.lib.rf_new_bias(q.tau_s,q.stationary_sigma_m,q.initial_sigma_m) if q.activation=='always' else self.lib.rf_new()
        return BiasFusion(self,h,active=q.activation=='always')

def innovation_terms(s,row):
    """The native local-frame PVT innovation; no reference or road lookup."""
    rotation=ned_rotation(s.anchor)
    pos=rotation@(ecef(row[1:4])-ecef(s.anchor))
    vel=rotation@ned_rotation(row[1:4]).T@row[4:7]
    rp=pos-s.position;rv=vel-s.velocity
    ps=max(float(np.mean(row[7:10])),.1);vs=max(float(np.mean(row[10:13])),.01)
    sp=s.P[6:8,6:8]+np.eye(2)*ps**2
    sv=s.P[3:5,3:5]+np.eye(2)*vs**2
    return float(rp[:2]@np.linalg.solve(sp,rp[:2])),float(rv[:2]@np.linalg.solve(sv,rv[:2]))

def position_velocity_consistency(rows):
    """Short-window disagreement, used ONLY to select a noise model.

    The diagnostic is not fused as another measurement. Receiver P/V temporal
    correlations are not provided, so the variance is a conservative heuristic,
    not a statistical integrity bound. Velocity uncertainty is fully correlated
    across the short window instead of being averaged away at high sample rate.
    """
    rows=np.asarray(rows,dtype=float);t=rows[:,0];dt=np.diff(t)
    rotation=ned_rotation(rows[0,1:4]);p=(ecef(rows[:,1:4])-ecef(rows[0,1:4]))@rotation.T
    velocity=np.array([rotation@ned_rotation(row[1:4]).T@row[4:7] for row in rows])
    displacement=np.sum((velocity[:-1,:2]+velocity[1:,:2])*(dt/2)[:,None],axis=0)
    residual=p[-1,:2]-displacement
    ps=np.mean(rows[:,7:10],axis=1);vs=np.mean(rows[:,10:13],axis=1)
    duration=t[-1]-t[0]
    sigma_v=float(np.sum(dt*(vs[:-1]+vs[1:])/2))
    variance=2*(ps[0]**2+ps[-1]**2)+sigma_v**2+(.1*duration)**2+.25**2
    variance=max(variance,1e-6)
    return float(np.linalg.norm(residual)),float(residual@residual/variance)

class BiasFusion(NativeFusion):
    def __init__(self,library,handle,active=False):
        super().__init__(library,handle);self.active=active;self.activation_t=-1.
        self.history=deque(maxlen=32);self.pending=None;self.diagnostic=(0.,0.,0.,0.)
    def clone(self):
        if not self.handle:raise RuntimeError('Clone after close')
        out=BiasFusion(self.library,self.library.lib.rf_clone(self.handle),self.active)
        out.activation_t=self.activation_t;out.history=deque((r.copy() for r in self.history),maxlen=32)
        out.pending=None if self.pending is None else self.pending.copy();out.diagnostic=self.diagnostic
        return out
    def prepare_gnss(self,row):
        row=np.asarray(row,dtype=float)
        if row.shape!=(14,) or not np.isfinite(row[:13]).all() or np.any(row[7:13]<0):raise ValueError('Invalid GNSS input')
        if self.pending is not None:
            if np.array_equal(row,self.pending,equal_nan=True):return
            raise ValueError('Another GNSS epoch is already pending')
        if self.history and row[0]<=self.history[-1][0]:raise ValueError('Duplicate/out-of-order GNSS')
        if self.history and row[0]-self.history[-1][0]>self.library.config.max_gap_s:self.history.clear()
        q=self.library.config
        while self.history and row[0]-self.history[0][0]>q.window_s+1e-8:self.history.popleft()
        s=super().snapshot()
        if not self.active and s.ready and s.usable and len(self.history)>=2 and row[0]-self.history[0][0]>=.9:
            distance,consistency=position_velocity_consistency([*self.history,row])
            prediction,velocity=innovation_terms(s,row)
            self.diagnostic=(distance,consistency,prediction,velocity)
            if distance>=q.consistency_floor_m and consistency>q.consistency_nis and prediction>q.prediction_nis and velocity<q.velocity_nis:
                rc=self.library.lib.rf_activate_bias(self.handle,q.tau_s,q.stationary_sigma_m,q.initial_sigma_m,q.shock_sigma_m)
                if rc:raise RuntimeError(f'Bias activation failed: {rc}')
                self.active=True;self.activation_t=float(row[0])
        self.pending=row.copy()
    def gnss(self,row):
        self.prepare_gnss(row);super().gnss(row)
        self.history.append(self.pending.copy());self.pending=None
    def snapshot(self):
        s=super().snapshot();extra=np.zeros(66,dtype=np.float64)
        rc=self.library.lib.rf_bias_snapshot(self.handle,extra.ctypes.data_as(POINTER),len(extra))
        if rc:raise RuntimeError(f'GNSS-bias snapshot failed: {rc}')
        s.gnss_bias=extra[:3];s.gnss_bias_covariance=extra[3:12].reshape(3,3);s.gnss_bias_cross=extra[12:].reshape(18,3)
        s.augmented_covariance=np.block([[s.P,s.gnss_bias_cross],[s.gnss_bias_cross.T,s.gnss_bias_covariance]])
        s.bias_active=self.active;s.activation_t=self.activation_t;s.bias_diagnostic=self.diagnostic
        # Check rollback equality for the detector and covariance too. Preserve
        # missing heading using a deterministic sentinel, never NaN equality.
        history=np.nan_to_num(np.asarray(list(self.history),dtype=float).ravel(),nan=1234567.)
        pending=[] if self.pending is None else np.nan_to_num(self.pending,nan=1234567.)
        s.raw=np.r_[s.raw,extra,float(self.active),self.activation_t,self.diagnostic,len(self.history),history,pending]
        return s
    def inject(self,dx,P):
        if self.active:raise RuntimeError('Marginal overwrite loses GNSS-bias correlation')
        super().inject(dx,P)
    def condition_interval(self,H,alpha,beta):
        h=np.ascontiguousarray(H,dtype=np.float64)
        if h.shape!=(18,) or not np.isfinite(h).all() or not np.isfinite([alpha,beta]).all() or beta>0:raise ValueError('Invalid interval moments')
        s=self.snapshot();u=s.augmented_covariance@np.r_[h,np.zeros(3)];posterior=s.augmented_covariance+beta*np.outer(u,u)
        if np.linalg.eigvalsh((posterior+posterior.T)/2)[0]<-1e-7*max(1.,np.diag(posterior).max()):raise ValueError('Indefinite augmented map posterior')
        if not self.active:
            super().inject(alpha*(s.P@h),posterior[:18,:18]);return
        rc=self.library.lib.rf_bias_interval(self.handle,h.ctypes.data_as(POINTER),len(h),alpha,beta)
        if rc:raise RuntimeError(f'Augmented map interval rejected: {rc}')
