"""Opt-in research Schmidt map-error adapter around an instrumented native EKF.

This wrapper requires the separately built `rf_transition` ABI, which records
actual native IMU F, sensor (I-KH), ESKF reset G and reanchor operations. It
refuses to reconstruct missing Jacobians, treat repeated map patches as
independent noise, or silently fall back to another estimator after a reseed.

Only *Gaussian* lateral factors with one local constant two-dimensional map
shift are supported. The more realistic finite-width corridor truncation still
requires a separately qualified correlated-moment derivation.
"""
from __future__ import annotations
import ctypes as C
from dataclasses import dataclass
import math
import numpy as np

DIM=18
MAP_DIM=2


def covariance_psd(p, atol=1e-6):
    """Validate raw and variance-normalized covariance without changing it.

    Navigation coordinates have different units and scales. A tolerance based
    only on the largest diagonal can hide impossible cross-correlations in a
    low-variance state. Positive diagonal congruence preserves PSD, so also
    check the dimensionless correlation matrix. Zero-variance directions must
    have exactly zero cross covariance; no jitter, clipping or repair is used.
    """
    p=np.asarray(p,float)
    if p.ndim != 2 or p.shape[0] == 0 or p.shape[0] != p.shape[1]:
        raise ValueError('Covariance must be a nonempty square matrix')
    if not math.isfinite(atol) or atol < 0:
        raise ValueError('Invalid covariance validation tolerance')
    if not np.all(np.isfinite(p)) or not np.allclose(p,p.T,rtol=0,atol=atol):
        raise ValueError('Nonfinite or asymmetric covariance')
    diagonal=np.diag(p)
    if np.any(diagonal < 0.):
        raise ValueError('Negative variance in navigation-map covariance')
    zero=diagonal == 0.
    if np.any(p[zero,:] != 0.) or np.any(p[:,zero] != 0.):
        raise ValueError('Zero-variance state has nonzero cross covariance')
    # Preserve the original check in addition to the scale-aware check.
    scale=max(1.,float(np.max(diagonal)))
    if np.linalg.eigvalsh((p+p.T)*.5)[0] < -atol*scale:
        raise ValueError('Non-positive-semidefinite navigation-map covariance')
    active=~zero
    if not np.any(active):
        return
    std=np.sqrt(diagonal[active])
    with np.errstate(over='ignore', invalid='ignore', divide='ignore'):
        normalized=p[np.ix_(active,active)]/std[:,None]/std[None,:]
    if not np.isfinite(normalized).all():
        raise ValueError('Nonfinite variance-normalized covariance')
    if not np.allclose(normalized,normalized.T,rtol=0,atol=atol):
        raise ValueError('Asymmetric variance-normalized covariance')
    if np.linalg.eigvalsh((normalized+normalized.T)*.5)[0] < -atol:
        raise ValueError('Non-positive-semidefinite variance-normalized covariance')


def compose_joint(p,c,u):
    return np.block([[p,c],[c.T,u]])


@dataclass(frozen=True)
class CrossTransition:
    valid: bool
    operators: int
    matrix: np.ndarray


class NativeTransition:
    """One-shot transition output. No implicit approximations or stale replay."""
    def __init__(self, nav):
        self.nav=nav
        lib=nav.library.lib
        if not hasattr(lib,'rf_transition'):
            raise RuntimeError('Required instrumented rf_transition symbol missing')
        self.fn=lib.rf_transition
        self.fn.argtypes=[C.c_void_p,C.POINTER(C.c_double),C.c_size_t]
        self.fn.restype=C.c_int

    def take(self):
        values=np.zeros(326,np.float64)
        rc=self.fn(self.nav.handle,values.ctypes.data_as(C.POINTER(C.c_double)),326)
        if rc:
            raise RuntimeError(f'Native transition export failed ({rc})')
        if not np.isfinite(values).all() or values[0] not in (0.,1.) or values[1] < 0 or not values[1].is_integer():
            raise RuntimeError('Malformed native transition descriptor')
        matrix=values[2:].reshape(DIM,DIM)
        return CrossTransition(bool(values[0]),int(values[1]),matrix)


@dataclass(frozen=True)
class SchmidtResult:
    applied: bool
    reason: str
    nis: float | None
    native_step_m: float


class NativeSchmidt:
    """Persistent native 18-state EKF + fixed-map Schmidt cross-moments.

    Sensor events MUST pass through this class and in the same order as the raw
    control; native snapshots are read for actual P, and native injection owns
    mean/covariance/reset. Map error m is a fixed 2-D shift for ONE patch.
    """
    def __init__(self, nav, patch_id, map_covariance, *, cross=None):
        if not isinstance(patch_id,str) or not patch_id:
            raise ValueError('Stable canonical map patch ID is required')
        self.nav=nav
        self.transition=NativeTransition(nav)
        self.patch_id=patch_id
        self.U=np.asarray(map_covariance,dtype=float)
        if self.U.shape!=(MAP_DIM,MAP_DIM):
            raise ValueError('Expected 2x2 map error covariance')
        covariance_psd(self.U)
        self.C=np.zeros((DIM,MAP_DIM)) if cross is None else np.asarray(cross,dtype=float).copy()
        if self.C.shape!=(DIM,MAP_DIM) or not np.isfinite(self.C).all():
            raise ValueError('Invalid map cross covariance')
        self.used_cells=set()
        self.map_updates=0
        self.engaged=False
        self.failed=False
        self.imu_events=0
        self.gnss_events=0

    def _take(self):
        tr=self.transition.take()
        if not tr.valid:
            if self.engaged:
                self.failed=True
                raise RuntimeError('Native reseed/untraced transform: correlated map estimator invalid')
            # Before first accepted map update, cross-covariance is zero.
            if np.any(self.C):
                raise RuntimeError('Invalid initial map cross covariance on reseed')
            return tr
        self.C=tr.matrix@self.C
        if self.engaged:
            covariance_psd(compose_joint(self.nav.snapshot().P,self.C,self.U),atol=2e-5)
        return tr

    def feed(self,kind,row):
        if self.failed: raise RuntimeError('Native map estimator has been invalidated')
        if kind not in ('imu','gnss'): raise ValueError('Unknown sensor type')
        getattr(self.nav,kind)(row)
        self._take()
        if kind=='imu':self.imu_events+=1
        else:self.gnss_events+=1

    def factor(self, *, patch_id, cell, residual_m, h, j, local_sigma_m,
               innovation_gate=16., max_attitude_rad=.25, max_mount_rad=.15):
        if self.failed:raise RuntimeError('Native map estimator has been invalidated')
        if patch_id!=self.patch_id:
            raise ValueError('Cannot relabel correlated map error as an independent patch')
        if not isinstance(cell,str) or not cell:
            raise ValueError('Stable canonical map cell key required')
        if cell in self.used_cells:
            return SchmidtResult(False,'ALREADY_USED',None,0.)
        if not all(math.isfinite(x) for x in (residual_m,local_sigma_m,innovation_gate)) or local_sigma_m<=0 or innovation_gate<=0:
            raise ValueError('Invalid lateral observation')
        h=np.asarray(h,dtype=float);j=np.asarray(j,dtype=float)
        if h.shape!=(DIM,) or j.shape!=(MAP_DIM,) or not np.isfinite(h).all() or not np.isfinite(j).all():
            raise ValueError('Invalid map observation Jacobian')
        state=self.nav.snapshot()
        if not state.ready or not state.usable:
            return SchmidtResult(False,'NATIVE_UNAVAILABLE',None,0.)
        p=np.asarray(state.P,dtype=float)
        covariance_psd(compose_joint(p,self.C,self.U),atol=2e-5)
        # Innovation variance accounts for all navigation-map cross terms.
        crossph=p@h+self.C@j
        s=float(h@crossph + j@(self.C.T@h +self.U@j) + local_sigma_m**2)
        if not math.isfinite(s) or s<=0:raise ValueError('Nonpositive innovation covariance')
        nis=float(residual_m*residual_m/s)
        if nis>innovation_gate:
            return SchmidtResult(False,'INNOVATION_GATE',nis,0.)
        k=crossph/s
        dx=k*residual_m
        if np.linalg.norm(dx[:3])>max_attitude_rad or np.linalg.norm(dx[15:18])>max_mount_rad:
            return SchmidtResult(False,'ERROR_STATE_GUARD',nis,0.)
        # Full Schmidt Joseph covariance simplified for K_map=0.
        post=p-np.outer(crossph,crossph)/s
        post=(post+post.T)/2
        cpost=self.C-np.outer(k,h@self.C+j@self.U)
        covariance_psd(compose_joint(post,cpost,self.U),atol=2e-5)
        # Native EKF owns actual mean injection and SO(3) error-state reset.
        self.nav.inject(dx,post)
        self.C=cpost
        self.engaged=True
        self._take() # consumes exact native reset G (and validates it)
        self.used_cells.add(cell)
        self.map_updates+=1
        return SchmidtResult(True,'APPLIED',nis,float(np.linalg.norm(dx[6:9])))

    def close(self):
        self.nav.close()
