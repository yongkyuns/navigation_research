"""Opt-in interval road constraints with retained native/map cross covariance.

The navigation moments match one linear Gaussian interval event. Keeping the
map mean/covariance fixed is a Schmidt-style moment approximation, NOT exact
joint Bayesian conditioning or an accuracy/consistency guarantee. Only one
fixed map-translation patch is supported, with unchanged spatial reuse guards.
"""
from __future__ import annotations
from dataclasses import dataclass
import hashlib
import json
import math
import numpy as np
from native_schmidt import NativeSchmidt, compose_joint, covariance_psd
from routefusion.filter import interval_moments


@dataclass(frozen=True)
class CorridorMoments:
    dx: np.ndarray | None
    covariance: np.ndarray | None
    cross: np.ndarray | None
    reason: str
    log_probability: float
    innovation_variance: float
    information_fraction: float
    conditional_mean: float
    conditional_variance: float


def corridor_moments(p, c, u, h, j, cross_track_m, half_width_m, local_sigma_m):
    """Scalar interval moment match followed by fixed-map Schmidt projection.

    With a=Cov([x,m],y), S=Var(y), and v=Var(y | -w<=y<=w),
    beta=1-v/S. Pxx and Pxm receive -beta*a_x*[a_x,a_m]^T/S;
    U is retained. For beta>0 this equals a Gaussian Schmidt update with
    S_eff=S/beta and residual_eff=(E[y|interval]-E[y])/beta.
    This equivalence establishes PSD, not nonlinear road-estimator consistency.
    """
    p, c, u, h, j = [np.asarray(x, dtype=float) for x in (p, c, u, h, j)]
    for value, shape in [(p,(18,18)), (c,(18,2)), (u,(2,2)), (h,(18,)), (j,(2,))]:
        if value.shape != shape or not np.isfinite(value).all():
            raise ValueError('Malformed navigation/map moments or Jacobian')
    if (not all(math.isfinite(x) for x in (cross_track_m, half_width_m, local_sigma_m))
            or half_width_m <= 0 or local_sigma_m <= 0):
        raise ValueError('Finite positive corridor width and local sigma required')
    r = local_sigma_m**2
    if not math.isfinite(r):
        raise ValueError('Local variance overflow')
    prior = compose_joint(p, c, u)
    covariance_psd(prior, atol=2e-5)
    g = np.r_[h,j]
    a = prior @ g
    s = float(g @ a + r)
    if not math.isfinite(s) or s <= 0:
        raise ValueError('Nonpositive corridor innovation variance')
    logz, mean, var = interval_moments(cross_track_m, s, -half_width_m, half_width_m)
    if not math.isfinite(logz) or logz < -12.:
        return CorridorMoments(None,None,None,'CORRIDOR_INCONSISTENT',logz,s,0.,mean,var)
    beta = 1. - var/s
    if not all(math.isfinite(x) for x in (mean,var,beta)) or not 0. <= beta <= 1.:
        raise ValueError('Invalid conditional interval moments')
    if beta == 0.:
        if abs(mean-cross_track_m) > 1e-12*max(1.,abs(cross_track_m)):
            raise ValueError('Unresolved interval mean with zero information')
        return CorridorMoments(None,None,None,'NO_INFORMATION',logz,s,0.,mean,var)
    dx = a[:18] * ((mean-cross_track_m)/s)
    post = p - (beta/s)*np.outer(a[:18],a[:18])
    post = (post+post.T)*.5
    cpost = c - (beta/s)*np.outer(a[:18],a[18:])
    covariance_psd(compose_joint(post,cpost,u), atol=2e-5)
    return CorridorMoments(dx,post,cpost,'APPLIED',logz,s,beta,mean,var)


class NativeCorridorSchmidt(NativeSchmidt):
    """Use native event transitions even during periods without map support."""
    def __init__(self, nav, patch_id, map_covariance, *, cross=None):
        super().__init__(nav,patch_id,np.asarray(map_covariance,dtype=float).copy(),cross=cross)
        self.U.setflags(write=False)

    def feed(self,kind,row):
        # An event may already have changed native state before a trace or PSD
        # check fails. Such an instance cannot subsequently resume as if valid.
        if kind not in ('imu','gnss'):
            raise ValueError('Unknown sensor type')
        try:
            super().feed(kind,row)
        except Exception:
            self.failed=True
            raise

    def interval_factor(self, *, patch_id, cell, cross_track_m, half_width_m,
                        h, j, local_sigma_m, max_attitude_rad=.25, max_mount_rad=.15):
        if self.failed:
            raise RuntimeError('Native map estimator has been invalidated')
        if patch_id != self.patch_id:
            raise ValueError('Cannot replace the shared map patch at an edge change')
        if not isinstance(cell,str) or not cell:
            raise ValueError('Canonical spatial observation key required')
        if not all(math.isfinite(x) and x > 0 for x in (max_attitude_rad,max_mount_rad)):
            raise ValueError('Invalid native increment guards')
        note=dict(applied=False,reason='ALREADY_USED',information_fraction=0.,
                  log_probability=None,innovation_variance=None,correction_norm_m=0.)
        if cell in self.used_cells:
            return note
        state=self.nav.snapshot()
        if not state.ready or not state.usable:
            return {**note,'reason':'NATIVE_UNAVAILABLE'}
        result=corridor_moments(state.P,self.C,self.U,h,j,cross_track_m,half_width_m,local_sigma_m)
        note.update(reason=result.reason,information_fraction=result.information_fraction,
                    log_probability=result.log_probability if math.isfinite(result.log_probability) else None,
                    innovation_variance=result.innovation_variance)
        if result.dx is None:
            return note
        if (np.linalg.norm(result.dx[:3]) > max_attitude_rad
                or np.linalg.norm(result.dx[15:18]) > max_mount_rad):
            return {**note,'reason':'ERROR_STATE_GUARD'}
        try:
            self.nav.inject(result.dx,result.covariance)
            self.C=result.cross
            self.engaged=True
            self._take()  # exact native injection reset, including its PSD check
        except Exception:
            self.failed=True
            raise
        self.used_cells.add(cell)
        self.map_updates+=1
        return {**note,'applied':True,'correction_norm_m':float(np.linalg.norm(result.dx[6:9]))}


def map_patch_id(map_response_sha256, origin):
    """One experimental extract-wide translation, never one new patch per edge."""
    if (not isinstance(map_response_sha256,str) or len(map_response_sha256)!=64
            or any(x not in '0123456789abcdef' for x in map_response_sha256)):
        raise ValueError('Expected map-source SHA-256')
    p=np.asarray(origin,dtype=float)
    if p.shape!=(3,) or not np.isfinite(p).all():
        raise ValueError('Expected fixed local-frame origin')
    return hashlib.sha256(json.dumps([map_response_sha256,p.tolist()],separators=(',',':')).encode()).hexdigest()
