"""Persistent 18-state map fusion with explicit geometry support and uncertainty.

A rejected/missing factor changes neither native state nor covariance. Default
navigation is untouched. The optional corridor likelihood is an uncertain road
occupancy model, not a surveyed lane fix or continuous independent observation.
"""
from dataclasses import dataclass
import math
import numpy as np
from routefusion.filter import interval_moments


@dataclass(frozen=True)
class MeasurementConfig:
    sigma_m: float = .25
    map_sigma_m: float = 1.5
    cell_m: float = 10.
    corridor: bool = True
    taper: bool = True
    max_distance_m: float = 30.
    max_attitude_rad: float = .25
    max_mount_rad: float = .15

    def __post_init__(self):
        positive = (self.sigma_m, self.cell_m, self.max_distance_m,
                    self.max_attitude_rad, self.max_mount_rad)
        if not all(math.isfinite(x) and 0 < x < 1e6 for x in positive):
            raise ValueError('Invalid measurement configuration')
        if not math.isfinite(self.map_sigma_m) or not 0 <= self.map_sigma_m < 1e6:
            raise ValueError('Invalid map uncertainty')


def lateral_update(P, H, cross, sigma, half_width=0.):
    """Gaussian centerline or interval moment update of a linear lateral factor."""
    if P.shape != (18,18) or H.shape != (18,):
        raise ValueError('Expected unaugmented native error state')
    if not np.isfinite(P).all() or not np.isfinite(H).all() or not math.isfinite(cross):
        raise ValueError('Nonfinite factor')
    if not math.isfinite(sigma) or sigma <= 0 or not math.isfinite(sigma*sigma):
        raise ValueError('Invalid factor noise')
    if not math.isfinite(half_width) or half_width < 0:
        raise ValueError('Invalid corridor width')
    if not np.allclose(P, P.T, atol=1e-7, rtol=0):
        raise ValueError('Asymmetric covariance')
    P=(P+P.T)/2
    if np.linalg.eigvalsh(P)[0] < -1e-7*max(1.,float(np.diag(P).max())):
        raise ValueError('Indefinite prior covariance')
    ph=P@H
    variance=float(H@ph)
    S=variance+sigma*sigma
    if variance < -1e-8 or S <= 0:
        raise ValueError('Invalid innovation covariance')
    if half_width > 0:
        logz, mean, var = interval_moments(cross, S, -half_width, half_width)
        if logz < -12:
            return None, None, dict(reason='CORRIDOR_INCONSISTENT')
        if not all(math.isfinite(x) for x in (logz, mean, var)):
            raise ValueError('Nonfinite interval moments')
        innovation = mean-cross
        posterior = P+np.outer(ph,ph)*(var-S)/(S*S)
    else:
        innovation = -cross
        K=ph/S
        A=np.eye(18)-np.outer(K,H)
        posterior=A@P@A.T+sigma*sigma*np.outer(K,K)
    return ph*innovation/S, (posterior+posterior.T)/2, dict(
        pre_cross_m=float(cross), prior_lateral_sigma_m=math.sqrt(max(0.,variance)),
        lateral_gain=variance/S, effective_sigma_m=float(sigma))


class SupportedMapEKF:
    def __init__(self, nav, roads, config=None, *, enabled=True):
        self.nav, self.roads = nav, roads
        self.config=config or MeasurementConfig()
        self.enabled=enabled
        self.version=roads.store.version
        self.invalidated=False
        self.last_t=-math.inf
        self.update_count=0
        self.last_update_t=None
        self.cells=set()
        self.closed=False
        self.sensor_counts={'imu':0,'gnss':0}

    def feed(self, kind, row):
        if self.closed:
            raise RuntimeError('Filter is closed')
        if kind not in self.sensor_counts:
            raise ValueError('Unknown sensor')
        getattr(self.nav,kind)(row)
        self.sensor_counts[kind]+=1

    def close(self):
        if not self.closed:
            self.nav.close()
            self.closed=True

    def step(self, t, path, support):
        if self.closed:
            raise RuntimeError('Filter is closed')
        if not math.isfinite(t) or t <= self.last_t:
            raise ValueError('Map epochs must increase')
        self.last_t=t
        state=self.nav.snapshot()
        if hasattr(state,'gnss_bias') or state.P.shape != (18,18):
            raise ValueError('Only the original 18-state EKF is supported')
        if not np.isfinite(state.P).all():
            raise RuntimeError('Nonfinite native covariance')
        note=dict(t_s=t, updated=False, reset=False, reason='NO_SUPPORT',
                  output_source='persistent_map_ekf', update_count=self.update_count,
                  native_position_step_m=0., pre_cross_m=None, post_cross_m=None,
                  effective_sigma_m=None, support_reason=support['reason'],
                  path_key=None if path is None else path.key)
        def skip(reason):
            note['reason']=reason
            return state,note
        if self.roads.store.version != self.version:
            self.invalidated=True
        if not state.ready or not state.usable:
            return skip('NATIVE_UNAVAILABLE')
        if not self.enabled:
            return skip('MAP_DISABLED')
        if self.invalidated:
            return skip('MAP_CHANGED')
        if path is None or not support['supported']:
            return skip('NO_SUPPORT')
        if any(not self.roads.store.available(e) for e in path.edges):
            return skip('MISSING_GEOMETRY')
        p,_,velocity=state.pose(self.roads.frame)
        if not np.isfinite(p).all() or not np.isfinite(velocity).all():
            raise RuntimeError('Nonfinite native pose')
        if not self.roads.covers(state.lla) or not self.roads.store.is_loaded(p):
            return skip('MISSING_GEOMETRY')
        try:
            q=path.project(p)
        except ValueError:
            return skip('AMBIGUOUS_PROJECTION')
        if not 0 <= q.s <= path.length:
            return skip('OUTSIDE_PATH')
        speed=float(np.linalg.norm(velocity))
        if speed > 2 and q.tangent@velocity < .2*speed:
            return skip('HEADING_INCOMPATIBLE')
        if q.distance > self.config.max_distance_m:
            return skip('TOO_DISTANT')
        # Stable geometric identity: splitting graph edges cannot buy more factors.
        cell=(self.version,path.key,math.floor(q.s/self.config.cell_m))
        note['cell']=list(cell)
        if cell in self.cells:
            return skip('CELL_ALREADY_USED')
        quality=min(float(support['quality']),path.quality(q.s,support['taper_m'])) if self.config.taper else 1.
        if quality <= 1e-9:
            return skip('UNSUPPORTED_BOUNDARY')
        sigma=math.sqrt((self.config.sigma_m**2+self.config.map_sigma_m**2
                         +path.deviation_m**2)/quality)
        H=np.zeros(18)
        H[6:9]=q.normal@self.roads.frame.from_ned(state.anchor)[:2]
        dx,posterior,metrics=lateral_update(state.P,H,q.cross,sigma,
                                           path.width if self.config.corridor else 0.)
        note.update(metrics)
        if dx is None:
            return state,note
        if (np.linalg.norm(dx[:3]) > self.config.max_attitude_rad
                or np.linalg.norm(dx[15:18]) > self.config.max_mount_rad):
            return skip('ERROR_STATE_GUARD')
        self.nav.inject(dx,posterior)
        after=self.nav.snapshot()
        if not after.ready or not np.isfinite(after.P).all():
            raise RuntimeError('Invalid native posterior')
        Q=(after.P+after.P.T)/2
        if np.linalg.eigvalsh(Q)[0] < -1e-6*max(1.,float(np.diag(Q).max())):
            raise RuntimeError('Indefinite native posterior')
        ap,_,_=after.pose(self.roads.frame)
        if not np.isfinite(ap).all():
            raise RuntimeError('Nonfinite native posterior position')
        self.cells.add(cell)
        self.update_count+=1
        self.last_update_t=t
        note.update(updated=True,reason='CORRIDOR_UPDATE' if self.config.corridor else 'CENTERLINE_UPDATE',
                    post_cross_m=path.project(ap).cross, update_count=self.update_count,
                    native_position_step_m=float(np.linalg.norm(ap-p)))
        return after,note
