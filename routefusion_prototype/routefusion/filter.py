"""Planar navigation reference and numerically explicit Gaussian operations.

State: [east, north, v_e, v_n, yaw, b_ax, b_ay, b_g, g_e, g_n, m_e, m_n, gv_e, gv_n]
Units: metres, seconds, radians. Yaw is CCW from east, body x forward/y left.
g and gv are colored GNSS position/velocity biases; m is persistent map error.
The two-dimensional inertial model assumes roll/pitch/gravity compensation has
already been done. It is NOT a complete 3-D strapdown INS implementation.
"""
from __future__ import annotations
from dataclasses import dataclass, field
import math
import numpy as np
from scipy.special import log_ndtr, logsumexp

N = 14
PXY, VXY, YAW, BA, BG, GB, MB = slice(0,2), slice(2,4), 4, slice(5,7), 7, slice(8,10), slice(10,12)
VB = slice(12,14)

def wrap(a: float) -> float:
    return float((a + np.pi) % (2*np.pi) - np.pi)

def sym(P: np.ndarray) -> np.ndarray:
    return (P + P.T)*0.5

def logdiffexp(a: float, b: float) -> float:
    if b >= a:
        return -np.inf
    return a + math.log(-math.expm1(b-a))

def interval_moments(mu: float, variance: float, lo: float, hi: float):
    """Log probability, conditional mean and variance of a truncated scalar.

    Stable CDF differencing; very remote intervals return -inf (fail closed)
    rather than an untrustworthy forced projection. Normal production uses a
    wider candidate search instead of treating a remote branch as certain.
    """
    if not (np.isfinite(variance) and variance > 0 and lo < hi):
        raise ValueError('invalid interval/variance')
    sigma = math.sqrt(variance)
    a, b = (lo-mu)/sigma, (hi-mu)/sigma
    if a > 0:
        logZ = logdiffexp(float(log_ndtr(-a)), float(log_ndtr(-b)))
    else:
        logZ = logdiffexp(float(log_ndtr(b)), float(log_ndtr(a)))
    if not np.isfinite(logZ) or logZ < -80:
        return -np.inf, mu, variance
    lognorm = 0.5*math.log(2*np.pi)
    pa = 0.0 if np.isinf(a) else math.exp(-0.5*a*a-lognorm-logZ)
    pb = 0.0 if np.isinf(b) else math.exp(-0.5*b*b-lognorm-logZ)
    shift = pa-pb
    ap = 0.0 if np.isinf(a) else a*pa
    bp = 0.0 if np.isinf(b) else b*pb
    vf = min(1.0, max(0.0, 1.0+ap-bp-shift*shift))
    return logZ, mu+sigma*shift, variance*vf

def condition_interval(x, P, H, mu: float, lo: float, hi: float, noise_var=0.0):
    """Moments of x | lo <= y <= hi, y~H*x+offset+Gaussian noise.

    mu is the predicted scalar y, allowing nonlinear local linearization.
    Exact first two moments for ONE linear Gaussian event. Returning a Gaussian
    approximation and applying other constraints afterwards is assumed-density
    filtering, not an exact multivariate truncated-Gaussian posterior.
    """
    cov = P @ H
    S = float(H @ cov + noise_var)
    logZ, m, v = interval_moments(mu, S, lo, hi)
    if not np.isfinite(logZ):
        return x.copy(), P.copy(), logZ
    xn = x + cov*((m-mu)/S)
    Pn = sym(P + np.outer(cov,cov)*((v-S)/(S*S)))
    return xn, Pn, logZ

def gaussian_update(x, P, innovation, H, R):
    S = sym(H @ P @ H.T + R)
    L = np.linalg.cholesky(S)
    q = np.linalg.solve(L, innovation)
    ll = -0.5*(len(innovation)*math.log(2*math.pi)
               + 2*np.log(np.diag(L)).sum() + q@q)
    K = np.linalg.solve(S, H@P).T
    xn = x + K@innovation
    A = np.eye(len(x))-K@H
    Pn = sym(A@P@A.T + K@R@K.T)  # Joseph form
    return xn, Pn, float(ll)

def mix_gaussians(xs, Ps, weights, angle_index=None):
    w = np.asarray(weights, dtype=float)
    w = w/w.sum()
    ref = np.asarray(xs[0]).copy()
    arr = np.asarray(xs).copy()
    if angle_index is not None:
        arr[:,angle_index] = ref[angle_index] + np.array([
            wrap(a-ref[angle_index]) for a in arr[:,angle_index]])
    mean = np.sum(w[:,None]*arr, axis=0)
    cov = np.zeros_like(Ps[0])
    for wi, xi, Pi in zip(w,arr,Ps):
        d = xi-mean
        cov += wi*(Pi+np.outer(d,d))
    if angle_index is not None:
        mean[angle_index] = wrap(mean[angle_index])
    return mean, sym(cov)

@dataclass(frozen=True)
class PVT:
    epoch: int
    t: float
    position: np.ndarray
    velocity: np.ndarray
    covariance: np.ndarray

    def __post_init__(self):
        if self.position.shape != (2,) or self.velocity.shape != (2,) or self.covariance.shape != (4,4):
            raise ValueError('PVT needs 2-D position/velocity and a full 4x4 covariance')
        if not np.isfinite(self.covariance).all():
            raise ValueError('non-finite PVT covariance')
        np.linalg.cholesky(sym(self.covariance))

@dataclass(frozen=True)
class FilterConfig:
    accel_density: float = 0.06       # m/s^2/sqrt(Hz), scenario tuning NOT ST spec
    gyro_density: float = math.radians(0.15)
    accel_bias_rw: float = 0.003
    gyro_bias_rw: float = math.radians(0.005)
    gnss_bias_tau_s: float = 80.0
    gnss_bias_sigma_m: float = 15.0
    gnss_velocity_bias_tau_s: float = 120.0
    gnss_velocity_bias_sigma_mps: float = 0.5
    outlier_probability: float = 0.08
    outlier_position_sigma_m: float = 45.0
    outlier_velocity_sigma_mps: float = 3.0
    nhc_sigma_mps: float = 0.35

@dataclass
class PlanarFilter:
    x: np.ndarray
    P: np.ndarray
    config: FilterConfig = field(default_factory=FilterConfig)
    last_pvt_epoch: int = -1

    def __post_init__(self):
        self.x = np.asarray(self.x, dtype=float).copy()
        self.P = sym(np.asarray(self.P, dtype=float).copy())
        if self.x.shape != (N,) or self.P.shape != (N,N):
            raise ValueError('expected 14-state planar filter')
        if not np.isfinite(self.x).all() or not np.isfinite(self.P).all():
            raise ValueError('non-finite filter initial condition')
        np.linalg.cholesky(self.P)

    def clone(self):
        return PlanarFilter(self.x, self.P, self.config, self.last_pvt_epoch)

    def predict(self, accel_body: np.ndarray, gyro_z: float, dt: float):
        if not (0 < dt <= 1.0) or not np.isfinite(accel_body).all() or not np.isfinite(gyro_z):
            raise ValueError('invalid inertial interval; split intervals above 1 s')
        c = self.config
        mid_yaw = self.x[YAW]+0.5*(gyro_z-self.x[BG])*dt
        cs,sn = math.cos(mid_yaw),math.sin(mid_yaw)
        R = np.array([[cs,-sn],[sn,cs]])
        acc = R@(accel_body-self.x[BA])
        dacc = np.array([-acc[1],acc[0]])
        F = np.eye(N)
        F[PXY,VXY] = np.eye(2)*dt
        F[PXY,YAW] = 0.5*dt*dt*dacc
        F[VXY,YAW] = dt*dacc
        F[PXY,BA] = -0.5*dt*dt*R
        F[VXY,BA] = -dt*R
        F[PXY,BG] = -0.25*dt**3*dacc
        F[VXY,BG] = -0.5*dt*dt*dacc
        F[YAW,BG] = -dt
        phi = math.exp(-dt/c.gnss_bias_tau_s)
        F[GB,GB] = np.eye(2)*phi
        phi_v = math.exp(-dt/c.gnss_velocity_bias_tau_s)
        F[VB,VB] = np.eye(2)*phi_v
        Q = np.zeros((N,N))
        for p,v in [(0,2),(1,3)]:
            q = c.accel_density**2
            Q[p,p] = q*dt**3/3
            Q[p,v] = Q[v,p] = q*dt*dt/2
            Q[v,v] = q*dt
        Q[YAW,YAW] = c.gyro_density**2*dt
        Q[BA,BA] = np.eye(2)*c.accel_bias_rw**2*dt
        Q[BG,BG] = c.gyro_bias_rw**2*dt
        Q[GB,GB] = np.eye(2)*c.gnss_bias_sigma_m**2*(1-phi*phi)
        Q[VB,VB] = np.eye(2)*c.gnss_velocity_bias_sigma_mps**2*(1-phi_v*phi_v)
        # m has no independent per-epoch noise: repeated geometry does not erase
        # its persistent uncertainty as though fresh maps had been observed.
        self.x[PXY] += self.x[VXY]*dt+0.5*acc*dt*dt
        self.x[VXY] += acc*dt
        self.x[YAW] = wrap(self.x[YAW]+(gyro_z-self.x[BG])*dt)
        self.x[GB] *= phi
        self.x[VB] *= phi_v
        self.P = sym(F@self.P@F.T+Q)

    def nhc(self):
        """Soft lateral velocity constraint, not a GNSS-derived heading update."""
        psi = self.x[YAW]
        n = np.array([-math.sin(psi), math.cos(psi)])
        t = np.array([math.cos(psi), math.sin(psi)])
        H = np.zeros((1,N))
        H[0,VXY] = n
        H[0,YAW] = -t@self.x[VXY]
        self.x,self.P,_ = gaussian_update(self.x,self.P,
            np.array([-n@self.x[VXY]]),H,
            np.array([[self.config.nhc_sigma_mps**2]]))
        self.x[YAW] = wrap(self.x[YAW])

    def update_pvt(self, pvt: PVT) -> float:
        """Use one receiver PVT observation exactly once in THIS hypothesis.

        Alternative route branches may all condition on the same observation;
        their resulting posteriors are alternatives, NEVER independent sensors.
        A two-component noise mixture has normalized marginal evidence, including
        determinant terms; moment matching retains between-component spread.
        """
        if pvt.epoch <= self.last_pvt_epoch:
            raise ValueError('duplicate or out-of-order GNSS epoch')
        H = np.zeros((4,N))
        H[:2,PXY] = H[:2,GB] = np.eye(2)
        H[2:,VXY] = H[2:,VB] = np.eye(2)
        z = np.r_[pvt.position,pvt.velocity]
        innov = z-H@self.x
        R0 = pvt.covariance
        R1 = R0 + np.diag([self.config.outlier_position_sigma_m**2]*2
                           + [self.config.outlier_velocity_sigma_mps**2]*2)
        eps = self.config.outlier_probability
        xs,Ps,lls = [],[],[]
        for prior,R in [(1-eps,R0),(eps,R1)]:
            x,P,ll = gaussian_update(self.x,self.P,innov,H,R)
            xs.append(x); Ps.append(P); lls.append(math.log(prior)+ll)
        total = float(logsumexp(lls))
        self.x,self.P = mix_gaussians(xs,Ps,np.exp(np.array(lls)-total),YAW)
        self.last_pvt_epoch = pvt.epoch
        return total
