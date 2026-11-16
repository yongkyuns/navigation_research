"""Linear-Gaussian Schmidt map-error reference (NOT wired into the native EKF).

A persistent local road-map error m has fixed nominal mean and covariance U;
its cross-covariance C=Cov(x,m) with navigation is retained and propagated.
The map state has zero Kalman gain; it is not estimated using the vehicle.
The native strapdown runtime must expose *its own* error-state transition,
GNSS update and reset Jacobians before this reference can become a native arm.

An observation_key guards duplicate uses of one mapped geometric cell. It does
not assert that observations in different cells have independent residual noise.
"""
from __future__ import annotations

import hashlib
import math
from dataclasses import dataclass
import numpy as np


def sym(a: np.ndarray) -> np.ndarray:
    return 0.5 * (a + a.T)


def checked_matrix(value, shape, label, *, psd=False):
    a = np.asarray(value, dtype=np.float64)
    if a.shape != shape or not np.isfinite(a).all():
        raise ValueError(f'{label} must be finite with shape {shape}')
    if psd:
        if not np.allclose(a, a.T, rtol=0, atol=1e-10):
            raise ValueError(f'{label} is not symmetric')
        scale = max(1., float(np.max(np.abs(np.diag(a)))))
        if np.linalg.eigvalsh(a)[0] < -1e-10 * scale:
            raise ValueError(f'{label} is not positive semidefinite')
    return a.copy()


@dataclass(frozen=True)
class UpdateResult:
    applied: bool
    reason: str
    innovation: tuple[float, ...] | None
    nis: float | None
    delta_state: tuple[float, ...] | None


class SchmidtMapReference:
    """Marginal navigation x (n) and static nuisance map error m (k).

    This is a mathematically explicit *linear* benchmark for a local map-error
    patch. It is not an approximation of the native inertial propagation. All
    ordinary sensor Jacobians and transition/reset Jacobians must be supplied.
    """

    def __init__(self, x, P, U, *, map_id: str, map_mean=None, C=None):
        self.x = np.asarray(x, dtype=np.float64).copy()
        if self.x.ndim != 1 or len(self.x) == 0 or not np.isfinite(self.x).all():
            raise ValueError('Navigation state must be a finite nonempty vector')
        self.n = len(self.x)
        u = np.asarray(U, dtype=np.float64)
        if u.ndim != 2 or u.shape[0] == 0 or u.shape[0] != u.shape[1]:
            raise ValueError('Map covariance must be square')
        self.k = len(u)
        if not isinstance(map_id, str) or not map_id.strip():
            raise ValueError('Map patch identity must be nonempty')
        self.map_id = map_id
        self.P = checked_matrix(P, (self.n, self.n), 'P', psd=True)
        self.U = checked_matrix(u, (self.k, self.k), 'U', psd=True)
        self.m = (np.zeros(self.k) if map_mean is None else
                  checked_matrix(map_mean, (self.k,), 'Map mean'))
        self.C = (np.zeros((self.n, self.k)) if C is None else
                  checked_matrix(C, (self.n, self.k), 'C'))
        self.used_observations: set[str] = set()
        self.map_updates = 0
        self.sensor_updates = 0
        self._check_joint()

    def joint_covariance(self) -> np.ndarray:
        return np.block([[self.P, self.C], [self.C.T, self.U]])

    def _check_joint(self):
        cov = self.joint_covariance()
        scale = max(1., float(np.diag(cov).max()))
        if not np.isfinite(cov).all() or np.linalg.eigvalsh(sym(cov))[0] < -1e-9 * scale:
            raise ValueError('Inconsistent joint navigation-map covariance')

    def predict(self, F, Q, *, control=None):
        """Propagate native navigation and its cross-covariance with map state.

        F must include all navigation error-state/reset transitions applicable
        to the same epoch; dropping GNSS/reset Jacobians invalidates the model.
        """
        f = checked_matrix(F, (self.n, self.n), 'F')
        q = checked_matrix(Q, (self.n, self.n), 'Q', psd=True)
        u = (np.zeros(self.n) if control is None else
             checked_matrix(control, (self.n,), 'control'))
        self.x = f @ self.x + u
        self.P = sym(f @ self.P @ f.T + q)
        self.C = f @ self.C
        self._check_joint()

    def reset(self, G):
        """Apply the native error-state reset Jacobian to ALL cross moments."""
        g = checked_matrix(G, (self.n, self.n), 'G')
        self.P = sym(g @ self.P @ g.T)
        self.C = g @ self.C
        self._check_joint()

    def _update(self, z, H, J, R, *, observation_key=None,
                chi2_gate=None, is_map=False):
        zz = np.asarray(z, dtype=float)
        if zz.ndim != 1 or not 1 <= len(zz) <= self.n+self.k or not np.isfinite(zz).all():
            raise ValueError('Invalid measurement vector')
        d = len(zz)
        h = checked_matrix(H, (d, self.n), 'H')
        j = checked_matrix(J, (d, self.k), 'J')
        r = checked_matrix(R, (d, d), 'R', psd=True)
        if chi2_gate is not None and (not math.isfinite(chi2_gate) or chi2_gate <= 0):
            raise ValueError('Invalid chi-square gate')
        if is_map:
            if not isinstance(observation_key, str) or not observation_key:
                raise ValueError('Geometric observation key required for map update')
            if observation_key in self.used_observations:
                return UpdateResult(False, 'ALREADY_USED', None, None, None)
        elif observation_key is not None:
            raise ValueError('Sensor observations cannot carry a map-cell key')

        h_full = np.hstack((h, j))
        prior = self.joint_covariance()
        innovation = zz - h @ self.x - j @ self.m
        s = sym(h_full @ prior @ h_full.T + r)
        try:
            chol = np.linalg.cholesky(s)
        except np.linalg.LinAlgError as exc:
            raise ValueError('Non-positive innovation covariance') from exc
        v = np.linalg.solve(chol, innovation)
        nis = float(v @ v)
        if chi2_gate is not None and nis > chi2_gate:
            return UpdateResult(False, 'INNOVATION_GATE', tuple(innovation), nis, None)

        # Kx is the only nonzero gain; Kmap=0 (Schmidt state held fixed).
        b = prior[:self.n] @ h_full.T
        kx = np.linalg.solve(s, b.T).T
        k_joint = np.vstack((kx, np.zeros((self.k, d))))
        a = np.eye(self.n+self.k) - k_joint @ h_full
        # Joseph-form update on the WHOLE augmented covariance, rather than
        # computing only Pxx and forgetting the navigation/map cross term.
        posterior = sym(a @ prior @ a.T + k_joint @ r @ k_joint.T)
        dx = kx @ innovation
        self.x = self.x + dx
        self.P = posterior[:self.n, :self.n]
        self.C = posterior[:self.n, self.n:]
        self.U = posterior[self.n:, self.n:]
        # The Schmidt map mean AND its covariance are unchanged by design.
        self._check_joint()
        if is_map:
            self.map_updates += 1
            self.used_observations.add(observation_key)
        else:
            self.sensor_updates += 1
        return UpdateResult(True, 'APPLIED', tuple(innovation), nis,
                            tuple(float(v) for v in dx))

    def sensor(self, z, H, R, *, chi2_gate=None):
        return self._update(z, H, np.zeros((len(z), self.k)), R,
                            chi2_gate=chi2_gate)

    def road(self, z, H, J, R, *, observation_key, map_id=None, chi2_gate=None):
        """Use an independently available road constraint, never the current EKF
        estimate projected back as a separate independent position observation.
        """
        if map_id is not None and map_id != self.map_id:
            raise ValueError('Different map patch needs a separate correlated error model')
        return self._update(z, H, J, R, observation_key=observation_key,
                            chi2_gate=chi2_gate, is_map=True)


def stable_patch_key(map_version, road_geometry_key):
    if not map_version or not road_geometry_key:
        raise ValueError('Map version and canonical geometry key are necessary')
    value = f'{map_version}:{road_geometry_key}'.encode('utf-8')
    return hashlib.sha256(value).hexdigest()[:24]
