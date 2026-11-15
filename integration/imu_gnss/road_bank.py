"""Alternative native INS/GNSS states with maneuver-aware road support.

Road weights remain conditional/approximate, not integrity probabilities. The
map-free runtime is never corrected by a branch. Junctions do not add a second
IMU-derived turn observation or pretend the vehicle follows a known turn curve.
"""
from dataclasses import dataclass
import math
import numpy as np
from scipy.special import logsumexp
from native import ecef, ned_rotation
from routefusion.filter import interval_moments, mix_gaussians
from intersections import (JunctionConfig, Maneuver, MissingTopology,
                           HypothesisOverflow, maneuver_paths, advance)
from recovery import ReplayWindow


@dataclass
class Branch:
    edge: str
    nav: object
    offset: np.ndarray
    logw: float
    cells: frozenset = frozenset()
    maneuver: Maneuver | None = None

    @property
    def mode(self):
        return self.maneuver.key if self.maneuver else ('road', self.edge)

    def fork(self, edge, weight=0.0, maneuver=None):
        return Branch(edge, self.nav.clone(), self.offset.copy(), self.logw + weight,
                      self.cells, self.maneuver if maneuver is None else maneuver)


class RoadBank:
    def __init__(self, base, roads, max_branches=24, config=None, *, recovery=True):
        if not isinstance(max_branches, int) or max_branches < 1:
            raise ValueError('Branch cap must be a positive integer')
        self.base, self.roads, self.cap = base, roads, max_branches
        self.config = config or JunctionConfig()
        self.branches = []
        self.dwell = 0
        self.previous = None
        self.last_score = -np.inf
        self.last_t = -np.inf
        self.max_pruned = 0.0
        self.map_updates = 0
        self.recoveries = 0
        self._record = recovery
        self.window = ReplayWindow(self.config.recovery_s, self.config.recovery_events)
        self._version = None

    def imu(self, row):
        self.base.imu(row)
        for branch in self.branches:
            branch.nav.imu(row)
        if self._record:
            self.window.append('imu', row)

    def gnss(self, row):
        score = row[0] - self.last_score >= 0.9
        for branch in self.branches:
            if hasattr(branch.nav, "prepare_gnss"):
                branch.nav.prepare_gnss(row)
            if score:
                branch.logw += self.gnss_evidence(branch.nav.snapshot(), row)
            branch.nav.gnss(row)
        self.base.gnss(row)
        if score:
            self.last_score = row[0]
        if self._record:
            self.window.append('gnss', row)

    @staticmethod
    def gnss_evidence(s, row):
        """Inherited approximate pre-update score; not exact gated-EKF evidence."""
        if not s.ready:
            return 0.0
        R = ned_rotation(s.anchor)
        pos = R @ (ecef(row[1:4]) - ecef(s.anchor))
        vel = R @ ned_rotation(row[1:4]).T @ row[4:7]
        position_mean = s.position + getattr(s, "gnss_bias", np.zeros(3))
        residual = np.r_[pos - position_mean, vel - s.velocity]
        indices = [6, 7, 8, 3, 4, 5]
        ps = max(float(np.mean(row[7:10])), 0.1)
        vs = max(float(np.mean(row[10:13])), 0.01)
        sigmas = np.array([ps, ps, 2.5 * ps, vs, vs, vs])
        S = s.P[np.ix_(indices, indices)] + np.diag(sigmas**2)
        if hasattr(s, "gnss_bias_cross"):
            # Marginalize the receiver-error state jointly, including correlation.
            hb = np.vstack((np.eye(3), np.zeros((3, 3))))
            cross = s.gnss_bias_cross[indices]
            S += cross @ hb.T + hb @ cross.T + hb @ s.gnss_bias_covariance @ hb.T
        evidence = []
        for prior, extra in [(0.95, np.zeros(6)),
                             (0.05, np.array([45.**2] * 3 + [3.**2] * 3))]:
            C = S + np.diag(extra)
            sign, logdet = np.linalg.slogdet(C)
            if sign <= 0:
                return -np.inf
            evidence.append(math.log(prior) - 0.5 * (
                6 * math.log(2 * math.pi) + logdet + residual @ np.linalg.solve(C, residual)))
        return float(logsumexp(evidence))

    def close_branches(self):
        for branch in self.branches:
            branch.nav.close()
        self.branches = []

    def reset(self):
        self.close_branches()
        self.window.clear()
        self.window.key = None
        self.dwell, self.previous = 0, None

    def seed(self, snapshot):
        p, P, v = snapshot.pose(self.roads.frame)
        radius = min(100., max(25., 4 * np.sqrt(max(np.linalg.eigvalsh(P)))))
        candidates = self.roads.store.candidates(p, radius)
        speed = np.linalg.norm(v)
        candidates = [e for e in candidates if speed < 2
                      or e.project(p).tangent @ v > 0.25 * speed]
        offsets = [([0, 0], .4), ([-3, 0], .15), ([3, 0], .15),
                   ([0, -3], .15), ([0, 3], .15)]
        # Generator: cold-start fanout does not allocate every native clone at once.
        for edge in candidates:
            for offset, weight in offsets:
                yield Branch(edge.id, self.base.clone(), np.array(offset, dtype=float),
                             math.log(weight / max(1, len(candidates))))

    def expand(self, branch):
        if branch.maneuver is not None:
            yield branch.fork(branch.edge)
            return
        edge = self.roads.store.edges[branch.edge]
        snapshot = branch.nav.snapshot()
        if not snapshot.ready:
            return
        p, _, velocity = snapshot.pose(self.roads.frame)
        projection = edge.project(p - branch.offset)
        lookahead = max(self.config.approach_min_m,
                        self.config.lookahead_s * np.linalg.norm(velocity))
        if not edge.successors or edge.length - projection.s > lookahead:
            yield branch.fork(branch.edge)
            return
        maneuvers = maneuver_paths(self.roads.store, branch.edge, self.config)
        if self._record and self.window.key != branch.edge:
            # Checkpoint contains ALL original observations through this epoch,
            # but no map posterior. Replay starts strictly AFTER that checkpoint.
            if self.window.nav is None:
                self.window.capture(self.base, self.last_t, self.last_score,
                                    branch.edge, self._version)
        prior = -math.log(len(maneuvers))
        for maneuver in maneuvers:
            yield branch.fork(branch.edge, prior, maneuver)

    def condition(self, branch):
        snapshot = branch.nav.snapshot()
        if not snapshot.ready:
            return -np.inf
        p, P, velocity = snapshot.pose(self.roads.frame)
        p = p - branch.offset
        edge = self.roads.store.edges[branch.edge]
        required = (branch.edge,) + (branch.maneuver.path if branch.maneuver else ())
        if any(not self.roads.store.available(edge_id) for edge_id in required):
            raise MissingTopology('Active maneuver geometry was evicted')
        if branch.maneuver is not None:
            old = branch.maneuver
            updated, edge, valid = advance(old, self.roads.store, p, P, velocity, self.config)
            if not valid:
                return -np.inf
            branch.maneuver = updated
            if updated is None:
                branch.edge = old.path[-1]
            if edge is None:
                return 0.0  # Junction support only; no fictitious turning measurement.
        projection = edge.project(p)
        if projection.s < -12 or projection.s > edge.length + 15:
            return -np.inf
        speed = np.linalg.norm(velocity)
        if speed > 2 and projection.tangent @ velocity < .2 * speed:
            return -np.inf
        H = np.zeros(18)
        H[6:9] = projection.normal @ self.roads.frame.from_ned(snapshot.anchor)[:2]
        cross_cov = snapshot.P @ H
        S = float(H @ cross_cov + 1.5**2)
        if S <= 0:
            return -np.inf
        logz, mean, var = interval_moments(projection.cross, S, -edge.half_width, edge.half_width)
        if logz < -12:
            return -np.inf
        cell = (self._version, edge.id, int(math.floor(projection.s / 10.)))
        if cell in branch.cells:
            return 0.0
        dx = cross_cov * ((mean - projection.cross) / S)
        posterior = snapshot.P + np.outer(cross_cov, cross_cov) * (var - S) / S**2
        if np.linalg.norm(dx[:3]) > .25 or np.linalg.norm(dx[15:18]) > .15:
            return -np.inf
        if hasattr(branch.nav, "condition_interval"):
            branch.nav.condition_interval(H, (mean - projection.cross) / S, (var - S) / S**2)
        else:
            branch.nav.inject(dx, posterior)
        branch.cells = branch.cells | {cell}
        self.map_updates += 1
        return logz

    def _select(self, parents):
        """Streaming beam; reserve one representative per maneuver/edge.

        At most cap+1 children coexist, plus the preceding <=cap parents. All
        valid children contribute to the normalization, including evicted ones.
        More distinct modes than the cap causes abstention, not false certainty.
        """
        retained = []
        modes = set()
        total = -np.inf
        try:
            for parent in parents:
                try:
                    for child in self.expand(parent):
                        try:
                            likelihood = self.condition(child)
                        except Exception:
                            child.nav.close()
                            raise
                        if not np.isfinite(likelihood + child.logw):
                            child.nav.close()
                            continue
                        child.logw += likelihood
                        total = float(np.logaddexp(total, child.logw))
                        modes.add(child.mode)
                        retained.append(child)
                        if len(modes) > self.cap:
                            raise HypothesisOverflow('Distinct maneuvers exceed branch cap')
                        if len(retained) > self.cap:
                            counts = {}
                            for b in retained:
                                counts[b.mode] = counts.get(b.mode, 0) + 1
                            duplicate_indices = [i for i, b in enumerate(retained)
                                                 if counts[b.mode] > 1]
                            worst = min(duplicate_indices, key=lambda i: retained[i].logw)
                            retained.pop(worst).nav.close()
                finally:
                    parent.nav.close()
            if not retained:
                return [], 0.0
            keep = float(logsumexp([b.logw for b in retained]))
            discarded = float(np.clip(-np.expm1(keep - total), 0., 1.))
            for branch in retained:
                branch.logw -= keep
            return retained, discarded
        except Exception:
            for branch in retained:
                branch.nav.close()
            raise
        finally:
            # Unvisited parents on a topology/overflow failure must also close.
            if isinstance(parents, list):
                for parent in parents:
                    parent.nav.close()

    def _recover(self, current):
        window = self.window
        if window.nav is None or window.attempted or window.version != self._version:
            return False
        window.attempted = True
        window.status = 'attempted'
        trial = RoadBank(window.nav.clone(), self.roads, self.cap, self.config, recovery=False)
        trial.last_score = window.score_t
        try:
            trial.step(window.t)
            for kind, value in window.events:
                if kind == 'map':
                    trial.step(value)
                else:
                    getattr(trial, kind)(np.asarray(value, dtype=float))
            if (trial.last_t != self.last_t or not trial.branches
                    or not np.array_equal(trial.base.snapshot().raw, current.raw)):
                window.status = 'failed'
                return False
            # REPLACE alternatives, never fuse replay output into the fallback.
            self.branches, trial.branches = trial.branches, []
            self.last_score = trial.last_score
            self.map_updates += trial.map_updates
            self.dwell, self.previous = 0, None
            self.recoveries += 1
            window.status = 'recovered'
            return True
        finally:
            trial.reset()
            trial.base.close()
            # Release raw-event storage immediately after the single attempt.
            window.events.clear()
            if window.nav is not None:
                window.nav.close()
                window.nav = None

    def _output(self, snapshot, discarded, recovered=False):
        p, P, _ = snapshot.pose(self.roads.frame)
        modes = {}
        phases = {}
        def report_key(branch):
            if branch.maneuver and branch.maneuver.phase == 'APPROACH':
                return ('road', branch.maneuver.incoming)
            return branch.mode

        for branch in self.branches:
            key = report_key(branch)
            modes[key] = modes.get(key, 0.) + math.exp(branch.logw)
            phase = branch.maneuver.phase if branch.maneuver else 'ROAD'
            phases[phase] = phases.get(phase, 0) + 1
        mode, weight = max(modes.items(), key=lambda item: item[1])
        # Directed-edge confidence, not OSM way/name confidence. Pending maneuvers
        # are never publishable exits, even with a single remaining candidate.
        pending = any(b.maneuver is not None for b in self.branches)
        traversing = any(b.maneuver and b.maneuver.phase != 'APPROACH' for b in self.branches)
        eligible = (not traversing and mode[0] == 'road' and weight > .97
                    and discarded < .05 and not recovered)
        self.dwell = self.dwell + 1 if eligible and mode == self.previous else int(eligible)
        self.previous = mode if eligible else None
        latched = eligible and self.dwell >= 3
        chosen = [b for b in self.branches if report_key(b) == mode]
        poses = [b.nav.snapshot().pose(self.roads.frame) for b in chosen]
        weights = np.exp([b.logw for b in chosen])
        mean, covariance = mix_gaussians([q[0] for q in poses], [q[1] for q in poses], weights)
        if latched and not pending:
            self.window.clear()
            self.window.key = None
        return dict(status='LATCHED' if latched else 'JUNCTION_AMBIGUOUS' if pending else 'AMBIGUOUS',
                    latched=latched, edge=mode[1] if latched else None,
                    group=self.roads.store.edges[mode[1]].group if latched else None,
                    top_weight=weight, pruned_mass=discarded, branches=len(self.branches), base=snapshot,
                    position=p, covariance=P, conditional_position=mean if latched else p,
                    conditional_covariance=covariance if latched else P,
                    phases=phases, maneuver_modes=len({b.mode for b in self.branches}),
                    future_exit=None, recovery_used=recovered,
                    recovery_status=self.window.status, recoveries=self.recoveries)

    def step(self, t, available=True):
        if not np.isfinite(t) or t <= self.last_t:
            raise ValueError('Repeated/out-of-order map epoch')
        self.last_t = t
        if self._record:
            self.window.append('map', t)
        snapshot = self.base.snapshot()
        out = dict(status='NOT_READY', latched=False, edge=None, group=None, top_weight=0.,
                   pruned_mass=0., branches=0, base=snapshot, phases={}, maneuver_modes=0,
                   recovery_used=False, recovery_status=self.window.status, recoveries=self.recoveries)
        if not snapshot.ready or not snapshot.usable:
            self.reset()
            return out
        p, P, _ = snapshot.pose(self.roads.frame)
        out.update(position=p, covariance=P, conditional_position=p, conditional_covariance=P)
        if not available or not self.roads.covers(snapshot.lla):
            self.reset()
            out['status'] = 'MAP_UNAVAILABLE'
            return out
        if not self.roads.store.is_loaded(p):
            self.reset()
            out['status'] = 'MAP_UNAVAILABLE'
            return out
        version = self.roads.store.version
        if self._version is not None and version != self._version:
            self.reset()
            self._version = version
            out['status'] = 'MAP_CHANGED'
            return out
        self._version = version
        parents = self.branches if self.branches else self.seed(snapshot)
        self.branches = []
        try:
            self.branches, discarded = self._select(parents)
        except (MissingTopology, HypothesisOverflow) as exc:
            self.reset()
            out['status'] = 'MAP_UNAVAILABLE' if isinstance(exc, MissingTopology) else 'HYPOTHESIS_OVERFLOW'
            out['reason'] = str(exc)
            return out
        recovered = False
        if not self.branches:
            recovered = self._recover(snapshot)
            if not recovered:
                self.dwell, self.previous = 0, None
                out['status'] = 'NO_CONSISTENT_ROUTE'
                out['recovery_status'] = self.window.status
                return out
        self.max_pruned = max(self.max_pruned, discarded)
        return self._output(snapshot, discarded, recovered)
