"""Opt-in map-only position re-localization, separate from normal navigation.

A small covariance describes one local hypothesis; it is not a reason to exclude
all distant routes forever. After repeated map inconsistency this adapter may
start ONE alternative localization prior. It never inflates the map-free filter,
changes GNSS covariance, invents a GNSS bias, or directly snaps a mean to a road.
The normal finite-width interval factor performs the actual position correction.
This is an explicit model-selection heuristic, not a calibrated integrity model.
"""
from dataclasses import dataclass
import math
import numpy as np
from routefusion.filter import interval_moments
from road_bank import RoadBank, Branch
from intersections import MissingTopology, HypothesisOverflow


@dataclass(frozen=True)
class RelocalizationConfig:
    position_sigma_m: float = 10.0
    max_corridor_excess_m: float = 30.0
    max_native_sigma_m: float = 2.0
    confirmations: int = 3
    min_duration_s: float = 1.8
    max_epoch_gap_s: float = 1.5
    min_travel_m: float = 5.0
    min_speed_mps: float = 2.0
    direction_cos: float = 0.5
    reject_log_probability: float = -12.0

    def __post_init__(self):
        values = (self.position_sigma_m, self.max_corridor_excess_m,
                  self.max_native_sigma_m, self.min_duration_s,
                  self.max_epoch_gap_s, self.min_travel_m, self.min_speed_mps)
        if (not all(math.isfinite(x) and x > 0 for x in values)
                or not isinstance(self.confirmations, int) or self.confirmations < 2
                or not 0 < self.direction_cos <= 1
                or not math.isfinite(self.reject_log_probability)
                or self.reject_log_probability >= 0):
            raise ValueError('Invalid bounded re-localization policy')


def require_search_coverage(roads, position, radius):
    """Do not infer that competing roads are absent from an incomplete cache.

    The complete square queried by TileStore.candidates must be loaded, including
    known-empty tiles. When a bounded extract is declared, the query must also
    lie inside its projected footprint; a loaded boundary tile alone is not proof
    that its out-of-extract portion was downloaded. This is a conservative local
    footprint check, not a substitute for map-version/coverage provenance.
    """
    position = np.asarray(position, dtype=float)
    lo, hi = position - radius, position + radius
    required = roads.store.tile_keys(lo, hi)
    if any(key not in roads.store.loaded for key in required):
        raise MissingTopology('Recovery search overlaps missing map tiles')
    bbox = getattr(roads, 'bbox', None)
    if bbox is not None:
        south, west, north, east = bbox
        footprint = roads.frame.project([[south, west, 0.], [south, east, 0.],
                                         [north, east, 0.], [north, west, 0.]])[:, :2]
        corners = np.array([[lo[0], lo[1]], [hi[0], lo[1]],
                            [hi[0], hi[1]], [lo[0], hi[1]]])
        for start, end in zip(footprint, np.roll(footprint, -1, axis=0)):
            tangent, delta = end - start, corners - start
            if np.any(tangent[0] * delta[:, 1] - tangent[1] * delta[:, 0] < -1e-6):
                raise MissingTopology('Recovery search leaves the declared map extract')


def inconsistent_edges(snapshot, roads, config, edge_ids=None):
    """Return rejected, reachable roads only if NONE of the candidates fits.

    A plausible competing road prevents an automatic loss declaration. Supplied
    route IDs narrow this test only in the explicitly trusted-route adapter.
    No reference coordinate, disturbed-case label or expected displacement enters.
    """
    if not snapshot.ready or not snapshot.usable:
        return ()
    if hasattr(snapshot, 'gnss_bias'):
        raise ValueError('Map-only recovery does not accept a GNSS-bias adapter')
    p, P, velocity = snapshot.pose(roads.frame)
    if (not np.isfinite(p).all() or not np.isfinite(P).all()
            or not np.isfinite(velocity).all()
            or np.linalg.norm(velocity) <= config.min_speed_mps
            or np.linalg.eigvalsh(P)[-1] > config.max_native_sigma_m**2):
        return ()
    if not roads.covers(snapshot.lla) or not roads.store.is_loaded(p):
        return ()
    if edge_ids is None:
        # Add the largest mapped half-width, so the range bound is corridor
        # distance, not inadvertently a smaller centerline-distance bound.
        width = max((e.half_width for e in roads.store.edges.values()), default=0.)
        candidates = roads.store.candidates(p, config.max_corridor_excess_m + width)
    else:
        candidates = []
        for eid in edge_ids:
            if eid not in roads.store.edges or not roads.store.available(eid):
                raise MissingTopology(eid)
            candidates.append(roads.store.edges[eid])
    bad = []
    for edge in candidates:
        if not roads.store.available(edge.id):
            raise MissingTopology(edge.id)
        q = edge.project(p)
        # Finite segments only: an extension behind a junction is not a route.
        if not 0 <= q.s <= edge.length:
            continue
        if q.tangent @ velocity < config.direction_cos * np.linalg.norm(velocity):
            continue
        if max(0., q.distance - edge.half_width) > config.max_corridor_excess_m:
            continue
        variance = float(q.normal @ P @ q.normal + 1.5**2)
        if variance <= 0:
            continue
        logz, _, _ = interval_moments(q.cross, variance, -edge.half_width, edge.half_width)
        if logz >= config.reject_log_probability:
            return ()  # Existing local explanation; do not force another road.
        bad.append(edge.id)
    if bad and edge_ids is None:
        # The absence of a fitting competitor is meaningful ONLY after the
        # entire search region is known. TileStore.candidates omits unloaded
        # tiles; checking just the receiver tile allowed false one-road latches.
        require_search_coverage(roads, p, config.max_corridor_excess_m + width)
    return tuple(sorted(bad))


class LossEvidence:
    """Bounded temporal confirmation, at most one activation per instance."""
    def __init__(self, config):
        self.config = config
        self.used = False
        self.activation_t = None
        self.count = 0
        self.start_t = self.last_t = None
        self.start_p = None
        self.common = frozenset()
        self.version = None
        self.invalidated = False

    def clear_pending(self):
        self.count = 0
        self.start_t = self.last_t = None
        self.start_p = None
        self.common = frozenset()

    def observe(self, t, position, rejected, version):
        if self.version is not None and version != self.version:
            self.invalidated = True
        self.version = version
        if self.used or self.invalidated:
            return False
        if not rejected:
            self.clear_pending()
            return False
        if self.last_t is not None and t <= self.last_t:
            raise ValueError('Recovery epochs must increase')
        current = frozenset(rejected)
        if (self.last_t is None or t-self.last_t > self.config.max_epoch_gap_s
                or not self.common.intersection(current)):
            self.count = 1
            self.start_t, self.start_p, self.common = t, np.array(position).copy(), current
        else:
            self.count += 1
            self.common &= current
        self.last_t = t
        if (self.count >= self.config.confirmations
                and t-self.start_t >= self.config.min_duration_s
                and np.linalg.norm(position-self.start_p) >= self.config.min_travel_m):
            self.used, self.activation_t = True, float(t)
            return True
        return False


def broaden_position(nav, frame, sigma_m):
    """Convolve candidate position with an independent horizontal translation.

    P' = P + E Sigma E^T is PSD and retains all existing cross-covariance.
    No attitude/velocity/bias variance or mean is deliberately changed. The mean
    moves only when the existing interval update is subsequently applied.
    """
    if not math.isfinite(sigma_m) or sigma_m <= 0:
        raise ValueError('Recovery uncertainty must be finite and positive')
    s = nav.snapshot()
    if s.P.shape != (18, 18) or hasattr(s, 'gnss_bias'):
        raise ValueError('Expected the unaugmented 18-state native EKF')
    a = frame.from_ned(s.anchor)[:2]
    P = s.P.copy()
    P[6:9, 6:9] += sigma_m**2 * a.T @ a
    nav.inject(np.zeros(18), P)


class TrustedRouteConditioner:
    """Known-route recovery; caller must explicitly supply a trusted road ID.

    On activation replace ONLY the candidate by a fresh map-free clone with the
    declared localization-loss prior. Original inputs through this epoch are
    already in that clone and must not be replayed/fused again. Previous map
    information is deliberately dropped. The caller owns branch.nav afterwards.
    """
    def __init__(self, roads, config=None):
        self.roads = roads
        self.config = config or RelocalizationConfig()
        self.evidence = LossEvidence(self.config)
        self.version = roads.store.version

    def condition(self, branch, helper, map_free, t):
        s = branch.nav.snapshot()
        if self.roads.store.version != self.version:
            raise MissingTopology('Map version changed during trusted-route recovery')
        p, _, _ = s.pose(self.roads.frame)
        if not self.roads.covers(s.lla) or not self.roads.store.is_loaded(p):
            raise MissingTopology('No loaded map coverage at recovery candidate')
        rejected = inconsistent_edges(s, self.roads, self.config, (branch.edge,))
        if self.evidence.observe(t, p, rejected, self.roads.store.version):
            replacement = map_free.clone()
            try:
                broaden_position(replacement, self.roads.frame, self.config.position_sigma_m)
            except Exception:
                replacement.close()
                raise
            branch.nav.close()
            branch.nav = replacement
            branch.cells = frozenset()  # fresh map-free state, no prior map factors
        return helper.condition(branch)


class RelocalizingRoadBank(RoadBank):
    """Optional automatic loss confirmation and one bounded bank replacement.

    This experiment disables the separate rollback/re-filtering feature: two
    recovery mechanisms must not repeatedly reset uncertainty. On ambiguity it
    keeps alternatives or abstains; a coherent incorrect parallel road remains
    indistinguishable without additional information.
    """
    def __init__(self, base, roads, max_branches=24, config=None, *, relocalization=None):
        super().__init__(base, roads, max_branches, config, recovery=False)
        self.loss_config = relocalization or RelocalizationConfig()
        self.loss = LossEvidence(self.loss_config)

    def _recovery_seed(self, snapshot):
        root = self.base.clone()
        try:
            broaden_position(root, self.roads.frame, self.loss_config.position_sigma_m)
            p, _, v = snapshot.pose(self.roads.frame)
            width = max((e.half_width for e in self.roads.store.edges.values()),default=0.)
            require_search_coverage(self.roads, p, self.loss_config.max_corridor_excess_m+width)
            edges = self.roads.store.candidates(p,self.loss_config.max_corridor_excess_m+width)
            candidates = []
            for edge in edges:
                q = edge.project(p)
                if (0 <= q.s <= edge.length and q.tangent@v >= self.loss_config.direction_cos*np.linalg.norm(v)
                        and max(0.,q.distance-edge.half_width) <= self.loss_config.max_corridor_excess_m):
                    candidates.append(edge)
            offsets = [([0,0],.4),([-3,0],.15),([3,0],.15),([0,-3],.15),([0,3],.15)]
            for edge in candidates:
                for offset,weight in offsets:
                    yield Branch(edge.id,root.clone(),np.array(offset,dtype=float),
                                 math.log(weight/max(1,len(candidates))))
        finally:
            root.close()

    def step(self, t, available=True):
        # Normal path is unchanged; supervisor reads the independent base.
        out = super().step(t,available)
        s = out['base']
        if out['status'] in ('NOT_READY','MAP_UNAVAILABLE','MAP_CHANGED','HYPOTHESIS_OVERFLOW'):
            self.loss.clear_pending()
            if out['status']=='MAP_CHANGED':self.loss.invalidated=True
        else:
            try:
                rejected = inconsistent_edges(s,self.roads,self.loss_config)
                p, _, _ = s.pose(self.roads.frame)
                trigger = self.loss.observe(t,p,rejected,self.roads.store.version)
                if trigger:
                    self.close_branches()
                    self.dwell,self.previous = 0,None
                    self.branches,discarded = self._select(self._recovery_seed(s))
                    if self.branches:
                        self.max_pruned = max(self.max_pruned,discarded)
                        out = self._output(s,discarded,recovered=True)
                    else:
                        out.update(status='NO_CONSISTENT_ROUTE',latched=False,edge=None,group=None,branches=0,
                                   conditional_position=out['position'],conditional_covariance=out['covariance'])
            except (MissingTopology,HypothesisOverflow) as exc:
                self.loss.clear_pending()
                self.reset()
                out.update(status='MAP_UNAVAILABLE' if isinstance(exc,MissingTopology) else 'HYPOTHESIS_OVERFLOW',
                           latched=False,edge=None,group=None,reason=str(exc),branches=0,
                           conditional_position=out['position'],conditional_covariance=out['covariance'])
        out['position_recovery_used'] = self.loss.used
        out['position_recovery_t'] = self.loss.activation_t
        return out
