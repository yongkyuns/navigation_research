"""Conservative intersection support, separate from the continuous filter.

A maneuver is an incoming edge plus an explicit connected path to an exit.
Inside its junction envelope the map supplies support, not an exact turning
curve or a second heading observation. No gyro sample is scored twice.
"""
from dataclasses import dataclass, replace
import math
import numpy as np


class MissingTopology(ValueError):
    """A declared successor is unavailable; absence is not negative evidence."""


class HypothesisOverflow(ValueError):
    """The topology/maneuver budget cannot represent all enumerated exits."""


@dataclass(frozen=True)
class JunctionConfig:
    approach_min_m: float = 25.0
    lookahead_s: float = 2.0
    radius_min_m: float = 12.0
    radius_max_m: float = 50.0
    connector_m: float = 15.0
    max_hops: int = 3
    max_paths: int = 32
    exit_cos: float = 0.5
    exit_epochs: int = 2
    recovery_s: float = 20.0
    recovery_events: int = 4096

    def __post_init__(self):
        scalars = (self.approach_min_m, self.lookahead_s, self.radius_min_m,
                   self.radius_max_m, self.connector_m, self.recovery_s)
        if not all(math.isfinite(v) and v > 0 for v in scalars):
            raise ValueError('Junction distances and durations must be positive')
        if (self.radius_max_m < self.radius_min_m or self.max_hops < 1
                or self.max_paths < 1 or self.exit_epochs < 1
                or self.recovery_events < 1 or not 0 < self.exit_cos <= 1):
            raise ValueError('Invalid junction budget or exit policy')


@dataclass(frozen=True)
class Maneuver:
    incoming: str
    path: tuple[str, ...]
    center: tuple[float, float]
    radius: float
    phase: str = 'APPROACH'
    exit_count: int = 0

    @property
    def key(self):
        return ('maneuver', self.incoming, self.path)


def maneuver_paths(store, incoming, cfg):
    """Follow declared, endpoint-connected successors, including short links.

    Missing edges and bounded-search exhaustion fail closed. A geometric
    crossing never introduces connectivity. Cycles terminate without assuming
    a new shortcut through the intersection.
    """
    root = store.edges[incoming]
    paths = []
    stack = [(incoming, (), frozenset((incoming,)))]
    visits = 0
    while stack:
        edge_id, path, visited = stack.pop()
        edge = store.edges[edge_id]
        if path and (edge.length >= cfg.connector_m or not edge.successors):
            paths.append(path)
            continue
        if path and len(path) >= cfg.max_hops:
            # An unresolved short-link chain must not masquerade as a full exit.
            raise HypothesisOverflow('Short-link look-ahead exhausted')
        for successor in sorted(set(edge.successors), reverse=True):
            if successor not in store.edges or not store.available(successor):
                raise MissingTopology(successor)
            target = store.edges[successor]
            if np.linalg.norm(edge.points[-1] - target.points[0]) > 1.0:
                raise MissingTopology('Disconnected successor: ' + successor)
            if successor in visited:
                continue
            visits += 1
            if visits > cfg.max_paths * cfg.max_hops:
                raise HypothesisOverflow('Junction search budget exceeded')
            stack.append((successor, path + (successor,), visited | {successor}))
        if len(paths) + len(stack) > cfg.max_paths:
            raise HypothesisOverflow('Too many junction paths')
    if root.successors and not paths:
        raise HypothesisOverflow('No acyclic exit within look-ahead')
    center = root.points[-1]
    # One envelope for ALL alternatives: no prior preference for short turns.
    radius = max(cfg.radius_min_m, 2.0 * root.half_width)
    for path in paths:
        for edge_id in path:
            edge = store.edges[edge_id]
            radius = max(radius, np.linalg.norm(edge.points[0] - center)
                         + 2.0 * edge.half_width)
    if radius > cfg.radius_max_m:
        raise HypothesisOverflow('Junction envelope exceeds configured bound')
    return [Maneuver(incoming, path, tuple(center), float(radius))
            for path in sorted(paths)]


def advance(maneuver, store, position, covariance, velocity, cfg):
    """Return (new maneuver or None, conditioning edge or None, valid).

    An unresolved junction supplies no interval update. Outgoing direction is
    tested only after clearing the common envelope, with uncertainty margin.
    Two consecutive supported exit epochs precede an ordinary road update.
    """
    incoming = store.edges[maneuver.incoming]
    outgoing = store.edges[maneuver.path[-1]]
    center = np.asarray(maneuver.center)
    delta = position - center
    tangent = incoming.delta[-1] / incoming.lengths[-1]
    sigma = math.sqrt(max(0.0, float(np.linalg.eigvalsh(covariance)[-1])))
    boundary = maneuver.radius + min(15.0, 2.0 * sigma)
    distance = float(np.linalg.norm(delta))
    progress = float(tangent @ delta)
    if maneuver.phase == 'APPROACH' and progress < -boundary:
        return maneuver, incoming, True
    if distance <= boundary:
        return replace(maneuver, phase='JUNCTION', exit_count=0), None, True
    # A reversing vehicle still on the approach must not be labelled an exit.
    if progress < -boundary:
        return replace(maneuver, phase='APPROACH', exit_count=0), incoming, True
    projection = outgoing.project(position)
    speed = float(np.linalg.norm(velocity))
    if projection.s <= 0 or projection.s > outgoing.length + 15.0:
        return maneuver, None, False
    # No course/heading decision while stopped in or just beyond the junction.
    if speed <= 2.0:
        return replace(maneuver, phase='EXIT_PENDING', exit_count=0), None, True
    if projection.tangent @ velocity < cfg.exit_cos * speed:
        return maneuver, None, False
    # Support gate only; the actual covariance-aware interval follows on exit.
    cross_sigma = math.sqrt(max(0.0, float(projection.normal @ covariance @ projection.normal)))
    if abs(projection.cross) > outgoing.half_width + 3.0 * math.hypot(cross_sigma, 1.5):
        return maneuver, None, False
    count = maneuver.exit_count + 1
    if count < cfg.exit_epochs:
        return replace(maneuver, phase='EXIT_PENDING', exit_count=count), None, True
    return None, outgoing, True
