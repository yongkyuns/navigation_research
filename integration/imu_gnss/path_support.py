"""Opt-in causal road support, independent of the corrected navigation state.

Only explicit, unbranched endpoint-connected edges can be stitched. No nearest
road switching, invented junction links or averaging of parallel roads occurs.
This is bounded continuation of independently accepted associations, not an HMM.
"""
from dataclasses import dataclass
import hashlib
import math
import numpy as np
from routefusion.tiles import Edge, Projection


def unit(v):
    norm = float(np.linalg.norm(v))
    if not math.isfinite(norm) or norm < 1e-9:
        raise ValueError('Degenerate geometry')
    return v / norm


def cross(a, b):
    return a[..., 0]*b[..., 1]-a[..., 1]*b[..., 0]


def normal(v):
    return np.array([-v[1], v[0]])


def smoothstep(value):
    z = float(np.clip(value, 0., 1.))
    return z*z*(3.-2.*z)


def simplify(points):
    """Remove only collinear forward vertices, not actual road shape."""
    out = []
    for p in np.asarray(points, dtype=float):
        if not np.isfinite(p).all():
            raise ValueError('Nonfinite geometry')
        if out and np.linalg.norm(p-out[-1]) < 1e-8:
            continue
        while len(out) >= 2:
            a, b = out[-1]-out[-2], p-out[-1]
            if a@b <= 0 or abs(float(cross(a, b))) > 1e-8*np.linalg.norm(a)*np.linalg.norm(b):
                break
            out.pop()
        out.append(p.copy())
    if len(out) < 2:
        raise ValueError('Degenerate path')
    return np.array(out)


class SmoothPath:
    """Analytic line/arc fillets with a bounded departure from the input line.

    Fillets exist only at ordinary (<60 degree) internal road vertices. Their
    maximal distance from either original segment is r*(1-cos(turn/2)). Sharp
    vertices are not rounded into an invented turn: their neighborhood fades out.
    """
    def __init__(self, edges, *, deviation_m=.5, trim_m=5.):
        if not edges or not math.isfinite(deviation_m) or not 0 <= deviation_m < 1e6:
            raise ValueError('Invalid geometry uncertainty')
        if not math.isfinite(trim_m) or not 0 < trim_m < 1e6:
            raise ValueError('Invalid corner support length')
        self.edges = tuple(e.id for e in edges)
        self.width = edges[0].half_width
        self.layer = edges[0].layer
        self.points = simplify(np.concatenate([e.points for e in edges]))
        self.polyline = Edge('support', self.points, (), self.width, self.layer)
        identity = np.round(self.points, 8).astype('<f8').tobytes()
        identity += np.array([self.width, self.layer], dtype='<f8').tobytes()
        self.key = hashlib.sha256(identity).hexdigest()[:24]
        self.deviation_m = 0.
        self.primitives, self.sharp = [], []
        self.length = 0.
        start = self.points[0]
        for i in range(1, len(self.points)-1):
            v = self.points[i]
            da, db = v-self.points[i-1], self.points[i+1]-v
            a, b = unit(da), unit(db)
            theta = math.atan2(float(cross(a, b)), float(a@b))
            angle = abs(theta)
            if 1e-7 < angle <= math.pi/3 and deviation_m > 0:
                tan = math.tan(angle/2)
                trim = min(trim_m, .24*np.linalg.norm(da), .24*np.linalg.norm(db),
                           deviation_m*tan/(1-math.cos(angle/2)))
                radius = trim/tan
                first, last = v-trim*a, v+trim*b
                self._line(start, first)
                sign = math.copysign(1., theta)
                center = first + sign*radius*normal(a)
                phi = math.atan2(first[1]-center[1], first[0]-center[0])
                length = radius*angle
                self.primitives.append(('arc', center, radius, phi, theta, self.length, length))
                self.length += length
                self.deviation_m = max(self.deviation_m, radius*(1-math.cos(angle/2)))
                start = last
            else:
                self._line(start, v)
                if angle > math.pi/3:
                    self.sharp.append(self.length)
                start = v
        self._line(start, self.points[-1])

    def _line(self, a, b):
        length = float(np.linalg.norm(b-a))
        if length > 1e-8:
            self.primitives.append(('line', a.copy(), b.copy(), self.length, length))
            self.length += length

    def project(self, p):
        candidates = []
        for i, primitive in enumerate(self.primitives):
            if primitive[0] == 'line':
                _, a, b, offset, length = primitive
                tangent = (b-a)/length
                progress = float((p-a)@tangent)
                clamped = float(np.clip(progress, 0., length))
                point = a + clamped*tangent
                s = offset+clamped
                if i == 0 and progress < 0:
                    s = progress
                if i == len(self.primitives)-1 and progress > length:
                    s = offset+progress
            else:
                _, center, radius, phi, theta, offset, length = primitive
                direction = math.atan2(p[1]-center[1], p[0]-center[0])
                delta = math.atan2(math.sin(direction-phi), math.cos(direction-phi))
                fraction = float(np.clip(delta/theta, 0., 1.))
                angle = phi+fraction*theta
                radial = np.array([math.cos(angle), math.sin(angle)])
                point = center+radius*radial
                tangent = math.copysign(1., theta)*normal(radial)
                s = offset+fraction*length
            n = normal(tangent)
            candidates.append(Projection(point, tangent, n, s, float(n@(p-point)),
                                         float(np.linalg.norm(p-point))))
        candidates.sort(key=lambda q: q.distance)
        best = candidates[0]
        # Separate nearby portions of a looping road are not one Gaussian fix.
        if any(abs(q.s-best.s) > 10 and abs(q.distance-best.distance) < .1
               for q in candidates[1:]):
            raise ValueError('Ambiguous projection on a looping path')
        return best

    def quality(self, s, taper_m):
        distance = min(s, self.length-s, *(abs(s-v) for v in self.sharp))
        return smoothstep(distance/taper_m)


class PathIndex:
    """Canonical paths ignore arbitrary graph segmentation, not true junctions."""
    def __init__(self, roads, *, deviation_m=.5):
        self.roads = roads
        self.version = roads.store.version
        edges = roads.store.edges
        incoming = {eid: [] for eid in edges}
        for edge in edges.values():
            for successor in set(edge.successors):
                if successor in incoming:
                    incoming[successor].append(edge.id)
        following = {}
        for edge in edges.values():
            successors = set(edge.successors)
            if len(successors) != 1:
                continue
            target = edges.get(next(iter(successors)))
            if target is None or len(incoming[target.id]) != 1 or target.id == edge.id:
                continue
            if (target.layer != edge.layer or target.half_width != edge.half_width
                    or np.linalg.norm(edge.points[-1]-target.points[0]) > 1e-6):
                continue
            if unit(edge.delta[-1])@unit(target.delta[0]) < .5:
                continue
            following[edge.id] = target.id
        previous = {b: a for a, b in following.items()}
        self.by_edge, self.paths = {}, {}
        visited = set()
        def add(chain):
            path = SmoothPath([edges[e] for e in chain], deviation_m=deviation_m)
            self.paths[path.key] = path
            for eid in chain:
                self.by_edge[eid] = path
                visited.add(eid)
        for start in sorted(edges):
            if start in previous or start in visited:
                continue
            chain, current = [], start
            while current not in visited and current not in chain:
                chain.append(current)
                if current not in following:
                    break
                current = following[current]
            add(chain)
        # Cycles need a distinct progress model. Leave them unstitched for now.
        for eid in sorted(set(edges)-visited):
            add([eid])


@dataclass(frozen=True)
class SupportConfig:
    max_age_s: float = 8.
    max_distance_m: float = 30.
    geometry_sigma_m: float = 1.5
    taper_m: float = 10.
    direction_cos: float = .5

    def __post_init__(self):
        values = (self.max_age_s, self.max_distance_m, self.geometry_sigma_m, self.taper_m)
        if not all(math.isfinite(x) and 0 < x < 1e6 for x in values):
            raise ValueError('Invalid support configuration')
        if not 0 < self.direction_cos <= 1:
            raise ValueError('Invalid direction gate')


def covered_search(roads, p, radius):
    if any(k not in roads.store.loaded for k in roads.store.tile_keys(p-radius, p+radius)):
        return False
    bbox = getattr(roads, 'bbox', None)
    if bbox is not None:
        south, west, north, east = bbox
        footprint = roads.frame.project([[south, west, 0], [south, east, 0],
                                         [north, east, 0], [north, west, 0]])[:, :2]
        corners = np.array([p+[-radius,-radius], p+[radius,-radius],
                            p+[radius,radius], p+[-radius,radius]])
        for a, b in zip(footprint, np.roll(footprint, -1, axis=0)):
            if np.any(cross(b-a, corners-a) < -1e-6):
                return False
    return True


class CausalSupport:
    def __init__(self, index, config=None):
        self.index, self.config = index, config or SupportConfig()
        self.path = None
        self.anchor_t = None
        self.last_t = -math.inf
        self.invalidated = False

    def select(self, t, snapshot, accepted_edge, association_status=None):
        if not math.isfinite(t) or t <= self.last_t:
            raise ValueError('Support epochs must increase')
        self.last_t = t
        roads, cfg = self.index.roads, self.config
        note = dict(reason='NO_SEED', supported=False, continued=False,
                    anchor_edge=accepted_edge, quality=0., path_key=None, taper_m=cfg.taper_m)
        if roads.store.version != self.index.version:
            self.invalidated = True
        if self.invalidated:
            note['reason'] = 'MAP_CHANGED'
            return None, note
        if association_status is not None and association_status not in (
                'LATCHED', 'AMBIGUOUS', 'JUNCTION_AMBIGUOUS'):
            self.path = None
            self.anchor_t = None
            note['reason'] = 'ASSOCIATION_INVALIDATED'
            return None, note
        if not snapshot.ready or not snapshot.usable:
            note['reason'] = 'NATIVE_UNAVAILABLE'
            return None, note
        p, P, velocity = snapshot.pose(roads.frame)
        if not all(np.isfinite(a).all() for a in (p, P, velocity)):
            raise ValueError('Nonfinite support state')
        if accepted_edge is not None:
            # Invalid new acceptance never silently keeps a previous road.
            self.path = self.index.by_edge.get(accepted_edge)
            self.anchor_t = t
        if self.path is None:
            return None, note
        path = self.path
        note['path_key'] = path.key
        if not roads.covers(snapshot.lla) or not roads.store.is_loaded(p) or any(
                not roads.store.available(e) for e in path.edges):
            note['reason'] = 'MISSING_GEOMETRY'
            return None, note
        try:
            q = path.project(p)
        except ValueError:
            note['reason'] = 'AMBIGUOUS_PROJECTION'
            return None, note
        if not 0 <= q.s <= path.length:
            note['reason'] = 'OUTSIDE_PATH'
            return None, note
        speed = float(np.linalg.norm(velocity))
        if q.distance > cfg.max_distance_m or (speed > 2 and q.tangent@velocity < cfg.direction_cos*speed):
            note['reason'] = 'INCOMPATIBLE_PATH'
            return None, note
        quality = path.quality(q.s, cfg.taper_m)
        if quality <= 1e-9:
            note['reason'] = 'UNSUPPORTED_BOUNDARY'
            return None, note
        if accepted_edge is None:
            if t-self.anchor_t > cfg.max_age_s:
                note['reason'] = 'SUPPORT_EXPIRED'
                return None, note
            if q.distance > path.width+3*math.sqrt(max(0.,float(q.normal@P@q.normal))+cfg.geometry_sigma_m**2):
                note['reason'] = 'CONTINUATION_INCONSISTENT'
                return None, note
            sigma = math.sqrt(max(0.,float(np.linalg.eigvalsh(P)[-1]))+cfg.geometry_sigma_m**2)
            radius = max(e.half_width for e in roads.store.edges.values())+3*sigma
            if radius > 100 or not covered_search(roads, p, radius):
                note['reason'] = 'INCOMPLETE_SEARCH_COVERAGE'
                return None, note
            for edge in roads.store.candidates(p, radius):
                if edge.id in path.edges:
                    continue
                other = edge.project(p)
                plausible = (0 <= other.s <= edge.length
                    and (speed <= 2 or other.tangent@velocity >= cfg.direction_cos*speed)
                    and other.distance <= edge.half_width+3*math.sqrt(
                        max(0.,float(other.normal@P@other.normal))+cfg.geometry_sigma_m**2))
                if plausible:
                    note['reason'] = 'COMPETING_CORRIDOR'
                    return None, note
            note['continued'] = True
        note.update(reason='CONTINUED_PATH' if note['continued'] else 'ACCEPTED_PATH',
                    supported=True, quality=quality, s_m=float(q.s))
        return path, note
