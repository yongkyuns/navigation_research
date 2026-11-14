"""Small versioned route-tile format: topology is explicit, never inferred
from 2-D line crossings. Tiles are JSON+gzip for inspectability, not a claim
of production wire-format optimality. All geometry uses one local metric frame.
"""
from __future__ import annotations
from dataclasses import dataclass
from pathlib import Path
import gzip, json, math
import numpy as np

@dataclass
class Projection:
    point: np.ndarray
    tangent: np.ndarray
    normal: np.ndarray
    s: float
    cross: float
    distance: float

@dataclass
class Edge:
    id: str
    points: np.ndarray
    successors: tuple[str,...] = ()
    half_width: float = 6.0
    layer: int = 0
    group: str = ''

    def __post_init__(self):
        self.points = np.asarray(self.points, dtype=float)
        if self.points.ndim != 2 or self.points.shape[1] != 2 or len(self.points)<2:
            raise ValueError('edge geometry must contain at least two 2-D points')
        self.delta = np.diff(self.points,axis=0)
        self.lengths = np.linalg.norm(self.delta,axis=1)
        if np.any(self.lengths < 1e-6) or self.half_width <= 0:
            raise ValueError('degenerate edge or nonpositive width')
        self.cumulative = np.r_[0,np.cumsum(self.lengths)]
        self.length = float(self.cumulative[-1])
        if not self.group:
            self.group = self.id

    def project(self, p) -> Projection:
        t = np.sum((p-self.points[:-1])*self.delta,axis=1)/(self.lengths**2)
        tc = np.clip(t,0,1)
        q = self.points[:-1]+tc[:,None]*self.delta
        i = int(np.argmin(np.sum((q-p)**2,axis=1)))
        tangent = self.delta[i]/self.lengths[i]
        normal = np.array([-tangent[1],tangent[0]])
        # Extend only the terminal segment's arc coordinate for endpoint tests;
        # nearest point itself remains within the actual finite polyline.
        frac = float(tc[i])
        if i==0 and t[i]<0: frac=float(t[i])
        if i==len(self.lengths)-1 and t[i]>1: frac=float(t[i])
        s = float(self.cumulative[i]+frac*self.lengths[i])
        return Projection(q[i],tangent,normal,s,float(normal@(p-q[i])),
                          float(np.linalg.norm(p-q[i])))

    def to_dict(self):
        return dict(id=self.id,points=self.points.tolist(),successors=list(self.successors),
                    half_width=self.half_width,layer=self.layer,group=self.group)

class TileStore:
    def __init__(self, tile_size=256.0, version='synthetic-v1', origin_wgs84=None):
        self.tile_size=float(tile_size); self.version=version
        self.origin_wgs84=origin_wgs84
        self.edges: dict[str,Edge] = {}
        self.tile_edges: dict[tuple[int,int],set[str]] = {}
        self.loaded: set[tuple[int,int]] = set()

    def key(self,p):
        return (math.floor(p[0]/self.tile_size),math.floor(p[1]/self.tile_size))

    def tile_keys(self, lo, hi):
        a,b=self.key(lo),self.key(hi)
        return [(i,j) for i in range(a[0],b[0]+1) for j in range(a[1],b[1]+1)]

    def add(self, edge: Edge):
        old=self.edges.get(edge.id)
        if old is not None and old.to_dict()!=edge.to_dict():
            raise ValueError('conflicting canonical edge ID')
        self.edges[edge.id]=edge
        margin=edge.half_width+5
        for key in self.tile_keys(edge.points.min(axis=0)-margin,
                                  edge.points.max(axis=0)+margin):
            self.loaded.add(key)
            self.tile_edges.setdefault(key,set()).add(edge.id)

    def cover(self,lo,hi):
        self.loaded.update(self.tile_keys(np.array(lo),np.array(hi)))

    def is_loaded(self,p):
        return self.key(p) in self.loaded

    def candidates(self,p,radius):
        ids=set()
        for key in self.tile_keys(p-radius,p+radius):
            if key in self.loaded: ids.update(self.tile_edges.get(key,()))
        return [self.edges[i] for i in sorted(ids)
                if self.edges[i].project(p).distance <= radius]

    def available(self,id):
        if id not in self.edges: return False
        return any(id in self.tile_edges.get(k,()) for k in self.loaded)

    def save(self,directory):
        directory=Path(directory); directory.mkdir(parents=True,exist_ok=True)
        for key in sorted(self.loaded):
            doc=dict(schema=1,version=self.version,tile_size=self.tile_size,key=list(key),
                     frame='local EN metres; one shared origin',origin_wgs84=self.origin_wgs84,
                     edges=[self.edges[i].to_dict() for i in sorted(self.tile_edges.get(key,()))])
            data=json.dumps(doc,separators=(',',':')).encode()
            with open(directory/f'{key[0]}_{key[1]}.json.gz','wb') as f:
                f.write(gzip.compress(data,mtime=0))

    @classmethod
    def load(cls,directory):
        result=None
        for path in sorted(Path(directory).glob('*.json.gz')):
            with gzip.open(path,'rt') as f: doc=json.load(f)
            if doc['schema']!=1: raise ValueError('unsupported tile schema')
            if result is None: result=cls(doc['tile_size'],doc['version'],doc.get('origin_wgs84'))
            if (result.version!=doc['version'] or result.tile_size!=doc['tile_size']
                or result.origin_wgs84!=doc.get('origin_wgs84')):
                raise ValueError('cannot mix tile versions or coordinate grids')
            key=tuple(doc['key']); result.loaded.add(key)
            result.tile_edges.setdefault(key,set())
            for d in doc['edges']:
                edge=Edge(d['id'],np.array(d['points']),tuple(d['successors']),
                          d['half_width'],d['layer'],d['group'])
                old=result.edges.get(edge.id)
                if old is not None and old.to_dict()!=edge.to_dict():
                    raise ValueError('conflicting canonical edge ID')
                result.edges[edge.id]=edge
                result.tile_edges[key].add(edge.id)
        if result is None: raise ValueError('no route tiles found')
        return result
