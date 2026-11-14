"""Compile an EXPLICIT DIRECTED road graph in GeoJSON into local route tiles.

This is not an OSM/Overture topology resolver. The server-side exporter must
supply directed edges, restrictions and successor IDs; crossings alone never
establish connectivity. GeoJSON coordinates are longitude, latitude[,height].
Height defaults to the local origin height. Local patches are limited to 10 km.
"""
import argparse,json,math
from pathlib import Path
import numpy as np
from .tiles import TileStore,Edge

def ecef(lon,lat,h=0.):
    lon,lat=math.radians(lon),math.radians(lat)
    a=6378137.; e2=6.6943799901413165e-3
    n=a/math.sqrt(1-e2*math.sin(lat)**2)
    return np.array([(n+h)*math.cos(lat)*math.cos(lon),
                     (n+h)*math.cos(lat)*math.sin(lon),
                     (n*(1-e2)+h)*math.sin(lat)])

def enu(points,origin):
    lon,lat,h=origin
    la,lo=math.radians(lat),math.radians(lon)
    R=np.array([[-math.sin(lo),math.cos(lo),0],
                [-math.sin(la)*math.cos(lo),-math.sin(la)*math.sin(lo),math.cos(la)],
                [math.cos(la)*math.cos(lo),math.cos(la)*math.sin(lo),math.sin(la)]])
    o=ecef(lon,lat,h)
    local=np.array([R@(ecef(p[0],p[1],p[2] if len(p)>2 else h)-o) for p in points])
    if len(local) and np.linalg.norm(local[:,:2],axis=1).max()>10_000:
        raise ValueError('local prototype patch exceeds 10 km; rebase/partition it')
    return local[:,:2]

def compile_graph(doc,origin,version,tile_size=256):
    if doc.get('type')!='FeatureCollection': raise ValueError('expected GeoJSON FeatureCollection')
    store=TileStore(tile_size,version,list(origin))
    for f in doc['features']:
        geom=f['geometry']; p=f['properties']
        if geom['type']!='LineString': raise ValueError('only LineString graph edges supported')
        for key in ('edge_id','successors','half_width_m'):
            if key not in p: raise ValueError('missing directed-graph property: '+key)
        store.add(Edge(str(p['edge_id']),enu(geom['coordinates'],origin),
                       tuple(map(str,p['successors'])),float(p['half_width_m']),
                       int(p.get('layer',0)),str(p.get('group',p['edge_id']))))
    # Cover the local patch's bounding rectangle, including empty cells. A real
    # compiler must explicitly distinguish surveyed-empty from not-downloaded.
    if store.edges:
        points=np.concatenate([e.points for e in store.edges.values()])
        store.cover(points.min(axis=0)-50,points.max(axis=0)+50)
    return store

def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('geojson'); ap.add_argument('output')
    ap.add_argument('--origin',type=float,nargs=3,metavar=('LON','LAT','HEIGHT'),required=True)
    ap.add_argument('--version',required=True)
    ap.add_argument('--tile-size',type=float,default=256)
    args=ap.parse_args()
    doc=json.loads(Path(args.geojson).read_text())
    store=compile_graph(doc,args.origin,args.version,args.tile_size)
    store.save(args.output)
    print(f'Wrote {len(store.loaded)} tiles and {len(store.edges)} canonical directed edges')

if __name__=='__main__': main()
