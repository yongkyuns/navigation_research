"""Independent OSM road graph; never constructs geometry from a receiver trace.

OSM node identity establishes connectivity; geometric crossings alone do not.
Widths are priors, not surveyed lane boundaries. Via-way/conditional restrictions
are recorded as unsupported; this is not a production routing compiler.
"""
from __future__ import annotations
import gzip, hashlib, json, math, re, urllib.parse, urllib.request
from collections import Counter, defaultdict
from pathlib import Path
import numpy as np
from routefusion.tiles import Edge,TileStore
from adapter import local_en

ATTRIBUTION='© OpenStreetMap contributors; Open Database License (ODbL) 1.0; https://www.openstreetmap.org/copyright'

def fetch_map(gnss,out:Path):
    out.mkdir(parents=True,exist_ok=True)
    bounds=[float(gnss['lat_deg'].min()-.002),float(gnss['lon_deg'].min()-.003),
            float(gnss['lat_deg'].max()+.002),float(gnss['lon_deg'].max()+.003)]
    bb=','.join(f'{n:.7f}' for n in bounds)
    highway='motorway|trunk|primary|secondary|tertiary|unclassified|residential|living_street|service|motorway_link|trunk_link|primary_link|secondary_link|tertiary_link'
    query=f'[out:json][timeout:90];way["highway"~"^({highway})$"]({bb})->.roads;(.roads;rel(bw.roads)["type"="restriction"];);out geom;'
    path=out/'osm.json.gz'
    if path.exists():
        raw=gzip.decompress(path.read_bytes());doc=json.loads(raw)
        provenance=json.loads((out/'provenance.json').read_text())
        if hashlib.sha256(raw).hexdigest()!=provenance['response_sha256'] or provenance['query']!=query:
            raise ValueError('Map cache checksum/query mismatch')
        return doc,provenance
    errors=[]
    for endpoint in ['https://overpass-api.de/api/interpreter','https://overpass.kumi.systems/api/interpreter']:
        try:
            req=urllib.request.Request(endpoint,data=urllib.parse.urlencode({'data':query}).encode(),
                headers={'User-Agent':'RouteFusion-research-replay/0.1','Content-Type':'application/x-www-form-urlencoded'})
            with urllib.request.urlopen(req,timeout=120) as response:raw=response.read()
            doc=json.loads(raw)
            if 'remark' in doc:raise ValueError('Incomplete Overpass reply: '+doc['remark'])
            if not any(e.get('type')=='way' for e in doc.get('elements',[])):raise ValueError('No roads in response')
            path.write_bytes(gzip.compress(raw,mtime=0))
            provenance={'endpoint':endpoint,'query':query,'bounds_s_w_n_e':bounds,
                        'response_sha256':hashlib.sha256(raw).hexdigest(),
                        'osm_base_timestamp':doc.get('osm3s',{}).get('timestamp_osm_base'),
                        'attribution':ATTRIBUTION,'selection':'Full-trip bounding box selects coverage only; no road geometry derived from GNSS',
                        'temporal_limit':'Map retrieval is not contemporaneous with the recording'}
            (out/'provenance.json').write_text(json.dumps(provenance,indent=2)+'\n')
            (out/'ATTRIBUTION.txt').write_text(ATTRIBUTION+'\n')
            return doc,provenance
        except Exception as exc:errors.append(endpoint+': '+str(exc))
    raise RuntimeError('Map download failed: '+'; '.join(errors))

def compile_osm(doc,origin,bounds,version):
    ways=[e for e in doc['elements'] if e.get('type')=='way' and e.get('nodes') and e.get('geometry')
          and e.get('tags',{}).get('access')!='no' and e.get('tags',{}).get('motor_vehicle')!='no']
    shared=Counter(n for w in ways for n in set(w['nodes']))
    store=TileStore(256,version,list(origin)); records=[]; outgoing=defaultdict(list)
    for w in ways:
        tags=w.get('tags',{}); nodes=w['nodes']; geom=w['geometry']
        if len(nodes)!=len(geom):raise ValueError('Incomplete OSM way geometry')
        xy=local_en([p['lon'] for p in geom],[p['lat'] for p in geom],origin[2],origin)
        cuts=[i for i,n in enumerate(nodes) if i in (0,len(nodes)-1) or shared[n]>1]
        direction=tags.get('oneway','yes' if tags.get('junction')=='roundabout' else 'no')
        if direction in ('reversible','alternating'):continue
        default=4.0 if tags.get('highway')=='service' else 6.0
        width=default
        try:
            token=tags.get('width','')
            if re.fullmatch(r'[0-9.]+(?: m)?',token):width=float(token.split()[0])/2
        except ValueError:pass
        width=float(np.clip(width,3,15))
        for lo,hi in zip(cuts[:-1],cuts[1:]):
            pts=xy[lo:hi+1]; keep=np.r_[True,np.linalg.norm(np.diff(pts,axis=0),axis=1)>1e-5];pts=pts[keep]
            if len(pts)<2:continue
            for reverse in ([True] if direction=='-1' else ([False] if direction in ('yes','1','true') else [False,True])):
                start,end=(nodes[hi],nodes[lo]) if reverse else (nodes[lo],nodes[hi])
                eid=f'osm:{w["id"]}:{nodes[lo]}:{nodes[hi]}:{"r" if reverse else "f"}'
                layer=int(tags.get('layer','0')) if re.fullmatch(r'-?\d+',tags.get('layer','0')) else 0
                edge=Edge(eid,pts[::-1].copy() if reverse else pts.copy(),half_width=width,layer=layer,group=eid)
                rec={'edge':edge,'start':start,'end':end,'way':w['id']}
                records.append(rec);outgoing[start].append(rec)
    restrictions=[];unsupported=0
    for rel in doc['elements']:
        if rel.get('type')!='relation':continue
        members=rel.get('members',[]); via=[m for m in members if m.get('role')=='via']
        fr=[m['ref'] for m in members if m.get('role')=='from' and m.get('type')=='way']
        to=[m['ref'] for m in members if m.get('role')=='to' and m.get('type')=='way']
        tag=rel.get('tags',{}).get('restriction','')
        if len(via)==len(fr)==len(to)==1 and via[0].get('type')=='node' and tag:
            restrictions.append((fr[0],to[0],via[0]['ref'],tag))
        else:unsupported+=1
    for rec in records:
        candidates=[r for r in outgoing[rec['end']] if not (r['way']==rec['way'] and r['end']==rec['start'])]
        for fr,to,via,tag in restrictions:
            if rec['way']==fr and rec['end']==via:
                candidates=[r for r in candidates if (r['way']==to if tag.startswith('only_') else r['way']!=to)]
        rec['edge'].successors=tuple(r['edge'].id for r in candidates)
        store.add(rec['edge'])
    south,west,north,east=bounds
    corners=local_en([west,east,west,east],[south,south,north,north],origin[2],origin)
    # Only fully covered interior grid cells count as available. Edge geometry
    # may extend beyond coverage; it does not declare those tiles complete.
    lo,hi=corners.min(axis=0),corners.max(axis=0)
    store.loaded={k for k in store.tile_keys(lo,hi)
                  if np.all(np.array(k)*store.tile_size>=lo) and np.all((np.array(k)+1)*store.tile_size<=hi)}
    return store,{'directed_edges':len(records),'loaded_tiles':len(store.loaded),
                  'simple_restrictions':len(restrictions),'unsupported_restrictions':unsupported,
                  'widths':'OSM width when parseable, otherwise 12 m road / 8 m service full-width prior',
                  'limitations':'No lane map, time-dependent restrictions, via-way restrictions, or reverse-driving mode'}
