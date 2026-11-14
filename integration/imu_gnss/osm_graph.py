"""Independent OSM -> explicit directed road graph, using canonical node IDs.

This research compiler handles one-way ways and shared-node junctions, NOT turn
restriction relations, conditional access, lane topology, or surveyed road widths.
"""
from collections import Counter,defaultdict
from pathlib import Path
import gzip,hashlib,json
import numpy as np
from native import Frame
from routefusion.tiles import Edge,TileStore

class RoadMap:
    def __init__(self,store,frame,bbox,metadata):
        self.store=store;self.frame=frame;self.bbox=np.array(bbox);self.metadata=metadata
    def covers(self,lla):
        s,w,n,e=self.bbox
        return s<=lla[0]<=n and w<=lla[1]<=e

def compile_osm(path,provenance):
    raw=gzip.decompress(Path(path).read_bytes());meta=json.loads(Path(provenance).read_text())
    if hashlib.sha256(raw).hexdigest()!=meta['response_sha256']:raise ValueError('OSM response hash mismatch')
    doc=json.loads(raw);bbox=meta['bbox_south_west_north_east'];s,w,n,e=bbox
    frame=Frame([(s+n)/2,(w+e)/2,0.]);store=TileStore(256,'osm-'+meta['response_sha256'][:16],frame.origin.tolist())
    nodes={v['id']:v for v in doc['elements'] if v['type']=='node'}
    ways=[v for v in doc['elements'] if v['type']=='way' and v.get('tags',{}).get('motor_vehicle')!='no' and v.get('tags',{}).get('access')!='no']
    counts=Counter(node for way in ways for node in way['nodes']);pending=[];outgoing=defaultdict(list)
    for way in ways:
        ids=way['nodes'];tags=way.get('tags',{});one=tags.get('oneway','yes' if tags.get('junction')=='roundabout' or tags.get('highway')=='motorway' else 'no')
        cuts=sorted({0,len(ids)-1}|{i for i in range(1,len(ids)-1) if counts[ids[i]]>1})
        try:half=float(tags.get('width','12'))/2
        except ValueError:half=6.
        half=float(np.clip(half,3.,12.))
        for a,b in zip(cuts,cuts[1:]):
            section=ids[a:b+1]
            if not all(k in nodes for k in section):raise ValueError('Incomplete OSM nodes')
            dirs=[1] if one in ('yes','1','true') else [-1] if one=='-1' else [1,-1]
            for direction in dirs:
                seq=section if direction==1 else section[::-1]
                xy=frame.project([[nodes[k]['lat'],nodes[k]['lon'],0.] for k in seq])[:,:2]
                keep=np.r_[True,np.linalg.norm(np.diff(xy,axis=0),axis=1)>1e-5];xy=xy[keep]
                if len(xy)<2:continue
                id=f"{way['id']}:{a}:{b}:{direction}"
                entry={'id':id,'points':xy,'start':seq[0],'end':seq[-1],'width':half,'group':str(way['id']),'piece':(way['id'],a,b),'layer':int(tags.get('layer','0')) if tags.get('layer','0').lstrip('-').isdigit() else 0}
                pending.append(entry);outgoing[seq[0]].append(entry)
    for d in pending:
        successors=tuple(x['id'] for x in outgoing[d['end']] if x['piece']!=d['piece'])
        store.add(Edge(d['id'],d['points'],successors,d['width'],d['layer'],d['group']))
    xy=frame.project([[s,w,0],[n,e,0]])[:,:2];store.cover(xy.min(0),xy.max(0))
    return RoadMap(store,frame,bbox,{**meta,'directed_edges':len(store.edges),'compiler_limits':['no turn-restriction relations','unsurveyed width defaults','no lane/height constraints','current map may differ from recording date']})
