"""Verified generic-replay inputs. References are loaded only by evaluators."""
from pathlib import Path
import gzip,hashlib,json
import numpy as np
IMU=['t_s','gx_radps','gy_radps','gz_radps','ax_mps2','ay_mps2','az_mps2']
GNSS=['t_s','lat_deg','lon_deg','height_m','vn_mps','ve_mps','vd_mps','pos_std_n_m','pos_std_e_m','pos_std_d_m','vel_std_n_mps','vel_std_e_mps','vel_std_d_mps','heading_rad']

def read_csv(path,expected=None):
    with gzip.open(path,'rt') if str(path).endswith('.gz') else open(path) as f:
        header=f.readline().strip().split(',')
        if expected is not None and header!=expected:raise ValueError(f'Unexpected columns: {path}')
        data=np.loadtxt(f,delimiter=',',ndmin=2)
    if data.shape[1]!=len(header) or not np.isfinite(data[:,0]).all() or np.any(np.diff(data[:,0])<=0):raise ValueError(f'Invalid shape or timestamps: {path}')
    return data

def load_inputs(directory):
    directory=Path(directory);m=json.loads((directory/'manifest.json').read_text());files=m['files'];data={}
    hashes={}
    for name,columns in [('imu',IMU),('gnss',GNSS)]:
        info=files[name];p=directory/info['path'];raw=p.read_bytes();sha=hashlib.sha256(raw).hexdigest()
        if sha!=info['sha256'] or len(raw)!=info['bytes']:raise ValueError(f'Dataset hash mismatch: {p}')
        a=read_csv(p,columns)
        if len(a)!=info['rows']:raise ValueError(f'Dataset row count mismatch: {p}')
        finite=a if name=='imu' else a[:,:13]
        if not np.isfinite(finite).all():raise ValueError(f'Non-finite measurements: {p}')
        data[name]=a;hashes[name]=sha
    return data['imu'],data['gnss'],{'id':m['dataset_id'],'manifest_sha256':hashlib.sha256((directory/'manifest.json').read_bytes()).hexdigest(),'input_sha256':hashes,'rows':{k:len(v) for k,v in data.items()},'reference_available':'reference_position' in files,'source':m.get('source',{})}

def load_reference(directory):
    directory=Path(directory);m=json.loads((directory/'manifest.json').read_text());info=m['files'].get('reference_position')
    if not info:return None
    p=directory/info['path']
    if hashlib.sha256(p.read_bytes()).hexdigest()!=info['sha256']:raise ValueError('Reference hash mismatch')
    return read_csv(p)

def events(imu,gnss):
    """Exactly the upstream replay ordering: IMU first on equal timestamps."""
    i=j=0
    while i<len(imu) or j<len(gnss):
        if i<len(imu) and (j==len(gnss) or imu[i,0]<=gnss[j,0]):yield 'imu',imu[i];i+=1
        else:yield 'gnss',gnss[j];j+=1
