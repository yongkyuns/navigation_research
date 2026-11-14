"""Recorded generic IMU/PVT -> experimental planar RouteFusion inputs.

Calibration uses a disjoint prefix of raw IMU/PVT, never reference streams or map.
The evaluated suffix uses an IMU-only complementary tilt adapter. This is NOT
an integration of the source project's full 3-D ESKF. See README.md limitations.
"""
from __future__ import annotations
import gzip, hashlib, json, math
from pathlib import Path
import numpy as np
from scipy.ndimage import median_filter
from scipy.spatial.transform import Rotation
from routefusion.filter import N, PlanarFilter, FilterConfig, PVT

SOURCE_SHA = '8a34212c7befe30f8d3eae4ad708cc0dcd4aa0d6'

def load_stream(folder: Path, kind: str):
    meta = json.loads((folder/'manifest.json').read_text())['files'][kind]
    path = folder/meta['path']
    data = path.read_bytes()
    if hashlib.sha256(data).hexdigest() != meta['sha256']:
        raise ValueError('Input checksum mismatch: '+str(path))
    with gzip.open(path, 'rt') as f:
        a = np.genfromtxt(f, delimiter=',', names=True)
    if a.ndim != 1 or len(a) != meta['rows']:
        raise ValueError('Input row count mismatch')
    if not np.isfinite(a['t_s']).all() or np.any(np.diff(a['t_s']) <= 0):
        raise ValueError('Nonfinite or non-increasing timestamps')
    required = [k for k in a.dtype.names if k != 'heading_rad']
    if any(not np.isfinite(a[k]).all() for k in required):
        raise ValueError('Nonfinite required data')
    return a

def vectors(a, names):
    return np.column_stack([a[n] for n in names])

def local_en(lon, lat, height, origin):
    """WGS84 ECEF to a single local EN frame; suitable for these short trips."""
    def ecef(lo, la, h):
        lo, la = np.deg2rad(lo), np.deg2rad(la)
        n = 6378137.0/np.sqrt(1-6.6943799901413165e-3*np.sin(la)**2)
        return np.stack(((n+h)*np.cos(la)*np.cos(lo), (n+h)*np.cos(la)*np.sin(lo),
                         (n*(1-6.6943799901413165e-3)+h)*np.sin(la)),axis=-1)
    lo, la, h = origin
    delta = ecef(np.asarray(lon),np.asarray(lat),np.asarray(height))-ecef(lo,la,h)
    lo, la = np.deg2rad([lo,la])
    rotation = np.array([[-np.sin(lo),np.cos(lo),0],
                         [-np.sin(la)*np.cos(lo),-np.sin(la)*np.sin(lo),np.cos(la)]])
    return delta@rotation.T

def calibrate(imu, gnss, cutoff):
    """Fixed mount calibration using only data at or before cutoff."""
    a = imu[imu['t_s'] <= cutoff]
    g = gnss[gnss['t_s'] <= cutoff]
    if len(g)<12 or len(a)<100: raise ValueError('Insufficient calibration prefix')
    at, gt = a['t_s'], g['t_s']
    acc = vectors(a,['ax_mps2','ay_mps2','az_mps2'])
    gyro = vectors(a,['gx_radps','gy_radps','gz_radps'])
    speed = np.hypot(g['ve_mps'],g['vn_mps'])
    prev = np.clip(np.searchsorted(gt,at,side='right')-1,0,len(gt)-1)
    stationary = (speed[prev]<0.4)&(np.linalg.norm(gyro,axis=1)<0.08)
    if stationary.sum()<100:
        raise ValueError('No adequate stationary calibration interval; mount not assumed')
    up = np.median(acc[stationary],axis=0); gravity = float(np.linalg.norm(up)); up /= gravity
    bias = np.median(gyro[stationary],axis=0)
    seed = np.eye(3)[int(np.argmin(abs(up)))]
    u = seed-up*np.dot(seed,up); u /= np.linalg.norm(u)
    v = np.cross(up,u)
    speed_s = median_filter(speed,size=3,mode='nearest')
    course = np.arctan2(g['vn_mps'],g['ve_mps']) # CCW from east
    hs, targets = [], []
    for k in range(1,len(g)):
        dt = gt[k]-gt[k-1]
        if not 0.1<dt<2.5 or min(speed[k],speed[k-1])<2: continue
        delta = float((course[k]-course[k-1]+np.pi)%(2*np.pi)-np.pi)
        target = np.array([(speed_s[k]-speed_s[k-1])/dt,
                           .5*(speed[k]+speed[k-1])*delta/dt])
        mask = (at>=gt[k-1])&(at<gt[k])
        if mask.sum()<3 or np.linalg.norm(target)>5 or abs(delta)>.35: continue
        mean = acc[mask].mean(axis=0)
        hs.append([mean@u,mean@v]); targets.append(target)
    X,Y = np.asarray(hs),np.asarray(targets)
    if len(X)<10 or float(np.std(Y,axis=0).max())<.15:
        raise ValueError('Mount yaw unobservable in calibration prefix')
    X0,Y0 = X-X.mean(axis=0),Y-Y.mean(axis=0)
    U,s,Vt = np.linalg.svd(X0.T@Y0)
    Q = U@np.diag([1.,np.linalg.det(U@Vt)])@Vt
    forward = u*Q[0,0]+v*Q[1,0]
    residual = X@Q-Y
    return {'cutoff_s':float(cutoff),'up_body':up.tolist(),'forward_body':forward.tolist(),
            'gyro_bias_radps':bias.tolist(),'accel_bias_planar':residual.mean(axis=0).tolist(),
            'gravity_norm':gravity,'stationary_samples':int(stationary.sum()),
            'fit_intervals':len(X),'fit_singular_values':s.tolist(),
            'fit_rms_mps2':float(np.sqrt(np.mean((residual-residual.mean(axis=0))**2))),
            'method':'prefix-only stationary tilt/gyro bias and proper-rotation GNSS acceleration fit'}

def planar_controls(imu, calibration):
    """Causal IMU-only tilt propagation after a fixed prefix calibration.

    Gyro propagation plus a 60-second accelerometer tilt correction when the
    force norm is close to calibrated gravity. Sustained acceleration can bias
    this approximation. No suffix GNSS, references or map enters this function.
    """
    times = imu['t_s']
    acc = vectors(imu,['ax_mps2','ay_mps2','az_mps2'])
    gyro = vectors(imu,['gx_radps','gy_radps','gz_radps'])-calibration['gyro_bias_radps']
    up = np.array(calibration['up_body']); forward = np.array(calibration['forward_body'])
    result = np.zeros((len(imu),4)); result[:,0]=times
    for i in range(len(imu)):
        dt = 0.0 if i==0 else times[i]-times[i-1]
        if dt>.2: raise ValueError('IMU gap >0.2 s requires explicit reseed, not implicit filling')
        if i:
            up = Rotation.from_rotvec(-gyro[i-1]*dt).apply(up)
        norm = np.linalg.norm(acc[i])
        if abs(norm-calibration['gravity_norm'])<.5 and np.linalg.norm(gyro[i])<.15:
            blend = -math.expm1(-dt/60.)
            up = (1-blend)*up+blend*acc[i]/norm
        up /= np.linalg.norm(up)
        tilt = float(forward@up)
        if abs(tilt)>.9: raise ValueError('Vehicle forward axis nearly vertical; planar model invalid')
        f = forward-up*tilt; f /= np.linalg.norm(f)
        left = np.cross(up,f)
        yaw_rate = (gyro[i]@up-tilt*(gyro[i]@forward))/(1-tilt*tilt)
        result[i,1:] = [acc[i]@f,acc[i]@left,yaw_rate]
    return result

def predict_to(tracker, controls, start, end, max_increment=.05):
    """Use only already observed samples; preserve time integrals in <=50ms packets."""
    if not start<end or end>controls[-1,0]+1e-9: raise ValueError('Invalid propagation interval')
    t=start; idx=max(0,int(np.searchsorted(controls[:,0],t,side='right')-1))
    packet_start=t; acc=np.zeros(2); gyro=0.
    while t<end-1e-10:
        boundary=min(end,packet_start+max_increment,
                     controls[idx+1,0] if idx+1<len(controls) else end)
        dt=boundary-t
        if dt<=1e-12:
            idx+=1; continue
        acc+=controls[idx,1:3]*dt; gyro+=controls[idx,3]*dt; t=boundary
        if t>=packet_start+max_increment-1e-10 or t>=end-1e-10:
            elapsed=t-packet_start
            tracker.predict(acc/elapsed,gyro/elapsed,elapsed)
            packet_start=t;acc=np.zeros(2);gyro=0.
        if idx+1<len(controls) and t>=controls[idx+1,0]-1e-10:idx+=1

def initialize(row, xy, calibration, epoch):
    x=np.zeros(N); x[:2]=xy; x[2:4]=[row['ve_mps'],row['vn_mps']]
    x[4]=math.atan2(x[3],x[2]);x[5:7]=calibration['accel_bias_planar']
    std=np.array([max(3,row['pos_std_e_m']),max(3,row['pos_std_n_m']),
                  max(.5,row['vel_std_e_mps']),max(.5,row['vel_std_n_mps']),
                  math.radians(15),.5,.5,math.radians(.3),15,15,5,5,.5,.5])
    config=FilterConfig(accel_density=.2,gyro_density=math.radians(.3),
                        accel_bias_rw=.01,gyro_bias_rw=math.radians(.01))
    return PlanarFilter(x,np.diag(std**2),config,last_pvt_epoch=epoch)

def observation(row,xy,epoch):
    std=np.array([max(3,row['pos_std_e_m']),max(3,row['pos_std_n_m']),
                  max(.5,row['vel_std_e_mps']),max(.5,row['vel_std_n_mps'])])
    return PVT(epoch,float(row['t_s']),xy.copy(),np.array([row['ve_mps'],row['vn_mps']]),np.diag(std**2))
