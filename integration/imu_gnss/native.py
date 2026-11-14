"""Lifetime-managed ctypes adapter to the pinned Rust SensorFusion runtime."""
from __future__ import annotations
import ctypes as C
from pathlib import Path
import numpy as np

POINTER=C.POINTER(C.c_double)

def ecef(lla):
    v=np.asarray(lla,dtype=float);lat,lon=np.deg2rad(v[...,0]),np.deg2rad(v[...,1]);h=v[...,2]
    a=6378137.;e2=6.6943799901413165e-3;rn=a/np.sqrt(1-e2*np.sin(lat)**2)
    return np.stack(((rn+h)*np.cos(lat)*np.cos(lon),(rn+h)*np.cos(lat)*np.sin(lon),(rn*(1-e2)+h)*np.sin(lat)),axis=-1)

def ned_rotation(lla):
    lat,lon=np.deg2rad(np.asarray(lla)[:2]);s,c=np.sin(lat),np.cos(lat);sl,cl=np.sin(lon),np.cos(lon)
    return np.array([[-s*cl,-s*sl,c],[-sl,cl,0],[-c*cl,-c*sl,-s]])

class Frame:
    def __init__(self,origin):
        self.origin=np.array(origin,dtype=float);self.ecef=ecef(self.origin)
        self.R=ned_rotation(self.origin)[[1,0,2],:].copy();self.R[2]*=-1
    def project(self,lla): return (ecef(lla)-self.ecef)@self.R.T
    def from_ned(self,anchor):return self.R@ned_rotation(anchor).T

class Snapshot:
    def __init__(self,data):
        self.raw=data;self.ready=bool(data[0]);self.usable=bool(data[1]);self.state=int(data[2]);self.reason=int(data[3]);self.reanchors=int(data[4]);self.t=data[5];self.events=int(data[6]);self.bytes=int(data[7])
        self.anchor=data[8:11];self.lla=data[11:14];self.velocity=data[14:17];self.q=data[17:21];self.mount=data[21:25];self.position=data[25:28];self.P=data[34:].reshape(18,18)
    def pose(self,frame):
        A=frame.from_ned(self.anchor);p=frame.project(self.anchor)+A@self.position
        return p[:2],(A@self.P[6:9,6:9]@A.T)[:2,:2],A[:2]@self.velocity

class Library:
    def __init__(self,path):
        self.path=Path(path);self.lib=C.CDLL(str(self.path.resolve()))
        self.lib.rf_new.argtypes=[];self.lib.rf_new.restype=C.c_void_p
        self.lib.rf_clone.argtypes=[C.c_void_p];self.lib.rf_clone.restype=C.c_void_p
        self.lib.rf_free.argtypes=[C.c_void_p];self.lib.rf_free.restype=None
        for name in ['rf_imu','rf_gnss','rf_snapshot']:
            fn=getattr(self.lib,name);fn.argtypes=[C.c_void_p,POINTER,C.c_size_t];fn.restype=C.c_int
        self.lib.rf_moments.argtypes=[C.c_void_p,POINTER,POINTER,C.c_size_t];self.lib.rf_moments.restype=C.c_int
    def create(self):return NativeFusion(self,self.lib.rf_new())

class NativeFusion:
    def __init__(self,library,handle):
        if not handle:raise RuntimeError('Native allocation failed')
        self.library=library;self.handle=handle
    def close(self):
        if getattr(self,'handle',None):self.library.lib.rf_free(self.handle);self.handle=None
    def __del__(self):self.close()
    def clone(self):return NativeFusion(self.library,self.library.lib.rf_clone(self.handle))
    def imu(self,row): self._feed('rf_imu',row,7)
    def gnss(self,row):self._feed('rf_gnss',row,14)
    def _feed(self,name,row,n):
        x=np.ascontiguousarray(row,dtype=np.float64)
        if x.shape!=(n,):raise ValueError('Wrong input shape')
        rc=getattr(self.library.lib,name)(self.handle,x.ctypes.data_as(POINTER),n)
        if rc:raise ValueError(f'{name} rejected sample ({rc}) at {x[0]}')
    def snapshot(self):
        x=np.zeros(358,dtype=np.float64)
        if self.library.lib.rf_snapshot(self.handle,x.ctypes.data_as(POINTER),358):raise RuntimeError('Snapshot failed')
        return Snapshot(x)
    def inject(self,dx,P):
        dx=np.ascontiguousarray(dx,dtype=np.float64);P=np.ascontiguousarray((P+P.T)/2,dtype=np.float64)
        if dx.shape!=(18,) or P.shape!=(18,18) or not np.isfinite(dx).all() or not np.isfinite(P).all():raise ValueError('Invalid moment update')
        if np.linalg.eigvalsh(P)[0]<-1e-7*max(1.,np.max(np.diag(P))):raise ValueError('Indefinite map posterior')
        rc=self.library.lib.rf_moments(self.handle,dx.ctypes.data_as(POINTER),P.ctypes.data_as(POINTER),18)
        if rc:raise RuntimeError(f'Moment injection rejected ({rc})')
