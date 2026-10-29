"""Bounded raw-LiDAR stationarity check, independent of frozen LIO output.
Point-to-plane observability rejects a lone floor/empty scene. Thresholds are
conservative TEST settings, not real-robot qualification.
"""
import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation

def downsample(points):
    p=np.asarray(points,dtype=float).reshape(-1,3)
    p=p[np.isfinite(p).all(axis=1)]
    p=p[(np.linalg.norm(p,axis=1)>.5)&(np.linalg.norm(p,axis=1)<12.)]
    if len(p)<200:raise ValueError('insufficient finite points')
    _,i=np.unique(np.floor(p/.12).astype(np.int32),axis=0,return_index=True)
    p=p[np.sort(i)]
    return p[np.linspace(0,len(p)-1,min(4000,len(p)),dtype=int)]

def displacement(anchor,current):
    a=downsample(anchor);b=downsample(current);tree=cKDTree(a)
    _,neighbors=tree.query(a,k=12)
    local=a[neighbors];center=local.mean(axis=1)
    centered=local-center[:,None,:]
    vals,vecs=np.linalg.eigh(np.einsum('nki,nkj->nij',centered,centered)/12)
    normals=vecs[:,:,0];planar=vals[:,0]<.1*np.maximum(vals[:,1],1e-9)
    r=np.eye(3);t=np.zeros(3);ratio=0.
    for _ in range(8):
        transformed=b@r.T+t
        distances,index=tree.query(transformed)
        valid=(distances<.18)&planar[index]
        if valid.mean()<.5 or valid.sum()<200:raise ValueError('low overlap/planarity')
        p=transformed[valid];n=normals[index[valid]]
        residual=np.einsum('ij,ij->i',n,p-a[index[valid]])
        j=np.column_stack((n,np.cross(p,n)))
        scale=np.sqrt((j*j).mean(axis=0))
        if np.any(scale<1e-4):raise ValueError('unobservable axes')
        v=np.linalg.eigvalsh((j/scale).T@(j/scale))
        ratio=float(v[0]/v[-1])
        if ratio<1e-4:raise ValueError('degenerate geometry')
        delta=np.linalg.lstsq(j,-residual,rcond=None)[0]
        dr=Rotation.from_rotvec(delta[3:]).as_matrix()
        r=dr@r;t=dr@t+delta[:3]
        if np.linalg.norm(delta)<1e-6:break
    moved=b@r.T+t;d,i=tree.query(moved);valid=(d<.18)&planar[i]
    residual=np.einsum('ij,ij->i',normals[i[valid]],moved[valid]-a[i[valid]])
    rmse=float(np.sqrt(np.mean(residual**2)))
    return dict(translation=float(np.linalg.norm(t)),angle=float(np.linalg.norm(Rotation.from_matrix(r).as_rotvec())),
                rmse=rmse,overlap=float(valid.mean()),hessian_ratio=ratio,
                stationary=bool(np.linalg.norm(t)<=.015 and np.linalg.norm(Rotation.from_matrix(r).as_rotvec())<=.015 and rmse<=.02))

class Stillness:
    def __init__(self):self.anchor=None;self.since=None;self.last=-1e20;self.report={}
    def reset(self):self.anchor=None;self.since=None;self.last=-1e20
    def observe(self,points,t):
        if self.anchor is None or t-self.last>.4:
            self.anchor=downsample(points);self.since=t;self.last=t;return False
        result=displacement(self.anchor,points);self.report=result;self.last=t
        if not result['stationary']:self.anchor=downsample(points);self.since=t
        return result['stationary'] and t-self.since>=2.
