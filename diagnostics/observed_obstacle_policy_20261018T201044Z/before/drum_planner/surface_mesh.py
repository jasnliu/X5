"""One depth image -> visible-surface triangles, strictly for visualization.

Neighbors are connected in the camera's original pixel grid, never globally.
Every original pixel in a decimated cell must support a continuous surface.
Holes, depth boundaries and unseen backsides are intentionally NOT filled.
"""
from dataclasses import dataclass

import cv2
import numpy as np

from camera_calibration.core import checked_transform


@dataclass(frozen=True)
class SurfaceMesh:
    vertices: np.ndarray
    faces: np.ndarray
    source_pixels: np.ndarray
    camera_depth_m: np.ndarray
    stats: dict


def depth_metres(raw, scale):
    raw=np.asarray(raw)
    if raw.ndim!=2 or min(raw.shape)<2:raise ValueError('Expected a 2D depth image')
    if not np.isfinite(scale) or not 1e-5<=scale<=.01:raise ValueError('Invalid depth scale')
    depth=raw.astype(np.float32)*scale
    good=np.isfinite(depth)&(raw>0)&(raw<65535)&(depth>=.15)&(depth<=5.)
    return np.where(good,depth,0).astype(np.float32)


def build_surface(raw,scale,meta,transform,*,step=8,depth_jump_m=.03,
                  relative_jump=.02,max_edge_m=.12):
    if not isinstance(step,int) or not 1<=step<=32:raise ValueError('Mesh grid step must be 1–32 pixels')
    if not all(np.isfinite(v) and v>=0 for v in (depth_jump_m,relative_jump,max_edge_m)) or max_edge_m==0:
        raise ValueError('Invalid surface thresholds')
    depth=depth_metres(raw,scale);height,width=depth.shape
    T=checked_transform(transform);K=np.asarray(meta['depth']['K'],dtype=float)
    if K.shape!=(3,3) or not np.isfinite(K).all() or K[0,0]<=0 or K[1,1]<=0:
        raise ValueError('Invalid depth intrinsics')
    # Include the final row/column: decimation never crops the native FOV.
    xs=np.unique(np.r_[np.arange(0,width,step),width-1])
    ys=np.unique(np.r_[np.arange(0,height,step),height-1])
    xx,yy=np.meshgrid(xs,ys);z=depth[yy,xx].ravel()
    points=np.c_[(xx.ravel()-K[0,2])*z/K[0,0],(yy.ravel()-K[1,2])*z/K[1,1],z]
    points=points@T[:3,:3].T+T[:3,3]
    ids=np.arange(len(z)).reshape(len(ys),len(xs))
    a=ids[:-1,:-1].ravel();b=ids[:-1,1:].ravel()
    c=ids[1:,:-1].ravel();d=ids[1:,1:].ravel()
    # Top-left anchored min/max cover ALL pixels between the sampled corners,
    # so a one-pixel invalid strip or depth discontinuity cannot be bridged.
    kernel=np.ones((step+1,step+1),np.uint8)
    low=cv2.erode(depth,kernel,anchor=(0,0),borderType=cv2.BORDER_REPLICATE)
    high=cv2.dilate(depth,kernel,anchor=(0,0),borderType=cv2.BORDER_REPLICATE)
    cell_low=low[np.ix_(ys[:-1],xs[:-1])].ravel()
    cell_high=high[np.ix_(ys[:-1],xs[:-1])].ravel()
    continuous=(cell_low>0)&((cell_high-cell_low)<=depth_jump_m+relative_jump*cell_low)
    faces=np.concatenate([np.c_[a,b,d],np.c_[a,d,c]])
    candidates=len(faces);faces=faces[np.tile(continuous,2)]
    triangles=points[faces]
    edges=triangles-np.roll(triangles,1,axis=1)
    lengths=np.linalg.norm(edges,axis=2)
    area2=np.linalg.norm(np.cross(triangles[:,1]-triangles[:,0],triangles[:,2]-triangles[:,0]),axis=1)
    faces=faces[np.all(lengths<=max_edge_m,axis=1)&(area2>1e-12)]
    used,remap=np.unique(faces.ravel(),return_inverse=True)
    vertices=points[used];faces=remap.reshape(-1,3).astype(np.int32)
    pixels=np.c_[xx.ravel(),yy.ravel()][used].astype(np.int32);camera_z=z[used]
    for array in (vertices,faces,pixels,camera_z):array.setflags(write=False)
    stats=dict(vertices=len(vertices),triangles=len(faces),candidate_triangles=candidates,
               rejected_triangles=candidates-len(faces),grid_step_pixels=step,
               native_resolution=[width,height],valid_pixels=int(np.count_nonzero(depth)),
               depth_jump_m=depth_jump_m,relative_depth_jump=relative_jump,max_edge_m=max_edge_m,
               hole_filling=False,hidden_surface_reconstruction=False,preview_only=True)
    return SurfaceMesh(vertices,faces,pixels,camera_z,stats)
