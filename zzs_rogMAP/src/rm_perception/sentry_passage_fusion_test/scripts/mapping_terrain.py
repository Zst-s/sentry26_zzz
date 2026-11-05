"""Mapping-only removal of terrain returns inside confirmed hollow regions."""
import copy
from math import cos, sin, isfinite
import numpy as np
from scipy.spatial.transform import Rotation


def clear_mapping_terrain(terrain, geometry, transform, cells):
    w,h,r,ox,oy=geometry
    rr=terrain.info.resolution
    if (w<=0 or h<=0 or not isfinite(r) or r<=0 or not isfinite(rr) or rr<=0 or
            terrain.info.width<=0 or terrain.info.height<=0 or
            len(terrain.data)!=terrain.info.width*terrain.info.height or
            any(type(i)is not int or not 0<=i<w*h for i in cells)):
        raise ValueError('INVALID_MAPPING_INTERIOR')
    rigid=np.asarray(transform,dtype=float)
    if rigid.shape!=(7,) or not np.isfinite(rigid).all():raise ValueError('INVALID_MAPPING_TRANSFORM')
    roll,pitch,yaw=Rotation.from_quat(rigid[3:]).as_euler('xyz')
    if abs(roll)>.1 or abs(pitch)>.1:raise ValueError('MAPPING_TRANSFORM_NOT_PLANAR')
    out=copy.deepcopy(terrain)
    if not cells:return out
    indices=np.fromiter(cells,dtype=np.int64);c,s=cos(yaw),sin(yaw)
    wx=ox+(indices%w+.5)*r-rigid[0];wy=oy+(indices//w+.5)*r-rigid[1]
    x,y=c*wx+s*wy,-s*wx+c*wy
    # Match TerrainLayer's exact source-cell overlap, including border cells.
    # This may remove the overlapping fraction of one side-wall boundary cell.
    half=.5*r*(abs(c)+abs(s));o=terrain.info.origin.position
    x0=np.floor((x-half-o.x)/rr).astype(int);x1=np.floor((x+half-1e-9-o.x)/rr).astype(int)
    y0=np.floor((y-half-o.y)/rr).astype(int);y1=np.floor((y+half-1e-9-o.y)/rr).astype(int)
    data=np.asarray(terrain.data,dtype=np.int8).reshape(terrain.info.height,terrain.info.width).copy()
    for a,b,d,e in zip(x0,x1,y0,y1):
        a,b=max(0,a),min(terrain.info.width-1,b);d,e=max(0,d),min(terrain.info.height-1,e)
        if a<=b and d<=e:data[d:e+1,a:b+1]=0
    out.data=data.ravel().tolist()
    return out


class MappingTerrainOutput:
    """One processed version per source/region association; never refresh its stamp."""
    def __init__(self):
        self.invalidate()

    def invalidate(self):
        self.source_key = self.scope = self.context = None
        self.transform = self.coverage = self.output = None
        self.reason = 'WAIT_ASSOCIATED_TERRAIN'

    @staticmethod
    def key(terrain):
        stamp=terrain.header.stamp
        return (terrain.header.frame_id,stamp.sec,stamp.nanosec,
                terrain.info,bytes(memoryview(terrain.data)))

    def render(self, terrain, context, memory, now, transform=None):
        from rigid_transform import context_transform,transforms_close
        if memory.state not in ('KNOWN_PASSAGE','UNKNOWN') or not context.valid():
            self.invalidate();self.reason='TERRAIN_ASSOCIATION_UNAVAILABLE';return None
        scope=(context.epoch,context.frame,context.geometry())
        associated=(self.scope==scope and self.context is not None and
                    transforms_close(context_transform(self.context),context_transform(context),.25,.15))
        if not associated:self.invalidate()
        key=self.key(terrain);coverage=frozenset(memory.cells)
        same_source=associated and key==self.source_key
        same_transform=(transform is None or self.transform is not None and
                        np.allclose(transform,self.transform,rtol=0.,atol=1e-9))
        if same_source and coverage==self.coverage and same_transform:
            self.reason='SAME_SOURCE_PROCESSED_OUTPUT';return copy.deepcopy(self.output)
        # A cached exact-time transform may reapply newly admitted geometry to
        # the very same source. It must never be reused for a different scan.
        if same_source and transform is None:transform=self.transform
        age=now-(terrain.header.stamp.sec+terrain.header.stamp.nanosec*1e-9)
        if not same_source and not 0<=age<=.7:
            self.reason='STALE_UNPROCESSED_TERRAIN';return None
        if transform is None:
            self.reason='WAIT_TERRAIN_EXACT_TF';return None
        output=clear_mapping_terrain(terrain,memory._geometry(context),transform,memory.cells)
        self.scope,self.context,self.source_key=scope,context,copy.deepcopy(key)
        self.transform,self.coverage,self.output=tuple(transform),coverage,output
        self.reason='PROCESSED_MAPPING_TERRAIN'
        return copy.deepcopy(output)
