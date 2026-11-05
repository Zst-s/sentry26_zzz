"""Short-lived wall boundary evidence for static hollow-region association.

A wall fit is evidence, not a clearing mask. Only wall_bounded_cells may use it,
anchored to classified hollow history and live free-space checks.
"""
from collections import Counter, deque
from itertools import combinations
from math import isfinite
import numpy as np
from scipy.spatial.transform import Rotation
from rigid_transform import normalize_transform


def within_wall_span(span, lower, upper):
    # Strict physical bounds; ignore sub-micrometre float32/transform noise.
    return lower+1e-6 < span < upper-1e-6


class WallObservationHistory:
    def __init__(self, wall_height_delta_min=.26, wall_height_delta_max=.60, wall_occupancy_ratio_min=.80):
        if (not all(isfinite(v) for v in (wall_height_delta_min,wall_height_delta_max,wall_occupancy_ratio_min)) or
                not 0<=wall_height_delta_min<wall_height_delta_max or
                not 0<=wall_occupancy_ratio_min<=1):
            raise ValueError('INVALID_WALL_THRESHOLDS')
        self.wall_height_delta_min=wall_height_delta_min
        self.wall_height_delta_max=wall_height_delta_max
        self.wall_occupancy_ratio_min=wall_occupancy_ratio_min
        self.frames = deque(maxlen=16)
        self.last_stamp = None
        self.resolution = None
        self._planes = []
        self._dirty = True

    def clear(self):
        self.frames.clear()
        self.last_stamp = self.resolution = None
        self._planes = []
        self._dirty = True

    @staticmethod
    def cloud_points(msg, expected_frame):
        from sensor_msgs.msg import PointField
        if (msg.header.frame_id != expected_frame or msg.point_step <= 0 or
                msg.width*msg.height > 100_000 or msg.width < 0 or msg.height < 0 or
                msg.row_step < msg.width*msg.point_step or
                len(msg.data) != msg.row_step*msg.height or len(msg.data) > 4_000_000):
            raise ValueError('INVALID_WALL_CLOUD')
        fields = {v.name:v for v in msg.fields}
        if len(fields) != len(msg.fields) or any(
                name not in fields or fields[name].datatype != PointField.FLOAT32 or
                fields[name].count != 1 or fields[name].offset < 0 or
                fields[name].offset+4 > msg.point_step for name in ('x','y','z')):
            raise ValueError('INVALID_WALL_CLOUD_FIELDS')
        dtype = np.dtype(dict(names=['x','y','z'],formats=[('>' if msg.is_bigendian else '<')+'f4']*3,
                              offsets=[fields[n].offset for n in ('x','y','z')],itemsize=msg.point_step))
        values = np.ndarray((msg.height,msg.width),dtype=dtype,buffer=msg.data,
                            strides=(msg.row_step,msg.point_step))
        points = np.column_stack([values[n].ravel() for n in ('x','y','z')]).astype(float)
        return points[np.isfinite(points).all(axis=1)]

    def add(self, points, stamp, now, resolution, transform):
        if (not isfinite(stamp) or not isfinite(now) or stamp <= 0 or not 0 <= now-stamp <= .7 or
                not isfinite(resolution) or not 0 < resolution <= 1):
            raise ValueError('STALE_OR_INVALID_WALL_OBSERVATION')
        if self.last_stamp is not None and stamp <= self.last_stamp:
            return False
        points = np.asarray(points,dtype=float)
        if points.ndim != 2 or points.shape[1] != 3 or len(points)>100_000 or not np.isfinite(points).all():
            raise ValueError('INVALID_WALL_POINTS')
        rigid = normalize_transform(transform)
        if self.resolution is not None and abs(resolution-self.resolution)>1e-6:
            self.clear()
        self.resolution = resolution
        # Match the isolated candidate's occupied-column evidence window.
        points = points[(points[:,2]>=-.35)&(points[:,2]<=1.5)]
        columns = {}
        for x,y,z in points:
            key = (int(np.floor(x/resolution)),int(np.floor(y/resolution)))
            columns.setdefault(key,set()).add(round(float(z),5))
        wall = set()
        for key,z in columns.items():
            span=max(z)-min(z)
            if (within_wall_span(span,self.wall_height_delta_min,self.wall_height_delta_max) and
                    (len(z)-1)*resolution/span>=self.wall_occupancy_ratio_min-1e-6):
                wall.add(key)
        # Fitted wall evidence must retain the same column admission policy.
        # Nearby sparse roof returns are not wall support merely by proximity.
        keys=np.floor(points[:,:2]/resolution).astype(np.int64)
        accepted=np.fromiter((tuple(key) in wall for key in keys),dtype=bool,count=len(keys))
        world = Rotation.from_quat(rigid[3:]).apply(points[accepted])+rigid[:3]
        self.frames.append((stamp,wall,world,rigid))
        self.last_stamp=stamp
        self._dirty=True
        return True

    def planes(self, now):
        if not isfinite(now):raise ValueError('INVALID_WALL_CLOCK')
        while self.frames and now-self.frames[0][0]>8.:
            self.frames.popleft();self._dirty=True
        if not self.frames or not 0<=now-self.frames[-1][0]<=.7:
            return []
        if not self._dirty:return self._planes
        seen=Counter(key for _,wall,_,_ in self.frames for key in wall)
        stable={key for key,count in seen.items() if count>=2}
        if not stable:
            self._planes=[];self._dirty=False;return []
        # Sparse components avoid allocating an image spanning an unbounded map.
        groups=[];remaining=set(stable)
        while remaining:
            start=min(remaining);remaining.remove(start);group=[start];queue=[start]
            while queue:
                x,y=queue.pop()
                for dx in (-1,0,1):
                    for dy in (-1,0,1):
                        key=x+dx,y+dy
                        if key in remaining:
                            remaining.remove(key);group.append(key);queue.append(key)
            groups.append(group)
        cloud=np.concatenate([frame[2] for frame in self.frames])
        r=self.resolution;rigid=self.frames[-1][3];rotation=Rotation.from_quat(rigid[3:]);out=[]
        for group in groups:
            centres=np.array([[(x+.5)*r,(y+.5)*r,0]for x,y in group])
            # Reuse the local fit at measured seed locations along the full wall.
            # A single component centroid discarded already observed endpoints.
            world_centres=(rotation.apply(centres)+rigid[:3])[:,:2]
            tiles=np.floor(world_centres/(2*r)).astype(np.int64)
            seeds={}
            for tile,point in zip(tiles,world_centres):
                key=tuple(tile)
                distance=float(np.linalg.norm(point-(tile+.5)*(2*r)))
                if key not in seeds or distance<seeds[key][0]:seeds[key]=(distance,point)
            for _,seed in seeds.values():
                points=cloud[np.linalg.norm(cloud[:,:2]-seed,axis=1)<.15]
                points=np.unique(np.round(points/.005).astype(np.int64),axis=0)*.005
                xy=np.unique(points[:,:2],axis=0);best=None
                # A dense/unbounded candidate is skipped, never allowed to stall the node.
                if len(xy)>128:continue
                for i,j in combinations(range(len(xy)),2):
                    axis=xy[j]-xy[i];length=np.linalg.norm(axis)
                    if length<.05:continue
                    axis/=length;normal=np.array([-axis[1],axis[0]]);origin=(xy[i]+xy[j])*.5
                    if abs((seed-origin)@normal)>.025:continue
                    residual=np.abs((points[:,:2]-origin)@normal);inside=residual<=.025+1e-8;p=points[inside]
                    if len(p)<3 or not within_wall_span(float(np.ptp(p[:,2])),self.wall_height_delta_min,self.wall_height_delta_max):continue
                    score=(len(p),-float(np.mean(residual[inside])))
                    if best is None or score>best[0]:best=(score,origin,normal,p)
                if best is None:continue
                _,origin,normal,support=best
                out.append(dict(tag=len(out),valid=True,origin=origin.tolist(),normal=normal.tolist(),support_xy=np.unique(support[:,:2],axis=0).tolist(),
                                support_z_min=float(support[:,2].min()),support_z_max=float(support[:,2].max()),
                                wall_span_limits=[self.wall_height_delta_min,self.wall_height_delta_max]))
        self._planes=merge_wall_segments(out,r);self._dirty=False
        return self._planes


def fit_support_line(points):
    points=np.asarray(points,dtype=float)
    origin=points.mean(0)
    _,_,axes=np.linalg.svd(points-origin,full_matrices=False)
    normal=axes[-1]
    residual=float(np.max(np.abs((points-origin)@normal)))
    return origin,normal,residual


def parallel_wall_supports(first,second,resolution):
    # Test a shared tangent using each side's own origin; wall separation
    # must not dominate the fit. Reuse the existing half-voxel fit accuracy.
    groups=[np.asarray(wall['support_xy'],dtype=float)for wall in (first,second)]
    centred=np.concatenate([points-points.mean(0)for points in groups])
    return fit_support_line(centred)[2]<=resolution*.5+1e-9


def merge_wall_segments(planes, resolution):
    """Join measured contiguous coplanar segments; never extrapolate their support."""
    groups=[dict(p,support_xy=np.unique(np.asarray(p['support_xy']),axis=0).tolist())for p in planes]
    changed=True
    while changed:
        changed=False
        for i in range(len(groups)):
            for j in range(i+1,len(groups)):
                a,b=groups[i],groups[j]
                pa,pb=np.asarray(a['support_xy']),np.asarray(b['support_xy'])
                # Neighbouring measured fragments only; no long-range line joining.
                if np.min(np.linalg.norm(pa[:,None,:]-pb[None,:,:],axis=2))>resolution+1e-9:continue
                height={}
                if all('support_z_min' in v and 'support_z_max' in v for v in (a,b)):
                    lower=max(v.get('wall_span_limits',[.26,.60])[0] for v in (a,b))
                    upper=min(v.get('wall_span_limits',[.26,.60])[1] for v in (a,b))
                    low=min(a['support_z_min'],b['support_z_min'])
                    high=max(a['support_z_max'],b['support_z_max'])
                    if not within_wall_span(high-low,lower,upper):continue
                    height=dict(support_z_min=low,support_z_max=high,wall_span_limits=[lower,upper])
                p=np.unique(np.concatenate((pa,pb)),axis=0)
                origin,normal,residual=fit_support_line(p)
                if residual>resolution*.5+1e-9:continue
                merged=dict(valid=True,tag=min(a['tag'],b['tag']),origin=origin.tolist(),normal=normal.tolist(),support_xy=p.tolist(),**height)
                groups[i]=merged;groups.pop(j);changed=True;break
            if changed:break
    return groups


def validate_wall_planes(planes):
    for p in planes:
        try:
            origin=np.asarray(p['origin'],dtype=float);normal=np.asarray(p['normal'],dtype=float)
            support=np.asarray(p['support_xy'],dtype=float)
        except (KeyError,TypeError,ValueError) as exc:raise ValueError('INVALID_WALL') from exc
        if (not p.get('valid') or origin.shape!=(2,) or normal.shape!=(2,) or
            support.ndim!=2 or support.shape[1]!=2 or len(support)<2 or
            not np.isfinite(origin).all() or not np.isfinite(normal).all() or
            not np.isfinite(support).all()):raise ValueError('INVALID_WALL')
        if np.linalg.norm(normal)<1e-9:raise ValueError('INVALID_WALL_NORMAL')
        try:
            low,high=float(p['support_z_min']),float(p['support_z_max'])
            lower,upper=p['wall_span_limits']
        except (KeyError,TypeError,ValueError) as exc:raise ValueError('MISSING_WALL_HEIGHT_SUPPORT')from exc
        if (not np.isfinite([low,high,lower,upper]).all() or
                not within_wall_span(high-low,lower,upper)):
            raise ValueError('INVALID_WALL_HEIGHT_SUPPORT')
