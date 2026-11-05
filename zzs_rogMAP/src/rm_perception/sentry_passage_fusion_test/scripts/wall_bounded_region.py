"""Mapping-only hollow interior between measured walls; no live-obstacle veto."""
from itertools import combinations
import numpy as np
from scipy.ndimage import label,binary_erosion
from scipy.spatial import ConvexHull
from passage_region import clip,polygon_cells
from wall_observation import merge_wall_segments,within_wall_span,validate_wall_planes,parallel_wall_supports


def _half_plane(poly, origin, normal):
    normal=np.asarray(normal,dtype=float);normal/=np.linalg.norm(normal)
    basis=np.array([normal,[-normal[1],normal[0]]]);origin=np.asarray(origin)
    local=(np.asarray(poly)-origin)@basis.T
    clipped=clip(local.tolist(),0,0.,True)
    return (np.asarray(clipped)@basis+origin).tolist() if clipped else []



def wall_support_cells(wall, geometry):
    """Rasterize the measured wall strip, including its occupied voxel area."""
    r=geometry[2];normal=np.asarray(wall['normal'],float)
    normal/=np.linalg.norm(normal);axis=np.array([-normal[1],normal[0]])
    basis=np.array([normal,axis]);support=np.asarray(wall['support_xy'])@basis.T
    half=.5*r*np.abs(basis).sum(axis=1)
    lo=support.min(axis=0)-half;hi=support.max(axis=0)+half
    local=np.array([[lo[0],lo[1]],[hi[0],lo[1]],[hi[0],hi[1]],[lo[0],hi[1]]])
    return polygon_cells((local@basis).tolist(),geometry)


def wall_bounded_cells(geometry, grid, observed_cells, planes, *, support_extent="overlap"):
    if support_extent not in ("overlap", "measured_union", "observed_roof"):raise ValueError("INVALID_SUPPORT_EXTENT")
    w,h,r,ox,oy=geometry
    if w<=0 or h<=0 or r<=0 or len(grid)!=w*h:raise ValueError('INVALID_GRID')
    if any(type(i)is not int or not 0<=i<w*h for i in observed_cells):raise ValueError('INVALID_CELLS')
    validate_wall_planes(planes)
    if not planes:return set(),[]
    planes=merge_wall_segments(planes,r)
    observed=np.zeros(w*h,bool);observed[list(observed_cells)]=True
    surface_core=binary_erosion(observed.reshape(h,w),np.ones((3,3))).ravel()
    components,count=label(observed.reshape(h,w),np.ones((3,3)))
    selected=set();details=[];protected=set();wall_cache={}
    for tag in range(1,count+1):
        ids=np.flatnonzero(components.ravel()==tag)
        core_ids=ids[surface_core[ids]]
        if not len(core_ids):continue
        centres=np.array([[ox+(i%w+.5)*r,oy+(i//w+.5)*r]for i in ids]);centre=centres.mean(0)
        corners=np.concatenate([centres+[dx*r/2,dy*r/2]for dx in [-1,1]for dy in [-1,1]])
        roof_polygon=corners[ConvexHull(corners).vertices].tolist() if support_extent == "observed_roof" else None
        core_centres=np.array([[ox+(i%w+.5)*r,oy+(i//w+.5)*r]for i in core_ids])
        choices=[]
        for p,q in combinations(planes,2):
            if not parallel_wall_supports(p,q,r):continue
            # Equal spans at different elevations are not one passage's sides.
            zlo=max(p['support_z_min'],q['support_z_min'])
            zhi=min(p['support_z_max'],q['support_z_max'])
            common_min=max(p['wall_span_limits'][0],q['wall_span_limits'][0])
            if zhi-zlo<=common_min+1e-6:continue
            normals=[];distances=[]
            for wall in [p,q]:
                n=np.array(wall['normal'],float);n/=np.linalg.norm(n)
                if (centre-wall['origin'])@n<0:n=-n
                normals.append(n);distances.append(float((centre-wall['origin'])@n))
            # Distinct opposite faces; prevent combining the two faces of one corner.
            if min(distances)<=r/2 or normals[0]@normals[1]>-2**-.5:continue
            across=normals[0]-normals[1];across/=np.linalg.norm(across);axis=np.array([-across[1],across[0]])
            depths=corners@axis;lo,hi=float(depths.min()),float(depths.max())
            # Physical wall segments, not their infinite fitted planes, bound depth.
            supports=[np.asarray(wall['support_xy'])@axis for wall in [p,q]]
            overlap_lo=max(lo,*(float(v.min()) for v in supports))
            overlap_hi=min(hi,*(float(v.max()) for v in supports))
            if overlap_hi-overlap_lo < r:continue
            if support_extent in ("measured_union", "observed_roof"):
                lo=max(lo,min(float(v.min()) for v in supports))
                hi=min(hi,max(float(v.max()) for v in supports))
            else:
                lo,hi=overlap_lo,overlap_hi
            local=(core_centres@axis>=lo)&(core_centres@axis<=hi)
            for wall,n in zip([p,q],normals):
                local &= (core_centres-np.asarray(wall['origin']))@n > r*.5
            if not np.any(local):continue
            choices.append(((sum(distances),-int(np.count_nonzero(local))),p,q,normals,axis.tolist(),[lo,hi]))
        if not choices:continue
        # Clip in nearest-pair order; rejected farther pairs need no raster work.
        choices.sort(key=lambda a:a[0]);chosen=None
        segments=[]
        if support_extent == "observed_roof":
            # Follow one connected pair of wall chains for this roof component.
            # Other parallel surfaces do not become passage sides by union alone.
            connected=[choices[0]];remaining=choices[1:];near_cache={}
            def touches(a,b):
                key=tuple(sorted((a['tag'],b['tag'])))
                if key not in near_cache:
                    aa=np.asarray(a['support_xy']);bb=np.asarray(b['support_xy'])
                    near_cache[key]=float(np.min(np.linalg.norm(aa[:,None,:]-bb[None,:,:],axis=2)))<=r+1e-9
                return near_cache[key]
            changed=True
            while changed:
                changed=False
                for choice in remaining[:]:
                    p,q=choice[1:3]
                    if any((touches(p,a[1]) and touches(q,a[2])) or
                           (touches(p,a[2]) and touches(q,a[1])) for a in connected):
                        connected.append(choice);remaining=[item for item in remaining if item is not choice];changed=True
            choices=connected
        for _,p,q,normals,axis,depths in choices:
            axis=np.asarray(axis)
            association_depths=depths
            if support_extent == "observed_roof":
                # Wall samples are voxel centres. Their half-cell footprint is
                # supported, but their fitted line must not span the entire roof.
                half_axis=.5*r*float(np.abs(axis).sum())
                supports=[np.asarray(wall['support_xy'])@axis for wall in (p,q)]
                projected=corners@axis
                depths=[max(float(projected.min()),min(float(v.min()) for v in supports)-half_axis),
                        min(float(projected.max()),max(float(v.max()) for v in supports)+half_axis)]
            lo,hi=depths
            # Walls only trim the classified passage footprint; they never
            # create a wider clearing domain outside that observed structure.
            poly=roof_polygon or [[ox,oy],[ox+w*r,oy],[ox+w*r,oy+h*r],[ox,oy+h*r]]
            for wall,n in zip([p,q],normals):
                poly=_half_plane(poly,wall['origin'],n)
                if not poly:break
            if poly:poly=_half_plane(poly,axis*lo,axis)
            if poly:poly=_half_plane(poly,axis*hi,-axis)
            if poly:
                chosen=(poly,[p['tag'],q['tag']],axis.tolist(),depths,association_depths,[(wall,n.tolist())for wall,n in zip([p,q],normals)])
                segments.append(chosen)
                if support_extent != "observed_roof":break
        for chosen in segments:
            poly,tags,axis,depths,association_depths,boundaries=chosen
            walls,normals=zip(*boundaries)
            lo,hi=depths
            cells=polygon_cells(poly,geometry)
            # Keep wall cells: require the whole static cell inside both finite sides.
            half_axis=.5*r*float(np.abs(axis).sum())
            interior=set()
            wall_limits=[.5*r*np.abs(n).sum()+float(np.max(np.abs((np.asarray(wall['support_xy'])-wall['origin'])@n)))
                         for wall,n in zip(walls,normals)]
            for cell in cells:
                centre=np.array([ox+(cell%w+.5)*r,oy+(cell//w+.5)*r])
                if not lo+half_axis-1e-9<=centre@axis<=hi-half_axis+1e-9:continue
                if all((centre-np.asarray(wall['origin']))@n > limit+1e-9
                       for wall,n,limit in zip(walls,normals,wall_limits)):
                    interior.add(cell)
            cells=interior
            wall_cells=set()
            for wall in walls:
                if wall['tag'] not in wall_cache:wall_cache[wall['tag']]=wall_support_cells(wall,geometry)
                wall_cells.update(wall_cache[wall['tag']])
            protected.update(wall_cells)
            selected.update(cells);details.append(dict(component=tag,observed_cells=ids.tolist(),walls=tags,wall_cells=sorted(wall_cells),polygon=poly,axis=axis,depths=depths,association_depths=association_depths,cells=sorted(cells),wall_boundaries=[dict(tag=wall['tag'],origin=wall['origin'],normal=n,support_xy=wall['support_xy'])for wall,n in boundaries]))
    selected.difference_update(protected)
    for detail in details:detail['cells']=sorted(set(detail['cells'])-protected)
    return selected,details
