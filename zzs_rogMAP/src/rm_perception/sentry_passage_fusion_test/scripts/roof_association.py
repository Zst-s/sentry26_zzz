"""Associate finite wall evidence with measured hollow height intervals.

Missing interior returns are allowed; no connected raw roof is required.
"""
import numpy as np
from scipy.ndimage import label
from scipy.spatial.transform import Rotation
from scipy.spatial import cKDTree
from wall_bounded_region import wall_bounded_cells


def world_column_samples(parts):
    rows=[]
    for part in parts:
        a=np.asarray(part['columns'],float);tr=part['map_transform'];rot=Rotation.from_quat(tr[3:])
        low=rot.apply(a[:,:3])+tr[:3];up=a[:,:3].copy();up[:,2]=a[:,3];up=rot.apply(up)+tr[:3]
        rows.append(np.column_stack((low,up)))
    return np.concatenate(rows)if rows else np.empty((0,6))


def associated_wall_bounded_cells(geometry, grid, observed_cells, planes, samples, *, support_extent="measured_union"):
    if support_extent not in ("measured_union", "observed_roof"):
        raise ValueError("INVALID_ASSOCIATION_EXTENT")
    w, h, r, ox, oy = geometry
    # Reuse established input validation before constructing component arrays.
    wall_bounded_cells(geometry, grid, set(), planes)
    samples = np.asarray(samples, dtype=float)
    if samples.ndim != 2 or samples.shape[1] != 6 or not np.isfinite(samples).all():
        raise ValueError('INVALID_ROOF_COLUMNS')
    if any(type(i) is not int or not 0 <= i < w*h for i in observed_cells):
        raise ValueError('INVALID_ROOF_CELLS')
    mask = np.zeros(w*h, dtype=bool)
    mask[list(observed_cells)] = True
    tags, count = label(mask.reshape(h, w), np.ones((3, 3)))
    positions = np.floor((samples[:, :2] - [ox, oy])/r).astype(int)
    valid = ((positions[:, 0] >= 0) & (positions[:, 0] < w) &
             (positions[:, 1] >= 0) & (positions[:, 1] < h))
    sample_tags = np.zeros(len(samples), dtype=int)
    sample_tags[valid] = tags[positions[valid, 1], positions[valid, 0]]
    selected, details = set(), []
    for tag in range(1, count+1):
        ids = set(int(i) for i in np.flatnonzero(tags.ravel() == tag))
        component_samples = samples[sample_tags == tag]
        if not len(component_samples):
            continue
        component_positions = positions[sample_tags == tag]
        columns, inverse = np.unique(component_positions, axis=0, return_inverse=True)
        intervals = np.array([np.median(component_samples[inverse == index][:, [2, 5]], axis=0)
                              for index in range(len(columns))])
        tree = cKDTree(np.array([ox, oy])+(columns+.5)*r)
        associated = []
        for wall in planes:
            support = np.asarray(wall['support_xy'], dtype=float)
            _, nearest = tree.query(support)
            # The wall sample must lie in this roof cell or its immediate
            # grid neighbours; height similarity cannot associate remote walls.
            roof_centres=np.array([ox,oy])+(columns[nearest]+.5)*r
            adjacent=np.all(np.abs(support-roof_centres)<=1.5*r+1e-9,axis=1)
            if np.count_nonzero(adjacent)<2:
                continue
            support=support[adjacent];nearest=nearest[adjacent]
            # Containing a lower unrelated roof is insufficient: both measured
            # height endpoints must agree for the same finite wall support.
            if (abs(float(intervals[nearest, 0].min())-wall['support_z_min']) > r+1e-9 or
                    abs(float(intervals[nearest, 1].max())-wall['support_z_max']) > r+1e-9):
                continue
            inside = ((intervals[nearest, 0] >= wall['support_z_min'] - r) &
                      (intervals[nearest, 1] <= wall['support_z_max'] + r))
            if np.count_nonzero(inside) >= 2:
                associated.append(dict(wall, support_xy=support[inside].tolist()))
        cells, regions = wall_bounded_cells(geometry, grid, ids, associated, support_extent=support_extent)
        for region in regions:
            region['component'] = tag
        selected.update(cells)
        details.extend(regions)
    return selected, details
