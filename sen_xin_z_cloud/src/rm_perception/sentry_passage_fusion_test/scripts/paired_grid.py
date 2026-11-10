"""Coherent static-only correction; shared by mapping display and export."""
import copy
from grid_fingerprint import grid_fingerprint

def frozen_navigation_grid(grid, mask, status, now):
    """Apply only an associated, freshly confirmed passage mask to a COPY."""
    if (not isinstance(status, dict) or status.get('virtual_preview') is not False or
            status.get('observation_source') != 'rog_map_hollow_structure' or
            status.get('map_associated') is not True or
            status.get('state') not in ('KNOWN_PASSAGE', 'UNKNOWN') or
            not isinstance(status.get('map_session_id'), str) or
            not status['map_session_id'].startswith('mapping-')):
        detail = ({key: status.get(key) for key in
                   ('state','reason','memory_reason','record_error','map_associated','observation_source','map_session_id')}
                  if isinstance(status,dict) else {'status':'missing or invalid'})
        raise ValueError('Static correction unavailable: '+str(detail))
    if (grid.info != mask.info or grid.header.frame_id != mask.header.frame_id or
            len(mask.data) != len(grid.data) or
            any(value not in (0, 100) for value in mask.data)):
        raise ValueError('Passage mask geometry or values do not match the map')
    stamp = mask.header.stamp.sec + mask.header.stamp.nanosec * 1e-9
    if not 0 <= now-stamp <= 1.0:
        raise ValueError('Passage mask is stale or from the future')
    if (status.get('map_fingerprint') != grid_fingerprint(grid) or
            status.get('mask_fingerprint') != grid_fingerprint(mask)):
        raise ValueError('Passage status does not identify this exact map/mask pair')
    if status['state'] != 'KNOWN_PASSAGE' and any(mask.data):
        raise ValueError('Unknown passage state cannot clear static obstacles')
    result,cleared = apply_static_mask(grid,mask)
    return result, {'cleared_static_cells': len(cleared),
                    'source_map_fingerprint': grid_fingerprint(grid),
                    'mask_fingerprint': grid_fingerprint(mask),
                    'map_session_id': status['map_session_id'],
                    'mask_stamp': [mask.header.stamp.sec, mask.header.stamp.nanosec]}


def apply_static_mask(grid, mask):
    """Apply a caller-associated mask; never read terrain or inflation costs."""
    if (grid.header.frame_id != 'map' or mask.header.frame_id != 'map' or
            grid.info != mask.info or len(grid.data) != len(mask.data) or
            any(value not in (0,100) for value in mask.data)):
        raise ValueError('STATIC_MASK_GEOMETRY_MISMATCH')
    grid_fingerprint(grid)
    result = copy.deepcopy(grid)
    cleared = [i for i, (cell, proof) in enumerate(zip(grid.data, mask.data))
               if cell == 100 and proof == 100]
    for index in cleared:
        result.data[index] = 0
    return result,cleared


def freeze_static_pair(grid,mask,status,now):
    source_stamp=grid.header.stamp.sec+grid.header.stamp.nanosec*1e-9
    mask_stamp=mask.header.stamp.sec+mask.header.stamp.nanosec*1e-9
    if not 0 < source_stamp <= mask_stamp <= now:
        raise ValueError('STATIC_SNAPSHOT_STAMP_MISMATCH')
    corrected,proof=frozen_navigation_grid(grid,mask,status,now)
    proof.update(correction_policy='confirmed_hollow_memory_mask',
                 source_map_stamp=[grid.header.stamp.sec,grid.header.stamp.nanosec],
                 corrected_map_fingerprint=grid_fingerprint(corrected))
    return copy.deepcopy(grid),corrected,proof
