"""Saved static-clearing ranges for decision readers; never an entry permit."""
import hashlib
import json
import math
from pathlib import Path
import yaml


def file_digest(path):
    value = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            value.update(chunk)
    return value.hexdigest()


def read_json(path):
    if Path(path).stat().st_size > 16_000_000:
        raise ValueError('Passage metadata is too large')
    with Path(path).open(encoding='utf-8') as stream:
        result = json.load(stream)
    if not isinstance(result, dict):
        raise ValueError('Expected a JSON object')
    return result


def load_passage_ranges(map_yaml):
    """Read only ranges bound by the manifest to this exact saved YAML/image.

    Region IDs belong to one snapshot. Neither region membership nor a
    representative point asserts current clearance or an entry/exit direction.
    """
    map_yaml = Path(map_yaml).absolute()
    if map_yaml.suffix != '.yaml':
        raise ValueError('Expected the corrected map YAML path')
    ranges_path = map_yaml.with_suffix('.passages.json')
    manifest = read_json(map_yaml.with_suffix('.registration.json'))
    ranges = read_json(ranges_path)
    if (manifest.get('version') != 1 or manifest.get('frame') != 'map' or
            ranges.get('version') != 1 or ranges.get('frame') != 'map' or
            ranges.get('units') != 'm' or ranges.get('map_yaml') != map_yaml.name or
            ranges.get('semantics') != 'confirmed_passage_clearing_mask_4_connected_regions' or
            ranges.get('ids_scope') != 'this_saved_snapshot_only'):
        raise ValueError('Unsupported passage metadata or wrong map name')
    try:
        with map_yaml.open(encoding='utf-8') as stream:
            metadata = yaml.safe_load(stream)
    except yaml.YAMLError as error:
        raise ValueError('Invalid map YAML') from error
    image_name = metadata['image']
    if not isinstance(image_name, str) or Path(image_name).name != image_name:
        raise ValueError('Expected a map image next to its YAML')
    files = manifest.get('files', {})
    for path in (map_yaml, map_yaml.parent / image_name, ranges_path):
        if files.get('map/' + path.name) != file_digest(path):
            raise ValueError('Saved map/ranges hash mismatch: ' + path.name)
    proof = manifest.get('static_correction', {})
    for key in ('source_map_fingerprint', 'mask_fingerprint', 'map_session_id', 'mask_stamp'):
        if key not in ranges or ranges[key] != proof.get(key):
            raise ValueError('Passage proof does not match the saved map: ' + key)
    grid = ranges['grid']
    width, height = grid['width'], grid['height']
    resolution = grid['resolution']
    origin = grid['origin_xy']
    yaw = grid['origin_yaw']
    if (type(width) is not int or type(height) is not int or min(width, height) <= 0 or
            type(resolution) not in (int, float) or not math.isfinite(resolution) or resolution <= 0 or
            not isinstance(origin, list) or len(origin) != 2 or
            any(type(v) not in (int, float) or not math.isfinite(v) for v in origin + [yaw]) or
            grid.get('span_end_exclusive') is not True):
        raise ValueError('Invalid passage grid')
    expected = [resolution] + origin + [yaw]
    actual = [metadata['resolution']] + metadata['origin']
    if len(actual) != 4 or any(not math.isfinite(float(v)) for v in actual) or any(
            not math.isclose(float(x), float(y), rel_tol=1e-7, abs_tol=1e-7)
            for x, y in zip(expected, actual)):
        raise ValueError('Passage geometry differs from the map YAML')
    regions = ranges['regions']
    if not isinstance(regions, list):
        raise ValueError('Invalid passage regions')
    rows = {}
    for number, region in enumerate(regions, 1):
        if type(region.get('id')) is not int or region['id'] != number:
            raise ValueError('Invalid passage region ID')
        count = 0
        for span in region['cell_spans_y_xbegin_xend']:
            if (len(span) != 3 or any(type(v) is not int for v in span) or
                    not 0 <= span[0] < height or not 0 <= span[1] < span[2] <= width):
                raise ValueError('Invalid passage cell span')
            rows.setdefault(span[0], []).append((span[1], span[2]))
            count += span[2] - span[1]
        if type(region.get('cell_count')) is not int or count <= 0 or count != region['cell_count']:
            raise ValueError('Invalid passage cell count')
        for key, size in (('center_xy', 2), ('representative_point_xy', 2), ('bounds_xy', 4)):
            values = region[key]
            if (not isinstance(values, list) or len(values) != size or
                    any(type(v) not in (int, float) or not math.isfinite(v) for v in values)):
                raise ValueError('Invalid passage coordinates: ' + key)
    for intervals in rows.values():
        ordered = sorted(intervals)
        if any(left[1] > right[0] for left, right in zip(ordered, ordered[1:])):
            raise ValueError('Overlapping passage regions')
    return ranges


def passage_regions(mask, proof):
    """Describe exact confirmed mask cells, without changing or extending them."""
    width, height = mask.info.width, mask.info.height
    resolution = float(mask.info.resolution)
    ox, oy = mask.info.origin.position.x, mask.info.origin.position.y
    q = mask.info.origin.orientation
    yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
    def world(x,y):
        return [ox+resolution*(math.cos(yaw)*x-math.sin(yaw)*y),
                oy+resolution*(math.sin(yaw)*x+math.cos(yaw)*y)]
    remaining = {i for i, value in enumerate(mask.data) if value == 100}
    regions = []
    # Sorted seeds give deterministic IDs within this saved snapshot only.
    for seed in sorted(remaining):
        if seed not in remaining:
            continue
        remaining.remove(seed)
        cells, pending = [], [seed]
        while pending:
            cell = pending.pop()
            cells.append(cell)
            x, y = cell % width, cell // width
            for nx, ny in ((x-1,y),(x+1,y),(x,y-1),(x,y+1)):
                neighbor = ny*width+nx
                if 0 <= nx < width and 0 <= ny < height and neighbor in remaining:
                    remaining.remove(neighbor)
                    pending.append(neighbor)
        rows = {}
        for cell in cells:
            rows.setdefault(cell//width, []).append(cell % width)
        spans = []
        for y, xs in sorted(rows.items()):
            xs.sort()
            start = last = xs[0]
            for x in xs[1:]:
                if x != last+1:
                    spans.append([y, start, last+1])
                    start = x
                last = x
            spans.append([y, start, last+1])
        xs, ys = [c % width for c in cells], [c//width for c in cells]
        cx, cy = sum(xs)/len(cells), sum(ys)/len(cells)
        inside = min(cells, key=lambda c: ((c % width-cx)**2+(c//width-cy)**2, c))
        corners=[world(x,y) for x in (min(xs),max(xs)+1) for y in (min(ys),max(ys)+1)]
        regions.append(dict(id=len(regions)+1, cell_count=len(cells),
            center_xy=world(cx+.5,cy+.5),
            representative_point_xy=world(inside % width+.5,inside//width+.5),
            bounds_xy=[min(p[0] for p in corners),min(p[1] for p in corners),
                       max(p[0] for p in corners),max(p[1] for p in corners)],
            cell_spans_y_xbegin_xend=spans))
    return dict(version=1, frame='map', units='m',
        semantics='confirmed_passage_clearing_mask_4_connected_regions',
        ids_scope='this_saved_snapshot_only',
        source_map_fingerprint=proof['source_map_fingerprint'],
        mask_fingerprint=proof['mask_fingerprint'],
        map_session_id=proof['map_session_id'], mask_stamp=proof['mask_stamp'],
        grid=dict(resolution=resolution, width=width, height=height,
                  origin_xy=[ox,oy], origin_yaw=yaw, span_end_exclusive=True), regions=regions)

