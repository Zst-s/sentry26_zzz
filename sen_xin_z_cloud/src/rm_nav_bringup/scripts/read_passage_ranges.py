#!/usr/bin/env python3
"""Print map-bound static-clearing regions as JSON for decision readers."""
import argparse
import json
from pathlib import Path
import sys
from ament_index_python.packages import get_package_prefix

sys.path.insert(0, str(Path(get_package_prefix('sentry_passage_fusion_test')) /
                       'lib/sentry_passage_fusion_test'))
from passage_ranges import load_passage_ranges


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('map_yaml', type=Path, help='Exact corrected map YAML used by navigation')
    args = parser.parse_args()
    try:
        ranges = load_passage_ranges(args.map_yaml)
    except (OSError, ValueError, TypeError, KeyError, AttributeError, OverflowError) as error:
        parser.exit(2, f'洞口范围读取失败: {error}\n')
    print(json.dumps(ranges, indent=2, allow_nan=False))


if __name__ == '__main__':
    main()
