#!/usr/bin/env python3
"""Freeze the current ZLIO bundle, SLAM graph and verified corrected grid."""
import argparse
import copy
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import time

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import DurabilityPolicy, QoSProfile
from slam_toolbox.srv import SerializePoseGraph
from std_msgs.msg import String
from std_srvs.srv import Trigger

from ament_index_python.packages import get_package_prefix
sys.path.insert(0, str(Path(get_package_prefix('sentry_passage_fusion_test')) /
                       'lib/sentry_passage_fusion_test'))
from paired_grid import freeze_static_pair
from passage_ranges import file_digest as digest, passage_regions



class Exporter:
    def __init__(self):
        self.node = rclpy.create_node('zlio_mapping_export')
        self.map = self.mask = self.status = None
        self.received = {}
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.node.create_subscription(OccupancyGrid, '/map',
                                      lambda msg: self.set('map', msg), latched)
        self.node.create_subscription(OccupancyGrid, '/passage/verified_static_clearing',
                                      lambda msg: self.set('mask', msg), 1)
        self.node.create_subscription(String, '/passage/memory_status',
                                      lambda msg: self.set_status(msg), 1)

    def set(self, key, msg):
        setattr(self, key, msg)
        self.received[key] = time.monotonic()

    def set_status(self, msg):
        try:
            self.set('status', json.loads(msg.data))
        except (ValueError, TypeError):
            self.status = None

    def call(self, service_type, name, request, timeout=60):
        client = self.node.create_client(service_type, name)
        try:
            if not client.wait_for_service(timeout_sec=5):
                raise RuntimeError(f'服务不可用: {name}')
            future = client.call_async(request)
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout)
            if not future.done() or future.result() is None:
                raise RuntimeError(f'服务超时: {name}')
            return future.result()
        finally:
            self.node.destroy_client(client)

    def capture(self):
        deadline = time.monotonic() + 10
        reason = '等待 /map、清障掩码和状态'
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            if self.map is None or self.mask is None or self.status is None:
                continue
            if any(time.monotonic() - self.received.get(key, 0) > 1
                   for key in ('mask', 'status')):
                reason = '清障证据过期'
                continue
            try:
                mask = copy.deepcopy(self.mask)
                raw, corrected, proof = freeze_static_pair(
                    copy.deepcopy(self.map), mask, copy.deepcopy(self.status),
                    self.node.get_clock().now().nanoseconds * 1e-9)
                return raw, corrected, proof, passage_regions(mask, proof)
            except (ValueError, TypeError, OverflowError) as error:
                reason = str(error)
        raise RuntimeError('无法冻结已验证二维地图: ' + reason)

    def save_grid(self, grid, prefix):
        topic = '/zlio_export/grid_' + prefix.name
        publisher = self.node.create_publisher(
            OccupancyGrid, topic,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        command = ['ros2', 'run', 'nav2_map_server', 'map_saver_cli', '-f', str(prefix),
                   '-t', topic, '--ros-args', '-p', 'map_subscribe_transient_local:=true',
                   '-p', 'save_map_timeout:=10.0']
        process = subprocess.Popen(command, stdout=subprocess.PIPE,
                                   stderr=subprocess.STDOUT, text=True)
        try:
            deadline = time.monotonic() + 30
            while process.poll() is None and time.monotonic() < deadline:
                publisher.publish(grid)
                rclpy.spin_once(self.node, timeout_sec=0.05)
            if process.poll() is None:
                raise RuntimeError('保存二维图超时')
            output = process.communicate()[0]
            if process.returncode:
                raise RuntimeError('map_saver_cli 失败: ' + output[-2000:])
        finally:
            if process.poll() is None:
                process.terminate()
                process.communicate(timeout=5)
            self.node.destroy_publisher(publisher)


def export(workspace, name):
    source_root = workspace / 'src/rm_nav_bringup'
    installed_root = workspace / 'install/rm_nav_bringup/share/rm_nav_bringup'
    roots = [source_root]
    if installed_root.is_dir() and installed_root.resolve() != source_root.resolve():
        roots.append(installed_root)
    map_dir = source_root / 'map'
    bundle = workspace / 'maps' / (name + '_raw')
    if not all((root / folder).is_dir() for root in roots for folder in ('map', 'PCD')):
        raise RuntimeError('缺少 rm_nav_bringup 的 map 或 PCD 目录')
    suffixes = ('.yaml', '.pgm', '_raw.yaml', '_raw.pgm',
                '.posegraph', '.data', '.registration.json', '.passages.json')
    if any(os.path.lexists(path) for root in roots for path in
           [root / 'PCD' / name] + [root / 'map' / (name + suffix) for suffix in suffixes]):
        raise RuntimeError('目标地图名已存在，拒绝覆盖')
    rclpy.init()
    worker = Exporter()
    try:
        stopped = worker.call(Trigger, '/stop', Trigger.Request(), timeout=30)
        if not stopped.success:
            raise RuntimeError('/stop 失败: ' + stopped.message)
        saved = worker.call(Trigger, '/backend_node/save_map', Trigger.Request(), timeout=180)
        if not saved.success:
            raise RuntimeError('ZLIO 地图包保存失败: ' + saved.message)
        expected_message = 'saved runtime map to ' + str(bundle)
        if saved.message != expected_message:
            raise RuntimeError(f'后端保存路径与地图名不一致: {saved.message!r}；预期 {bundle}')
        if not (bundle / 'keyframes.txt').is_file() or not (bundle / 'map.pcd').is_file():
            raise RuntimeError(f'后端输出不在预期位置: {bundle}；请保持 mapping.sh 的 WORLD 一致')
        with tempfile.TemporaryDirectory(prefix='.zlio-export-', dir=map_dir) as directory:
            stage = Path(directory)
            graph = worker.call(SerializePoseGraph, '/slam_toolbox/serialize_map',
                                SerializePoseGraph.Request(filename=str(stage / name)),
                                timeout=180)
            if graph.result != SerializePoseGraph.Response.RESULT_SUCCESS:
                raise RuntimeError('SLAM posegraph 保存失败')
            raw, corrected, proof, ranges = worker.capture()
            worker.save_grid(corrected, stage / name)
            worker.save_grid(raw, stage / (name + '_raw'))
            ranges['map_yaml'] = name + '.yaml'
            (stage / (name + '.passages.json')).write_text(
                json.dumps(ranges, indent=2, allow_nan=False) + '\n')
            required = [name + suffix for suffix in
                        ('.yaml', '.pgm', '_raw.yaml', '_raw.pgm', '.posegraph', '.data', '.passages.json')]
            if any(not (stage / item).is_file() or not (stage / item).stat().st_size
                   for item in required):
                raise RuntimeError('二维图或 posegraph 输出不完整')
            manifest = {'version': 1, 'lio': 'zlio', 'frame': 'map',
                        'bundle': name, 'static_correction': proof,
                        'source_map_stamp': [raw.header.stamp.sec,
                                             raw.header.stamp.nanosec],
                        'files': {}}
            for item in required:
                manifest['files']['map/' + item] = digest(stage / item)
            for path in sorted(bundle.rglob('*')):
                if path.is_file():
                    manifest['files']['PCD/' + name + '/' + str(path.relative_to(bundle))] = \
                        digest(path)
            (stage / (name + '.registration.json')).write_text(
                json.dumps(manifest, indent=2, allow_nan=False) + '\n')
            # 先准备完整的先验包；所有公开名称只允许首次创建。
            published = []
            try:
                for root in roots:
                    staging_bundle = root / 'PCD' / ('.' + name + '.staging')
                    if os.path.lexists(staging_bundle):
                        raise RuntimeError(f'残留暂存目录: {staging_bundle}')
                    shutil.copytree(bundle, staging_bundle)
                    for item in required + [name + '.registration.json']:
                        destination = root / 'map' / item
                        if not any(destination.resolve() == path.resolve()
                                   for path in published):
                            os.link(stage / item, destination)
                            published.append(destination)
                    bundle_destination = root / 'PCD' / name
                    if not any(bundle_destination.resolve() == path.resolve()
                               for path in published):
                        staging_bundle.rename(bundle_destination)
                        published.append(bundle_destination)
                    elif staging_bundle.exists():
                        shutil.rmtree(staging_bundle)
            except BaseException:
                for path in reversed(published):
                    if path.is_dir():
                        shutil.rmtree(path)
                    else:
                        path.unlink()
                raise
            finally:
                for root in roots:
                    staging_bundle = root / 'PCD' / ('.' + name + '.staging')
                    if staging_bundle.exists():
                        shutil.rmtree(staging_bundle)
        print(f'已保存 map/{name}.yaml、map/{name}.posegraph 、map/{name}.passages.json 和 PCD/{name}/')
    finally:
        worker.node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('workspace', type=Path)
    parser.add_argument('name')
    arguments = parser.parse_args()
    if not arguments.name.isascii() or not arguments.name.replace('_', '').replace('-', '').isalnum():
        parser.error('地图名只能包含英文字母、数字、下划线和短横线')
    try:
        export(arguments.workspace.resolve(), arguments.name)
    except Exception as error:
        sys.exit(f'地图导出失败: {error}')
