#!/usr/bin/env python3
"""只读静态验收：不编译、不运行 launch、不连接底盘。"""
import argparse
import ast
import copy
from pathlib import Path
import xml.etree.ElementTree as ET

import yaml


def load(path):
    with path.open(encoding="utf-8") as stream:
        return yaml.safe_load(stream)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("workspace", type=Path)
    args = parser.parse_args()
    root = args.workspace.resolve()
    bringup = root / "src/rm_nav_bringup"
    config = bringup / "config/reality"
    old = load(config / "nav2_params_real.yaml")
    new = load(config / "nav2_params_rog_map_real.yaml")
    restored = copy.deepcopy(new)
    expected = {
        "local_costmap": ["terrain_layer", "inflation_layer"],
        "global_costmap": ["static_layer", "terrain_layer", "inflation_layer"],
    }
    for name, plugins in expected.items():
        params = restored[name][name]["ros__parameters"]
        assert params["plugins"] == plugins
        layer = params.pop("terrain_layer")
        assert layer["plugin"] == "sentry_terrain_bridge::TerrainLayer"
        assert layer["topic"] == "/terrain/grid"
        assert layer["data_timeout"] > 0
        params["plugins"] = old[name][name]["ros__parameters"]["plugins"]
    assert restored == old, "发现感知层之外的 Nav2 参数变化"
    print("PASS: Nav2 仅改变感知插件，控制器/规划器/速度/原地图参数保持一致")

    cfg = load(config / "rog_map_real.yaml")["sentry_terrain"]["ros__parameters"]
    rog = cfg["rog_map"]
    assert rog["frame_id"] == "odom"
    assert not rog["ros_callback"]["enable"]
    assert not rog["projection"]["prior_map"]["enable"]
    assert not rog["load_pcd_en"]
    assert rog["decay"]["clear_time"] > rog["decay"]["keep_time"]
    assert cfg["cloud_topic"] == "/cloud_registered_body"
    assert cfg["cloud_coordinates"] == "imu"
    assert cfg["mapping_scan_enabled"] is False
    assert cfg["mapping_scan_frame"] == "livox_frame"
    assert 0 < cfg["mapping_scan_range_min"] < cfg["mapping_scan_range_max"] <= 5.0
    projection = rog["projection"]
    assert projection["wall_height_delta_min"] > projection["surface_height_delta_max"]
    assert projection["tunnel_height_delta_min"] > projection["surface_height_delta_max"]
    assert projection["tunnel_occupancy_ratio_max"] < projection["wall_occupancy_ratio_min"]
    print("PASS: 地形坐标、分类门限、衰减和输入配置自洽")

    for name in ["fastlio_mid360_real.yaml", "pointlio_mid360_real.yaml"]:
        lio = load(config / name)["/**"]["ros__parameters"]
        assert len(lio["mapping"]["extrinsic_T"]) == 3
        assert not lio["mapping"]["extrinsic_est_en"], "在线外参估计需要同步更新感知射线起点"
        assert lio["publish"]["scan_publish_en"]
    launch_source = (bringup / "launch/bringup_real.launch.py").read_text(encoding="utf-8")
    ast.parse(launch_source)
    assert "'/rog_map/scan'" in launch_source
    assert "'mapping_scan_enabled'" in launch_source
    print("PASS: launch 语法及本车固定外参输入")

    packages = {}
    for manifest in (root / "src").rglob("package.xml"):
        if any((parent / "COLCON_IGNORE").exists() for parent in manifest.parents if parent != root):
            continue
        name = ET.parse(manifest).getroot().findtext("name")
        assert name not in packages, f"重复功能包 {name}: {manifest} 与 {packages.get(name)}"
        packages[name] = manifest
    assert "rog_map" in packages and "sentry_terrain_bridge" in packages
    plugin = ET.parse(root / "src/rm_perception/sentry_terrain_bridge/terrain_plugins.xml")
    assert plugin.getroot().find("class").get("type") == "sentry_terrain_bridge::TerrainLayer"
    print(f"PASS: {len(packages)} 个功能包未重名，插件 XML 名称正确")
    print("注意：这些结果不是编译、ROS 通信、运动安全或实车验收通过。")


if __name__ == "__main__":
    main()
