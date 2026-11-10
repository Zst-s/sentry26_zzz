"""不启动硬件，验证坐标/速度转换和实机 launch 分支。"""
import importlib.util
from pathlib import Path

import numpy as np
import pytest
from scipy.spatial.transform import Rotation
from launch import LaunchContext
from launch.actions import GroupAction
from launch_ros.actions import Node
from launch_ros.utilities import evaluate_parameters
from nav_msgs.msg import Odometry

PACKAGE = Path(__file__).resolve().parents[1]
ZLOCAL = PACKAGE.parent / 'z-localization'


def load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


adapter = load('zlio_nav_adapter', PACKAGE / 'scripts/zlio_nav_adapter.py')
fake_velocity = load('fake_vel_transform', PACKAGE / 'scripts/fake_vel_transform.py')
launch = load('bringup_real', PACKAGE / 'launch/bringup_real.launch.py')


def message():
    msg = Odometry()
    msg.header.frame_id = 'odom'
    msg.header.stamp.sec = 12
    msg.child_frame_id = 'imu'
    msg.pose.pose.orientation.w = 1.
    msg.pose.covariance = np.eye(6).ravel().tolist()
    msg.twist.covariance = np.eye(6).ravel().tolist()
    return msg


def test_rotating_robot_center_has_zero_linear_speed():
    # 车体原点静止，IMU 在车体前方 1 m，机器人以 2 rad/s 旋转。
    msg = message()
    msg.pose.pose.position.x = 1.
    msg.twist.twist.linear.y = 2.
    msg.twist.twist.angular.z = 2.
    result = adapter.body_odometry(msg, np.array([-1., 0., 0.]), Rotation.identity())
    assert result.pose.pose.position.x == 0.
    assert result.twist.twist.linear.y == 0.
    assert result.twist.twist.angular.z == 2.
    assert result.header == msg.header
    assert result.child_frame_id == 'base_link'
    assert msg.child_frame_id == 'imu'
    assert msg.twist.twist.linear.y == 2.


def test_fake_odometry_rotates_real_velocity_and_covariance():
    msg = message()
    msg.child_frame_id = 'base_link'
    msg.pose.pose.orientation.z = np.sqrt(.5)
    msg.pose.pose.orientation.w = np.sqrt(.5)
    msg.twist.twist.linear.x = 1.0
    msg.twist.twist.angular.z = 0.4
    covariance = np.diag([1., 4., 9., 16., 25., 36.])
    msg.twist.covariance = covariance.ravel().tolist()
    result = fake_velocity.fake_odometry(msg)
    assert result.child_frame_id == 'base_link_fake'
    assert result.header == msg.header
    assert result.twist.twist.linear.x == pytest.approx(0., abs=1e-12)
    assert result.twist.twist.linear.y == pytest.approx(1.)
    assert result.twist.twist.angular.z == pytest.approx(0.4)
    np.testing.assert_allclose(np.diag(np.asarray(result.twist.covariance).reshape(6, 6)),
                               [4., 1., 9., 25., 16., 36.], atol=1e-12)
    assert msg.twist.twist.linear.x == 1.0
    assert msg.twist.twist.angular.z == 0.4


def test_mount_rotation_and_covariance():
    msg = message()
    msg.twist.twist.linear.x = 1.
    rotation = Rotation.from_euler('z', 90, degrees=True)
    result = adapter.body_odometry(msg, np.zeros(3), rotation)
    assert result.twist.twist.linear.x == pytest.approx(0., abs=1e-12)
    assert result.twist.twist.linear.y == pytest.approx(-1.)
    np.testing.assert_allclose(np.array(result.twist.covariance).reshape(6, 6), np.eye(6), atol=1e-12)


def test_lever_arm_pose_and_covariance():
    msg = message()
    msg.pose.pose.orientation.z = np.sqrt(.5)
    msg.pose.pose.orientation.w = np.sqrt(.5)
    result = adapter.body_odometry(msg, np.array([1., 0., 0.]), Rotation.identity())
    assert result.pose.pose.position.y == pytest.approx(1.)
    covariance = np.array(result.pose.covariance).reshape(6, 6)
    assert covariance[0, 0] == pytest.approx(2.)
    assert covariance[0, 5] == pytest.approx(-1.)
    assert np.linalg.eigvalsh(covariance).min() > 0


def test_actual_urdf_and_lidar_geometry():
    xml = launch.xacro.process_file(str(PACKAGE / 'urdf/sentry_robot_real.xacro')).toxml()
    transform = launch.sensor_geometry(xml, ZLOCAL / 'config/lio.yaml')
    np.testing.assert_allclose(transform[:3, 3], [-.011, -.19329, -.19588])
    np.testing.assert_allclose(transform[:3, :3], np.eye(3))


@pytest.mark.parametrize('mode', ['nav', 'mapping'])
@pytest.mark.parametrize('driver', ['true', 'false'])
def test_launch_modes(tmp_path, monkeypatch, mode, driver):
    calls = []
    def record(package, filename, arguments):
        calls.append((package, filename, arguments))
        return GroupAction(actions=[])
    monkeypatch.setattr(launch, 'include', record)
    monkeypatch.setattr(launch, 'get_package_share_directory', lambda _: str(PACKAGE))
    context = LaunchContext()
    context.launch_configurations.update({
        'mode': mode, 'lio': 'zlio', 'localization': 'zlio', 'use_sim_time': 'true',
        'frontend_config': str(ZLOCAL / 'config/lio.yaml'),
        'backend_config': str(ZLOCAL / 'config/backend.yaml'),
        'prior_map': str(tmp_path), 'map': str(PACKAGE / 'map/328.yaml'),
        'save_directory': str(tmp_path / 'saved'), 'use_uwb': 'false',
        'lidar_topic': '/livox/lidar', 'imu_topic': '/livox/imu',
        'imu_acceleration_scale': '9.81', 'lio_rviz': 'false', 'nav_rviz': 'false',
        'driver': driver, 'driver_config': str(PACKAGE / 'config/reality/MID360_config.json'),
        'enable_fake_velocity': 'false', 'cruise': 'false',
        'params_file': str(PACKAGE / 'config/reality/nav2_params_real.yaml'),
    })
    # 只验证路径存在，地图内容由 nav2_map_server 校验。
    context.launch_configurations['map'] = str(PACKAGE / 'config/reality/nav2_params_real.yaml')
    actions = launch.setup(context)
    overlay_path = Path(calls[0][2]['backend_config'])
    try:
        backend_launch = load('backend_launch', ZLOCAL / 'src/backend/launch/backend.launch.py')
        config = backend_launch.load_params(overlay_path)
        assert config['common']['body']['frame'] == 'base_link'
        assert config['output']['legacy']['publish_tf'] is False
        assert config['uwb']['enabled'] is False
        assert config['common']['body']['offset']['y'] == pytest.approx(-.19329)
        assert calls[0][2]['driver'] == 'false'
        assert calls[0][2]['coordinated_reset'] == 'true'
        assert calls[0][2]['mode'] == ('localization' if mode == 'nav' else 'mapping')
        nodes = [a for a in actions if isinstance(a, Node)]
        for node in nodes:
            evaluate_parameters(context, node._Node__parameters)
        packages = [n.node_package for n in nodes]
        assert ('livox_ros_driver2' in packages) == (driver == 'true')
        assert ('slam_toolbox' in packages) == (mode == 'mapping')
        assert any(c[1] == 'map_server_launch.py' for c in calls) == (mode == 'nav')
        assert any(c[1] == 'bringup_rm_navigation.py' for c in calls) == (mode == 'nav')
        if mode == 'mapping':
            slam = nodes[packages.index('slam_toolbox')]
            params = evaluate_parameters(context, slam._Node__parameters)[-1]
            assert params['transform_publish_period'] == 0.
            assert params['odom_frame'] == params['map_frame'] == 'map'
    finally:
        overlay_path.unlink()


def test_old_algorithms_are_rejected():
    context = LaunchContext()
    context.launch_configurations.update(mode='nav', lio='fastlio', localization='icp')
    with pytest.raises(ValueError, match='Z-Localization'):
        launch.setup(context)
