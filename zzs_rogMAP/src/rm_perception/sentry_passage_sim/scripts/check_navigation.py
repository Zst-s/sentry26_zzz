#!/usr/bin/env python3
"""Bounded isolated closed-loop check. Sends one SIMULATION goal only."""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import time

import rclpy
from action_msgs.msg import GoalStatus
from gazebo_msgs.msg import ContactsState
from gazebo_msgs.srv import SpawnEntity, DeleteEntity
from geometry_msgs.msg import Twist
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import ManageLifecycleNodes
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import OccupancyGrid, Odometry, Path as NavPath
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from generate_world import make_world
from navigation_assets import obstacle_boxes


def intersects(x, y, yaw, box):
    # Separating axis test, actual asymmetric rectangle against each wall area.
    c, s = math.cos(yaw), math.sin(yaw)
    robot = [(x+c*a-s*b, y+s*a+c*b) for a,b in
             [(-.32,-.29),(.30,-.29),(.30,.29),(-.32,.29)]]
    x0,y0,x1,y1 = box
    wall = [(x0,y0),(x1,y0),(x1,y1),(x0,y1)]
    for ax,ay in [(1.,0.),(0.,1.),(c,s),(-s,c)]:
        r = [px*ax+py*ay for px,py in robot]
        w = [px*ax+py*ay for px,py in wall]
        if max(r) <= min(w)+1e-6 or max(w) <= min(r)+1e-6:
            return False
    return True


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    if os.environ.get('ROS_DOMAIN_ID') != '229' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        raise SystemExit('Refusing to send a goal outside simulation domain 229')
    rclpy.init()
    node = Node('passage_navigation_check', parameter_overrides=[Parameter('use_sim_time', value=True)])
    state = dict(odom=None, terrain=None, cloud=None, map_hash=None, map_changed=False,
                 local_grid=None, path=None, nonzero=0, collisions=0, contacts=0, samples=0, max_speed=0.)
    boxes = obstacle_boxes(make_world())
    samples = []
    def odom(msg):
        state['odom'] = msg
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        speed = math.hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y)
        state['max_speed'] = max(state['max_speed'], speed)
        state['samples'] += 1
        if any(intersects(p.x,p.y,yaw,box) for box in boxes):
            state['collisions'] += 1
        samples.append([msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9,p.x,p.y,yaw,speed])
    def grid(msg):
        fingerprint = hashlib.sha256(bytes(v+1 for v in msg.data)).hexdigest()
        if state['map_hash'] is not None and state['map_hash'] != fingerprint:
            state['map_changed'] = True
        state['map_hash'] = fingerprint
    def drive(msg):
        state['nonzero'] += math.hypot(msg.linear.x,msg.linear.y) > .01
    subscriptions = [
        node.create_subscription(Odometry,'/passage_sim/base_odom',odom,qos_profile_sensor_data),
        node.create_subscription(OccupancyGrid,'/map',grid,QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)),
        node.create_subscription(OccupancyGrid,'/terrain/grid',lambda m: state.update(terrain=m),10),
        node.create_subscription(PointCloud2,'/passage_sim/body_points',lambda m: state.update(cloud=m),qos_profile_sensor_data),
        node.create_subscription(OccupancyGrid,'/local_costmap/costmap',lambda m: state.update(local_grid=m),10),
        node.create_subscription(NavPath,'/plan',lambda m: state.update(path=m),10),
        node.create_subscription(Twist,'/passage_sim/drive_cmd_vel',drive,10),
        node.create_subscription(ContactsState,'/passage_sim/bumper_states',
                                 lambda m: state.update(contacts=state['contacts']+sum(1 for contact in m.states
                                    if 'ground::' not in contact.collision1_name and 'ground::' not in contact.collision2_name)),10)]
    client = ActionClient(node,NavigateToPose,'/navigate_to_pose')
    def wait(predicate, timeout, label):
        end = time.monotonic()+timeout
        while time.monotonic() < end:
            rclpy.spin_once(node,timeout_sec=.05)
            if predicate():
                return
        raise RuntimeError('Timeout: '+label)
    result = dict(passed=False, truth_pose=True, real_hardware=False)
    handle = None
    spawned = False
    delete = node.create_client(DeleteEntity, '/delete_entity')
    def cell(grid, x, y):
        if grid is None:
            return -1
        col = math.floor((x-grid.info.origin.position.x)/grid.info.resolution)
        row = math.floor((y-grid.info.origin.position.y)/grid.info.resolution)
        if 0 <= col < grid.info.width and 0 <= row < grid.info.height:
            return grid.data[row*grid.info.width+col]
        return -1
    dynamic_x, dynamic_y = 3.6, 1.2
    def obstacle_values(grid):
        # Rays observe box faces, not the unobserved interior center.
        return [cell(grid,dynamic_x+dx,dynamic_y+dy) for dx in (-.15,-.05,.05,.15) for dy in (-.15,-.05,.05,.15)]
    try:
        wait(lambda: all(state[k] is not None for k in ('odom','terrain','cloud','map_hash')),60,'inputs')
        p = state['odom'].pose.pose.position
        if math.hypot(p.x,p.y) > .05:
            raise RuntimeError('Requires a fresh simulation at the initial position')
        lifecycle = node.create_client(GetState, '/velocity_smoother/get_state')
        wait(lifecycle.service_is_ready,30,'Nav2 lifecycle service')
        active_deadline = time.monotonic()+30
        while True:
            active = lifecycle.call_async(GetState.Request())
            wait(active.done,3,'Nav2 lifecycle response')
            if active.result().current_state.id == 3:
                break
            if time.monotonic() > active_deadline:
                raise RuntimeError('Nav2 did not become active')
            until = time.monotonic()+.2
            wait(lambda: time.monotonic() >= until,1,'activation wait')
        wait(lambda: client.server_is_ready(),10,'Nav2 action')
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = node.get_clock().now().to_msg()
        goal.pose.pose.position.x = 4.2
        goal.pose.pose.orientation.w = 1.
        future = client.send_goal_async(goal)
        wait(future.done,10,'goal acceptance')
        handle = future.result()
        if not handle.accepted:
            raise RuntimeError('Goal rejected')
        future = handle.get_result_async()
        wait(future.done,100,'whole-vehicle passage')
        result['action_status'] = future.result().status
        if result['action_status'] != GoalStatus.STATUS_SUCCEEDED:
            raise RuntimeError('Navigation failed')
        # Keep observing actual simulated motion after the action result.
        until = time.monotonic()+2.
        wait(lambda: time.monotonic() >= until,3.,'stop observation')
        p = state['odom'].pose.pose.position
        q = state['odom'].pose.pose.orientation
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        rear_x = min(p.x+math.cos(yaw)*a-math.sin(yaw)*b
                     for a,b in [(-.32,-.29),(.30,-.29),(.30,.29),(-.32,.29)])
        result.update(final_xy=[p.x,p.y], rear_x=rear_x,
                      final_speed=math.hypot(state['odom'].twist.twist.linear.x,state['odom'].twist.twist.linear.y))
        inside = [sample for sample in samples if 2.0 <= sample[1] <= 3.2]
        result['traversed_inside_tunnel'] = bool(inside) and all(abs(sample[2]) < .31 for sample in inside)
        result['passed'] = (result['traversed_inside_tunnel'] and rear_x > 3.2 and abs(p.x-4.2) <= .2 and abs(p.y) <= .2 and
                            result['final_speed'] < .02 and state['collisions'] == 0 and
                            state['contacts'] == 0 and state['nonzero'] > 0 and not state['map_changed'])
        if not result['passed']:
            raise RuntimeError('Passage/stop/footprint checks failed')
        baseline_overlay = obstacle_values(state['local_grid'])
        result['dynamic_baseline_cells'] = baseline_overlay
        spawn = node.create_client(SpawnEntity, '/spawn_entity')
        wait(lambda: spawn.service_is_ready() and delete.service_is_ready(),10,'Gazebo entity services')
        request = SpawnEntity.Request()
        request.name = 'navigation_check_obstacle'
        request.reference_frame = 'world'
        request.initial_pose.position.x = dynamic_x
        request.initial_pose.position.y = dynamic_y
        request.initial_pose.position.z = .3
        request.initial_pose.orientation.w = 1.
        request.xml = '''<sdf version="1.6"><model name="navigation_check_obstacle"><static>true</static>
<link name="body"><collision name="body"><geometry><box><size>.25 .25 .6</size></box></geometry></collision>
<visual name="body"><geometry><box><size>.25 .25 .6</size></box></geometry></visual></link></model></sdf>'''
        future = spawn.call_async(request)
        wait(future.done,10,'insert temporary obstacle')
        if not future.result().success:
            raise RuntimeError(future.result().status_message)
        spawned = True
        wait(lambda: 100 in obstacle_values(state['terrain']) and
                     max(obstacle_values(state['local_grid'])) >= 99 and
                     any(value > previous for value,previous in zip(obstacle_values(state['local_grid']),baseline_overlay)),20,'dynamic obstacle marking')
        result['dynamic_obstacle_marked'] = True
        request = DeleteEntity.Request()
        request.name = 'navigation_check_obstacle'
        future = delete.call_async(request)
        wait(future.done,10,'remove temporary obstacle')
        if not future.result().success:
            raise RuntimeError(future.result().status_message)
        spawned = False
        wait(lambda: 100 not in obstacle_values(state['terrain']) and
                     obstacle_values(state['local_grid']) == baseline_overlay,25,'dynamic overlay removal')
        result['dynamic_obstacle_cleared'] = True
        result['dynamic_restored_cells'] = obstacle_values(state['local_grid'])
        result['static_walls_retained'] = all(cell(state['local_grid'],2.6,y) >= 99 for y in (-.7,.7))
        result['passed'] = result['static_walls_retained'] and not state['map_changed']
    except Exception as exc:
        result['passed'] = False
        result['error'] = str(exc)
        result['last_obstacle_cells'] = dict(terrain=obstacle_values(state['terrain']),
                                             local=obstacle_values(state['local_grid']))
    finally:
        if spawned and delete.service_is_ready():
            request = DeleteEntity.Request()
            request.name = 'navigation_check_obstacle'
            cleanup = delete.call_async(request)
            try:
                wait(cleanup.done,3,'obstacle cleanup')
            except RuntimeError:
                pass
        if handle is not None and not result['passed']:
            cancel = handle.cancel_goal_async()
            try:
                wait(cancel.done,3,'cancel')
            except RuntimeError:
                pass
        # Deactivate/clean up plugins before terminating their processes.
        manager = node.create_client(ManageLifecycleNodes, '/lifecycle_manager_navigation/manage_nodes')
        if manager.service_is_ready():
            request = ManageLifecycleNodes.Request()
            request.command = ManageLifecycleNodes.Request.SHUTDOWN
            shutdown = manager.call_async(request)
            try:
                wait(shutdown.done,10,'navigation lifecycle shutdown')
                result['lifecycle_shutdown_clean'] = bool(shutdown.result().success)
            except RuntimeError:
                result['lifecycle_shutdown_clean'] = False
        result.update({k:state[k] for k in ('samples','max_speed','nonzero','collisions','contacts','map_hash','map_changed')})
        output = Path(args.output)
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(result,indent=2))
        output.with_suffix('.poses.json').write_text(json.dumps(samples))
        print(json.dumps(result,indent=2),flush=True)
        del subscriptions
        node.destroy_node()
        rclpy.shutdown()
    if not result['passed']:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
