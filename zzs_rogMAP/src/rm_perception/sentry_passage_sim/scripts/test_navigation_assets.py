"""Check physical geometry, static-map preservation and controller wiring contracts."""
import tempfile
import unittest
from pathlib import Path
from generate_world import make_world
from navigation_assets import navigation_world, obstacle_boxes, write_map, navigation_parameters, FOOTPRINT


class NavigationAssets(unittest.TestCase):
    def test_real_vehicle_and_sensor_are_distinct(self):
        root = navigation_world(make_world(dense=True, mobile=True))
        rig = root.find(".//model[@name='sensor_rig']")
        self.assertEqual(rig.findtext("link[@name='base_link']/collision/geometry/box/size"), '.62 .58 .30')
        self.assertEqual(rig.findtext("plugin[@name='truth']/body_name"), 'passage_lidar')
        self.assertEqual(rig.findtext("plugin[@name='planar_move']/robot_base_frame"), 'base_link')
        self.assertEqual(rig.findtext("plugin[@name='planar_move']/publish_odom_tf"), 'true')
        self.assertIsNotNone(rig.find("link[@name='passage_lidar']/sensor"))

    def test_map_keeps_all_sides_and_wall_but_no_overhead_roof(self):
        root = navigation_world(make_world(dense=True, mobile=True))
        boxes = obstacle_boxes(root)
        self.assertEqual(len(boxes), 7)
        self.assertFalse(any(x0 < 2.6 < x1 and y0 < 0 < y1 for x0,y0,x1,y1 in boxes))
        self.assertTrue(any(x0 < 2.6 < x1 and y0 < .7 < y1 for x0,y0,x1,y1 in boxes))
        with tempfile.TemporaryDirectory() as directory:
            write_map(root, directory)
            self.assertEqual((Path(directory)/'navigation.pgm').read_bytes().count(b'P5'), 1)

    def test_derived_configuration_does_not_mutate_source(self):
        source = {name: {'ros__parameters': {}} for name in ('controller_server','bt_navigator','velocity_smoother')}
        source['planner_server'] = {'ros__parameters': {'GridBased': {}}}
        source['controller_server']['ros__parameters']['FollowPath'] = {'max_vel_x': 1.}
        for name in ('global_costmap','local_costmap'):
            source[name] = {name: {'ros__parameters': {}}}
        derived = navigation_parameters(source)
        self.assertNotIn('odom_topic', source['controller_server']['ros__parameters'])
        for name in ('global_costmap','local_costmap'):
            cost = derived[name][name]['ros__parameters']
            self.assertEqual(cost['footprint'], FOOTPRINT)
            self.assertTrue(cost['terrain_layer']['marking_only'])
            self.assertFalse(cost['static_layer']['footprint_clearing_enabled'])
        teb = derived['controller_server']['ros__parameters']['FollowPath']
        self.assertEqual(teb['max_vel_x'], 1.)
        self.assertEqual(teb['feasibility_check_no_poses'], -1)


if __name__ == '__main__':
    unittest.main()
