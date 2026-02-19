# Copyright 2026 Theo Morel
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
import time
import unittest

from ament_index_python.packages import get_package_share_directory
from controller_manager_msgs.srv import ListControllers
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import launch_testing.actions
import pytest
import rclpy
from tf2_ros import Buffer, TransformListener


@pytest.mark.launch_test
def generate_test_description():
    """Launch full system integration test."""
    # Packages
    pkg_bringup = FindPackageShare('mine_explorer_bringup')
    resources_dir = get_package_share_directory('mine_explorer_resources')
    mapping_dir = get_package_share_directory('mine_explorer_mapping')
    control_dir = get_package_share_directory('mine_explorer_control')

    files_to_check = [
        os.path.join(resources_dir, 'worlds', 'world_1.world'),
        os.path.join(mapping_dir, 'config', 'slam_2d_params.yaml'),
        os.path.join(control_dir, 'config', 'ekf.yaml'),
        os.path.join(resources_dir, 'trajectories', 'trajectory_1.yaml')
    ]

    for f in files_to_check:
        if not os.path.exists(f):
            pytest.fail(f'Missing file for the test : {f}')

    # Launch Bringup (Gazebo + EKF + SLAM)
    bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                pkg_bringup,
                'launch',
                'bringup_mine_slam2d_ekf.launch.py'
            ])
        ]),
        launch_arguments={'world': 'world_1.world'}.items()
    )

    return LaunchDescription([
        bringup_launch,
        launch_testing.actions.ReadyToTest(),
    ])


class TestFullSystemIntegration(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_bringup_node')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self.node)

    def tearDown(self):
        self.node.destroy_node()

    def verify_controller_active(self, controller_name):
        """Verify active controller."""
        client = self.node.create_client(ListControllers, '/controller_manager/list_controllers')

        if client.wait_for_service(timeout_sec=5.0):
            request = ListControllers.Request()
            future = client.call_async(request)
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=2.0)

            if future.result() is not None:
                for controller in future.result().controller:
                    if controller.name == controller_name and controller.state == 'active':
                        return True
        return False

    def test_system_flow(self):
        """Verify : Controllers, Topics et TFs."""
        start_time = time.time()
        timeout = 90.0

        # Controllers
        self.node.get_logger().info('Waiting for controllers activation...')
        controller_timeout = time.time() + 30.0

        while time.time() < controller_timeout:
            base_ok = self.verify_controller_active('base_controller')
            js_ok = self.verify_controller_active('joint_state_broadcaster')

            if base_ok and js_ok:
                break

            self.node.get_logger().info('Controllers not ready... next test in 2s')
            time.sleep(2.0)
            rclpy.spin_once(self.node, timeout_sec=0.1)

        self.assertTrue(base_ok,
                        'base_controller is not active !')
        self.assertTrue(js_ok,
                        'joint_state_broadcasternot is not active !')

        # Topics
        checked_topics = {
            '/sensors/lidar/data': False,
            '/sensors/lidar/scan_2d': False,
            '/map': False,
            '/odometry/filtered': False
        }
        found_tf_map_odom = False
        found_tf_odom_base = False

        while (time.time() - start_time) < timeout:
            rclpy.spin_once(self.node, timeout_sec=1.0)

            # Topics
            current_topics = [t[0] for t in self.node.get_topic_names_and_types()]
            for topic in checked_topics.keys():
                if topic in current_topics:
                    checked_topics[topic] = True

            # TF map -> odom (SLAM)
            if not found_tf_map_odom:
                try:
                    if self.tf_buffer.can_transform('map', 'odom', rclpy.time.Time()):
                        found_tf_map_odom = True
                except Exception:
                    pass

            # TF odom -> base_footprint (EKF)
            if not found_tf_odom_base:
                try:
                    if self.tf_buffer.can_transform('odom', 'base_footprint', rclpy.time.Time()):
                        found_tf_odom_base = True
                except Exception:
                    pass
            if all(checked_topics.values()) and found_tf_map_odom and found_tf_odom_base:
                break

        for topic, found in checked_topics.items():
            self.assertTrue(found, f'Topic {topic} not found !')

        self.assertTrue(found_tf_odom_base, 'TF odom -> base_footprint not found')
        self.assertTrue(found_tf_map_odom, 'TF map -> odom not found')
