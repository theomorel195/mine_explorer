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
import unittest

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import launch_testing.actions
import pytest
import rclpy
from tf2_ros import Buffer, TransformListener


@pytest.mark.launch_test
def generate_test_description():
    """Launch gazebo sim, robot_mover and slam."""
    # Packages
    pkg_mapping = FindPackageShare('mine_explorer_mapping')
    pkg_gazebo = FindPackageShare('mine_explorer_gazebo')
    pkg_resources = FindPackageShare('mine_explorer_resources')

    # Configuration file
    config_path = os.path.join(
        get_package_share_directory('mine_explorer_mapping'),
        'config', 'slam_2d_params.yaml'
    )
    if not os.path.exists(config_path):
        pytest.fail(f'Configuration file not found at {config_path}')

    # Gazebo simulation
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([pkg_gazebo, 'launch', 'gazebo.launch.py'])
        ]),
        launch_arguments={'world': 'world_1.world'}.items()
    )

    # SLAM
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([pkg_mapping, 'launch', 'slam_2d.launch.py'])
        ])
    )

    # Robot_mover script
    traj_path = PathJoinSubstitution([pkg_resources, 'trajectories', 'trajectory_1.yaml'])
    mover_node = ExecuteProcess(
        cmd=['ros2', 'run', 'mine_explorer_resources', 'robot_mover', traj_path],
        output='screen'
    )

    return LaunchDescription([
        gazebo_launch,
        slam_launch,
        TimerAction(period=5.0, actions=[mover_node]),
        launch_testing.actions.ReadyToTest(),
    ])


class TestSlamSystem(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_slam_node')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self.node)

    def tearDown(self):
        self.node.destroy_node()

    def test_topics_and_tf(self):
        """Verify topics."""
        start_time = self.node.get_clock().now()
        timeout = 30.0

        found_scan = False
        found_map = False

        while (self.node.get_clock().now() - start_time).nanoseconds / 1e9 < timeout:
            rclpy.spin_once(self.node, timeout_sec=0.1)

            # Topics
            topic_list = [t[0] for t in self.node.get_topic_names_and_types()]
            if '/sensors/lidar/scan_2d' in topic_list:
                found_scan = True
            if '/map' in topic_list:
                found_map = True

            if found_scan and found_map:
                break

        self.assertTrue(found_scan, 'Topic /sensors/lidar/scan_2d not found')
        self.assertTrue(found_map, 'Topic /map not found')
