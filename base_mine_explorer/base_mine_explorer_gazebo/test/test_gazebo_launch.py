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
import launch
import launch.actions
import launch_testing.actions
import launch_testing.asserts
import pytest
import rclpy
from sensor_msgs.msg import JointState


@pytest.mark.launch_test
def generate_test_description():
    """Generate the launch description for testing Gazebo integration."""
    pkg_gazebo = get_package_share_directory('base_mine_explorer_gazebo')
    launch_file_path = os.path.join(pkg_gazebo, 'launch', 'gazebo.launch.py')

    included_launch = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(launch_file_path)
    )

    return launch.LaunchDescription([
        included_launch,
        launch_testing.actions.ReadyToTest(),
    ])


class TestGazeboIntegration(unittest.TestCase):
    """Full integration test suite for Gazebo and ROS 2 controllers."""

    @classmethod
    def setUpClass(cls):
        """Initialize ROS 2 Python client library."""
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        """Shutdown ROS 2 Python client library."""
        rclpy.shutdown()

    def setUp(self):
        """Create a temporary node for each test case."""
        self.node = rclpy.create_node('test_integration_node')

    def tearDown(self):
        """Destroy the node after each test case."""
        self.node.destroy_node()

    def test_nodes_and_communication(self):
        """
        Check if nodes stay alive and /joint_states is active.

        This replaces the old simple sleep test with real data validation.
        """
        msgs_received = []
        sub = self.node.create_subscription(
            JointState,
            '/joint_states',
            lambda msg: msgs_received.append(msg),
            10
        )

        # Wait up to 20s for the first message
        # Use clock().now() instead of seconds_now() for Humble compatibility
        end_time = self.node.get_clock().now().nanoseconds + 20e9
        while rclpy.ok() and len(msgs_received) == 0:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if self.node.get_clock().now().nanoseconds > end_time:
                break

        self.node.destroy_subscription(sub)
        self.assertGreater(len(msgs_received), 0, 'No messages received on /joint_states!')

    def test_base_controller_is_available(self):
        """Check if the base_controller has been successfully loaded."""
        end_time = self.node.get_clock().now().nanoseconds + 15e9
        controller_found = False

        while rclpy.ok() and self.node.get_clock().now().nanoseconds < end_time:
            topic_names_and_types = self.node.get_topic_names_and_types()
            topics = [t[0] for t in topic_names_and_types]

            if '/base_controller/cmd_vel_unstamped' and '/base_controller/odom' in topics:
                controller_found = True
                break
            rclpy.spin_once(self.node, timeout_sec=0.5)

        self.assertTrue(controller_found, 'Base controller topics not found!')


@launch_testing.post_shutdown_test()
class TestGazeboLaunchAfterShutdown(unittest.TestCase):
    """Checks exit codes to ensure a clean shutdown."""

    def test_exit_codes(self, proc_info):
        """Check if all processes exited with code 0 or SIGINT (-2)."""
        launch_testing.asserts.assertExitCodes(proc_info, allowable_exit_codes=[0, -2])
