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
    pkg_gazebo = get_package_share_directory('mine_explorer_gazebo')
    launch_file_path = os.path.join(pkg_gazebo, 'launch', 'gazebo.launch.py')

    included_launch = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(launch_file_path)
    )

    return launch.LaunchDescription([
        included_launch,
        launch_testing.actions.ReadyToTest(),
    ])


def is_link_present(node, link_name: str) -> bool:
    """Check if a link exists in the robot_description."""
    from urdf_parser_py.urdf import URDF
    robot_description_param = '/robot_description'
    if not node.has_parameter(robot_description_param):
        return False
    urdf_str = node.get_parameter(robot_description_param).get_parameter_value().string_value
    robot = URDF.from_xml_string(urdf_str)
    return any(link.name == link_name for link in robot.links)


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

    def test_controllers_are_available(self):
        """Check if the controllers has been successfully loaded."""
        end_time = self.node.get_clock().now().nanoseconds + 15e9
        arm_controller_found = False
        base_controller_found = False

        while rclpy.ok() and self.node.get_clock().now().nanoseconds < end_time:
            topic_names_and_types = self.node.get_topic_names_and_types()
            topics = [t[0] for t in topic_names_and_types]

            if '/arm_controller/joint_trajectory' in topics:
                arm_controller_found = True
            if '/base_controller/cmd_vel_unstamped' in topics:
                base_controller_found = True

            if arm_controller_found and base_controller_found:
                break

            rclpy.spin_once(self.node, timeout_sec=0.5)

        self.assertTrue(arm_controller_found, 'Arm controller topic not found!')
        self.assertTrue(base_controller_found, 'Base controller topic not found!')

    def test_lidar_topic_if_included(self):
        """Check if LIDAR is present in the URDF and its topic publishes data."""
        if not is_link_present(self.node, 'lidar_base_link'):
            self.skipTest('LIDAR not included in the URDF, skipping test.')

        msgs_received = []
        lidar_topic = '/sensors/lidar/data'
        sub = self.node.create_subscription(
            rclpy.qos.QoSProfile(depth=10),
            lidar_topic,
            lambda msg: msgs_received.append(msg),
            10
        )

        # Wait up to 15s for first LIDAR message
        end_time = self.node.get_clock().now().nanoseconds + 15e9
        while rclpy.ok() and len(msgs_received) == 0:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if self.node.get_clock().now().nanoseconds > end_time:
                break

        self.node.destroy_subscription(sub)
        self.assertGreater(len(msgs_received), 0, f'No messages received on {lidar_topic}!')


@launch_testing.post_shutdown_test()
class TestGazeboLaunchAfterShutdown(unittest.TestCase):
    """Checks exit codes to ensure a clean shutdown."""

    def test_exit_codes(self, proc_info):
        """Check if all processes exited with code 0 or SIGINT (-2)."""
        launch_testing.asserts.assertExitCodes(proc_info, allowable_exit_codes=[0, -2])
