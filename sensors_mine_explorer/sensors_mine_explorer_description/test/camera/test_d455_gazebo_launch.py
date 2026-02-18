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
    pkg_gazebo = get_package_share_directory('sensors_mine_explorer_description')
    launch_file_path = os.path.join(pkg_gazebo, 'launch', 'camera_d455_display.launch.py')

    included_launch = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(launch_file_path)
    )

    return launch.LaunchDescription([
        included_launch,
        launch_testing.actions.ReadyToTest(),
    ])


class TestGazeboIntegration(unittest.TestCase):
    """Full integration test suite for Gazebo and topics."""

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

    def test_camera_topics_is_available(self):
        """Check if the camera topics is available."""
        end_time = self.node.get_clock().now().nanoseconds + 15e9
        topic_found = False

        while rclpy.ok() and self.node.get_clock().now().nanoseconds < end_time:
            topic_names_and_types = self.node.get_topic_names_and_types()
            topics = [t[0] for t in topic_names_and_types]

            if '/camera/d455/points' and '/camera/d455/depth/image_raw' in topics:
                topic_found = True
                break
            rclpy.spin_once(self.node, timeout_sec=0.5)

        self.assertTrue(topic_found, 'Topics not found!')


@launch_testing.post_shutdown_test()
class TestGazeboLaunchAfterShutdown(unittest.TestCase):
    """Checks exit codes to ensure a clean shutdown."""

    def test_exit_codes(self, proc_info):
        """Check if all processes exited with code 0 or SIGINT (-2) or -6."""
        launch_testing.asserts.assertExitCodes(proc_info, allowable_exit_codes=[0, -2, -6])
