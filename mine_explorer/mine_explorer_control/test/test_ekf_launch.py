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
import math
import unittest

from ament_index_python.packages import get_package_share_directory
import launch
import launch.actions
import launch_testing.actions
import launch_testing.asserts
import pytest
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


@pytest.mark.launch_test
def generate_test_description():
    """Generate the launch description for testing EKF and odometry."""
    pkg_gazebo = get_package_share_directory('mine_explorer_gazebo')
    gazebo_launch_path = os.path.join(pkg_gazebo, 'launch', 'gazebo.launch.py')

    pkg_control = get_package_share_directory('mine_explorer_control')
    ekf_launch_path = os.path.join(pkg_control, 'launch', 'localization.launch.py')

    included_gazebo = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(gazebo_launch_path)
    )
    included_ekf = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(ekf_launch_path)
    )

    return launch.LaunchDescription([
        included_gazebo,
        included_ekf,
        launch_testing.actions.ReadyToTest(),
    ])


def is_file_present(path):
    """Check if a file exists."""
    return os.path.isfile(path)


class TestEkfConfiguration(unittest.TestCase):
    """Verify that EKF configuration files exist."""

    def test_ekf_yaml_exists(self):
        """Check that ekf.yaml exists in the config folder."""
        pkg_control = get_package_share_directory('mine_explorer_control')
        ekf_file_path = os.path.join(pkg_control, 'config', 'ekf.yaml')
        self.assertTrue(is_file_present(ekf_file_path),
                        f"EKF configuration file not found at {ekf_file_path}")


class TestEkfIntegration(unittest.TestCase):
    """Integration tests combining EKF and a simulation with robot movement."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_ekf_node')

    def tearDown(self):
        self.node.destroy_node()

    def test_odom_filtered_active_and_movement(self):
        """Launch EKF + simulation, send velocity commands (linear + rotation)."""
        msgs_received = []
        sub = self.node.create_subscription(
            Odometry,
            '/odometry/filtered',
            lambda msg: msgs_received.append(msg),
            10
        )

        cmd_vel_pub = self.node.create_publisher(Twist, '/base_controller/cmd_vel_unstamped', 10)

        end_time = self.node.get_clock().now().nanoseconds + 15e9
        while rclpy.ok() and len(msgs_received) == 0:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if self.node.get_clock().now().nanoseconds > end_time:
                break

        self.assertGreater(len(msgs_received), 0, "/odometry/filtered did not publish any messages!")

        twist_msg = Twist()
        for _ in range(3):
            # Forward 5s
            twist_msg.linear.x = 0.2
            twist_msg.angular.z = 0.0
            for _ in range(50):
                cmd_vel_pub.publish(twist_msg)
                rclpy.spin_once(self.node, timeout_sec=0.1)

            # Turn 5s
            twist_msg.linear.x = 0.0
            twist_msg.angular.z = 0.3
            for _ in range(50):
                cmd_vel_pub.publish(twist_msg)
                rclpy.spin_once(self.node, timeout_sec=0.1)

        # Stopped
        twist_msg.linear.x = 0.0
        twist_msg.angular.z = 0.0
        cmd_vel_pub.publish(twist_msg)

        self.node.destroy_subscription(sub)

        self.assertGreater(len(msgs_received), 0, 'No messages received on /odometry/filtered!')

        first_msg = msgs_received[0]
        last_msg = msgs_received[-1]
        dx = math.hypot(last_msg.pose.pose.position.x - first_msg.pose.pose.position.x,
                        last_msg.pose.pose.position.y - first_msg.pose.pose.position.y)
        self.assertGreater(dx, 0.0, 'Robot did not move according to EKF output!')


@launch_testing.post_shutdown_test()
class TestEkfLaunchAfterShutdown(unittest.TestCase):
    """Checks exit codes to ensure a clean shutdown."""

    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info, allowable_exit_codes=[0, -2])
