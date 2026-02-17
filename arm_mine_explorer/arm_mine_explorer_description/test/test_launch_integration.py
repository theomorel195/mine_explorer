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

import unittest

import launch
import launch_ros.actions
import launch_testing.actions
import pytest


@pytest.mark.launch_test
def generate_test_description():
    """Launch the robot_state_publisher node with the description."""
    robot_description_content = '<?xml version="1.0"?><robot name="test"></robot>'

    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description_content}]
    )

    return launch.LaunchDescription([
        robot_state_publisher_node,
        launch_testing.actions.ReadyToTest(),
    ]), {'rsp_node': robot_state_publisher_node}


class TestLinkPublication(unittest.TestCase):

    def test_node_starts(self, proc_info, rsp_node):
        """Check if the robot_state_publisher node starts correctly."""
        assert proc_info
