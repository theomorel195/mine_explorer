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
import launch_ros.actions
import launch_testing.actions
import pytest
import xacro


@pytest.mark.launch_test
def generate_test_description():
    """Launch the robot_state_publisher node with the description."""
    pkg_name = 'sensors_mine_explorer_description'
    pkg_path = get_package_share_directory(pkg_name)
    xacro_file = os.path.join(pkg_path, 'urdf', 'lidar', 'hdl32e_standalone.urdf.xacro')

    mappings = {
        'prefix': '',
    }

    robot_description_config = xacro.process_file(xacro_file, mappings=mappings)
    robot_xml = robot_description_config.toxml()

    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_xml}]
    )

    return launch.LaunchDescription([
        robot_state_publisher_node,
        launch_testing.actions.ReadyToTest(),
    ]), {'rsp_node': robot_state_publisher_node}


class TestLinkPublication(unittest.TestCase):

    def test_node_starts(self, proc_info, rsp_node):
        """Check if the robot_state_publisher node starts correctly."""
        assert proc_info
