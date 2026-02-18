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

from ament_index_python.packages import get_package_share_directory
import pytest
import xacro


def test_xacro_parsing():
    """Test if the Xacro file parses correctly with all default arguments."""
    pkg_name = 'mine_explorer_description'
    arm_pkg_name = 'arm_mine_explorer_description'
    try:
        pkg_path = get_package_share_directory(pkg_name)
    except Exception:
        pytest.fail(f'Package {pkg_name} not found.')

    try:
        arm_pkg_path = get_package_share_directory(arm_pkg_name)
    except Exception:
        pytest.fail(f'Package {arm_pkg_name} not found.')

    xacro_file = os.path.join(pkg_path, 'urdf', 'robot.urdf.xacro')

    mappings = {
        'prefix_arm': 'arm_',
        'name_arm': 'ArmMineExplorerSystem',
        'prefix_base': '',
        'name_base': 'MobileBaseMineExplorerSystem',
        'name_lidar': 'lidar',
        'use_sim': 'true',
        'joint_limit_params': os.path.join(arm_pkg_path, 'config', 'joint_limits.yaml'),
        'kinematics_params': os.path.join(arm_pkg_path, 'config', 'default_kinematics.yaml'),
        'physical_params': os.path.join(arm_pkg_path, 'config', 'physical_parameters.yaml'),
        'visual_params': os.path.join(arm_pkg_path, 'config', 'visual_parameters.yaml'),
        'initial_positions_file': os.path.join(arm_pkg_path, 'config', 'initial_positions.yaml'),
        'safety_limits': 'true',
        'safety_pos_margin': '0.15',
        'safety_k_position': '20',
    }

    try:
        robot_description_config = xacro.process_file(xacro_file, mappings=mappings)
        robot_xml = robot_description_config.toxml()
    except Exception as e:
        pytest.fail(f'Xacro parsing failed! Error: {e}')

    assert robot_xml is not None, 'Generated URDF XML is empty.'
    assert '<robot' in robot_xml, 'Generated XML does not contain a <robot> tag.'
    assert 'base_link' in robot_xml, 'The root link base_link was not found in URDF.'
