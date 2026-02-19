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
import yaml


def test_gazebo_controller_config():
    """Verify that the controller manager YAML is valid and contains controllers."""
    pkg_path = get_package_share_directory('mine_explorer_gazebo')
    config_path = os.path.join(pkg_path, 'config', 'gazebo_controller_manager.yaml')

    assert os.path.exists(config_path), f'Config file not found: {config_path}'

    with open(config_path, 'r') as f:
        try:
            config_data = yaml.safe_load(f)
        except yaml.YAMLError as e:
            assert False, f'Error parsing YAML: {e}'

    assert 'controller_manager' in config_data, 'Missing controller_manager key'
    ros_parameters = config_data['controller_manager']['ros__parameters']
    assert 'arm_controller' in ros_parameters, 'arm_controller not defined in config'
    assert 'base_controller' in ros_parameters, 'base_controller not defined in config'
    assert 'joint_state_broadcaster' in ros_parameters, 'broadcaster not defined'


def test_world_file_exists():
    """Check if the empty world model file exists."""
    pkg_path = get_package_share_directory('mine_explorer_resources')
    world_path = os.path.join(pkg_path, 'worlds', 'empty.world')
    assert os.path.exists(world_path), f'World file missing: {world_path}'
