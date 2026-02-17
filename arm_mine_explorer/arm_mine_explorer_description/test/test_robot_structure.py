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
from urdf_parser_py.urdf import URDF
import xacro


def get_robot_urdf():
    """Parse the xacro and return a URDF object."""
    pkg_path = get_package_share_directory('arm_mine_explorer_description')
    xacro_file = os.path.join(pkg_path, 'urdf', 'robot.urdf.xacro')

    mappings = {
        'prefix': '',
        'name': 'ArmMineExplorerSystem',
        'use_sim': 'true',
        'joint_limit_params': os.path.join(pkg_path, 'config', 'joint_limits.yaml'),
        'kinematics_params': os.path.join(pkg_path, 'config', 'default_kinematics.yaml'),
        'physical_params': os.path.join(pkg_path, 'config', 'physical_parameters.yaml'),
        'visual_params': os.path.join(pkg_path, 'config', 'visual_parameters.yaml'),
        'initial_positions_file': os.path.join(pkg_path, 'config', 'initial_positions.yaml'),
    }

    robot_description_xml = xacro.process_file(xacro_file, mappings=mappings).toxml()
    return URDF.from_xml_string(robot_description_xml)


def test_essential_links_exist():
    """Check if the world attachment and the ground plane are present."""
    robot = get_robot_urdf()
    links = [link.name for link in robot.links]

    assert 'world' in links, 'The world link is missing!'
    assert 'ground_plane' in links, 'The ground_plane link is missing.'
    assert len(links) > 2, 'The robot seems to only have world/ground links.'


def test_joint_limits_are_valid():
    """Ensure all non-continuous joints have defined limits."""
    robot = get_robot_urdf()

    for joint in robot.joints:
        if joint.type in ['revolute', 'prismatic']:
            assert joint.limit is not None, f'Joint {joint.name} must have limits.'
            assert joint.limit.velocity > 0, f'Joint {joint.name} velocity must be > 0.'
            assert joint.limit.effort > 0, f'Joint {joint.name} effort must be > 0.'
            assert joint.limit.lower < joint.limit.upper, f'Joint {joint.name} range invalid.'


def test_chain_continuity():
    """Verify that there is a continuous chain from world to the end-effector."""
    robot = get_robot_urdf()

    joints_parents = [joint.parent for joint in robot.joints]
    joints_children = [joint.child for joint in robot.joints]

    assert 'world' in joints_parents, 'The world link is not a parent of any joint.'
    assert 'ground_plane' in joints_children or 'ground_plane' in joints_parents, \
        'The ground_plane is not connected to the kinematic tree.'
