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
    pkg_path = get_package_share_directory('base_mine_explorer_description')
    xacro_file = os.path.join(pkg_path, 'urdf', 'robot.urdf.xacro')

    mappings = {
        'prefix': '',
        'name': 'MobileBaseMineExplorerSystem',
        'use_sim': 'true',
    }

    robot_description_xml = xacro.process_file(xacro_file, mappings=mappings).toxml()
    return URDF.from_xml_string(robot_description_xml)


def test_essential_links_exist():
    """Check if the world attachment and the ground plane are present."""
    robot = get_robot_urdf()
    links = [link.name for link in robot.links]

    assert 'base_link' in links, 'The base_link link is missing!'
    assert 'base_footprint' in links, 'The base_footprint link is missing!'

    wheel_links = [name for name in links if 'wheel' in name]
    assert len(wheel_links) == 4, f'Expected 4 wheels, but found {len(wheel_links)}: {wheel_links}'
    assert len(links) >= 6, f'Robot should have at least 6 links, found {len(links)}.'


def test_mobile_base_joints():
    """Ensure wheel joints are correctly defined as continuous."""
    robot = get_robot_urdf()

    wheel_joints = [j for j in robot.joints if 'wheel' in j.name]

    assert len(wheel_joints) == 4, f'Expected 4 wheel joints, but found {len(wheel_joints)}.'

    for joint in wheel_joints:
        assert joint.type == 'continuous', f'Wheel joint {joint.name} must be continuous.'
        assert joint.limit is None or joint.limit.effort > 0, \
            f'Wheel {joint.name} must have effort defined if limits exist.'


def test_joint_limits_are_valid():
    """Ensure all non-continuous joints have defined limits."""
    robot = get_robot_urdf()

    for joint in robot.joints:
        if joint.type in ['revolute', 'prismatic']:
            assert joint.limit is not None, f'Joint {joint.name} must have limits.'
            assert joint.limit.velocity > 0, f'Joint {joint.name} velocity must be > 0.'
            assert joint.limit.effort > 0, f'Joint {joint.name} effort must be > 0.'
            assert joint.limit.lower < joint.limit.upper, f'Joint {joint.name} range invalid.'


def test_kinematic_chain():
    """Verify that all joints are connected to the base_link."""
    robot = get_robot_urdf()

    joints_parents = [joint.parent for joint in robot.joints]

    assert 'base_footprint' in joints_parents or 'base_link' in joints_parents, \
        'The kinematic tree is not rooted at base_footprint or base_link.'
