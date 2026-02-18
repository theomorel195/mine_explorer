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
    pkg_path = get_package_share_directory('mine_explorer_description')
    arm_pkg_path = get_package_share_directory('arm_mine_explorer_description')
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

    robot_description_xml = xacro.process_file(xacro_file, mappings=mappings).toxml()
    return URDF.from_xml_string(robot_description_xml)


def test_essential_links_exist():
    """Check if base_link, base_footprint and the arm_base_link are present."""
    robot = get_robot_urdf()
    links = [link.name for link in robot.links]

    assert 'base_link' in links, 'The base_link link is missing!'
    assert 'base_footprint' in links, 'The base_footprint link is missing!'
    assert 'arm_base_link' in links, 'The arm_base_link link is missing!'

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
    """
    Verify the robot kinematic structure.

    1. All base joints are connected to base_link.
    2. arm_base_joint is fixed to top_plate_link.
    3. There is a continuous chain from arm_base_joint to the end effector.
    """
    robot = get_robot_urdf()

    tree = {}
    for joint in robot.joints:
        parent = joint.parent
        child = joint.child
        tree.setdefault(parent, []).append(child)

    # 1. base connectivity
    base_children = tree.get('base_link', [])
    assert base_children, 'No joints connected to base_link.'

    base_joints = [j for j in robot.joints if j.parent == 'base_link']
    assert len(base_joints) > 0, 'No joints attached to base_link.'

    # 2. arm_base_joint -> top_plate_link (fixed joint)
    matching_joints = [
        j for j in robot.joints
        if (j.name == 'arm_base_joint' and
            j.parent == 'top_plate_link' and
            j.child == 'arm_base_link')
    ]
    assert len(matching_joints) == 1, 'Connection top_plate_link <-> arm_base_link failed.'
    assert matching_joints[0].type == 'fixed', 'arm_base_joint must be fixed.'

    assert len(matching_joints) == 1, \
        'arm_base_joint is not properly connected to top_plate_link.'

    assert matching_joints[0].type == 'fixed', \
        'arm_base_joint must be connected to top_plate_link with a fixed joint.'

    # 3. continuity to end effector
    end_effector = 'arm_tool0'

    visited = set()
    stack = ['arm_base_link']

    while stack:
        current = stack.pop()
        if current == end_effector:
            break
        if current in visited:
            continue
        visited.add(current)
        # ajouter tous les enfants existants, s'il y en a
        stack.extend(tree.get(current, []))

    assert end_effector in visited or end_effector == current, \
        'No continuous kinematic chain from arm_base_link to end effector.'
