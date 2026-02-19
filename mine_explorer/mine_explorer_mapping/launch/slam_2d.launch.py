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
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Launch slam 2D."""
    pkg_share = get_package_share_directory('mine_explorer_mapping')

    pointcloud_to_laserscan_node = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        remappings=[
            ('cloud_in', '/sensors/lidar/data'),
            ('scan', '/sensors/lidar/scan_2d')
        ],
        parameters=[{
            'target_frame': 'lidar_base_link',
            'min_height': -0.1,
            'max_height': 0.1,
            'angle_min': -2.35619,
            'angle_max': 2.34746,
            'angle_increment': 0.008726,
            'range_min': 0.4,
            'range_max': 30.0,
            'use_inf': True
        }]
    )

    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            os.path.join(pkg_share, 'config', 'slam_2d_params.yaml'),
            {'use_sim_time': True}
        ]
    )

    return LaunchDescription([
        pointcloud_to_laserscan_node,
        slam_toolbox_node
    ])
