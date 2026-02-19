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
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Launch the simulation with EKF and SLAM for localization."""
    # Packages
    pkg_gazebo = get_package_share_directory('mine_explorer_gazebo')
    pkg_control = get_package_share_directory('mine_explorer_control')
    pkg_mapping = get_package_share_directory('mine_explorer_mapping')

    # Arguments
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='empty.world',
        description='Monde Gazebo à charger'
    )

    # Gazebo
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': LaunchConfiguration('world')}.items()
    )

    # EKF
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_control, 'launch', 'localization.launch.py')
        )
    )

    # SLAM 2D
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_mapping, 'launch', 'slam_2d.launch.py')
        )
    )

    return LaunchDescription([
        world_arg,
        gazebo_launch,
        localization_launch,
        slam_launch
    ])
