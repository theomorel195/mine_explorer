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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Launch the simulation of the mobile robot in gazebo."""
    # Packages
    base_description_pkg_share = FindPackageShare('base_mine_explorer_description')
    base_gazebo_pkg_share = FindPackageShare('base_mine_explorer_gazebo')

    # Launch arguments
    declared_arguments = [
        DeclareLaunchArgument('prefix', default_value=''),
        DeclareLaunchArgument('name', default_value='MobileBaseMineExplorerSystem'),
        DeclareLaunchArgument('use_sim', default_value='true'),
        DeclareLaunchArgument(
            'world',
            default_value=PathJoinSubstitution([
                base_gazebo_pkg_share, 'worlds', 'empty_world.model'
            ]))
    ]

    # Robot description
    robot_description_file = Command([
        PathJoinSubstitution([
            FindExecutable(name='xacro')
        ]),
        ' ',
        PathJoinSubstitution([
            base_description_pkg_share, 'urdf', 'robot.urdf.xacro'
        ]),
        ' ',
        'prefix:=', LaunchConfiguration('prefix'),
        ' ',
        'name:=', LaunchConfiguration('name'),
        ' ',
        'use_sim:=', LaunchConfiguration('use_sim'),
        ' ',
    ])

    # Robot state publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[
            {'robot_description': robot_description_file},
            {'use_sim_time': LaunchConfiguration('use_sim')},
        ],
        output='screen',
    )

    # Controllers
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager',
            '/controller_manager'
        ],
        output='screen',
    )

    base_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['base_controller'],
        output='screen',
    )

    delay_controllers = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[
                base_controller_spawner,
            ],
        )
    )

    # Gazebo
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare('gazebo_ros'),
                    'launch',
                    'gazebo.launch.py'
                ]
            )
        ),
        launch_arguments={
            'verbose': 'false',
            'world': LaunchConfiguration('world'),
        }.items(),
    )

    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', LaunchConfiguration('name'),
        ],
        output='screen',
    )

    # Launch
    return LaunchDescription(
        declared_arguments + [
            gazebo_launch,
            robot_state_publisher_node,
            joint_state_broadcaster_spawner,
            delay_controllers,
            spawn_robot,
        ]
    )
