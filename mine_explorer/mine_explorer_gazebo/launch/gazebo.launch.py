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
    """Launch the simulation of the robotic arm in gazebo."""
    # Packages
    arm_description_pkg_share = FindPackageShare('arm_mine_explorer_description')
    mine_description_pkg_share = FindPackageShare('mine_explorer_description')

    # Arguments
    declared_arguments = [
        DeclareLaunchArgument('prefix_arm', default_value='arm_'),
        DeclareLaunchArgument('prefix_base', default_value=''),
        DeclareLaunchArgument('name_arm', default_value='ArmMineExplorerSystem'),
        DeclareLaunchArgument('name_base', default_value='MobileBaseMineExplorerSystem'),
        DeclareLaunchArgument('name_lidar', default_value='lidar'),
        DeclareLaunchArgument('use_sim', default_value='true'),
        DeclareLaunchArgument(
            'joint_limit_params',
            default_value=PathJoinSubstitution([
                arm_description_pkg_share, 'config', 'joint_limits.yaml'
            ])),
        DeclareLaunchArgument(
            'kinematics_params',
            default_value=PathJoinSubstitution([
                arm_description_pkg_share, 'config', 'default_kinematics.yaml'
            ])),
        DeclareLaunchArgument(
            'physical_params',
            default_value=PathJoinSubstitution([
                arm_description_pkg_share, 'config', 'physical_parameters.yaml'
            ])),
        DeclareLaunchArgument(
            'visual_params',
            default_value=PathJoinSubstitution([
                arm_description_pkg_share, 'config', 'visual_parameters.yaml'
            ])),
        DeclareLaunchArgument(
            'initial_positions_file',
            default_value=PathJoinSubstitution([
                arm_description_pkg_share, 'config', 'initial_positions.yaml'
            ])),
        DeclareLaunchArgument('safety_limits', default_value='true'),
        DeclareLaunchArgument('safety_pos_margin', default_value='0.15'),
        DeclareLaunchArgument('safety_k_position', default_value='20'),
        DeclareLaunchArgument(
            'world',
            default_value='empty.world',
        )
    ]

    # Robot description
    robot_description_file = Command([
        PathJoinSubstitution([
            FindExecutable(name='xacro')
        ]),
        ' ',
        PathJoinSubstitution([
            mine_description_pkg_share,
            'urdf',
            'robot.urdf.xacro'
        ]),
        ' ',
        'prefix_arm:=', LaunchConfiguration('prefix_arm'),
        ' ',
        'prefix_base:=', LaunchConfiguration('prefix_base'),
        ' ',
        'name_arm:=', LaunchConfiguration('name_arm'),
        ' ',
        'name_base:=', LaunchConfiguration('name_base'),
        ' ',
        'name_lidar:=', LaunchConfiguration('name_lidar'),
        ' ',
        'use_sim:=', LaunchConfiguration('use_sim'),
        ' ',
        'joint_limit_params:=', LaunchConfiguration('joint_limit_params'),
        ' ',
        'kinematics_params:=', LaunchConfiguration('kinematics_params'),
        ' ',
        'physical_params:=', LaunchConfiguration('physical_params'),
        ' ',
        'visual_params:=', LaunchConfiguration('visual_params'),
        ' ',
        'initial_positions_file:=', LaunchConfiguration('initial_positions_file'),
        ' ',
        'safety_limits:=', LaunchConfiguration('safety_limits'),
        ' ',
        'safety_pos_margin:=', LaunchConfiguration('safety_pos_margin'),
        ' ',
        'safety_k_position:=', LaunchConfiguration('safety_k_position'),
    ])

    # Robot state publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[
            {'robot_description': robot_description_file,
             'use_sim_time': True,
             'publish_frequency': 50.0},
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

    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller'],
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
                arm_controller_spawner,
                base_controller_spawner,
            ],
        )
    )

    # Rviz
    rviz_config_file = PathJoinSubstitution(
        [
            mine_description_pkg_share,
            'rviz',
            'model_with_sensors.rviz'
        ]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_config_file],
    )

    # Gazebo
    world_file = PathJoinSubstitution([
        FindPackageShare('mine_explorer_resources'),
        'worlds',
        LaunchConfiguration('world')
    ])

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
            'world': world_file,
        }.items(),
    )

    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'mine_explorer_system',
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
            rviz_node,
            spawn_robot,
        ]
    )
