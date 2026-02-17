#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # -----------------------
    # Packages
    # -----------------------
    base_description_pkg_share = FindPackageShare('base_mine_explorer_description')
    base_gazebo_pkg_share = FindPackageShare('base_mine_explorer_gazebo')
    
    # -----------------------
    # Launch arguments
    # -----------------------
    declared_arguments = [
        DeclareLaunchArgument("prefix", default_value=''),
        DeclareLaunchArgument("name", default_value='MobileBaseMineExplorerSystem'),
        DeclareLaunchArgument("use_sim", default_value='true'),
        DeclareLaunchArgument("world", 
            default_value=PathJoinSubstitution([base_gazebo_pkg_share, 'worlds', 'empty_world.model']))
    ]

    # -----------------------
    # Robot description
    # -----------------------
    robot_description_file = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]), ' ',
        PathJoinSubstitution([base_description_pkg_share, 'urdf', 'robot.urdf.xacro']), ' ',
        'prefix:=', LaunchConfiguration("prefix"), ' ',
        'name:=', LaunchConfiguration("name"), ' ',
        'use_sim:=', LaunchConfiguration("use_sim"), ' ',
    ])


    # -----------------------
    # ros2_control
    # -----------------------
    controller_manager_config = PathJoinSubstitution(
        [
            base_gazebo_pkg_share,
            'config',
            'gazebo_controller_manager.yaml',
        ]
    )

    # -----------------------
    # Robot state publisher
    # -----------------------
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[
            {'robot_description': robot_description_file},
            {'use_sim_time': LaunchConfiguration("use_sim")},
        ],
        output='screen',
    )

    # -----------------------
    # Controllers
    # -----------------------
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

    # -----------------------
    # Gazebo
    # -----------------------
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
            'world': LaunchConfiguration("world"),
        }.items(),
    )

    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', LaunchConfiguration("name"),
        ],
        output='screen',
    )

    # -----------------------
    # Launch
    # -----------------------
    return LaunchDescription(
        declared_arguments + [
            gazebo_launch,
            robot_state_publisher_node,
            joint_state_broadcaster_spawner,
            delay_controllers,
            spawn_robot,
        ]
    )
