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
    # Package
    # -----------------------
    sensors_description_pkg_share = FindPackageShare('sensors_mine_explorer_description')
    
    # -----------------------
    # Launch arguments
    # -----------------------
    declared_arguments = [
        DeclareLaunchArgument("prefix", default_value=''),
        DeclareLaunchArgument("world", 
            default_value=PathJoinSubstitution([sensors_description_pkg_share, 'worlds', 'empty_world.model']))
    ]

    # -----------------------
    # Robot description
    # -----------------------
    robot_description_file = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]), ' ',
        PathJoinSubstitution([sensors_description_pkg_share, 'urdf', 'camera', 'd455_standalone.urdf.xacro']), ' ',
        'prefix:=', LaunchConfiguration("prefix"), ' ',
    ])


    # -----------------------
    # Rviz
    # -----------------------
    rviz_config_file = PathJoinSubstitution(
        [
            sensors_description_pkg_share,
            'rviz',
            'camera_d455_model.rviz'
        ]
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )

    # -----------------------
    # Robot State Publisher
    # -----------------------
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[
            {'robot_description': robot_description_file},
        ],
        output='screen',
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
            '-entity', 'Camera_3D_RealSense_D455_System',
        ],
        output='screen',
    )

    # -----------------------
    # Launch
    # -----------------------
    return LaunchDescription(
        declared_arguments + [
            gazebo_launch,
            rviz_node,
            robot_state_publisher_node,
            spawn_robot,
        ]
    )
