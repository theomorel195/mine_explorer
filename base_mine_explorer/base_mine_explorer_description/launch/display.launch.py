#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command
from launch.substitutions import FindExecutable
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    
    # -----------------------
    # Package
    # -----------------------
    base_description_pkg_share = FindPackageShare('base_mine_explorer_description')
    
    # -----------------------
    # Arguments
    # -----------------------
    declared_arguments = [
        DeclareLaunchArgument("prefix", default_value=''),
        DeclareLaunchArgument("name", default_value='MobileBaseMineExplorerSystem'),
        DeclareLaunchArgument("use_sim", default_value='true'),
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
    # Rviz
    # -----------------------
    rviz_config_file = PathJoinSubstitution(
        [
            FindPackageShare('base_mine_explorer_description'),
            'rviz',
            'model.rviz'
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
    # Joint State Publisher
    # -----------------------
    joint_state_publisher_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
    )

    # -----------------------
    # Robot State Publisher
    # -----------------------
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[{'robot_description': robot_description_file}],
    )
    
    # -----------------------
    # Launch
    # -----------------------
    return LaunchDescription(
        declared_arguments + [
            joint_state_publisher_node,
            robot_state_publisher_node,
            rviz_node,
        ]
    )
