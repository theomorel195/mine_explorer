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
    # Packages
    # -----------------------
    arm_description_pkg_share = FindPackageShare('arm_mine_explorer_description')
    mine_description_pkg_share = FindPackageShare('mine_explorer_description')

    # -----------------------
    # Arguments
    # -----------------------
    declared_arguments = [
        DeclareLaunchArgument("prefix_arm", default_value='arm_'),
        DeclareLaunchArgument("prefix_base", default_value=''),
        DeclareLaunchArgument("name_arm", default_value='ArmMineExplorerSystem'),
        DeclareLaunchArgument("name_base", default_value='MobileBaseMineExplorerSystem'),
        DeclareLaunchArgument("name_lidar", default_value='lidar'),
        DeclareLaunchArgument("use_sim", default_value='true'),
        
        DeclareLaunchArgument("joint_limit_params", 
            default_value=PathJoinSubstitution([arm_description_pkg_share, 'config', 'joint_limits.yaml'])),
        DeclareLaunchArgument("kinematics_params", 
            default_value=PathJoinSubstitution([arm_description_pkg_share, 'config', 'default_kinematics.yaml'])),
        DeclareLaunchArgument("physical_params", 
            default_value=PathJoinSubstitution([arm_description_pkg_share, 'config', 'physical_parameters.yaml'])),
        DeclareLaunchArgument("visual_params", 
            default_value=PathJoinSubstitution([arm_description_pkg_share, 'config', 'visual_parameters.yaml'])),
        DeclareLaunchArgument("initial_positions_file", 
            default_value=PathJoinSubstitution([arm_description_pkg_share, 'config', 'initial_positions.yaml'])),
        
        DeclareLaunchArgument("safety_limits", default_value="true"),
        DeclareLaunchArgument("safety_pos_margin", default_value="0.15"),
        DeclareLaunchArgument("safety_k_position", default_value="20"),
    ]
    
    # -----------------------
    # Robot description
    # -----------------------
    robot_description_file = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]), ' ',
        PathJoinSubstitution([mine_description_pkg_share, 'urdf', 'robot.urdf.xacro']), ' ',
        'prefix_arm:=', LaunchConfiguration("prefix_arm"), ' ',
        'prefix_base:=', LaunchConfiguration("prefix_base"), ' ',
        'name_arm:=', LaunchConfiguration("name_arm"), ' ',
        'name_base:=', LaunchConfiguration("name_base"), ' ',
        'name_lidar:=', LaunchConfiguration("name_lidar"), ' ',
        'use_sim:=', LaunchConfiguration("use_sim"), ' ',
        'joint_limit_params:=', LaunchConfiguration("joint_limit_params"), ' ',
        'kinematics_params:=', LaunchConfiguration("kinematics_params"), ' ',
        'physical_params:=', LaunchConfiguration("physical_params"), ' ',
        'visual_params:=', LaunchConfiguration("visual_params"), ' ',
        'initial_positions_file:=', LaunchConfiguration("initial_positions_file"), ' ',
        'safety_limits:=', LaunchConfiguration("safety_limits"), ' ',
        'safety_pos_margin:=', LaunchConfiguration("safety_pos_margin"), ' ',
        'safety_k_position:=', LaunchConfiguration("safety_k_position"),
    ])

    # -----------------------
    # Rviz
    # -----------------------
    rviz_config_file = PathJoinSubstitution(
        [
            mine_description_pkg_share,
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
