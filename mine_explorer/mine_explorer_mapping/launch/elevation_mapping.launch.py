from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    config_file = os.path.join(
        get_package_share_directory('mine_explorer_mapping'),
        'config',
        'elevation_mapping_params.yaml'
    )

    elevation_mapping_node = Node(
        package='mine_explorer_mapping',
        executable='elevation_mapping_node',
        name='elevation_mapping_node',
        output='screen',
        parameters=[config_file]
    )

    return LaunchDescription([
        elevation_mapping_node
    ])