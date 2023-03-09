from importlib.resources import path
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    ads_map = os.path.join(
        get_package_share_directory('ros_ads_node'),
        'config',
        'configuration.yaml'
        )
    node=Node(
        package = 'ros_ads_node',
        name = 'Ads Node',
        executable = 'ros_ads_node',
        parameters = [{'YAML_config_file' :ads_map}]
    )

    description = [node]

    return LaunchDescription(description)
