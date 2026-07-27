from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    package_share = get_package_share_directory("micro_lqr_controller")
    default_config = os.path.join(package_share, "config", "lqr.yaml")
    config = LaunchConfiguration("config")
    return LaunchDescription([
        DeclareLaunchArgument(
            "config",
            default_value=default_config,
            description="Path to controller YAML file",
        ),
        DeclareLaunchArgument(
            "show_poles",
            default_value="false",
            description="Compatibility argument; pole monitor is not started by this package",
        ),
        Node(
            package="micro_lqr_controller",
            executable="micro_lqr_node",
            name="micro_lqr_controller",
            output="screen",
            parameters=[config],
        ),
    ])
