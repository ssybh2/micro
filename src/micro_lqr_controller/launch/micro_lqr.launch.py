from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    package_share = get_package_share_directory("micro_lqr_controller")
    default_config = os.path.join(package_share, "config", "lqr.yaml")

    config = LaunchConfiguration("config")
    show_poles = LaunchConfiguration("show_poles")
    pole_topic = LaunchConfiguration("pole_topic")
    pole_monitor_period_s = LaunchConfiguration("pole_monitor_period_s")

    return LaunchDescription([
        DeclareLaunchArgument(
            "config",
            default_value=default_config,
            description="Path to the cascade/LQR controller YAML file",
        ),
        DeclareLaunchArgument(
            "show_poles",
            default_value="true",
            description="Open the live closed-loop pole/unit-circle window",
        ),
        DeclareLaunchArgument(
            "pole_topic",
            default_value="/micro_lqr/poles",
            description="Closed-loop pole analysis topic",
        ),
        DeclareLaunchArgument(
            "pole_monitor_period_s",
            default_value="0.5",
            description="How often to re-read controller parameters and recompute poles",
        ),
        Node(
            package="micro_lqr_controller",
            executable="micro_lqr_node",
            name="micro_lqr_controller",
            output="screen",
            parameters=[config],
        ),
        Node(
            package="micro_lqr_controller",
            executable="micro_lqr_pole_monitor",
            name="micro_lqr_pole_monitor",
            output="screen",
            parameters=[{
                "controller_node_name": "/micro_lqr_controller",
                "pole_topic": pole_topic,
                "monitor_period_s": pole_monitor_period_s,
            }],
        ),
        Node(
            package="micro_lqr_controller",
            executable="pole_visualizer.py",
            name="micro_lqr_pole_visualizer",
            output="screen",
            emulate_tty=True,
            condition=IfCondition(show_poles),
            parameters=[{
                "pole_topic": pole_topic,
                "window_title": "Micro cascade local closed-loop poles",
            }],
        ),
    ])
