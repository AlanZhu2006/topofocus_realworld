#!/usr/bin/env python3
"""Launch only the Odin1 sensor/SLAM driver required by TopoFocus.

The vendor launch file also starts RViz and three visualization/reprojection
nodes. The Hub adapter consumes the driver's native image, SLAM cloud and
odometry topics directly, so a service deployment needs only
``host_sdk_sample``. This launch file contains no planner, controller or robot
command publisher.
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    package_dir = get_package_share_directory("odin_ros_driver")
    default_config = f"{package_dir}/config/control_command.yaml"
    config_file = DeclareLaunchArgument(
        "config_file",
        default_value=default_config,
        description="Odin1 driver control_command.yaml",
    )
    driver = Node(
        package="odin_ros_driver",
        executable="host_sdk_sample",
        name="host_sdk_sample",
        output="screen",
        parameters=[{"config_file": LaunchConfiguration("config_file")}],
    )
    return LaunchDescription([config_file, driver])
