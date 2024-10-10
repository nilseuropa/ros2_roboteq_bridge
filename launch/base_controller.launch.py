import os

from ament_index_python.packages import get_package_share_directory

import launch
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler, DeclareLaunchArgument
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

import xacro


def generate_launch_description():

    base_controller_node = Node(
        package="roboteq_bridge",
        executable="base_controller_node",
        parameters=[
            {"port_name": "/dev/roboteq"},
        ]
    )

    ld = LaunchDescription()

    ld.add_action(base_controller_node)

    return ld
