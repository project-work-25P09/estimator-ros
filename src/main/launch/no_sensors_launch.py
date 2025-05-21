#!/usr/bin/env python3
import os

import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    hw_monitor_path = os.path.join(
        get_package_share_directory('rpi_hw_monitor'),
        'launch',
        'hw_monitor_launch.py'
    )

    ld = LaunchDescription([
        Node(
            package="estimation",
            executable="simple_estimation.py",
            name="estimation_node",
            output="log",
        ),
        Node(
            package="server",
            executable="start_server.py",
            name="server_node",
            output="log",
        ),
    ])

    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([hw_monitor_path]),
        ),
    )

    return ld

if __name__ == "__main__":
    ls = launch.LaunchService()
    ls.include_launch_description(generate_launch_description())
    ls.run()
