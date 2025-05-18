#!/usr/bin/env python3
# filepath: /home/jtammisto/estimator-ros/src/server/launch/server.launch.py
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    """
    Launch the FastAPI server with ROS integration
    """
    # Get the package directory
    pkg_dir = get_package_share_directory('server')
    
    # Launch the FastAPI server
    server_node = ExecuteProcess(
        cmd=['python3', os.path.join(pkg_dir, 'scripts', 'start_server.py')],
        name='trajectory_server',
        output='screen'
    )
    
    return LaunchDescription([
        server_node
    ])