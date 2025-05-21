#!/usr/bin/env python3
import os

import launch
from launch import LaunchDescription
from launch.actions import RegisterEventHandler, EmitEvent
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    optical = LifecycleNode(
        package="optical_sensor",
        executable="start_optical.py",
        name="optical_sensor_node",
        namespace="/",
        output="log",
    )
    
    microstrain_path = os.path.join(
        get_package_share_directory('microstrain_inertial_driver'),
        'launch',
        'microstrain_launch.py'
    )

    hw_monitor_path = os.path.join(
        get_package_share_directory('rpi_hw_monitor'),
        'launch',
        'hw_monitor_launch.py'
    )

    ld = LaunchDescription([
        optical,
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
            PythonLaunchDescriptionSource([microstrain_path]),
            launch_arguments={
                # TODO: absolute path should be in share directory
                'params_file': '/home/project/estimator-ros/config/imu_params.yml'
            }.items()
        ),
    )
    
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([hw_monitor_path]),
        ),
    )

    ld.add_action(EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(optical),
            transition_id=Transition.TRANSITION_CONFIGURE,
        )
    ))

    ld.add_action(RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=optical,
            goal_state="inactive",
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(optical),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    )
                )
            ],
        )
    ))

    return ld

if __name__ == "__main__":
    ls = launch.LaunchService()
    ls.include_launch_description(generate_launch_description())
    ls.run()
