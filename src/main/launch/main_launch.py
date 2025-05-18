#!/usr/bin/env python3
from launch_ros.actions import LifecycleNode, LifecycleTransition
from lifecycle_msgs.msg import Transition

import launch
from launch import LaunchDescription


def generate_launch_description():
    ld = LaunchDescription(
        [
            LifecycleNode(
                package="optical_sensor",
                executable="start_optical.py",
                name="optical_sensor_node",
                namespace="/",
                output="log",
            ),
        ]
    )

    ld.add_action(
        LifecycleTransition(
            lifecycle_node_names=["optical_sensor_node"],
            transition_ids=[
                Transition.TRANSITION_CONFIGURE,
                Transition.TRANSITION_ACTIVATE,
            ],
        )
    )

    return ld


if __name__ == "__main__":
    ls = launch.LaunchService()
    ls.include_launch_description(generate_launch_description())
    ls.run()
