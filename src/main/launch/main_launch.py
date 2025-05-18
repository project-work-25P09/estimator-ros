#!/usr/bin/env python3
from launch_ros.actions import Node, LifecycleNode, LifecycleTransition
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
            LifecycleNode(
                package="microstrain_inertial_driver",
                executable="microstrain_launch.py",
                name="microstrain_intertial_node",
                namespace="/",
                output="log",
            ),
            Node(
                package="estimation",
                executable="simple_estimation.py",
                name="estimation_node",
                output="log",
            ),
            Node(
                package="server",
                executable="start_server_dash_full.py",
                name="server_node",
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
    # ld.add_action(
    #     LifecycleTransition(
    #         lifecycle_node_names=["microstrain_intertial_node"],
    #         transition_ids=[
    #             Transition.TRANSITION_CONFIGURE,
    #             Transition.TRANSITION_ACTIVATE,
    #         ],
    #     )
    # )

    return ld


if __name__ == "__main__":
    ls = launch.LaunchService()
    ls.include_launch_description(generate_launch_description())
    ls.run()
