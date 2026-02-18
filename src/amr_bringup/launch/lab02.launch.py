from launch import LaunchDescription
from launch_ros.actions import LifecycleNode, Node

import math


def generate_launch_description():

    simulation = False
    # start = (1.0, -1.0, 0.5 * math.pi)  # Outer corridor
    # start = (0.6, -0.6, 1.5 * math.pi)  # Inner corridor

    wall_follower_node = LifecycleNode(
        package="amr_control",
        executable="wall_follower",
        name="wall_follower",
        namespace="",
        output="screen",
        arguments=["--ros-args", "--log-level", "WARN"],
        parameters=[{"simulation": simulation}],
    )

    odometry_node = LifecycleNode(
        package="amr_turtlebot3",
        executable="odometry_node",
        name="odometry_node",
        namespace="",
        output="screen",
        arguments=["--ros-args", "--log-level", "WARN"],
    )

    lifecycle_manager_node = Node(
        package="amr_bringup",
        executable="lifecycle_manager",
        namespace="",
        output="screen",
        arguments=["--ros-args", "--log-level", "WARN"],
        parameters=[
            {
                "node_startup_order": [
                    "odometry_node",
                    "wall_follower",  # Must be started last
                ]
            }
        ],
    )

    return LaunchDescription(
        [
            wall_follower_node,
            odometry_node,
            lifecycle_manager_node,
        ]
    )
