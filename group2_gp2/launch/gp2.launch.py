# Name: Nam Facchetti & Yossaphat Kulvatunyou
# Module: gp2.launch.py

# Launch file that starts:
# 1. NavigateToGoal action server
# 2. NavigateToGoal action client

# - Loads goals.yaml into the client
# - Exposes goal_tolerance and yaw_tolerance as launch arguments

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description() -> LaunchDescription:
    """
    Generate the launch description for the GP2 system.

    Returns:
        LaunchDescription: Configured launch description.
    """

    # Resolve goals.yaml path
    package_name = "group2_gp2"
    pkg_share = get_package_share_directory(package_name)

    goals_file = os.path.join(pkg_share, "config", "goals.yaml")

    # Launch arguments
    goal_tolerance_arg = DeclareLaunchArgument(
        "goal_tolerance",
        default_value="0.10",
        description="Position tolerance (meters)",
    )

    yaw_tolerance_arg = DeclareLaunchArgument(
        "yaw_tolerance",
        default_value="0.05",
        description="Yaw tolerance (radians)",
    )

    goal_tolerance = LaunchConfiguration("goal_tolerance")
    yaw_tolerance = LaunchConfiguration("yaw_tolerance")

    # Action Server Node
    server_node = Node(
        package=package_name,
        executable="navigate_to_goal_server",
        name="navigate_to_goal_server",
        output="screen",
        emulate_tty=True,
        parameters=[
            {
                "goal_tolerance": goal_tolerance,
                "yaw_tolerance": yaw_tolerance,
            }
        ],
    )

    # Action Client Node
    client_node = Node(
        package=package_name,
        executable="navigate_to_goal_client",
        name="navigate_to_goal_client",
        output="screen",
        emulate_tty=True,
        parameters=[goals_file],
    )

    # Launch Description
    return LaunchDescription(
        [
            goal_tolerance_arg,
            yaw_tolerance_arg,
            server_node,
            client_node,
        ]
    )