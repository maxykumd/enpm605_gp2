# Name: Nam Facchetti & Yossaphat Kulvatunyou
# Module: gp2.launch.py

# Launch file that starts:
# 1. NavigateToGoal action server
# 2. NavigateToGoal action client

# Loads goals.yaml into the client
# Exposes goal_tolerance and yaw_tolerance as launch arguments

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description() -> LaunchDescription:
    """
    Generate the launch description for the NavigateToGoal action server and client.
    Loads goals from the goals.yaml file and sets up launch arguments for tolerances.
    (Referenced lecture 10 and lecture 11 launch files and parameter loading)

    Returns:
        LaunchDescription: The launch description containing the nodes and parameters to be launched.
    """

    # Resolve goals.yaml path
    package_name = "group2_gp2"
    pkg_share = get_package_share_directory(package_name)   # Get the share directory of the package to locate the goals.yaml file

    goals_file = os.path.join(pkg_share, "config", "goals.yaml")   # Construct the full path to the goals.yaml file in the config directory

    # Launch arguments
    goal_tolerance_arg = DeclareLaunchArgument(   # Declare a launch argument for goal tolerance with a default value of 0.10 meters
        "goal_tolerance",
        default_value = "0.10",
        description = "Position tolerance (meters)",
    )

    yaw_tolerance_arg = DeclareLaunchArgument(   # Declare a launch argument for yaw tolerance with a default value of 0.05 radians
        "yaw_tolerance",
        default_value = "0.05",
        description = "Yaw tolerance (radians)",
    )

    goal_tolerance = LaunchConfiguration("goal_tolerance")   # Create a LaunchConfiguration substitution for the goal tolerance argument to be used in node parameters
    yaw_tolerance = LaunchConfiguration("yaw_tolerance")   # Create a LaunchConfiguration substitution for the yaw tolerance argument to be used in node parameters

    # Action Server Node
    server_node = Node(   # Create the NavigateToGoal action server node with parameters for goal and yaw tolerances
        package=package_name,
        executable="navigate_to_goal_server",
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
    client_node = Node(   # Create the NavigateToGoal action client node, passing the path to the goals.yaml file as a parameter
        package=package_name,
        executable="navigate_to_goal_client",
        output="screen",
        emulate_tty=True,
        parameters=[goals_file],
    )

    # Launch Description
    return LaunchDescription(   # Return the launch description containing all the nodes and launch arguments to be launched together
        [
            goal_tolerance_arg,
            yaw_tolerance_arg,
            server_node,
            client_node,
        ]
    )