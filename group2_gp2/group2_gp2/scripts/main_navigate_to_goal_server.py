# Name: Yossaphat Kulvatunyou & Nam Facchetti 
# Module: main_navigate_to_goal_server.py

import rclpy
from rclpy.executors import MultiThreadedExecutor
from group2_gp2.navigate_to_goal_server import NavigateToGoalServer


def main(args=None):
    """
    Entry point for the navigate_to_goal_server executable.

    Creates and spins a NavigateToGoalServer node that provides the
    navigate action.

    Initialize ROS 2, spin the NavigateToGoalServer, and handle shutdown.
    Uses a MultiThreadedExecutor so that cancel requests can be
    processed while the execute callback is blocking.
    Args:
        args (list, optional): Command-line arguments passed to rclpy.init().
    """
    rclpy.init(args=args)
    node = NavigateToGoalServer("navigate_to_goal_server")
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt as e:
        print(f"Exception: {type(e).__name__}")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
