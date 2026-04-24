# Name: Nam Facchetti & Yossaphat Kulvatunyou
# Module: main_navigate_to_goal_client.py - Entry point for the NavigateToGoalClient node. Initializes ROS2, creates the node, and spins it.

import rclpy

from group2_gp2.navigate_to_goal_client import NavigateToGoalClient


def main(args=None) -> None:
    """
    Main function that initializes and runs the action client node.

    Args:
        args: Optional command-line arguments passed to ROS2.
    """
    rclpy.init(args=args)

    node = NavigateToGoalClient()

    try:
        node.send_next_goal()
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down NavigateToGoalClient node.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()