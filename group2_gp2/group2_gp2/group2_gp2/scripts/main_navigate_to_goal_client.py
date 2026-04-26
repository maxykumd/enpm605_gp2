# Name: Nam Facchetti & Yossaphat Kulvatunyou
# Module: main_navigate_to_goal_client.py - Entry point for the NavigateToGoalClient node. Initializes ROS2, creates the node, and spins it.

import rclpy
from group2_gp2.navigate_to_goal_client import NavigateToGoalClient

def main(args=None) -> None:
    """
    Main function to initialize ROS2 and run the NavigateToGoalClient node.

    Args:
        args: Optional command-line arguments for ROS2 initialization.
    """
    rclpy.init(args=args) 

    client_node = NavigateToGoalClient()   # Load goals from parameters and prepare the action client

    try:
        client_node.send_next_goal()   # Start the sequence by sending the first goal
        rclpy.spin(client_node)   # Keep the node alive to process action results and send subsequent goals
    except KeyboardInterrupt:
        client_node.get_logger().info("Shutting down NavigateToGoalClient node.")
    finally:
        client_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()