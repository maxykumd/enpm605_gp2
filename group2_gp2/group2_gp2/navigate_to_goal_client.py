# Name: Nam Facchetti & Yossaphat Kulvatunyou
# Module: navigate_to_goal_client.py - ROS2 Action Client that sends three sequential navigation goals to the NavigateToGoal action server.

from typing import List, Tuple
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from geometry_msgs.msg import Point, Pose
from group2_gp2_interfaces.action import NavigateToGoal
from scipy.spatial.transform import Rotation as R

class NavigateToGoalClient(Node):
    """
    Action client that sends three goals sequentially to the server. 
    (Referenced lecture 10 action_demo navigate_client.py)

    Attributes:
        _action_client (ActionClient): The ROS2 action client for NavigateToGoal.
        _goals (List[Tuple[float, float, float]]): List of goals loaded from parameters, each as (x, y, final_heading).
        _current_goal_index (int): Index of the current goal being processed.
        _results (List[Tuple[float, float]]): List of results for each goal, each as (total_distance, elapsed_time).
    """

    def __init__(self) -> None:
        """Initialize the action client node."""
        super().__init__("navigate_to_goal_client")   # Node name is navigate_to_goal_client as specified in the launch file

        self._action_client = ActionClient(   # Initialize the action client for the NavigateToGoal action
            self,
            NavigateToGoal,
            "navigate_to_goal",
        )

        # Load goals from parameters
        self._goals: List[Tuple[float, float, float]] = []   # List of (x, y, final_heading) tuples
        self._load_goals()

        self._current_goal_index: int = 0
        self._results: List[Tuple[float, float]] = []  # List of (total_distance, elapsed_time) tuples for each goal

    def _load_goals(self) -> None:
        """
        Load goals from ROS2 parameters. Expects parameters in the format:
            goal1.x, goal1.y, goal1.final_heading
            goal2.x, goal2.y, goal2.final_heading
            goal3.x, goal3.y, goal3.final_heading
        These parameters should be set by the launch file from the goals.yaml configuration.
        """
        for i in range(1, 4):
            prefix = f"goal{i}"

            self.declare_parameter(f"{prefix}.x", 0.0)   # Declare parameters for each goal's x, y, and final_heading with default values
            self.declare_parameter(f"{prefix}.y", 0.0)
            self.declare_parameter(f"{prefix}.final_heading", 0.0)

            x = self.get_parameter(f"{prefix}.x").value   # Pulling from YAML via launch file parameters
            y = self.get_parameter(f"{prefix}.y").value
            heading = self.get_parameter(f"{prefix}.final_heading").value   

            if x is None or y is None or heading is None:   # Check if any parameter is missing
                self.get_logger().error(f"Missing parameters for {prefix}")
                raise RuntimeError("Failed to load goals from YAML")

            self._goals.append((x, y, heading))

        self.get_logger().info(
            f"Loaded 3 goals: {[(round(g[0],2), round(g[1],2), round(g[2],2)) for g in self._goals]}"
        )

    def send_next_goal(self) -> None:
        """
        Send the next goal in sequence.
        """
        if self._current_goal_index >= len(self._goals):   # All goals have been processed
            self._print_summary()
            return

        x, y, heading = self._goals[self._current_goal_index]   # Get the current goal's parameters

        self.get_logger().info(
            f"--- Sending goal {self._current_goal_index+1}/3: "
            f"({x:.2f}, {y:.2f}, final_heading={heading:.2f}) ---" 
        )

        goal_msg = NavigateToGoal.Goal()   # Create the goal message to send to the action server
        goal_msg.goal_position = Point(x=x, y=y, z=0.0)
        goal_msg.final_heading = heading

        self.get_logger().info("Waiting for action server...")
        self._action_client.wait_for_server()

        send_goal_future = self._action_client.send_goal_async(   # Send the goal asynchronously and register callbacks for response and feedback
            goal_msg,
            feedback_callback=self.feedback_callback,
        )

        send_goal_future.add_done_callback(self.goal_response_callback)   # Register callback to handle goal acceptance/rejection and result processing

    def goal_response_callback(self, future) -> None:
        """
        Handle goal acceptance/rejection.
        """
        goal_handle: ClientGoalHandle = future.result()   # Get the goal handle from the future result

        if not goal_handle.accepted:   # Check if the goal was accepted by the server
            self.get_logger().error("Goal rejected by server. Aborting mission.")
            return

        self.get_logger().info("Goal accepted.")

        result_future = goal_handle.get_result_async()   # Register callback to handle the result when the action is complete
        result_future.add_done_callback(self.result_callback)   # Register callback to process the result once the action is complete

    def feedback_callback(self, feedback_msg) -> None:
        """
        Receive feedback from the server (throttled logging).
        """
        feedback = feedback_msg.feedback   # Extract feedback data from the message
        pose: Pose = feedback.current_pose   # Current pose of the robot from feedback
        dist = feedback.distance_remaining   # Distance remaining to the goal from feedback
        q = pose.orientation   # Quaternion orientation from feedback to compute yaw for logging
        yaw = R.from_quat([q.x, q.y, q.z, q.w]).as_euler("xyz")[2]   # Computing yaw from quaternion (lecture 11 p_controller_demo.py)
        self.get_logger().info(
            f"Feedback: pose=({pose.position.x:.2f}, "
            f"{pose.position.y:.2f}, yaw={yaw:.2f}) "
            f"remaining={dist:.2f}",
            throttle_duration_sec = 1.0,   # Throttle feedback logging to once per second to avoid spamming
        )

    def result_callback(self, future) -> None:
        """
        Handle result from server.
        """
        result = future.result().result   # Extract the result data from the future result

        if not result.success:   # Check if the goal was successfully reached according to the server's result
            self.get_logger().error(
                f"Goal {self._current_goal_index+1} failed. Aborting mission."
            )
            return

        self.get_logger().info(   # Log the result for the current goal, including total distance traveled and elapsed time
            f"Goal {self._current_goal_index+1}/3 succeeded. "
            f"total_distance={result.total_distance:.2f}, "
            f"elapsed_time={result.elapsed_time:.2f}s"
        )

        self._results.append(   # Store the result for summary printing at the end of the mission
            (result.total_distance, result.elapsed_time)
        )

        self._current_goal_index += 1
        self.send_next_goal()

    def _print_summary(self) -> None:
        """
        Print final mission summary.
        """
        self.get_logger().info("=" * 40)   # Print a summary of the mission results after all goals have been processed, including total distance and time for each goal
        self.get_logger().info("Mission complete. All 3 goals reached.")

        for i, (dist, time) in enumerate(self._results):
            self.get_logger().info(
                f"  Goal {i+1}: total_distance={dist:.2f}, "
                f"elapsed_time={time:.2f}s"
            )

        self.get_logger().info("=" * 40)   # Print a separator line for clarity in the logs

def main(args=None) -> None:
    """Main entry point."""
    rclpy.init(args=args)

    client_node = NavigateToGoalClient()   # Load goals from parameters and prepare the action client
    client_node.send_next_goal()   # Start the sequence by sending the first goal

    rclpy.spin(client_node)   # Keep the node alive to process action results and send subsequent goals

    client_node.destroy_node()
    rclpy.shutdown()