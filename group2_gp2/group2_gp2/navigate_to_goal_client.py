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
    """

    def __init__(self) -> None:
        """Initialize the action client node."""
        super().__init__("navigate_to_goal_client")

        self._action_client = ActionClient(
            self,
            NavigateToGoal,
            "navigate_to_goal",
        )

        # Load goals from parameters
        self._goals: List[Tuple[float, float, float]] = []
        self._load_goals()

        self._current_goal_index: int = 0
        self._results: List[Tuple[float, float]] = []  # (distance, time)

    def _load_goals(self) -> None:
        """
        Load goal1, goal2, goal3 from parameters.
        """
        for i in range(1, 4):
            prefix = f"goal{i}"

            self.declare_parameter(f"{prefix}.x", 0.0)
            self.declare_parameter(f"{prefix}.y", 0.0)
            self.declare_parameter(f"{prefix}.final_heading", 0.0)

            x = self.get_parameter(f"{prefix}.x").value
            y = self.get_parameter(f"{prefix}.y").value
            heading = self.get_parameter(f"{prefix}.final_heading").value

            if x is None or y is None or heading is None:
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
        if self._current_goal_index >= len(self._goals):
            self._print_summary()
            return

        x, y, heading = self._goals[self._current_goal_index]

        self.get_logger().info(
            f"--- Sending goal {self._current_goal_index+1}/3: "
            f"({x:.2f}, {y:.2f}, final_heading={heading:.2f}) ---"
        )

        goal_msg = NavigateToGoal.Goal()
        goal_msg.goal_position = Point(x=x, y=y, z=0.0)
        goal_msg.final_heading = heading

        self.get_logger().info("Waiting for action server...")
        self._action_client.wait_for_server()

        send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback,
        )

        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future) -> None:
        """
        Handle goal acceptance/rejection.
        """
        goal_handle: ClientGoalHandle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected by server. Aborting mission.")
            return

        self.get_logger().info("Goal accepted.")

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def feedback_callback(self, feedback_msg) -> None:
        """
        Receive feedback from the server (throttled logging).
        """
        feedback = feedback_msg.feedback
        pose: Pose = feedback.current_pose
        dist = feedback.distance_remaining
        q = pose.orientation
        yaw = R.from_quat([q.x, q.y, q.z, q.w]).as_euler("xyz")[2]
        self.get_logger().info(
            f"Feedback: pose=({pose.position.x:.2f}, "
            f"{pose.position.y:.2f}, yaw={yaw:.2f}) "
            f"remaining={dist:.2f}",
            throttle_duration_sec=1.0,
        )

    def result_callback(self, future) -> None:
        """
        Handle result from server.
        """
        result = future.result().result

        if not result.success:
            self.get_logger().error(
                f"Goal {self._current_goal_index+1} failed. Aborting mission."
            )
            return

        self.get_logger().info(
            f"Goal {self._current_goal_index+1}/3 succeeded. "
            f"total_distance={result.total_distance:.2f}, "
            f"elapsed_time={result.elapsed_time:.2f}s"
        )

        self._results.append(
            (result.total_distance, result.elapsed_time)
        )

        self._current_goal_index += 1
        self.send_next_goal()

    def _print_summary(self) -> None:
        """
        Print final mission summary.
        """
        self.get_logger().info("=" * 40)
        self.get_logger().info("Mission complete. All 3 goals reached.")

        for i, (dist, time) in enumerate(self._results):
            self.get_logger().info(
                f"  Goal {i+1}: total_distance={dist:.2f}, "
                f"elapsed_time={time:.2f}s"
            )

        self.get_logger().info("=" * 40)


def main(args=None) -> None:
    """Main entry point."""
    rclpy.init(args=args)

    client_node = NavigateToGoalClient()
    client_node.send_next_goal()

    rclpy.spin(client_node)

    client_node.destroy_node()
    rclpy.shutdown()