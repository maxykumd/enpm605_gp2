# Name: Yossaphat Kulvatunyou & Nam Facchetti 
# Module: navigate_to_goal_server.py

import math
import time
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup

from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from scipy.spatial.transform import Rotation as R

from group2_gp2_interfaces.action import NavigateToGoal

class NavigateToGoalServer(Node):
    """ROS 2 Navigate to Server that drives the robot to a goal pose."""
    MAX_LINEAR = 0.5    # m/s
    MAX_ANGULAR = 1.0   # rad/s

    def __init__(self, node_name: str) -> None:
        """Initialize the NavigateToGoalServer node.

        Args:
            node_name: Name to register this node with in the ROS 2 graph.
        """
        super().__init__(node_name)

        # Gains and tolerances
        self.declare_parameter("k_rho", 0.4)
        self.declare_parameter("k_alpha", 0.8)
        self.declare_parameter("k_yaw", 0.8)
        self.declare_parameter("goal_tolerance", 0.10)
        self.declare_parameter("yaw_tolerance", 0.05)

        self._k_rho = self.get_parameter("k_rho").value
        self._k_alpha = self.get_parameter("k_alpha").value
        self._k_yaw = self.get_parameter("k_yaw").value
        self._tolerance = self.get_parameter("goal_tolerance").value
        self._yaw_tolerance = self.get_parameter("yaw_tolerance").value
        self._odom_received = False

        # pose of robot
        self._x = 0.0
        self._y = 0.0
        self._yaw = 0.0

        self._cb_group = ReentrantCallbackGroup()
        
        # Publishers
        self._cmd_pub = self.create_publisher(TwistStamped, "cmd_vel", 10)
         # Subscribers
        self._odom_sub = self.create_subscription(
            Odometry, "odometry/filtered", self._odom_callback, 10, callback_group=self._cb_group
        )

        self._action_server = ActionServer(
            self,
            NavigateToGoal,
            "navigate_to_goal",
            execute_callback=self._execute_callback,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=self._cb_group
        )
        self.get_logger().info(f"NavigateToGoal server ready. Gains: k_rho={self._k_rho}, k_alpha={self._k_alpha}, k_yaw={self._k_yaw}")
    
    def _stop_robot(self) -> None:
        """Stop the robot by publish zero to cmd_vel"""
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.twist.linear.x = 0.0
        cmd.twist.angular.z = 0.0
        self._cmd_pub.publish(cmd)


    def _odom_callback(self, msg: Odometry) -> None:
        """Update the robot's pose from an odometry message.

        Extracts the x/y position and yaw angle (converted from the
        orientation quaternion via scipy) and stores them for use in
        the control loop.

        Args:
            msg: Odometry message from ``odometry/filtered``.
        """
        
        self._x = msg.pose.pose.position.x
        self._y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self._yaw = R.from_quat([q.x, q.y, q.z, q.w]).as_euler("xyz")[2]
        
        if not self._odom_received:
            self._odom_received = True
            self.get_logger().info("First odometry received. Control loop active.")

    def _goal_callback(self, goal_request) -> GoalResponse:
        """Accept all incoming goal request.

        Args:
            goal_request: The incoming goal request.

        Returns:
            GoalResponse.ACCEPT or GoalResponse.REJECT.
        """
        goal_x = goal_request.x
        goal_y = goal_request.y
        goal_yaw = goal_request.final_heading
        self.get_logger().info(f"Goal accepted: ({goal_x:.2f}, {goal_y:.2f}, final_heading={goal_yaw:.2f})")        
        return GoalResponse.ACCEPT
    
    def _cancel_callback(self, goal_handle) -> CancelResponse:
        """Accept all cancel requests.

        Args:
            goal_handle: The goal handle for the cancel request.

        Returns:
            CancelResponse.ACCEPT.
        """
        self.get_logger().warn("Cancel requested. Stopping the robot and finalizing the goal.")
        return CancelResponse.ACCEPT
    
    def _execute_callback(self, goal_handle) -> NavigateToGoal.Result:
        """Get goal from client and drive until goal, 
        then rotate until correct direction and reutrn the result

        Args:
            goal_handle: The goal handle (x,y,final_heading).

        Returns:
            NavigateToGoal.Result
        """

        goal_x = goal_handle.request.x
        goal_y = goal_handle.request.y
        goal_yaw = goal_handle.request.final_heading
        total_dist = 0.0
        
        result = NavigateToGoal.Result()
        feedback_msg = NavigateToGoal.Feedback()

        start_time = time.time()
        rate = self.create_rate(20) # loop run 20 time per sec
        feedback_counter = 0


        rho = math.sqrt((goal_x - self._x)**2 + (goal_y - self._y)**2)
        prev_x = self._x
        prev_y = self._y

        # Phase 1 - Position loop to drive to goal position
        while rho > self._tolerance:

            # Check for cancellation before
            if goal_handle.is_cancel_requested:
                self._stop_robot()
                goal_handle.canceled()
                result.success = False
                result.total_distance = total_dist
                result.elapsed_time = float(time.time() - start_time)
                self.get_logger().info(f"Goal canceled: total_distance=2.{total_dist}, elapsed_time={result._elapsed_time}s.")                                         
                return result

            dx = goal_x - self._x
            dy = goal_y - self._y
            rho = math.sqrt(dx**2 + dy**2) # dist robot to goal
            angle_to_goal = math.atan2(dy, dx) #angle robot to goal
            alpha = math.atan2( # heading error (diff angle to goal and current yaw)
                math.sin(angle_to_goal - self._yaw),
                math.cos(angle_to_goal - self._yaw),
            )
            cmd = TwistStamped()
            cmd.header.stamp = self.get_clock().now().to_msg()
            cmd.twist.linear.x = max(
                -self.MAX_LINEAR, min(self.MAX_LINEAR, self._k_rho * rho)
            )
            cmd.twist.angular.z = max(
                -self.MAX_ANGULAR,
                min(self.MAX_ANGULAR, self._k_alpha * alpha),
            )
            self._cmd_pub.publish(cmd)
            # self.get_logger().info(
            #     f"[position] pose=({self._x:.2f}, {self._y:.2f}, "
            #     f"yaw={self._yaw:.2f}) rho={rho:.2f} alpha={alpha:.2f} "
            #     f"cmd=(v={cmd.twist.linear.x:.2f}, "
            #     f"w={cmd.twist.angular.z:.2f})",
            #     throttle_duration_sec=1.0,
            # )


            total_dist += math.sqrt((self._x - prev_x)**2 + (self._y - prev_y)**2)
            prev_x = self._x
            prev_y = self._y

            # publish feedback at 2 Hz (every 10 iterations at 20Hz)
            feedback_counter += 1
            if feedback_counter % 10 == 0:
                feedback_msg.current_x = self._x
                feedback_msg.current_y = self._y
                feedback_msg.distance_remaining = rho
                goal_handle.publish_feedback(feedback_msg)
            rate.sleep()


        yaw_error = math.atan2(
            math.sin(goal_yaw - self._yaw),
            math.cos(goal_yaw - self._yaw),)
        
        # Phase 2: rotate in place to reach the desired yaw
        while abs(yaw_error) > self._yaw_tolerance:
            yaw_error = math.atan2(
                math.sin(goal_yaw - self._yaw),
                math.cos(goal_yaw - self._yaw),
            )

            # Check for cancellation before
            if goal_handle.is_cancel_requested:
                self._stop_robot()
                goal_handle.canceled()
                result.success = False
                result.total_distance = total_dist
                result.elapsed_time = float(time.time() - start_time)
                self.get_logger().info(f"Goal canceled: total_distance=2.{total_dist}, elapsed_time={result._elapsed_time}s.")                                         
                return result

            # Publish only angular vel for rotation
            cmd = TwistStamped()
            cmd.header.stamp = self.get_clock().now().to_msg()                             
            cmd.twist.angular.z = max(
                -self.MAX_ANGULAR,
                min(self.MAX_ANGULAR, self._k_yaw * yaw_error)
                )
            self._cmd_pub.publish(cmd)

            # publish feedback at 2 Hz (every 10 iterations at 20Hz)
            feedback_counter += 1
            if feedback_counter % 10 == 0:
                feedback_msg.current_x = self._x
                feedback_msg.current_y = self._y
                feedback_msg.distance_remaining = 0.0
                goal_handle.publish_feedback(feedback_msg)
            rate.sleep()

        # Mark goal as succeeded
        self._stop_robot()
        goal_handle.succeed()
        self.get_logger().info(f"Goal reached: ({goal_x:.2f}, {goal_y:.2f}, final_heading={goal_yaw:.2f})")
        result.success = True
        result.total_distance = total_dist
        result.elapsed_time = float( time.time() - start_time)
        
        return result
    
