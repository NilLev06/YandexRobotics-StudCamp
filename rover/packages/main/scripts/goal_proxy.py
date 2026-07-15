#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String, Float32MultiArray
# from std_srvs.srv import Empty
from slam_toolbox.srv import Reset


class GoalProxy(Node):
    def __init__(self):
        super().__init__("goal_proxy")

        # Action client for Nav2"s navigate_to_pose
        self.action_client = ActionClient(
            self,
            NavigateToPose,
            "navigate_to_pose"
        )

        # Subscriber for incoming goals
        self.goal_sub = self.create_subscription(
            PoseStamped,
            "/goal",
            self.goal_callback,
            10
        )

        # Publisher for status updates
        self.status_pub = self.create_publisher(
            String,
            "/goal/status",
            10
        )
        self.status_pub2 = self.create_publisher(
            Float32MultiArray,
            "/goal/feedback",
            10
        )

        # Service to cancel the current goal
        self.cancel_srv = self.create_service(
            Reset,
            "/goal/cancel",
            self.cancel_callback,
            callback_group=MutuallyExclusiveCallbackGroup()
        )

        # State variables
        self.current_goal_handle = None   # active goal handle
        self.pending_goal = None          # new goal while active/cancelling
        self.goal_future = None           # future from send_goal_async
        self.result_future = None         # future from get_result_async
        self.get_logger().info("Goal proxy started")

    def goal_callback(self, msg: PoseStamped):
        """Called when a new goal is received on /goal."""
        # If there is an active goal, cancel it and store the new one
        if self.current_goal_handle is not None:
            self.get_logger().info("New goal received, cancelling current goal...")
            self.pending_goal = msg
            self.cancel_current_goal()
        else:
            # No active goal, send immediately
            self.send_goal(msg)

    def cancel_current_goal(self):
        """Cancel the currently active goal (if any)."""
        if self.current_goal_handle is not None:
            cancel_future = self.current_goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(self.cancel_done_callback)
        else:
            # No active goal, but maybe there is a pending one
            if self.pending_goal:
                self.send_goal(self.pending_goal)
                self.pending_goal = None

    def cancel_done_callback(self, future):
        """Called after cancellation of the current goal finishes."""
        # After cancellation, send any pending goal
        if self.pending_goal:
            self.send_goal(self.pending_goal)
            self.pending_goal = None

    def send_goal(self, goal_msg: PoseStamped):
        """Send a goal to the navigate_to_pose action server."""
        # Ensure the action server is ready
        if not self.action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().error("Action server not available!")
            self.publish_status("ACTION CALL ERROR")
            return

        # Create action goal
        action_goal = NavigateToPose.Goal()
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        action_goal.pose = goal_msg


        # Send goal with feedback callback
        self.goal_future = self.action_client.send_goal_async(
            action_goal,
            feedback_callback=self.feedback_callback
        )
        self.goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """Callback when the action server responds to the goal request."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info("Goal rejected")
            self.publish_status("GOAL REJECTED")
            self.current_goal_handle = None
            return

        self.get_logger().info("Goal accepted")
        # self.publish_status("Goal accepted")
        self.current_goal_handle = goal_handle

        # Wait for the final result
        self.result_future = goal_handle.get_result_async()
        self.result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        """Callback when the action finishes (success, aborted, cancelled)."""
        result = future.result()
        status = result.status
        self.current_goal_handle = None

        # Publish final status
        if status == 4:          # GoalStatus.STATUS_SUCCEEDED
            self.publish_status("GOAL SUCCESS")
        elif status == 5:        # GoalStatus.STATUS_CANCELED
            self.publish_status("GOAL CANCELLED")
        elif status == 6:        # GoalStatus.STATUS_ABORTED
            self.publish_status("GOAL ABORTED")
        else:
            self.publish_status(f"GOAL STATUS: {status}")

        # If there is a pending goal, send it now
        if self.pending_goal:
            self.send_goal(self.pending_goal)
            self.pending_goal = None

    def feedback_callback(self, feedback_msg):
        """Called whenever the action server sends feedback."""
        feedback = feedback_msg.feedback
        # Convert Duration to seconds (float)
        nav_time = feedback.navigation_time.sec + feedback.navigation_time.nanosec * 1e-9
        status_str = (
            f'distance: {feedback.distance_remaining:.2f} m, '
            f'time: {nav_time:.2f} s, '
            f'retries: {feedback.number_of_recoveries}'
        )
        self.publish_status(status_str)
        data_msg = Float32MultiArray()
        data_msg.data = [
            float(feedback.distance_remaining),
            float(nav_time),
            float(feedback.number_of_recoveries)
        ]
        self.status_pub2.publish(data_msg)

    def publish_status(self, status_text: str):
        """Publish a status update on /goal/status."""
        msg = String()
        msg.data = status_text
        self.status_pub.publish(msg)
        # self.get_logger().info(f"Status: {status_text}")

    def cancel_callback(self, request, response):
        """Service callback for /goal/cancel. Cancels current goal if active."""
        if self.current_goal_handle is not None:
            self.get_logger().info("Cancel service called, cancelling goal")
            cancel_future = self.current_goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(self.cancel_done_callback)
        else:
            self.get_logger().info("Cancel service called but no active goal")
        return response


def main(args=None):
    rclpy.init(args=args)
    node = GoalProxy()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
