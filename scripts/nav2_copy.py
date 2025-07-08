#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, PoseStamped
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import Bool
from rclpy.action import ActionClient
import math
import tf2_ros
import time  # Used for timeout checking

class Nav2GoalSender(Node):
    def __init__(self):
        super().__init__('nav2_goal_sender')
        
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.subscription = self.create_subscription(PoseStamped, '/exploration_goal', self.obstacle_callback, 10)
        self.done_publisher = self.create_publisher(Bool, '/goal_done', 10)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.goal_active = False
        self.goal_handle = None
        self.goal_start_time = None

        self.get_logger().info('Waiting for Nav2 action server...')
        self._action_client.wait_for_server()
        self.get_logger().info('Nav2 action server connected!')

    def obstacle_callback(self, msg):
        """
        Receive new goal from /exploration_goal.
        """
        goal_x = msg.pose.position.x
        goal_y = msg.pose.position.y
        robot_x, robot_y = self.get_robot_position()
        distance = self.calculate_distance((robot_x, robot_y), (goal_x, goal_y))

        if distance < 0.1:
            self.get_logger().info(f"Goal too close. Skipping.")
            return

        if self.goal_active:
            time_elapsed = time.time() - self.goal_start_time
            if time_elapsed > 10:
                self.get_logger().warn(f"Goal stuck. Canceling after {time_elapsed:.2f} seconds.")
                self.cancel_goal()
            else:
                return

        self.get_logger().info(f"Sending goal to ({goal_x}, {goal_y})")
        self.send_goal(goal_x, goal_y)

    def send_goal(self, x, y):
        """
        Send a goal to Nav2's action server.
        """
        if self.goal_active:
            return

        self.goal_active = True
        self.goal_start_time = time.time()

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.w = 1.0

        self._goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self._feedback_callback)
        self._goal_future.add_done_callback(self._goal_response_callback)

    def _goal_response_callback(self, future):
        """
        Handle goal acceptance or rejection.
        """
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            self.goal_active = False
            return

        self.get_logger().info('Goal accepted')
        self._result_future = self.goal_handle.get_result_async()
        self._result_future.add_done_callback(self._result_callback)

    def _result_callback(self, future):
        """
        Handle the goal result.
        """
        result = future.result().result
        if result:
            self.get_logger().info('Goal reached!')
            self.done_publisher.publish(Bool(data=True))  # Notify exploration node
        else:
            self.get_logger().info('Failed to reach goal')

        self.goal_active = False

    def cancel_goal(self):
        """
        Cancel the current goal if it is stuck.
        """
        if self.goal_handle:
            self.get_logger().warn("Canceling goal due to timeout.")
            cancel_future = self.goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(self._cancel_callback)
            self.goal_active = False

    def _cancel_callback(self, future):
        """
        Handle cancellation callback.
        """
        self.get_logger().info("Goal canceled. Ready for new goal.")

    def calculate_distance(self, p1, p2):
        """
        Calculate Euclidean distance between two points.
        """
        return math.sqrt((p2[0] - p1[0]) ** 2 + (p2[1] - p1[1]) ** 2)

    def get_robot_position(self):
        """
        Get the robot's current position from TF.
        """
        try:
            transform = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            return transform.transform.translation.x, transform.transform.translation.y
        except tf2_ros.LookupException:
            return 0.0, 0.0

    def _feedback_callback(self, feedback_msg):
        """Receive feedback from the action server"""
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Distance remaining: {feedback.distance_remaining:.2f} meters')


def main(args=None):
    rclpy.init(args=args)
    nav2_goal_sender = Nav2GoalSender()

    try:
        rclpy.spin(nav2_goal_sender)
    except KeyboardInterrupt:
        pass

    nav2_goal_sender.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
