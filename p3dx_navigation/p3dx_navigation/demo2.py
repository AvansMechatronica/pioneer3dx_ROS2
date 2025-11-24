#!/usr/bin/env python3

# Naam Student:
# Studentnummer:
# Datum:
# Verklaring: Door het inleveren van dit bestand verklaar ik dat ik deze opdracht zelfstandig heb uitgevoerd en 
# dat ik geen code van anderen heb gebruikt. Tevens ga ik akkoord met de beoordeling van deze opdracht.

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
import time


class MoveRobotNode(Node):
    def __init__(self):
        super().__init__('demo2')
        # Create an action client for the 'NavigateToPose' action
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.get_logger().info('Waiting for navigate_to_pose action server...')
        self._action_client.wait_for_server()
        self.get_logger().info('Action server available!')
        self.done = True

    def send_goal(self, x, y):
        """Send a goal to the action server."""
        self.goal_msg = NavigateToPose.Goal()
        self.goal_msg.pose.header.frame_id = 'map'
        self.goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        self.goal_msg.pose.pose.position.x = x
        self.goal_msg.pose.pose.position.y = y
        self.goal_msg.pose.pose.position.z = 0.0
        self.goal_msg.pose.pose.orientation.x = 0.0
        self.goal_msg.pose.pose.orientation.y = 0.0
        self.goal_msg.pose.pose.orientation.z = 0.0
        self.goal_msg.pose.pose.orientation.w = 1.0 # Default facing forward

        self.get_logger().info(f"Sending goal to position (x={self.goal_msg.pose.pose.position.x}, y={self.goal_msg.pose.pose.position.y})")

        #self._action_client.send_goal_async(self.goal_msg, feedback_callback=self._feedback_callback).add_done_callback(self._goal_response_callback)
        #self._action_client.send_goal(self.goal_msg)
        goal_future = self._action_client.send_goal_async(self.goal_msg, feedback_callback=self._feedback_callback)
        goal_future.add_done_callback(self._goal_response_callback)

        self.done = False
        while not self.done:
            time.sleep(0.2)
            rclpy.spin_once(self)


    def _goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal was rejected by the action server')
            return

        self.get_logger().info('Goal accepted, waiting for result...')
        goal_handle.get_result_async().add_done_callback(self._result_callback)

    def _feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f"Feedback received: {feedback.current_pose.pose.position}")

    def _result_callback(self, future):
        result = future.result().result
        if result is None:
            self.get_logger().error('Goal execution failed.')
        else:
            self.get_logger().info('Goal reached successfully!')
        self.done = True

def main(args=None):
    rclpy.init(args=args)
    node = MoveRobotNode()

    # Define multiple goals
    goals = [
        (0.5, 1.0),  # First goal (x, y) (0.5, 1.0)
        (-7.8, -1.5),  # Second goal (x, y)
    ]

    # Send goals sequentially
    for x, y in goals:
        node.send_goal(x, y)
    
    # Shutdown after achieving all goals
    rclpy.shutdown()

if __name__ == '__main__':
    main()
