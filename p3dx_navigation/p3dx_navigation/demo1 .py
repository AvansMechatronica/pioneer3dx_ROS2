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


class MoveRobotNode(Node):
    def __init__(self):
        super().__init__('demo1')

        # Create an action client for the 'NavigateToPose' action
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.get_logger().info('Waiting for navigate_to_pose action server...')
        self._action_client.wait_for_server()
        self.get_logger().info('Action server available!')

        # Create a goal
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = 0.13
        goal_msg.pose.pose.position.y = 1.44
        goal_msg.pose.pose.position.z = 0.0
        goal_msg.pose.pose.orientation.x = 0.0
        goal_msg.pose.pose.orientation.y = 0.0
        goal_msg.pose.pose.orientation.z = 0.0
        goal_msg.pose.pose.orientation.w = 1.0

        self.get_logger().info(f"Sending goal to position (x={goal_msg.pose.pose.position.x}, y={goal_msg.pose.pose.position.y})")
        self._send_goal(goal_msg)

    def _send_goal(self, goal_msg):
        # Send the goal to the action server
        self._action_client.send_goal_async(goal_msg, feedback_callback=self._feedback_callback).add_done_callback(self._goal_response_callback)

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
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = MoveRobotNode()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
