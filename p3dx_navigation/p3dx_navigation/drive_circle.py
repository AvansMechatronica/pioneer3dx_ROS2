#!/usr/bin/env python3
## Node to drive the TurtleBot in circles.

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class DriveCircle(Node):
    def __init__(self):
        super().__init__('drive_circle')
        # Publisher to the 'cmd_vel' topic
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer = self.create_timer(0.5, self.publish_velocity)  # Timer for 2Hz (0.5s interval)

        # Initialize Twist message
        self.move = Twist()
        self.move.linear.x = 0.2  # Linear velocity in the x-axis
        self.move.angular.z = 0.5  # Angular velocity in the z-axis
        self.get_logger().info('Driving TurtleBot in circles...')

    def publish_velocity(self):
        self.publisher_.publish(self.move)  # Publish velocity message
        self.get_logger().info(f'Published velocity: linear.x={self.move.linear.x}, angular.z={self.move.angular.z}')


def main(args=None):
    rclpy.init(args=args)
    node = DriveCircle()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Stopping the TurtleBot...')
    finally:
        # Stop the robot before shutting down
        stop_move = Twist()
        node.publisher_.publish(stop_move)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
