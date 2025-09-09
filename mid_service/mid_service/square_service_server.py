#!/usr/bin/env python3
import time
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty

class SquareServiceServer(Node):
    def __init__(self):
        super().__init__('square_service_server')

        # Publisher for velocity commands
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Service server
        self.srv = self.create_service(Empty, 'square_service', self.handle_square)

        self.get_logger().info("✅ Service '/square_service' ready.")

    def handle_square(self, request, response):
        # Parameters
        side_length = 0.5       # meters
        linear_speed = 0.1      # m/s
        angular_speed = 1.0     # rad/s

        # Time to move forward & turn
        forward_time = side_length / linear_speed
        turn_time = (math.pi/2) / angular_speed  # 90°

        twist = Twist()

        for i in range(4):
            # 1. Move forward
            twist.linear.x = linear_speed
            twist.angular.z = 0.0
            self._publish_for(forward_time, twist)

            # 2. Turn left 90°
            twist.linear.x = 0.0
            twist.angular.z = angular_speed
            self._publish_for(turn_time, twist)

        # Stop robot at the end
        self.stop_robot()
        self.get_logger().info("✅ Finished square path.")
        return response

    def _publish_for(self, duration, msg):
        """Publish a Twist message for a given duration"""
        start = time.time()
        while time.time() - start < duration:
            self.pub.publish(msg)
            time.sleep(0.1)

    def stop_robot(self):
        """Send zero velocity"""
        self.pub.publish(Twist())

def main(args=None):
    rclpy.init(args=args)
    node = SquareServiceServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
