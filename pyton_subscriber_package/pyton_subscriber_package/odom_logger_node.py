import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import math


class OdomLogger(Node):
    def __init__(self):
        super().__init__('odom_logger')
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        self.subscription  # prevent unused variable warning
        self.get_logger().info("OdomLogger started, listening to /odom")

    def odom_callback(self, msg):
        # Get position
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        # Get orientation quaternion
        q = msg.pose.pose.orientation
        qx, qy, qz, qw = q.x, q.y, q.z, q.w

        # Convert quaternion -> yaw (Euler)
        # yaw = atan2(2*(wz + xy), 1 - 2*(y^2 + z^2))
        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        self.get_logger().info(f"Position: x={x:.2f}, y={y:.2f}, yaw={math.degrees(yaw):.2f}°")


def main(args=None):
    rclpy.init(args=args)
    node = OdomLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info("Shutting down OdomLogger...")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
