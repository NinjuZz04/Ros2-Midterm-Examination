import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class CirclePublisher(Node):
    def __init__(self):
        super().__init__('circle_publisher')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Timer to publish velocity every 0.1 seconds
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        # Linear velocity and angular velocity for a circle (radius 0.5 meters)
        self.linear_speed = 0.2  # meters per second
        self.angular_speed = 0.4  # radians per second (approx. for a circle of radius ~0.5 meters)
        
        self.get_logger().info(f"Publishing velocity: Linear Speed = {self.linear_speed} m/s, Angular Speed = {self.angular_speed} rad/s")

    def timer_callback(self):
        msg = Twist()
        msg.linear.x = self.linear_speed
        msg.angular.z = self.angular_speed
        self.publisher_.publish(msg)

    def stop_robot(self):
        """Set velocity to 0"""
        stop_msg = Twist()
        stop_msg.linear.x = 0.0
        stop_msg.angular.z = 0.0
        self.publisher_.publish(stop_msg)
        self.get_logger().info("Node destroyed: Robot stopped.")

    def destroy_node(self):
        """Override destroy_node to stop robot first"""
        # Stop robot
        self.stop_robot()
        # Cancel timer to prevent callback after destroy
        if self.timer is not None:
            self.timer.cancel()
        # Call parent destroy
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = CirclePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()