#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty

class SquareServiceClient(Node):
    def __init__(self):
        super().__init__('square_service_client')
        self.cli = self.create_client(Empty, 'square_service')

        # Wait until the service is available
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')

        # Create an empty request
        self.req = Empty.Request()

        # Call the service
        self.future = self.cli.call_async(self.req)
        self.future.add_done_callback(self.callback)

    def callback(self, future):
        try:
            future.result()
            self.get_logger().info('✅ Service call successful: robot is moving in a square')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = SquareServiceClient()
    rclpy.spin(node)  # keeps node alive until callback completes
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
