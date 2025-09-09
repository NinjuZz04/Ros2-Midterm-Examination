import math
import sys

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from mid_action.action import Rotate



class RotateClient(Node):
    """
    Simple Rotate action client.
    - Goal: Rotate.Goal.angle (radians)
    - Feedback: remaining_angle (radians)
    - Result: success (bool)
    """

    def __init__(self) -> None:
        super().__init__('rotate_action_client')
        self.client: ActionClient = ActionClient(self, Rotate, 'rotate')
        self._result_future = None  # set later after goal accepted

    def send_goal_deg(self, angle_deg: float) -> None:
        """Send rotate goal using degree input; converts to radians internally."""
        angle_rad: float = math.radians(angle_deg)

        self.get_logger().info('Waiting for action server...')
        self.client.wait_for_server()

        goal = Rotate.Goal()
        goal.angle = float(angle_rad)  # explicit cast for safety

        self.get_logger().info(f'Sending goal: {angle_deg:.2f} deg ({angle_rad:.3f} rad)')
        goal_future = self.client.send_goal_async(goal, feedback_callback=self.feedback_cb)
        goal_future.add_done_callback(self.goal_response_cb)

    # ----- Callbacks -----

    def goal_response_cb(self, future) -> None:
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected')
            rclpy.shutdown()
            return

        self.get_logger().info('Goal accepted')
        self._result_future = goal_handle.get_result_async()
        self._result_future.add_done_callback(self.result_cb)

    def feedback_cb(self, fb_msg) -> None:
        remaining_rad: float = fb_msg.feedback.remaining_angle
        remaining_deg: float = math.degrees(remaining_rad)
        self.get_logger().info(f'Feedback: remaining {remaining_deg:.1f} deg')

    def result_cb(self, future) -> None:
        result = future.result().result
        if result.success:
            self.get_logger().info('Result: Goal reached successfully')
        else:
            self.get_logger().warning('Result: Goal aborted')
        rclpy.shutdown()


def main(args=None) -> None:
    rclpy.init(args=args)

    # default = 180 deg if no CLI argument
    try:
        deg = float(sys.argv[1]) if len(sys.argv) > 1 else 180.0
    except ValueError:
        print('Invalid angle. Please pass a number (degrees). Example: `ros2 run ... 90`')
        rclpy.shutdown()
        return

    node = RotateClient()
    node.send_goal_deg(deg)
    rclpy.spin(node)


if __name__ == '__main__':
    main()

