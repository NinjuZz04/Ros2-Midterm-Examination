#!/usr/bin/env python3
import math
import sys

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from mid_action.action import Rotate


class RotateClient(Node):
    """
    Simple Rotate action client node.
    - Sends a rotation goal (angle in radians) to the action server.
    - Prints feedback as 'remaining_angle' (how much more to rotate).
    - Prints final result (success or aborted).
    """

    def __init__(self) -> None:
        # Initialize the node with the name "rotate_action_client"
        super().__init__('rotate_action_client')

        # Create an ActionClient:
        #   - self: reference to this node
        #   - Rotate: the action type (defined in mid_action/action/Rotate.action)
        #   - 'rotate': action name (must match the server)
        self.client: ActionClient = ActionClient(self, Rotate, 'rotate')

        # Will hold the future result of the goal (set later in callbacks)
        self._result_future = None

    # ------------------ Public method ------------------

    def send_goal_deg(self, angle_deg: float) -> None:
        """
        Send a rotation goal to the action server.
        Input angle is in degrees → converted to radians for the server.
        """
        # Convert degrees → radians
        angle_rad: float = math.radians(angle_deg)

        # Wait until the action server is ready to receive goals
        self.get_logger().info('Waiting for action server...')
        self.client.wait_for_server()

        # Create the goal message
        goal = Rotate.Goal()
        goal.angle = float(angle_rad)  # explicit cast for safety

        # Log the goal in both degrees and radians
        self.get_logger().info(f'Sending goal: {angle_deg:.2f} deg ({angle_rad:.3f} rad)')

        # Send the goal asynchronously, attach feedback callback
        goal_future = self.client.send_goal_async(goal, feedback_callback=self.feedback_cb)

        # When the server responds, trigger goal_response_cb
        goal_future.add_done_callback(self.goal_response_cb)

    # ------------------ Callbacks ------------------

    def goal_response_cb(self, future) -> None:
        """
        Called when the action server responds to our goal request.
        Checks if the goal was accepted or rejected.
        """
        goal_handle = future.result()
        if not goal_handle or not goal_handle.accepted:
            self.get_logger().error('Goal rejected ❌')
            rclpy.shutdown()
            return

        self.get_logger().info('Goal accepted ✅')

        # Ask for the result asynchronously, trigger result_cb later
        self._result_future = goal_handle.get_result_async()
        self._result_future.add_done_callback(self.result_cb)

    def feedback_cb(self, fb_msg) -> None:
        """
        Called whenever the server sends feedback.
        Feedback includes 'remaining_angle' (radians).
        We print it in degrees for readability.
        """
        remaining_rad: float = fb_msg.feedback.remaining_angle
        remaining_deg: float = math.degrees(remaining_rad)
        self.get_logger().info(f'Feedback: remaining {remaining_deg:.1f}°')

    def result_cb(self, future) -> None:
        """
        Called when the server finishes the goal (success or failure).
        """
        try:
            result = future.result().result
            if result.success:
                self.get_logger().info('Result: Goal reached successfully 🎉')
            else:
                self.get_logger().warning('Result: Goal aborted ⚠️')
        except Exception as e:
            self.get_logger().error(f'Failed to get result: {e}')
        finally:
            # Always shut down the node when finished
            rclpy.shutdown()


# ------------------ Main entry ------------------

def main(args=None) -> None:
    # Initialize ROS 2 Python client library
    rclpy.init(args=args)

    # Default angle = 180° if user does not provide argument
    try:
        deg = float(sys.argv[1]) if len(sys.argv) > 1 else 180.0
    except ValueError:
        print('❌ Invalid angle. Please pass a number (degrees). Example: `ros2 run <pkg> rotate_action_client 90`')
        rclpy.shutdown()
        return

    # Create the client node
    node = RotateClient()

    # Send the rotation goal (in degrees)
    node.send_goal_deg(deg)

    # Keep the node alive until goal finishes
    rclpy.spin(node)


if __name__ == '__main__':
    main()
