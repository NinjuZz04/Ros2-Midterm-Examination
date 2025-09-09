import math
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from mid_action.action import Rotate


def normalize_angle(a: float) -> float:
    """Wrap angle to [-pi, pi)."""
    a = (a + math.pi) % (2.0 * math.pi)
    if a < 0:
        a += 2.0 * math.pi
    return a - math.pi


class RotateActionServer(Node):
    def __init__(self) -> None:
        super().__init__('rotate_action_server')

        # -------- Parameters --------
        self.declare_parameter('kp', 0.5)                # proportional gain
        self.declare_parameter('max_w', 1.2)             # max |angular z|
        self.declare_parameter('min_w', 0.15)            # min |angular z|
        self.declare_parameter('tolerance_deg', 5.0)     # stop window (deg)
        self.declare_parameter('odom_topic', '/odom')    # odom topic name

        # Pre-fetch parameter values to avoid repeated lookups
        self._kp: float = float(self.get_parameter('kp').value)
        self._max_w: float = float(self.get_parameter('max_w').value)
        self._min_w: float = float(self.get_parameter('min_w').value)
        self._tol: float = math.radians(float(self.get_parameter('tolerance_deg').value))
        self._odom_topic: str = str(self.get_parameter('odom_topic').value)

        # -------- Publishers / Subscribers --------
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self._x: float = 0.0
        self._y: float = 0.0
        self._yaw: float = 0.0
        self._have_odom: bool = False

        self.create_subscription(
            Odometry, self._odom_topic, self._odom_cb, 25
        )

        # -------- Action Server --------
        self._server = ActionServer(
            self,
            Rotate,
            'rotate',
            execute_callback=self.execute_cb,
            goal_callback=self.goal_cb,
            cancel_callback=self.cancel_cb,
        )

        self.get_logger().info('rotate_action_server ready')

    # ---------------- Callbacks ----------------

    def _odom_cb(self, msg: Odometry) -> None:
        """Cache x, y, yaw from /odom."""
        self._x = msg.pose.pose.position.x
        self._y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self._yaw = math.atan2(siny_cosp, cosy_cosp)

        self._have_odom = True

    def goal_cb(self, goal_request: Rotate.Goal) -> GoalResponse:
        self.get_logger().info(f'Goal received: angle={goal_request.angle:.3f} rad')
        return GoalResponse.ACCEPT

    def cancel_cb(self, _goal_handle) -> CancelResponse:
        self.get_logger().info('Cancel requested')
        return CancelResponse.ACCEPT

    def execute_cb(self, goal_handle) -> Rotate.Result:
        """Run a simple P-controller on yaw until target reached stably."""
        # Wait for first odom
        while rclpy.ok() and not self._have_odom:
            rclpy.spin_once(self, timeout_sec=0.1)

        # Read goal and compute target yaw
        target_delta: float = float(goal_handle.request.angle)   # radians
        start_yaw: float = self._yaw
        target_yaw: float = normalize_angle(start_yaw + target_delta)

        # Control loop settings
        fb = Rotate.Feedback()
        cmd = Twist()
        dt: float = 0.1
        stable_hits: int = 0
        needed_hits: int = 5

        while rclpy.ok():
            # Handle cancel
            if goal_handle.is_cancel_requested:
                self._stop()
                goal_handle.canceled()
                res = Rotate.Result()
                res.success = False
                self.get_logger().info('Goal canceled')
                return res

            rclpy.spin_once(self, timeout_sec=0.0)

            # Error (remaining angle) and feedback
            remain: float = normalize_angle(target_yaw - self._yaw)
            fb.remaining_angle = float(remain)
            goal_handle.publish_feedback(fb)

            self.get_logger().info(
                f'odom: x={self._x:.3f}, y={self._y:.3f}, yaw={math.degrees(self._yaw):.1f}° '
                f'(remain={math.degrees(remain):.1f}°)'
            )

            # Stable-in-tolerance check
            if abs(remain) <= self._tol:
                stable_hits += 1
                if stable_hits >= needed_hits:
                    break
            else:
                stable_hits = 0

            # P-control on angular velocity with deadband/min speed and clamp
            w: float = self._kp * remain
            w = max(-self._max_w, min(self._max_w, w))
            if abs(w) < self._min_w:
                w = math.copysign(self._min_w, w)

            cmd.linear.x = 0.0
            cmd.angular.z = float(w)
            self.cmd_pub.publish(cmd)

            # Fixed-rate loop using time.sleep (simple and good enough here)
            time.sleep(dt)

        # Success
        self._stop()
        goal_handle.succeed()
        res = Rotate.Result()
        res.success = True
        self.get_logger().info('Goal reached successfully')
        return res

    def _stop(self) -> None:
        """Publish zero cmd to stop the robot."""
        self.cmd_pub.publish(Twist())


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RotateActionServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
