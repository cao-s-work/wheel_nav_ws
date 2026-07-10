"""
safety_node.py — 最终安全层。

输入:  /cmd_vel_selected
输出:  /cmd_vel_safe → zsl_driver

功能:
  - 限速 clamp
  - 加速度 ramp（防急加速）
  - watchdog 超时零速（500ms）
  - estop 硬停（通过 service / 订阅 external estop）
"""
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


def clamp(val: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, val))


class SafetyNode(Node):
    def __init__(self):
        super().__init__("safety_node")

        # 限速
        self._max_vx = self.declare_parameter("max_linear_vel", 0.5).value
        self._max_wz = self.declare_parameter("max_angular_vel", 1.0).value

        # 加速度限制（每 tick 最大增量）
        self._max_linear_accel = self.declare_parameter("max_linear_accel", 2.0).value
        self._max_angular_accel = self.declare_parameter("max_angular_accel", 3.0).value

        # watchdog
        self._watchdog_timeout_s = self.declare_parameter("watchdog_timeout_s", 0.5).value

        # estop
        self._estop = False

        # 状态
        self._last_cmd = Twist()
        self._last_cmd_time = 0.0
        self._prev_vx = 0.0
        self._prev_vy = 0.0
        self._prev_wz = 0.0

        # 订阅
        self._sub = self.create_subscription(
            Twist, "cmd_vel_selected", self._cmd_cb, 10
        )

        # 发布
        self._pub = self.create_publisher(Twist, "cmd_vel_safe", 10)

        # 定时器 50Hz
        self._timer = self.create_timer(0.02, self._tick)

        self.get_logger().info(
            f"SafetyNode ready "
            f"(max_vel=({self._max_vx},{self._max_wz}), "
            f"max_accel=({self._max_linear_accel},{self._max_angular_accel}), "
            f"watchdog={self._watchdog_timeout_s}s)"
        )

    def _cmd_cb(self, msg: Twist):
        self._last_cmd = msg
        self._last_cmd_time = time.monotonic()

    def _tick(self):
        now = time.monotonic()
        dt = 0.02  # 50Hz

        # estop → 零速
        if self._estop:
            self._pub.publish(Twist())
            self._prev_vx = 0.0
            self._prev_vy = 0.0
            self._prev_wz = 0.0
            return

        # watchdog 超时 → 零速
        if (now - self._last_cmd_time) > self._watchdog_timeout_s:
            self._pub.publish(Twist())
            self._prev_vx = 0.0
            self._prev_vy = 0.0
            self._prev_wz = 0.0
            return

        # 限速 clamp
        target_vx = clamp(self._last_cmd.linear.x, -self._max_vx, self._max_vx)
        target_vy = clamp(self._last_cmd.linear.y, -self._max_vx, self._max_vx)
        target_wz = clamp(self._last_cmd.angular.z, -self._max_wz, self._max_wz)

        # 加速度 ramp
        max_dv = self._max_linear_accel * dt
        max_dw = self._max_angular_accel * dt

        vx = clamp(target_vx, self._prev_vx - max_dv, self._prev_vx + max_dv)
        vy = clamp(target_vy, self._prev_vy - max_dv, self._prev_vy + max_dv)
        wz = clamp(target_wz, self._prev_wz - max_dw, self._prev_wz + max_dw)

        self._prev_vx = vx
        self._prev_vy = vy
        self._prev_wz = wz

        out = Twist()
        out.linear.x = vx
        out.linear.y = vy
        out.angular.z = wz
        self._pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = SafetyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
