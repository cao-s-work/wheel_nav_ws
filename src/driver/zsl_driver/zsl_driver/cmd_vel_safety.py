"""
cmd_vel_safety.py — 最终安全层。

输入:  /cmd_vel_selected  (来自 cmd_vel_mux)
输出:  /cmd_vel_safe       (到 zsl_driver)

功能:
  1. estop 锁存 — 立即零速，不经过 ramp
  2. watchdog 超时 → 零速
  3. 限速 clamp（前进/后退/横移/角速度分开）
  4. 加速度 ramp（正常加减速）
  5. 没输入时持续发布零速
"""
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist


def _clamp(value: float, minimum: float, maximum: float) -> float:
    return max(minimum, min(maximum, value))


def _approach(current: float, target: float, max_delta: float) -> float:
    if target > current:
        return min(current + max_delta, target)
    return max(current - max_delta, target)


class CmdVelSafety(Node):
    def __init__(self):
        super().__init__("cmd_vel_safety")

        # 发布频率
        publish_rate = self.declare_parameter("publish_rate", 50.0).value

        # watchdog
        self._watchdog_timeout = self.declare_parameter("watchdog_timeout", 0.5).value

        # 限速（前进/后退/横移分开）
        self._max_linear_vel = self.declare_parameter("max_linear_vel", 0.30).value
        self._max_reverse_vel = self.declare_parameter("max_reverse_vel", 0.15).value
        self._max_lateral_vel = self.declare_parameter("max_lateral_vel", 0.0).value
        self._max_angular_vel = self.declare_parameter("max_angular_vel", 0.50).value

        # 加速度
        self._max_linear_accel = self.declare_parameter("max_linear_accel", 0.30).value
        self._max_lateral_accel = self.declare_parameter("max_lateral_accel", 0.0).value
        self._max_angular_accel = self.declare_parameter("max_angular_accel", 0.50).value

        # estop 锁存
        self._estop_latched = False

        # 状态
        self._last_cmd = Twist()
        self._last_cmd_time = 0.0
        self._current_cmd = Twist()
        self._last_tick_time = 0.0

        # 订阅
        self._sub = self.create_subscription(
            Twist, "cmd_vel_selected", self._cmd_cb, 10
        )
        self._estop_sub = self.create_subscription(
            Bool, "~/estop", self._estop_cb, 10
        )

        # 发布
        self._safe_pub = self.create_publisher(Twist, "cmd_vel_safe", 10)

        # 定时器
        self._timer = self.create_timer(1.0 / publish_rate, self._tick)

        self.get_logger().info(
            f"CmdVelSafety ready "
            f"(vel=({self._max_reverse_vel}/{self._max_linear_vel}/{self._max_lateral_vel}/{self._max_angular_vel}), "
            f"accel=({self._max_linear_accel}/{self._max_lateral_accel}/{self._max_angular_accel}), "
            f"watchdog={self._watchdog_timeout}s)"
        )

    def _cmd_cb(self, msg: Twist):
        self._last_cmd = msg
        self._last_cmd_time = time.monotonic()

    def _estop_cb(self, msg: Bool):
        """急停锁存：收到 True 即锁死，不能自动解除。"""
        if msg.data:
            self._estop_latched = True
            self._current_cmd = Twist()
            self.get_logger().warn("ESTOP latched!")

    def _tick(self):
        now = time.monotonic()

        # 计算 dt，上限防止长时间未 tick 后的一次大跳变
        if self._last_tick_time <= 0.0:
            dt = 0.02
        else:
            dt = _clamp(now - self._last_tick_time, 0.0, 0.1)
        self._last_tick_time = now

        # --- 1. estop: 立即零速，不经过加速度限制 ---
        if self._estop_latched:
            self._current_cmd = Twist()
            self._safe_pub.publish(self._current_cmd)
            return

        # --- 2. watchdog 超时: 目标速度归零 ---
        stale = (
            self._last_cmd_time <= 0.0
            or now - self._last_cmd_time > self._watchdog_timeout
        )
        if stale:
            target = Twist()
        else:
            target = Twist()
            target.linear.x = self._last_cmd.linear.x
            target.linear.y = self._last_cmd.linear.y
            target.angular.z = self._last_cmd.angular.z

        # --- 3. 限速 clamp ---
        target.linear.x = _clamp(
            target.linear.x,
            -self._max_reverse_vel,
            self._max_linear_vel,
        )
        target.linear.y = _clamp(
            target.linear.y,
            -self._max_lateral_vel,
            self._max_lateral_vel,
        )
        target.angular.z = _clamp(
            target.angular.z,
            -self._max_angular_vel,
            self._max_angular_vel,
        )

        # --- 4. 加速度 ramp ---
        self._current_cmd.linear.x = _approach(
            self._current_cmd.linear.x,
            target.linear.x,
            self._max_linear_accel * dt,
        )
        self._current_cmd.linear.y = _approach(
            self._current_cmd.linear.y,
            target.linear.y,
            self._max_lateral_accel * dt,
        )
        self._current_cmd.angular.z = _approach(
            self._current_cmd.angular.z,
            target.angular.z,
            self._max_angular_accel * dt,
        )

        # --- 5. 发布 ---
        self._safe_pub.publish(self._current_cmd)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelSafety()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
