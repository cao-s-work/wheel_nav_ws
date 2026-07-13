"""
cmd_vel_safety.py — 最终安全层。

输入:  /cmd_vel_selected  (来自 cmd_vel_mux)
输出:  /cmd_vel_safe       (到 zsl_driver)

功能:
  1. estop/estop_latched — 立即零速，不经过 ramp
  2. read_only / SDK 连接状态 — 零速
  3. watchdog 输入超时 → 零速
  4. 限速 clamp
  5. 加速度 ramp
  6. 没输入时持续发布零速
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

        # watchdog 输入超时
        self._input_timeout = self.declare_parameter("input_timeout_s", 0.30).value

        # 限速
        self._max_vx = self.declare_parameter("max_vx", 0.30).value
        self._min_vx = self.declare_parameter("min_vx", -0.15).value
        self._max_vy = self.declare_parameter("max_vy", 0.0).value
        self._max_wz = self.declare_parameter("max_wz", 0.50).value

        # 加速度
        self._max_ax = self.declare_parameter("max_ax", 0.30).value
        self._max_ay = self.declare_parameter("max_ay", 0.0).value
        self._max_aw = self.declare_parameter("max_aw", 0.50).value

        # 状态
        self._estop_latched = False      # zsl_driver 锁存（唯一急停源）
        self._read_only = True           # SDK 保护 / 未连接
        self._last_cmd = Twist()
        self._last_cmd_time = 0.0
        self._current_output = Twist()
        self._last_tick_time = 0.0

        # 订阅: 速度输入
        self._sub = self.create_subscription(
            Twist, "cmd_vel_selected", self._cmd_cb, 10
        )

        # 订阅: 安全状态（来自 zsl_driver）
        self._estop_latched_sub = self.create_subscription(
            Bool, "~/estop_latched", self._estop_latched_cb, 10
        )
        self._read_only_sub = self.create_subscription(
            Bool, "~/read_only", self._read_only_cb, 10
        )

        # 发布
        self._safe_pub = self.create_publisher(Twist, "cmd_vel_safe", 10)

        # 定时器
        self._timer = self.create_timer(1.0 / publish_rate, self._tick)

        self.get_logger().info(
            f"CmdVelSafety ready "
            f"vel=(vx: {self._min_vx}~{self._max_vx}, vy: {self._max_vy}, wz: {self._max_wz}) "
            f"accel=(ax: {self._max_ax}, ay: {self._max_ay}, aw: {self._max_aw}) "
            f"input_timeout={self._input_timeout}s"
        )

    def _cmd_cb(self, msg: Twist):
        self._last_cmd = msg
        self._last_cmd_time = time.monotonic()

    def _estop_latched_cb(self, msg: Bool):
        """来自 zsl_driver 的锁存急停。双向处理，可恢复。"""
        previous = self._estop_latched
        self._estop_latched = bool(msg.data)
        if self._estop_latched:
            self._current_output = Twist()
            self.get_logger().error("ESTOP latched (driver)")
        elif previous:
            self.get_logger().info("ESTOP latch cleared; read_only still applies")

    def _read_only_cb(self, msg: Bool):
        self._read_only = msg.data
        if self._read_only:
            self._current_output = Twist()

    def _tick(self):
        now = time.monotonic()

        # 计算 dt，上限防止长时间未 tick 后的一次大跳变
        if self._last_tick_time <= 0.0:
            dt = 0.02
        else:
            dt = _clamp(now - self._last_tick_time, 0.0, 0.1)
        self._last_tick_time = now

        # --- 1. 安全闸: estop_latched / read_only → 立即零速 ---
        if self._estop_latched or self._read_only:
            self._current_output = Twist()
            self._safe_pub.publish(self._current_output)
            return

        # --- 2. watchdog 输入超时 → 立即零速（跳过 ramp）---
        stale = (
            self._last_cmd_time <= 0.0
            or now - self._last_cmd_time > self._input_timeout
        )
        if stale:
            self._current_output = Twist()
            self._safe_pub.publish(self._current_output)
            return

        # --- 3. 限速 clamp ---
        target = Twist()
        target.linear.x = self._last_cmd.linear.x
        target.linear.y = self._last_cmd.linear.y
        target.angular.z = self._last_cmd.angular.z
        target.linear.x = _clamp(target.linear.x, self._min_vx, self._max_vx)
        target.linear.y = _clamp(target.linear.y, -self._max_vy, self._max_vy)
        target.angular.z = _clamp(target.angular.z, -self._max_wz, self._max_wz)

        # --- 4. 加速度 ramp ---
        self._current_output.linear.x = _approach(
            self._current_output.linear.x,
            target.linear.x,
            self._max_ax * dt,
        )
        self._current_output.linear.y = _approach(
            self._current_output.linear.y,
            target.linear.y,
            self._max_ay * dt,
        )
        self._current_output.angular.z = _approach(
            self._current_output.angular.z,
            target.angular.z,
            self._max_aw * dt,
        )

        # --- 5. 发布 ---
        self._safe_pub.publish(self._current_output)


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
