"""
cmd_vel_mux.py — 多路 cmd_vel 合并 + watchdog。

订阅:
  /cmd_vel          (Nav2 velocity_smoother 平滑后)
  /cmd_vel_teleop   (Web 遥控)

优先级: Nav2 > teleop
  - Nav2 有活跃消息（500ms 内） → 转发 Nav2
  - 否则 → 转发 teleop（teleop watchdog 500ms 超时则零速）

发布:
  /cmd_vel_selected → safety_node

注意:
  - 所有时钟使用 time.monotonic()，避免系统校时跳变
  - 不依赖 threading.Lock 重入
"""
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class CmdVelMux(Node):
    def __init__(self):
        super().__init__("cmd_vel_mux")

        self._nav_timeout_s = self.declare_parameter("nav_timeout_s", 0.5).value
        self._teleop_timeout_s = self.declare_parameter("teleop_timeout_s", 0.5).value

        # 状态
        self._last_nav_cmd = Twist()
        self._last_nav_time = 0.0
        self._last_teleop_cmd = Twist()
        self._last_teleop_time = 0.0

        # 订阅
        self._nav_sub = self.create_subscription(
            Twist, "cmd_vel", self._nav_cb, 10
        )
        self._teleop_sub = self.create_subscription(
            Twist, "cmd_vel_teleop", self._teleop_cb, 10
        )

        # 发布
        self._sel_pub = self.create_publisher(Twist, "cmd_vel_selected", 10)

        # 定时器 50Hz
        self._timer = self.create_timer(0.02, self._tick)

        self.get_logger().info(
            f"CmdVelMux ready (nav_timeout={self._nav_timeout_s}s, "
            f"teleop_timeout={self._teleop_timeout_s}s)"
        )

    def _nav_cb(self, msg: Twist):
        self._last_nav_cmd = msg
        self._last_nav_time = time.monotonic()

    def _teleop_cb(self, msg: Twist):
        self._last_teleop_cmd = msg
        self._last_teleop_time = time.monotonic()

    def _tick(self):
        now = time.monotonic()
        nav_alive = (now - self._last_nav_time) < self._nav_timeout_s
        teleop_alive = (now - self._last_teleop_time) < self._teleop_timeout_s

        if nav_alive:
            self._sel_pub.publish(self._last_nav_cmd)
        elif teleop_alive:
            self._sel_pub.publish(self._last_teleop_cmd)
        else:
            self._sel_pub.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelMux()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
