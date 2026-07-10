"""
cmd_vel_mux.py — 多路 cmd_vel 合并 + watchdog。

订阅:
  /cmd_vel           (Nav2 velocity_smoother 平滑后)
  /cmd_vel_teleop    (Web 遥控)
  /web/teleop_active (Web 人工接管状态, transient-local QoS)

优先级:
  teleop_active=true 且 teleop_alive → teleop
  teleop_active=true 且 teleop 超时   → zero（绝不回退 Nav2）
  teleop_active=false 且 nav_alive   → Nav2
  其他                               → zero

发布:
  /cmd_vel_selected → safety_node
"""
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool


TELEOP_ACTIVE_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


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
        self._teleop_active = False

        # 订阅
        self._nav_sub = self.create_subscription(
            Twist, "cmd_vel", self._nav_cb, 10
        )
        self._teleop_sub = self.create_subscription(
            Twist, "cmd_vel_teleop", self._teleop_cb, 10
        )
        self._teleop_active_sub = self.create_subscription(
            Bool, "/web/teleop_active", self._teleop_active_cb, TELEOP_ACTIVE_QOS
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

    def _teleop_active_cb(self, msg: Bool):
        new_active = bool(msg.data)
        if new_active and not self._teleop_active:
            # 刚进入手动模式，清除历史命令
            self._last_teleop_cmd = Twist()
            self._last_teleop_time = 0.0
        if not new_active:
            # 退出手动模式，也清除历史命令
            self._last_teleop_cmd = Twist()
            self._last_teleop_time = 0.0
        self._teleop_active = new_active

    def _tick(self):
        now = time.monotonic()

        nav_alive = (
            self._last_nav_time > 0.0
            and now - self._last_nav_time < self._nav_timeout_s
        )
        teleop_alive = (
            self._last_teleop_time > 0.0
            and now - self._last_teleop_time < self._teleop_timeout_s
        )

        if self._teleop_active:
            # 处于人工接管模式时，绝不回退到 Nav2
            if teleop_alive:
                selected = self._last_teleop_cmd
            else:
                selected = Twist()
        elif nav_alive:
            selected = self._last_nav_cmd
        else:
            selected = Twist()

        self._sel_pub.publish(selected)


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
