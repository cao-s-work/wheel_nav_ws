"""
ros_api.py — ROS 2 状态聚合 + Service 封装。

收集所有节点状态信息用于 Dashboard 展示。
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import Bool, UInt32, Float32
from diagnostic_msgs.msg import DiagnosticArray
from sensor_msgs.msg import BatteryState
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Trigger, SetBool
import time


class RosApi:
    """状态聚合层。持有 ROS node 引用，订阅/服务调用均通过此 node。"""

    def __init__(self, node: Node):
        self._node = node
        self._drv_ns = "zsl_driver_node"

        # ——— 驱动状态订阅 ———
        self._battery = 0.0
        self._ctrl_mode = 0
        self._connected = False
        self._read_only = True
        self._cmd_watchdog = -1.0

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        node.create_subscription(
            BatteryState, f"/{self._drv_ns}/battery",
            self._cb_battery, qos)
        node.create_subscription(
            UInt32, f"/{self._drv_ns}/ctrl_mode",
            self._cb_ctrl_mode, qos)
        node.create_subscription(
            Bool, f"/{self._drv_ns}/connection",
            self._cb_connection, qos)
        node.create_subscription(
            Bool, f"/{self._drv_ns}/read_only",
            self._cb_read_only, qos)
        node.create_subscription(
            Float32, f"/{self._drv_ns}/cmd_watchdog",
            self._cb_cmd_watchdog, qos)

        # ——— 姿态 Service Client ———
        self._svc_stand_up = node.create_client(Trigger, f"/{self._drv_ns}/stand_up")
        self._svc_lie_down = node.create_client(Trigger, f"/{self._drv_ns}/lie_down")
        self._svc_crawl = node.create_client(Trigger, f"/{self._drv_ns}/crawl")
        self._svc_emergency_stop = node.create_client(Trigger, f"/{self._drv_ns}/emergency_stop")
        self._svc_reset_estop = node.create_client(Trigger, f"/{self._drv_ns}/reset_estop")
        self._svc_set_read_only = node.create_client(SetBool, f"/{self._drv_ns}/set_read_only")

    # ---- 订阅回调 ----

    def _cb_battery(self, msg):
        self._battery = msg.percentage

    def _cb_ctrl_mode(self, msg):
        self._ctrl_mode = msg.data

    def _cb_connection(self, msg):
        self._connected = msg.data

    def _cb_read_only(self, msg):
        self._read_only = msg.data

    def _cb_cmd_watchdog(self, msg):
        self._cmd_watchdog = msg.data

    # ---- 聚合摘要 ----

    def summary(self) -> dict:
        return {
            "connected": self._connected,
            "read_only": self._read_only,
            "battery": round(self._battery, 1),
            "ctrl_mode": self._ctrl_mode,
            "cmd_watchdog_s": round(self._cmd_watchdog, 3),
        }

    # ---- 运动 Service 调用 ----

    def _call_trigger(self, client, name: str) -> dict:
        if not client.wait_for_service(timeout_sec=1.0):
            return {"success": False, "message": f"{name} service unavailable"}
        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self._node, future, timeout_sec=5.0)
        if future.done():
            resp = future.result()
            return {"success": resp.success, "message": resp.message}
        return {"success": False, "message": "timeout"}

    def stand_up(self) -> dict:
        return self._call_trigger(self._svc_stand_up, "stand_up")

    def lie_down(self) -> dict:
        return self._call_trigger(self._svc_lie_down, "lie_down")

    def crawl(self) -> dict:
        return self._call_trigger(self._svc_crawl, "crawl")

    def emergency_stop(self) -> dict:
        return self._call_trigger(self._svc_emergency_stop, "emergency_stop")

    def reset_estop(self) -> dict:
        return self._call_trigger(self._svc_reset_estop, "reset_estop")

    def set_read_only(self, ro: bool) -> dict:
        if not self._svc_set_read_only.wait_for_service(timeout_sec=1.0):
            return {"success": False, "message": "service unavailable"}
        future = self._svc_set_read_only.call_async(SetBool.Request(data=ro))
        rclpy.spin_until_future_complete(self._node, future, timeout_sec=3.0)
        if future.done():
            resp = future.result()
            return {"success": resp.success, "message": resp.message}
        return {"success": False, "message": "timeout"}
