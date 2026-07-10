"""Robot status aggregation and safe ROS service wrappers."""
from __future__ import annotations

import math
import threading
import time
from typing import Any

from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool, Float32, UInt32
from std_srvs.srv import Empty, SetBool, Trigger

from .utils import EventJournal, quaternion_to_yaw, wait_future, yaw_to_quaternion


class RosApi:
    def __init__(self, node: Node, journal: EventJournal):
        self._node = node
        self._journal = journal
        self._lock = threading.RLock()

        self._driver_ns = str(node.get_parameter("driver_ns").value).strip("/")
        self._initialpose_topic = str(node.get_parameter("initialpose_topic").value)
        self._amcl_pose_topic = str(node.get_parameter("amcl_pose_topic").value)
        self._odom_topic = str(node.get_parameter("odom_topic").value)
        self._global_localization_service = str(node.get_parameter("global_localization_service").value)
        self._nomotion_update_service = str(node.get_parameter("nomotion_update_service").value)

        self._battery_raw = float("nan")
        self._ctrl_mode = 0
        self._connected = False
        self._read_only = True
        self._estop_latched = False
        self._cmd_watchdog = -1.0
        self._diagnostics: list[dict[str, Any]] = []
        self._pose: dict[str, Any] | None = None
        self._odom_pose: dict[str, Any] | None = None

        state_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        node.create_subscription(BatteryState, self._driver_topic("battery"), self._cb_battery, state_qos)
        node.create_subscription(UInt32, self._driver_topic("ctrl_mode"), self._cb_ctrl_mode, state_qos)
        node.create_subscription(Bool, self._driver_topic("connection"), self._cb_connection, state_qos)
        node.create_subscription(Bool, self._driver_topic("read_only"), self._cb_read_only, state_qos)
        node.create_subscription(Bool, self._driver_topic("estop_latched"), self._cb_estop, state_qos)
        node.create_subscription(Float32, self._driver_topic("cmd_watchdog"), self._cb_watchdog, state_qos)
        node.create_subscription(DiagnosticArray, "/diagnostics", self._cb_diagnostics, state_qos)
        node.create_subscription(PoseWithCovarianceStamped, self._amcl_pose_topic, self._cb_amcl_pose, state_qos)
        node.create_subscription(Odometry, self._odom_topic, self._cb_odom, state_qos)

        self._initialpose_pub = node.create_publisher(PoseWithCovarianceStamped, self._initialpose_topic, 10)

        self._svc_stand = node.create_client(Trigger, self._driver_service("stand_up"))
        self._svc_lie = node.create_client(Trigger, self._driver_service("lie_down"))
        self._svc_crawl = node.create_client(Trigger, self._driver_service("crawl"))
        self._svc_estop = node.create_client(Trigger, self._driver_service("emergency_stop"))
        self._svc_reset_estop = node.create_client(Trigger, self._driver_service("reset_estop"))
        self._svc_read_only = node.create_client(SetBool, self._driver_service("set_read_only"))
        self._svc_global_localization = node.create_client(Empty, self._global_localization_service)
        self._svc_nomotion_update = node.create_client(Empty, self._nomotion_update_service)

    def _driver_topic(self, suffix: str) -> str:
        return f"/{self._driver_ns}/{suffix}"

    def _driver_service(self, suffix: str) -> str:
        return f"/{self._driver_ns}/{suffix}"

    @property
    def read_only(self) -> bool:
        with self._lock:
            return self._read_only

    @property
    def estop_latched(self) -> bool:
        with self._lock:
            return self._estop_latched

    def _cb_battery(self, msg: BatteryState) -> None:
        with self._lock:
            self._battery_raw = float(msg.percentage)

    def _cb_ctrl_mode(self, msg: UInt32) -> None:
        with self._lock:
            self._ctrl_mode = int(msg.data)

    def _cb_connection(self, msg: Bool) -> None:
        with self._lock:
            self._connected = bool(msg.data)

    def _cb_read_only(self, msg: Bool) -> None:
        with self._lock:
            self._read_only = bool(msg.data)

    def _cb_estop(self, msg: Bool) -> None:
        with self._lock:
            changed = self._estop_latched != bool(msg.data)
            self._estop_latched = bool(msg.data)
        if changed:
            level = "error" if msg.data else "success"
            self._journal.add(
                "Emergency stop latched" if msg.data else "Emergency stop reset",
                level,
                "driver",
            )

    def _cb_watchdog(self, msg: Float32) -> None:
        with self._lock:
            self._cmd_watchdog = float(msg.data)

    def _cb_diagnostics(self, msg: DiagnosticArray) -> None:
        items = []
        for status in msg.status[:40]:
            items.append(
                {
                    "name": status.name,
                    "level": int(status.level),
                    "message": status.message,
                    "hardware_id": status.hardware_id,
                    "values": {item.key: item.value for item in status.values[:20]},
                }
            )
        with self._lock:
            self._diagnostics = items

    @staticmethod
    def _pose_dict(pose, frame_id: str, stamp_s: float, source: str) -> dict[str, Any]:
        yaw = quaternion_to_yaw(
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        )
        return {
            "x": round(float(pose.position.x), 4),
            "y": round(float(pose.position.y), 4),
            "z": round(float(pose.position.z), 4),
            "yaw_deg": round(math.degrees(yaw), 2),
            "frame_id": frame_id,
            "source": source,
            "timestamp": stamp_s,
        }

    def _cb_amcl_pose(self, msg: PoseWithCovarianceStamped) -> None:
        stamp_s = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) / 1e9
        pose = self._pose_dict(msg.pose.pose, msg.header.frame_id or "map", stamp_s, "amcl")
        pose["covariance_xy"] = round(float(msg.pose.covariance[0] + msg.pose.covariance[7]), 5)
        with self._lock:
            self._pose = pose

    def _cb_odom(self, msg: Odometry) -> None:
        stamp_s = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) / 1e9
        pose = self._pose_dict(msg.pose.pose, msg.header.frame_id or "odom", stamp_s, "odometry")
        pose["linear_speed"] = round(float(msg.twist.twist.linear.x), 4)
        pose["angular_speed"] = round(float(msg.twist.twist.angular.z), 4)
        with self._lock:
            self._odom_pose = pose

    @staticmethod
    def _battery_percent(raw: float) -> float | None:
        if math.isnan(raw) or raw < 0.0:
            return None
        value = raw * 100.0 if raw <= 1.0 else raw
        return round(max(0.0, min(100.0, value)), 1)

    def summary(self) -> dict[str, Any]:
        with self._lock:
            battery = self._battery_raw
            pose = dict(self._pose) if self._pose else None
            odom_pose = dict(self._odom_pose) if self._odom_pose else None
            return {
                "connected": self._connected,
                "read_only": self._read_only,
                "estop_latched": self._estop_latched,
                "battery_percent": self._battery_percent(battery),
                "ctrl_mode": self._ctrl_mode,
                "cmd_watchdog_s": round(self._cmd_watchdog, 3),
                "pose": pose or odom_pose,
                "localization_pose": pose,
                "odometry_pose": odom_pose,
                "diagnostics": list(self._diagnostics),
            }

    def _call_trigger(self, client, name: str, timeout_s: float = 5.0) -> dict[str, Any]:
        if not client.wait_for_service(timeout_sec=1.0):
            return {"success": False, "message": f"{name} service unavailable"}
        future = client.call_async(Trigger.Request())
        ok, response, error = wait_future(future, timeout_s)
        if not ok or response is None:
            return {"success": False, "message": f"{name} failed: {error}"}
        result = {"success": bool(response.success), "message": response.message or name}
        self._journal.add(
            f"{name}: {result['message']}",
            "success" if result["success"] else "error",
            "driver",
        )
        return result

    def stand_up(self) -> dict[str, Any]:
        return self._call_trigger(self._svc_stand, "stand_up")

    def lie_down(self) -> dict[str, Any]:
        return self._call_trigger(self._svc_lie, "lie_down")

    def crawl(self) -> dict[str, Any]:
        return self._call_trigger(self._svc_crawl, "crawl")

    def emergency_stop(self) -> dict[str, Any]:
        return self._call_trigger(self._svc_estop, "emergency_stop", timeout_s=2.0)

    def reset_estop(self) -> dict[str, Any]:
        return self._call_trigger(self._svc_reset_estop, "reset_estop")

    def set_read_only(self, read_only: bool) -> dict[str, Any]:
        if not self._svc_read_only.wait_for_service(timeout_sec=1.0):
            return {"success": False, "message": "set_read_only service unavailable"}
        future = self._svc_read_only.call_async(SetBool.Request(data=bool(read_only)))
        ok, response, error = wait_future(future, 4.0)
        if not ok or response is None:
            return {"success": False, "message": f"set_read_only failed: {error}"}
        result = {"success": bool(response.success), "message": response.message}
        self._journal.add(
            f"read_only={read_only}: {response.message}",
            "success" if result["success"] else "error",
            "driver",
        )
        return result

    def set_initial_pose(
        self,
        x: float,
        y: float,
        yaw_deg: float,
        covariance_xy: float = 0.25,
        covariance_yaw: float = 0.0685,
    ) -> dict[str, Any]:
        message = PoseWithCovarianceStamped()
        message.header.frame_id = "map"
        message.header.stamp = self._node.get_clock().now().to_msg()
        message.pose.pose.position.x = float(x)
        message.pose.pose.position.y = float(y)
        qx, qy, qz, qw = yaw_to_quaternion(math.radians(float(yaw_deg)))
        message.pose.pose.orientation.x = qx
        message.pose.pose.orientation.y = qy
        message.pose.pose.orientation.z = qz
        message.pose.pose.orientation.w = qw
        message.pose.covariance[0] = max(1e-6, float(covariance_xy))
        message.pose.covariance[7] = max(1e-6, float(covariance_xy))
        message.pose.covariance[35] = max(1e-6, float(covariance_yaw))
        self._initialpose_pub.publish(message)
        self._journal.add(
            f"Initial pose published: x={x:.2f}, y={y:.2f}, yaw={yaw_deg:.1f}°",
            "success",
            "localization",
        )
        return {"success": True, "message": "initial pose published"}

    def _call_empty(self, client, name: str, timeout_s: float = 3.0) -> dict[str, Any]:
        if not client.wait_for_service(timeout_sec=1.0):
            return {"success": False, "message": f"{name} service unavailable"}
        future = client.call_async(Empty.Request())
        ok, _, error = wait_future(future, timeout_s)
        if not ok:
            return {"success": False, "message": f"{name} failed: {error}"}
        self._journal.add(f"{name} requested", "success", "localization")
        return {"success": True, "message": f"{name} requested"}

    def global_localization(self) -> dict[str, Any]:
        return self._call_empty(self._svc_global_localization, "global localization")

    def nomotion_update(self) -> dict[str, Any]:
        return self._call_empty(self._svc_nomotion_update, "no-motion update")
