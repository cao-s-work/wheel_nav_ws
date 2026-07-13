"""topic_rate_monitor — 独立轻量传感器频率监控节点.

订阅 /livox/lidar、/scan、/Odometry，仅统计帧率，每 0.5s 发布聚合 JSON
到 /system/topic_rates，供 Web 控制台和其他组件使用。

不依赖外部 ROS 消息反序列化后内容 — 回调仅计数，但 rclpy 仍需构造消息对象。
"""
from __future__ import annotations

import json
import threading
import time
from typing import Any

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, PointCloud2
from std_msgs.msg import String


class TopicRateMonitor(Node):
    def __init__(self) -> None:
        super().__init__("topic_rate_monitor")

        self.declare_parameter("lidar_topic", "/livox/lidar")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("odom_topic", "/Odometry")
        self.declare_parameter("output_topic", "/system/topic_rates")
        self.declare_parameter("publish_period_s", 0.5)
        self.declare_parameter("rate_window_s", 2.0)
        self.declare_parameter("alive_timeout_s", 1.5)

        self._lidar_topic = str(self.get_parameter("lidar_topic").value)
        self._scan_topic = str(self.get_parameter("scan_topic").value)
        self._odom_topic = str(self.get_parameter("odom_topic").value)
        self._output_topic = str(self.get_parameter("output_topic").value)
        self._publish_period = max(0.1, float(self.get_parameter("publish_period_s").value))
        self._rate_window = max(1.0, float(self.get_parameter("rate_window_s").value))
        self._alive_timeout = max(0.5, float(self.get_parameter("alive_timeout_s").value))

        self._tick_lock = threading.Lock()
        self._sub: dict[str, dict[str, Any]] = {
            "lidar":     {"topic": self._lidar_topic, "samples": [], "last": 0.0},
            "scan":      {"topic": self._scan_topic,  "samples": [], "last": 0.0},
            "odometry":  {"topic": self._odom_topic,  "samples": [], "last": 0.0},
        }

        sensor_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(
            PointCloud2, self._lidar_topic,
            lambda _: self._tick("lidar"), sensor_qos,
        )
        self.create_subscription(
            LaserScan, self._scan_topic,
            lambda _: self._tick("scan"), sensor_qos,
        )
        self.create_subscription(
            Odometry, self._odom_topic,
            lambda _: self._tick("odometry"), sensor_qos,
        )

        self._pub = self.create_publisher(String, self._output_topic, 10)
        self.create_timer(self._publish_period, self._publish)

        self.get_logger().info(
            f"Monitoring lidar={self._lidar_topic} scan={self._scan_topic} "
            f"odom={self._odom_topic} → {self._output_topic}"
        )

    def _tick(self, key: str) -> None:
        now = time.monotonic()
        with self._tick_lock:
            entry = self._sub[key]
            entry["samples"].append(now)
            entry["last"] = now
            cutoff = now - self._rate_window
            while entry["samples"] and entry["samples"][0] < cutoff:
                entry["samples"].pop(0)

    def _snapshot(self, key: str, now: float) -> dict[str, Any]:
        with self._tick_lock:
            entry = self._sub[key]
            samples = list(entry["samples"])
            last = entry["last"]

        if len(samples) >= 2:
            elapsed = max(1e-6, samples[-1] - samples[0])
            hz = round((len(samples) - 1) / elapsed, 2)
        else:
            hz = 0.0

        age = now - last if last > 0 else -1.0
        return {
            "topic": entry["topic"],
            "hz": hz,
            "alive": last > 0 and age <= self._alive_timeout,
            "age_s": round(age, 3) if age >= 0 else -1.0,
        }

    def _publish(self) -> None:
        now = time.monotonic()
        payload = {
            "stamp": now,
            "window_s": self._rate_window,
            "topics": {
                "lidar": self._snapshot("lidar", now),
                "scan": self._snapshot("scan", now),
                "odometry": self._snapshot("odometry", now),
            },
        }
        msg = String()
        msg.data = json.dumps(payload)
        self._pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    try:
        rclpy.spin(TopicRateMonitor())
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
