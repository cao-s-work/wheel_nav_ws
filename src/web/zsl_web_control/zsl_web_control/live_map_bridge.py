"""Live SLAM visualization bridge for the ZSL web console.

The bridge keeps the browser independent from rosbridge/RViz.  It converts the
latest nav_msgs/OccupancyGrid into a cached PNG and exposes lightweight JSON
containing the robot pose, travelled path and a down-sampled LaserScan overlay.
"""
from __future__ import annotations

import math
import struct
import threading
import time
import zlib
from collections import deque
from typing import Any

from nav_msgs.msg import OccupancyGrid
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener

from .utils import quaternion_to_yaw


def _png_chunk(chunk_type: bytes, payload: bytes) -> bytes:
    return (
        struct.pack(">I", len(payload))
        + chunk_type
        + payload
        + struct.pack(">I", zlib.crc32(chunk_type + payload) & 0xFFFFFFFF)
    )


def _encode_rgb_png(width: int, height: int, rgb_rows: bytes) -> bytes:
    """Encode filter-prefixed RGB scanlines as a PNG without Pillow."""
    signature = b"\x89PNG\r\n\x1a\n"
    ihdr = struct.pack(">IIBBBBB", width, height, 8, 2, 0, 0, 0)
    return b"".join(
        (
            signature,
            _png_chunk(b"IHDR", ihdr),
            _png_chunk(b"IDAT", zlib.compress(rgb_rows, level=6)),
            _png_chunk(b"IEND", b""),
        )
    )


def _rotate_vector(qx: float, qy: float, qz: float, qw: float, x: float, y: float, z: float) -> tuple[float, float, float]:
    """Rotate a vector by a unit quaternion without external dependencies."""
    tx = 2.0 * (qy * z - qz * y)
    ty = 2.0 * (qz * x - qx * z)
    tz = 2.0 * (qx * y - qy * x)
    return (
        x + qw * tx + (qy * tz - qz * ty),
        y + qw * ty + (qz * tx - qx * tz),
        z + qw * tz + (qx * ty - qy * tx),
    )


class LiveMapBridge:
    def __init__(self, node: Node):
        self._node = node
        self._lock = threading.RLock()

        self._enabled = bool(node.get_parameter("live_map_enabled").value)
        self._map_topic = str(node.get_parameter("map_topic").value)
        self._scan_topic = str(node.get_parameter("scan_topic").value)
        self._target_frame = str(node.get_parameter("live_map_frame").value).strip() or "map"
        self._robot_frame = str(node.get_parameter("live_robot_frame").value).strip() or "base_link"
        self._max_scan_points = max(20, int(node.get_parameter("live_scan_max_points").value))
        self._max_path_points = max(100, int(node.get_parameter("live_path_max_points").value))
        self._path_min_distance = max(0.005, float(node.get_parameter("live_path_min_distance").value))
        image_hz = max(0.2, float(node.get_parameter("live_map_image_rate_hz").value))
        self._image_period = 1.0 / image_hz

        self._map_version = 0
        self._map_received_at = 0.0
        self._last_version_at = 0.0
        self._map_meta: dict[str, Any] | None = None
        self._map_data = b""
        self._png_cache_version = -1
        self._png_cache = b""

        self._pose: dict[str, Any] | None = None
        self._pose_error = "waiting for TF"
        self._path: deque[tuple[float, float]] = deque(maxlen=self._max_path_points)

        self._scan_points: list[list[float]] = []
        self._scan_received_at = 0.0
        self._scan_frame = ""
        self._scan_error = "waiting for /scan"

        self._tf_buffer = Buffer(cache_time=Duration(seconds=15.0))
        self._tf_listener = TransformListener(self._tf_buffer, node, spin_thread=False)

        if not self._enabled:
            return

        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        scan_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        node.create_subscription(OccupancyGrid, self._map_topic, self._map_cb, map_qos)
        node.create_subscription(LaserScan, self._scan_topic, self._scan_cb, scan_qos)
        node.create_timer(0.1, self._update_robot_pose)

    @staticmethod
    def _stamp_to_time(msg_stamp) -> Time:
        if int(msg_stamp.sec) == 0 and int(msg_stamp.nanosec) == 0:
            return Time()
        return Time.from_msg(msg_stamp)

    def _lookup_transform(self, target: str, source: str, stamp: Time):
        if not source or source == target:
            return None
        try:
            return self._tf_buffer.lookup_transform(
                target,
                source,
                stamp,
                timeout=Duration(seconds=0.035),
            )
        except Exception:
            # The latest transform is usually preferable to dropping the whole
            # visualization when the scan stamp is a few milliseconds ahead.
            return self._tf_buffer.lookup_transform(
                target,
                source,
                Time(),
                timeout=Duration(seconds=0.035),
            )

    def _map_cb(self, msg: OccupancyGrid) -> None:
        width = int(msg.info.width)
        height = int(msg.info.height)
        if width <= 0 or height <= 0 or len(msg.data) < width * height:
            return

        q = msg.info.origin.orientation
        meta = {
            "width": width,
            "height": height,
            "resolution": float(msg.info.resolution),
            "origin": {
                "x": float(msg.info.origin.position.x),
                "y": float(msg.info.origin.position.y),
                "yaw_deg": math.degrees(quaternion_to_yaw(q.x, q.y, q.z, q.w)),
            },
            "frame_id": msg.header.frame_id or self._target_frame,
        }
        # OccupancyGrid is int8. Converting through modulo preserves -1 as 255
        # while keeping the representation compact.
        raw = bytes((int(value) & 0xFF) for value in msg.data[: width * height])
        now = time.monotonic()
        with self._lock:
            self._map_meta = meta
            self._map_data = raw
            self._map_received_at = now
            if self._map_version == 0 or now - self._last_version_at >= self._image_period:
                self._map_version += 1
                self._last_version_at = now
                self._png_cache_version = -1

    def _update_robot_pose(self) -> None:
        if not self._enabled:
            return
        try:
            transform = self._lookup_transform(self._target_frame, self._robot_frame, Time())
            if transform is None:
                return
            t = transform.transform.translation
            r = transform.transform.rotation
            pose = {
                "x": round(float(t.x), 4),
                "y": round(float(t.y), 4),
                "yaw_deg": round(math.degrees(quaternion_to_yaw(r.x, r.y, r.z, r.w)), 2),
                "frame_id": self._target_frame,
                "robot_frame": self._robot_frame,
                "timestamp": time.time(),
            }
            with self._lock:
                previous = self._path[-1] if self._path else None
                if previous is None or math.hypot(pose["x"] - previous[0], pose["y"] - previous[1]) >= self._path_min_distance:
                    self._path.append((pose["x"], pose["y"]))
                self._pose = pose
                self._pose_error = ""
        except Exception as exc:
            with self._lock:
                self._pose_error = f"TF {self._target_frame} <- {self._robot_frame}: {exc}"

    def _scan_cb(self, msg: LaserScan) -> None:
        if not self._enabled or not msg.ranges:
            return
        source_frame = msg.header.frame_id.strip()
        try:
            transform = self._lookup_transform(
                self._target_frame,
                source_frame,
                self._stamp_to_time(msg.header.stamp),
            )
            if transform is None:
                tx = ty = tz = 0.0
                qx = qy = qz = 0.0
                qw = 1.0
            else:
                tr = transform.transform.translation
                rot = transform.transform.rotation
                tx, ty, tz = float(tr.x), float(tr.y), float(tr.z)
                qx, qy, qz, qw = float(rot.x), float(rot.y), float(rot.z), float(rot.w)

            count = len(msg.ranges)
            step = max(1, math.ceil(count / self._max_scan_points))
            angle = float(msg.angle_min)
            points: list[list[float]] = []
            minimum = max(0.0, float(msg.range_min))
            maximum = float(msg.range_max)

            for index in range(0, count, step):
                distance = float(msg.ranges[index])
                current_angle = angle + index * float(msg.angle_increment)
                if not math.isfinite(distance) or distance < minimum or distance > maximum:
                    continue
                lx = distance * math.cos(current_angle)
                ly = distance * math.sin(current_angle)
                rx, ry, _ = _rotate_vector(qx, qy, qz, qw, lx, ly, 0.0)
                points.append([round(rx + tx, 3), round(ry + ty, 3)])

            with self._lock:
                self._scan_points = points
                self._scan_received_at = time.monotonic()
                self._scan_frame = source_frame
                self._scan_error = ""
        except Exception as exc:
            with self._lock:
                self._scan_error = f"scan TF failed: {exc}"

    def reset_path(self) -> dict[str, Any]:
        with self._lock:
            self._path.clear()
            if self._pose:
                self._path.append((float(self._pose["x"]), float(self._pose["y"])))
        return {"success": True, "message": "live mapping trajectory cleared"}

    def snapshot(self) -> dict[str, Any]:
        now = time.monotonic()
        with self._lock:
            meta = dict(self._map_meta) if self._map_meta else None
            if meta:
                meta["origin"] = dict(meta["origin"])
                meta["version"] = self._map_version
                meta["image_url"] = f"/api/v1/live_mapping/map.png?v={self._map_version}"
                meta["age_s"] = round(max(0.0, now - self._map_received_at), 3)
            pose = dict(self._pose) if self._pose else None
            path = [[round(x, 3), round(y, 3)] for x, y in self._path]
            scan = [list(point) for point in self._scan_points]
            scan_age = round(max(0.0, now - self._scan_received_at), 3) if self._scan_received_at else None
            return {
                "enabled": self._enabled,
                "available": bool(meta),
                "map": meta,
                "pose": pose,
                "path": path,
                "scan": {
                    "points": scan,
                    "count": len(scan),
                    "frame_id": self._scan_frame,
                    "age_s": scan_age,
                },
                "frames": {
                    "map": self._target_frame,
                    "robot": self._robot_frame,
                },
                "errors": {
                    "pose": self._pose_error,
                    "scan": self._scan_error,
                },
                "timestamp": time.time(),
            }

    def map_png(self) -> tuple[bytes | None, int]:
        with self._lock:
            version = self._map_version
            if version <= 0 or not self._map_meta or not self._map_data:
                return None, version
            if self._png_cache_version == version and self._png_cache:
                return self._png_cache, version
            meta = dict(self._map_meta)
            raw = self._map_data

        width = int(meta["width"])
        height = int(meta["height"])
        expected = width * height
        if len(raw) < expected:
            return None, version

        rows = bytearray()
        # OccupancyGrid row zero is the lower edge of the map. PNG row zero is
        # the upper edge, so rows are reversed to match RViz orientation.
        for grid_y in range(height - 1, -1, -1):
            rows.append(0)  # PNG filter type: None
            offset = grid_y * width
            for grid_x in range(width):
                value = raw[offset + grid_x]
                if value == 255:  # unknown (-1)
                    color = (22, 34, 50)
                elif value >= 65:
                    color = (14, 20, 29)
                elif value <= 25:
                    color = (230, 236, 240)
                else:
                    shade = max(45, min(215, 230 - int(value * 2.0)))
                    color = (shade, shade, shade)
                rows.extend(color)

        encoded = _encode_rgb_png(width, height, bytes(rows))
        with self._lock:
            if self._map_version == version:
                self._png_cache = encoded
                self._png_cache_version = version
        return encoded, version
