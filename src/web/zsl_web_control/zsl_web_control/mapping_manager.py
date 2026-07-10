"""Mapping, map library, map switching, and managed stack operations."""
from __future__ import annotations

import json
import os
import re
import struct
import subprocess
import threading
import time
import zlib
from pathlib import Path
from typing import Any
from urllib.parse import quote

import yaml
from nav2_msgs.srv import LoadMap, SaveMap
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan, PointCloud2

from .nav2_client import Nav2Client
from .process_manager import ProcessManager
from .utils import EventJournal, RateTracker, wait_future


class MappingManager:
    def __init__(
        self,
        node: Node,
        journal: EventJournal,
        processes: ProcessManager,
        nav2: Nav2Client,
    ):
        self._node = node
        self._journal = journal
        self._processes = processes
        self._nav2 = nav2
        self._lock = threading.RLock()

        self.map_root = Path(os.path.expanduser(str(node.get_parameter("map_root").value))).resolve()
        self.map_root.mkdir(parents=True, exist_ok=True)
        self._preview_root = self.map_root / ".preview_cache"
        self._preview_root.mkdir(parents=True, exist_ok=True)
        self._active_state_path = self.map_root / ".active_map.json"

        self._mapping_command = str(node.get_parameter("mapping_command").value)
        self._navigation_command = str(node.get_parameter("navigation_command").value)
        self._map_topic = str(node.get_parameter("map_topic").value)
        self._lidar_topic = str(node.get_parameter("lidar_topic").value)
        self._scan_topic = str(node.get_parameter("scan_topic").value)
        self._odom_topic = str(node.get_parameter("odom_topic").value)
        self._save_service_name = str(node.get_parameter("map_save_service").value)
        self._load_service_name = str(node.get_parameter("map_load_service").value)
        self._allow_cli_fallback = bool(node.get_parameter("map_save_cli_fallback").value)
        self._image_format = str(node.get_parameter("map_image_format").value).lower()
        if self._image_format not in {"png", "pgm", "bmp"}:
            self._image_format = "png"

        self._save_client = node.create_client(SaveMap, self._save_service_name)
        self._load_client = node.create_client(LoadMap, self._load_service_name)

        sensor_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        map_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        self._lidar_rate = RateTracker()
        self._scan_rate = RateTracker()
        self._odom_rate = RateTracker()
        self._map_rate = RateTracker(window_s=10.0)
        self._map_info: dict[str, Any] | None = None

        node.create_subscription(PointCloud2, self._lidar_topic, lambda _: self._lidar_rate.tick(), sensor_qos)
        node.create_subscription(LaserScan, self._scan_topic, lambda _: self._scan_rate.tick(), sensor_qos)
        node.create_subscription(Odometry, self._odom_topic, lambda _: self._odom_rate.tick(), sensor_qos)
        node.create_subscription(OccupancyGrid, self._map_topic, self._map_cb, map_qos)

        self._active_map = self._read_active_map()

    @staticmethod
    def _valid_name(name: str) -> str:
        value = str(name).strip()
        if not re.fullmatch(r"[\w\-\u4e00-\u9fff]{1,64}", value, flags=re.UNICODE):
            raise ValueError("map name may only contain letters, numbers, Chinese characters, _ and -")
        return value

    def _map_cb(self, msg: OccupancyGrid) -> None:
        self._map_rate.tick()
        with self._lock:
            self._map_info = {
                "width": int(msg.info.width),
                "height": int(msg.info.height),
                "resolution": round(float(msg.info.resolution), 4),
                "origin": {
                    "x": round(float(msg.info.origin.position.x), 3),
                    "y": round(float(msg.info.origin.position.y), 3),
                },
                "frame_id": msg.header.frame_id,
                "cells": int(msg.info.width * msg.info.height),
            }

    def _read_active_map(self) -> str | None:
        try:
            data = json.loads(self._active_state_path.read_text(encoding="utf-8"))
            name = data.get("name")
            return str(name) if name else None
        except Exception:
            return None

    def _write_active_map(self, name: str | None) -> None:
        self._active_map = name
        try:
            self._active_state_path.write_text(
                json.dumps({"name": name, "updated_at": time.time()}, ensure_ascii=False, indent=2),
                encoding="utf-8",
            )
        except Exception as exc:
            self._node.get_logger().warning(f"Failed to persist active map: {exc}")

    def _yaml_path(self, name: str) -> Path:
        safe = self._valid_name(name)
        path = (self.map_root / f"{safe}.yaml").resolve()
        if self.map_root not in path.parents:
            raise ValueError("invalid map path")
        return path

    @staticmethod
    def _resolve_image(yaml_path: Path, metadata: dict[str, Any]) -> Path | None:
        image = str(metadata.get("image", "")).strip()
        if not image:
            return None
        if image.startswith("file://"):
            image = image[7:]
        path = Path(image)
        if not path.is_absolute():
            path = yaml_path.parent / path
        try:
            return path.resolve()
        except Exception:
            return None

    def list_maps(self) -> list[dict[str, Any]]:
        maps: list[dict[str, Any]] = []
        for yaml_path in self.map_root.glob("*.yaml"):
            try:
                metadata = yaml.safe_load(yaml_path.read_text(encoding="utf-8")) or {}
                image_path = self._resolve_image(yaml_path, metadata)
                stat = yaml_path.stat()
                maps.append(
                    {
                        "name": yaml_path.stem,
                        "yaml_path": str(yaml_path),
                        "image_path": str(image_path) if image_path else None,
                        "image_exists": bool(image_path and image_path.exists()),
                        "resolution": metadata.get("resolution"),
                        "origin": metadata.get("origin"),
                        "mode": metadata.get("mode", "trinary"),
                        "negate": metadata.get("negate", 0),
                        "occupied_thresh": metadata.get("occupied_thresh"),
                        "free_thresh": metadata.get("free_thresh"),
                        "modified_at": stat.st_mtime,
                        "size_bytes": stat.st_size + (image_path.stat().st_size if image_path and image_path.exists() else 0),
                        "active": yaml_path.stem == self._active_map,
                        "preview_url": f"/api/v1/maps/{quote(yaml_path.stem)}/preview?v={int(stat.st_mtime)}",
                    }
                )
            except Exception as exc:
                maps.append(
                    {
                        "name": yaml_path.stem,
                        "yaml_path": str(yaml_path),
                        "valid": False,
                        "error": str(exc),
                        "active": yaml_path.stem == self._active_map,
                    }
                )
        return sorted(maps, key=lambda item: float(item.get("modified_at", 0)), reverse=True)

    def map_detail(self, name: str) -> dict[str, Any] | None:
        for item in self.list_maps():
            if item.get("name") == name:
                return item
        return None

    def save_map(self, name: str) -> dict[str, Any]:
        try:
            safe = self._valid_name(name)
        except ValueError as exc:
            return {"success": False, "message": str(exc)}
        prefix = (self.map_root / safe).resolve()

        if self._save_client.wait_for_service(timeout_sec=1.5):
            request = SaveMap.Request()
            request.map_topic = self._map_topic
            request.map_url = str(prefix)
            request.image_format = self._image_format
            request.map_mode = "trinary"
            request.free_thresh = 0.25
            request.occupied_thresh = 0.65
            future = self._save_client.call_async(request)
            ok, response, error = wait_future(future, 20.0)
            if ok and response is not None and bool(response.result):
                self._journal.add(f"Map saved: {safe}", "success", "mapping")
                return {"success": True, "message": f"map {safe} saved", "map": self.map_detail(safe)}
            if not self._allow_cli_fallback:
                return {"success": False, "message": f"SaveMap failed: {error or 'server returned false'}"}

        if not self._allow_cli_fallback:
            return {"success": False, "message": f"map saver service unavailable: {self._save_service_name}"}

        try:
            result = subprocess.run(
                ["ros2", "run", "nav2_map_server", "map_saver_cli", "-f", str(prefix)],
                stdin=subprocess.DEVNULL,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                timeout=30.0,
                check=False,
                text=True,
            )
        except Exception as exc:
            return {"success": False, "message": f"map_saver_cli failed: {exc}"}
        if result.returncode != 0:
            return {"success": False, "message": result.stdout[-800:] or "map_saver_cli failed"}
        self._journal.add(f"Map saved with CLI fallback: {safe}", "success", "mapping")
        return {"success": True, "message": f"map {safe} saved", "map": self.map_detail(safe)}

    def load_map(self, name: str) -> dict[str, Any]:
        try:
            yaml_path = self._yaml_path(name)
        except ValueError as exc:
            return {"success": False, "message": str(exc)}
        if not yaml_path.exists():
            return {"success": False, "message": f"map does not exist: {name}"}
        if not self._load_client.wait_for_service(timeout_sec=2.0):
            return {"success": False, "message": f"map load service unavailable: {self._load_service_name}"}

        request = LoadMap.Request()
        request.map_url = str(yaml_path)
        future = self._load_client.call_async(request)
        ok, response, error = wait_future(future, 12.0)
        if not ok or response is None:
            return {"success": False, "message": f"load map failed: {error}"}
        if int(response.result) != int(LoadMap.Response.RESULT_SUCCESS):
            return {"success": False, "message": f"map server returned code {int(response.result)}"}
        self._write_active_map(name)
        self._journal.add(f"Map loaded: {name}", "success", "mapping")
        return {"success": True, "message": f"map {name} loaded", "map": self.map_detail(name)}

    def delete_map(self, name: str) -> dict[str, Any]:
        try:
            yaml_path = self._yaml_path(name)
        except ValueError as exc:
            return {"success": False, "message": str(exc)}
        if name == self._active_map:
            return {"success": False, "message": "active map cannot be deleted"}
        if not yaml_path.exists():
            return {"success": False, "message": "map does not exist"}
        try:
            metadata = yaml.safe_load(yaml_path.read_text(encoding="utf-8")) or {}
            image_path = self._resolve_image(yaml_path, metadata)
            yaml_path.unlink()
            if image_path and image_path.exists() and image_path.parent == self.map_root:
                image_path.unlink()
            preview = self._preview_root / f"{name}.png"
            if preview.exists():
                preview.unlink()
        except Exception as exc:
            return {"success": False, "message": f"delete map failed: {exc}"}
        self._journal.add(f"Map deleted: {name}", "warning", "mapping")
        return {"success": True, "message": f"map {name} deleted"}

    @staticmethod
    def _png_chunk(chunk_type: bytes, data: bytes) -> bytes:
        return struct.pack(">I", len(data)) + chunk_type + data + struct.pack(">I", zlib.crc32(chunk_type + data) & 0xFFFFFFFF)

    @classmethod
    def _write_grayscale_png(cls, width: int, height: int, pixels: bytes, output: Path) -> None:
        rows = b"".join(b"\x00" + pixels[row * width : (row + 1) * width] for row in range(height))
        png = b"\x89PNG\r\n\x1a\n"
        png += cls._png_chunk(b"IHDR", struct.pack(">IIBBBBB", width, height, 8, 0, 0, 0, 0))
        png += cls._png_chunk(b"IDAT", zlib.compress(rows, 9))
        png += cls._png_chunk(b"IEND", b"")
        output.write_bytes(png)

    @staticmethod
    def _read_pgm(path: Path) -> tuple[int, int, bytes]:
        data = path.read_bytes()
        index = 0

        def token() -> bytes:
            nonlocal index
            while index < len(data):
                if data[index:index + 1] == b"#":
                    while index < len(data) and data[index:index + 1] not in {b"\n", b"\r"}:
                        index += 1
                elif data[index:index + 1].isspace():
                    index += 1
                else:
                    break
            start = index
            while index < len(data) and not data[index:index + 1].isspace() and data[index:index + 1] != b"#":
                index += 1
            return data[start:index]

        magic = token()
        width = int(token())
        height = int(token())
        max_value = int(token())
        if width <= 0 or height <= 0 or max_value <= 0:
            raise ValueError("invalid PGM header")
        while index < len(data) and data[index:index + 1].isspace():
            index += 1
        count = width * height
        if magic == b"P5":
            if max_value <= 255:
                raw = data[index:index + count]
            else:
                source = data[index:index + count * 2]
                raw = bytes(int.from_bytes(source[i:i + 2], "big") * 255 // max_value for i in range(0, len(source), 2))
        elif magic == b"P2":
            values = []
            while len(values) < count:
                values.append(int(token()) * 255 // max_value)
            raw = bytes(values)
        else:
            raise ValueError("unsupported PGM type")
        if len(raw) < count:
            raise ValueError("truncated PGM image")
        return width, height, raw[:count]

    def preview_path(self, name: str) -> Path | None:
        try:
            yaml_path = self._yaml_path(name)
            metadata = yaml.safe_load(yaml_path.read_text(encoding="utf-8")) or {}
            image_path = self._resolve_image(yaml_path, metadata)
        except Exception:
            return None
        if not image_path or not image_path.exists():
            return None
        if image_path.suffix.lower() in {".png", ".jpg", ".jpeg", ".webp", ".bmp"}:
            return image_path
        if image_path.suffix.lower() != ".pgm":
            return None
        cache = self._preview_root / f"{name}.png"
        if not cache.exists() or cache.stat().st_mtime < image_path.stat().st_mtime:
            try:
                width, height, pixels = self._read_pgm(image_path)
                self._write_grayscale_png(width, height, pixels, cache)
            except Exception as exc:
                self._node.get_logger().warning(f"PGM preview conversion failed: {exc}")
                return None
        return cache

    def start_mapping(self) -> dict[str, Any]:
        process_status = self._processes.status()
        managed_navigation = bool(process_status.get("navigation", {}).get("running"))
        if self._nav2.available(0.1) and not managed_navigation:
            return {
                "success": False,
                "message": "an externally started Nav2 stack is active; stop it before starting mapping",
            }
        self._nav2.cancel_goal()
        self._processes.stop("navigation", timeout_s=5.0)
        return self._processes.start("mapping", self._mapping_command)

    def stop_mapping(self) -> dict[str, Any]:
        return self._processes.stop("mapping")

    def start_navigation(self, map_name: str) -> dict[str, Any]:
        try:
            yaml_path = self._yaml_path(map_name)
        except ValueError as exc:
            return {"success": False, "message": str(exc)}
        if not yaml_path.exists():
            return {"success": False, "message": f"map does not exist: {map_name}"}
        process_status = self._processes.status()
        managed_mapping = bool(process_status.get("mapping", {}).get("running"))
        try:
            external_slam = "slam_toolbox" in set(self._node.get_node_names()) and not managed_mapping
        except Exception:
            external_slam = False
        if external_slam:
            return {
                "success": False,
                "message": "an externally started SLAM stack is active; stop it before starting navigation",
            }
        self._processes.stop("mapping", timeout_s=5.0)
        if self._nav2.available(0.2):
            loaded = self.load_map(map_name)
            if loaded.get("success"):
                return {"success": True, "message": "navigation is already running; selected map loaded"}
        result = self._processes.start(
            "navigation",
            self._navigation_command,
            substitutions={"map": str(yaml_path)},
        )
        if result.get("success"):
            self._write_active_map(map_name)
        return result

    def stop_navigation(self) -> dict[str, Any]:
        self._nav2.cancel_goal()
        return self._processes.stop("navigation")

    def status(self) -> dict[str, Any]:
        try:
            node_names = set(self._node.get_node_names())
        except Exception:
            node_names = set()
        with self._lock:
            map_info = dict(self._map_info) if self._map_info else None
        process_status = self._processes.status()
        return {
            "slam_active": "slam_toolbox" in node_names or bool(process_status.get("mapping", {}).get("running")),
            "navigation_active": self._nav2.available(0.0) or bool(process_status.get("navigation", {}).get("running")),
            "map_topic": self._map_rate.snapshot(),
            "lidar": self._lidar_rate.snapshot(),
            "scan": self._scan_rate.snapshot(),
            "odometry": self._odom_rate.snapshot(),
            "map_info": map_info,
            "active_map": self._active_map,
            "map_count": len(self.list_maps()),
            "map_root": str(self.map_root),
            "processes": process_status,
        }
