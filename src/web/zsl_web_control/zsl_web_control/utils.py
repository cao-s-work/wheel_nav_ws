"""Shared helpers for the ZSL commercial web gateway."""
from __future__ import annotations

import math
import threading
import time
from collections import deque
from typing import Any, Callable, Optional


def clamp(value: float, minimum: float, maximum: float) -> float:
    return max(minimum, min(maximum, value))


def yaw_to_quaternion(yaw_rad: float) -> tuple[float, float, float, float]:
    return (0.0, 0.0, math.sin(yaw_rad / 2.0), math.cos(yaw_rad / 2.0))


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def duration_to_seconds(duration: Any) -> float:
    return float(getattr(duration, "sec", 0)) + float(getattr(duration, "nanosec", 0)) / 1e9


def wait_future(future: Any, timeout_s: float) -> tuple[bool, Any, str]:
    """Wait for a rclpy future without nesting another executor spin."""
    event = threading.Event()
    future.add_done_callback(lambda _: event.set())
    if not event.wait(timeout=max(0.0, timeout_s)):
        return False, None, "timeout"
    try:
        return True, future.result(), ""
    except Exception as exc:  # pragma: no cover - depends on ROS runtime
        return False, None, str(exc)


class EventJournal:
    """Thread-safe in-memory event journal for the operator UI."""

    def __init__(self, max_items: int = 200):
        self._items: deque[dict[str, Any]] = deque(maxlen=max_items)
        self._lock = threading.Lock()
        self._sequence = 0

    def add(self, message: str, level: str = "info", source: str = "system", **extra: Any) -> None:
        with self._lock:
            self._sequence += 1
            item = {
                "id": self._sequence,
                "timestamp": time.time(),
                "level": level,
                "source": source,
                "message": message,
            }
            item.update(extra)
            self._items.append(item)

    def list(self, limit: int = 80) -> list[dict[str, Any]]:
        with self._lock:
            items = list(self._items)
        return items[-max(1, min(limit, 200)) :]


class RateTracker:
    """Compute a moving topic rate without relying on wall-clock jumps."""

    def __init__(self, window_s: float = 5.0):
        self._window_s = max(1.0, window_s)
        self._samples: deque[float] = deque()
        self._last_time = 0.0
        self._lock = threading.Lock()

    def tick(self) -> None:
        now = time.monotonic()
        with self._lock:
            self._last_time = now
            self._samples.append(now)
            cutoff = now - self._window_s
            while self._samples and self._samples[0] < cutoff:
                self._samples.popleft()

    def snapshot(self) -> dict[str, float | bool]:
        now = time.monotonic()
        with self._lock:
            samples = list(self._samples)
            last = self._last_time
        if len(samples) >= 2:
            elapsed = max(1e-6, samples[-1] - samples[0])
            hz = (len(samples) - 1) / elapsed
        else:
            hz = 0.0
        age = now - last if last > 0 else -1.0
        return {
            "hz": round(hz, 2),
            "age_s": round(age, 3) if age >= 0 else -1.0,
            "alive": bool(last > 0 and age <= max(2.0, self._window_s)),
        }


class RemoteRateState:
    """Lightweight cache for rate data received from external topic_rate_monitor.

    Maintains the same snapshot() interface as RateTracker so existing callers
    (mapping start worker, status()) need zero changes.

    If no update arrives within stale_timeout_s, snapshot() returns
    hz=0.0 / alive=False — so consumers can detect a crashed monitor node.
    """

    def __init__(self, stale_timeout_s: float = 2.0) -> None:
        self._lock = threading.Lock()
        self._stale_timeout_s = max(1.0, stale_timeout_s)
        self._received_at = 0.0
        self._data: dict[str, Any] = {
            "hz": 0.0,
            "alive": False,
            "age_s": -1.0,
        }

    def update(self, data: dict[str, Any]) -> None:
        now = time.monotonic()
        with self._lock:
            self._received_at = now
            self._data = {
                "hz": float(data.get("hz", 0.0)),
                "alive": bool(data.get("alive", False)),
                "age_s": float(data.get("age_s", -1.0)),
            }

    def snapshot(self) -> dict[str, Any]:
        now = time.monotonic()
        with self._lock:
            data = dict(self._data)
            received_at = self._received_at
        monitor_age = now - received_at if received_at > 0.0 else -1.0
        if received_at <= 0.0 or monitor_age > self._stale_timeout_s:
            return {
                "hz": 0.0,
                "alive": False,
                "age_s": -1.0,
                "monitor_age_s": round(monitor_age, 3) if monitor_age >= 0.0 else -1.0,
            }
        data["monitor_age_s"] = round(monitor_age, 3)
        return data
