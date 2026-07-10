"""Web-side teleoperation safety gate.

This is an additional guard. The final robot safety limit must still be enforced by
cmd_vel_safety before zsl_driver.
"""
from __future__ import annotations

import threading
import time

from .utils import clamp


class SafetyGate:
    def __init__(
        self,
        deadman_timeout_s: float = 0.35,
        max_vx: float = 0.30,
        max_reverse: float = 0.15,
        max_vy: float = 0.0,
        max_wz: float = 0.50,
    ):
        self.deadman_timeout_s = max(0.1, float(deadman_timeout_s))
        self.max_vx = abs(float(max_vx))
        self.max_reverse = abs(float(max_reverse))
        self.max_vy = abs(float(max_vy))
        self.max_wz = abs(float(max_wz))
        self._last_teleop = 0.0
        self._read_only = True
        self._lock = threading.Lock()

    @property
    def read_only(self) -> bool:
        with self._lock:
            return self._read_only

    @read_only.setter
    def read_only(self, value: bool) -> None:
        with self._lock:
            self._read_only = bool(value)

    def teleop_heartbeat(self) -> None:
        with self._lock:
            self._last_teleop = time.monotonic()

    @property
    def teleop_alive(self) -> bool:
        now = time.monotonic()
        with self._lock:
            last = self._last_teleop
        return last > 0.0 and now - last <= self.deadman_timeout_s

    def filter(self, vx: float, vy: float, wz: float) -> tuple[float, float, float]:
        now = time.monotonic()
        with self._lock:
            read_only = self._read_only
            alive = self._last_teleop > 0.0 and now - self._last_teleop <= self.deadman_timeout_s
        if read_only or not alive:
            return 0.0, 0.0, 0.0
        return (
            clamp(float(vx), -self.max_reverse, self.max_vx),
            clamp(float(vy), -self.max_vy, self.max_vy),
            clamp(float(wz), -self.max_wz, self.max_wz),
        )
