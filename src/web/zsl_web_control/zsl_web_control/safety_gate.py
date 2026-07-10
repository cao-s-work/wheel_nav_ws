"""
safety_gate.py — 速度安全闸门。

- 限幅：vx/vy 在 [-1,1], wz 在 [-1,1]
- deadman：外部心跳 300ms 无刷新 → 自动零速
- read_only：闸门关闭时速度归零
- 速度档位: slow=0.3, normal=0.6, fast=1.0
"""
import time
import threading
from dataclasses import dataclass, field


SPEED_LEVELS = {
    "slow": 0.3,
    "normal": 0.6,
    "fast": 1.0,
}


@dataclass
class SafetyGate:
    """线程安全的速度闸门。"""

    # 限幅约束
    vx_max: float = 1.0
    vy_max: float = 1.0
    wz_max: float = 1.0

    # deadman 开关
    deadman_timeout_s: float = 0.3  # 300ms 无心跳归零

    # 档位
    speed_level: str = "slow"

    # 横移开关
    lateral_enabled: bool = False

    # 内部状态
    _lock: threading.Lock = field(default_factory=threading.Lock)
    _last_heartbeat: float = 0.0
    _read_only: bool = True
    _last_vx: float = 0.0
    _last_vy: float = 0.0
    _last_wz: float = 0.0

    # =========================================================================
    # 心跳
    # =========================================================================

    def heartbeat(self):
        """外部（WebSocket/Browser）定时调用，维持 deadman。"""
        with self._lock:
            self._last_heartbeat = time.time()

    @property
    def heartbeat_alive(self) -> bool:
        with self._lock:
            return (time.time() - self._last_heartbeat) < self.deadman_timeout_s

    # =========================================================================
    # read_only 闸门
    # =========================================================================

    @property
    def read_only(self) -> bool:
        with self._lock:
            return self._read_only

    @read_only.setter
    def read_only(self, val: bool):
        with self._lock:
            self._read_only = val

    # =========================================================================
    # 速度过滤主入口
    # =========================================================================

    def filter(self, vx: float, vy: float, wz: float) -> tuple[float, float, float]:
        """
        返回安全过滤后的 (vx, vy, wz)。
        - read_only → (0,0,0)
        - deadman 超时 → (0,0,0)
        - lateral_enabled=False → vy=0
        - 限幅 clamped
        """
        with self._lock:
            if self._read_only:
                return (0.0, 0.0, 0.0)

            if not self.heartbeat_alive:
                return (0.0, 0.0, 0.0)

            scale = SPEED_LEVELS.get(self.speed_level, 0.3)

            vx = max(-self.vx_max, min(self.vx_max, vx)) * scale
            vy = max(-self.vy_max, min(self.vy_max, vy)) * scale
            wz = max(-self.wz_max, min(self.wz_max, wz)) * scale

            if not self.lateral_enabled:
                vy = 0.0

            self._last_vx = vx
            self._last_vy = vy
            self._last_wz = wz

            return (vx, vy, wz)
