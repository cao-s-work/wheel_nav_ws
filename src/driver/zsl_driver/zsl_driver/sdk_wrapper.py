"""
zsl_driver.sdk_wrapper — 封装 mc_sdk_zsl_1w_py SDK，提供线程安全的数据缓存。

架构对标铜锤 M1 的 RobotDriver，但适配 ZSL-1W 轮足 SDK：
- initRobot → connect
- standUp/standDown/passive → 姿态控制
- move(vx,vy,wz) → cmd_vel 映射
- getCurrentCtrlmode/getBatteryPower/checkConnect → 状态轮询

SDK 库路径: ~/gb_ws2/sdk/genisom_l1_sdk-main/lib/zsl-1w/aarch64/
Python 模块: mc_sdk_zsl_1w_py (编译于 Python 3.10)
"""

import os
import sys
import time
import threading
from dataclasses import dataclass


# ——— SDK 库路径解析（优先级：参数 sdk_lib_dir > 环境变量 > 自动识别 > 包内 fallback） ———
import ctypes as _ctypes
import platform as _platform


def _resolve_sdk_lib_dir(explicit_dir: str | None = None) -> str | None:
    """
    三级 fallback 解析 SDK .so 目录，按优先级返回第一个有效的路径。
    Returns: 有效目录绝对路径，或 None。
    """
    candidates: list[str] = []

    # 1) 显式参数（最高优先级）
    if explicit_dir:
        candidates.append(os.path.abspath(explicit_dir))

    # 2) 环境变量
    env_dir = os.environ.get("ZSL_SDK_LIB_DIR")
    if env_dir:
        candidates.append(os.path.abspath(env_dir))

    # 3) 自动识别 aarch64 / x86_64
    arch = _platform.machine()
    auto_path = os.path.join(
        os.path.expanduser("~"),
        "gb_ws2", "sdk", "genisom_l1_sdk-main", "lib", "zsl-1w", arch,
    )
    candidates.append(auto_path)

    # 4) 包内 sdk_lib（fallback）
    builtin = os.path.join(os.path.dirname(os.path.abspath(__file__)), "sdk_lib")
    candidates.append(builtin)

    for d in candidates:
        d = os.path.abspath(d)
        py_binding = os.path.join(d, f"mc_sdk_zsl_1w_py.cpython-{sys.version_info.major}{sys.version_info.minor}-{arch}-linux-gnu.so")
        if os.path.isfile(py_binding):
            return d

    return None  # 全部未找到


def _load_sdk(sdk_lib_dir: str):
    """延迟加载 SDK 模块（在 SdkWrapper.connect() 时调用）。"""
    if sdk_lib_dir not in sys.path:
        sys.path.insert(0, sdk_lib_dir)

    # 预加载 C++ 依赖 so
    try:
        for _fname in os.listdir(sdk_lib_dir):
            if _fname.startswith("libmc_sdk_") and _fname.endswith(".so"):
                _ctypes.CDLL(os.path.join(sdk_lib_dir, _fname), mode=_ctypes.RTLD_GLOBAL)
    except Exception:
        pass

    # 注入 LD_LIBRARY_PATH
    os.environ["LD_LIBRARY_PATH"] = sdk_lib_dir + ":" + os.environ.get("LD_LIBRARY_PATH", "")

    import mc_sdk_zsl_1w_py as sdk
    return sdk


# ——— 模式码 ———
MODE_PASSIVE = 0
MODE_STAND = 1
MODE_MOVE = 18
MODE_LIE_DOWN = 51


@dataclass
class RobotDataCache:
    """线程安全的数据缓存，对标铜锤的 RobotDataCache。"""
    lock: threading.Lock

    # 连接状态
    connected: bool = False
    read_only: bool = True

    # 机器人状态
    mode: int = 0              # 0=PASSIVE, 1=STAND, 18=MOVE, 51=LIE_DOWN
    battery: float = 0.0       # 电量百分比
    mode_updated: bool = False

    # 控制超时
    control_lost: bool = False
    control_available: bool = False


class SdkWrapper:
    """
    ZSL-1W SDK 的高层封装，线程安全。

    对标铜锤的 RobotDriver，但 ZSL-1W SDK 没有：
    - IMU/JointState/MotionData 推送 → 不在本节点处理
    - TakeControl/ReleaseControl 概念 → 无操作
    - 灯/头/姿态/摄像头 → 不支持
    """

    def __init__(self, read_only: bool = True, sdk_lib_dir: str | None = None):
        self._read_only = read_only
        self._app = None
        self._sdk = None  # 延迟加载
        self._cache = RobotDataCache(lock=threading.Lock())
        self._sdk_lib_dir = _resolve_sdk_lib_dir(sdk_lib_dir)

    # =========================================================================
    # 连接管理
    # =========================================================================

    def connect(self, local_ip: str, local_port: int, dog_ip: str, timeout: float = 5.0) -> bool:
        """
        初始化 SDK 连接。
        对标铜锤 client_->Connect(ip, port)。
        首次调用时延迟加载 SDK。
        """
        if self._sdk is None:
            if self._sdk_lib_dir is None:
                raise RuntimeError(
                    "ZSL-1W SDK .so 未找到！\n"
                    "  请通过以下任一方式指定：\n"
                    "  1) 参数: sdk_lib_dir:=/path/to/sdk\n"
                    "  2) 环境变量: export ZSL_SDK_LIB_DIR=/path/to/sdk\n"
                    "  3) 将 .so 放入 ~/gb_ws2/sdk/.../lib/zsl-1w/{arch}/\n"
                    "  4) 放入本包 zsl_driver/sdk_lib/ 目录\n"
                    f"  当前架构: {_platform.machine()}, Python {sys.version_info.major}.{sys.version_info.minor}"
                )
            self._sdk = _load_sdk(self._sdk_lib_dir)

        try:
            self._app = self._sdk.HighLevel()
            self._app.initRobot(local_ip, local_port, dog_ip)
            # 等待握手完成
            deadline = time.time() + timeout
            while time.time() < deadline:
                if self._app.checkConnect():
                    with self._cache.lock:
                        self._cache.connected = True
                    return True
                time.sleep(0.2)
            # 超时但 initRobot 没抛异常 → 可能已连接，再查一次
            connected = self._app.checkConnect()
            with self._cache.lock:
                self._cache.connected = connected
            return connected
        except Exception as e:
            print(f"[SdkWrapper] connect failed: {e}")
            return False

    def disconnect(self):
        """释放 SDK。"""
        self._app = None
        with self._cache.lock:
            self._cache.connected = False

    @property
    def connected(self) -> bool:
        with self._cache.lock:
            return self._cache.connected

    @property
    def read_only(self) -> bool:
        return self._read_only

    # =========================================================================
    # 姿态控制（对标铜锤 StandUp/LieDown/Crawl）
    # =========================================================================

    def stand_up(self) -> bool:
        """站立。ZSL-1W: standUp()。"""
        if not self._app or self._read_only:
            return False
        try:
            self._app.standUp()
            return True
        except Exception as e:
            print(f"[SdkWrapper] stand_up failed: {e}")
            return False

    def lie_down(self) -> bool:
        """
        安全趴下：先 crawl 匍匐 → cancelCrawl → passive。
        对标铜锤 LieDown。
        """
        if not self._app or self._read_only:
            return False
        try:
            self._app.crawl(0.3, 0.0, 0.0)
            time.sleep(1.5)
            self._app.cancelCrawl()
            time.sleep(0.5)
            self._app.passive()
            return True
        except Exception as e:
            print(f"[SdkWrapper] lie_down failed: {e}")
            return False

    def crawl(self) -> bool:
        """匍匐/下蹲。ZSL-1W: crawl(vx,vy,yaw_rate) 慢速匍匐。"""
        if not self._app or self._read_only:
            return False
        try:
            self._app.crawl(0.3, 0.0, 0.0)
            return True
        except Exception as e:
            print(f"[SdkWrapper] crawl failed: {e}")
            return False

    # =========================================================================
    # 运动控制（对标铜锤 Move(lr, fb, yaw)）
    # =========================================================================

    def move(self, vx: float, vy: float, wz: float) -> bool:
        """
        发送速度指令。
        铜锤: Move(left_right, forward_back, yaw)
        ZSL-1W: move(vx_forward, vy_lateral, wz_angular)

        参数映射：
          cmd_vel.linear.x  → vx (前进)
          cmd_vel.linear.y  → vy (左移)
          cmd_vel.angular.z → wz (逆时针)
        """
        if not self._app or self._read_only:
            return False
        try:
            self._app.move(vx, vy, wz)
            return True
        except Exception as e:
            print(f"[SdkWrapper] move({vx:.2f},{vy:.2f},{wz:.2f}) failed: {e}")
            return False

    def stop(self) -> bool:
        """停车。"""
        return self.move(0.0, 0.0, 0.0)

    # =========================================================================
    # 急停
    # =========================================================================

    def stop_force(self) -> bool:
        """发零速度，绕过 read_only 检查。"""
        if not self._app:
            return False
        try:
            self._app.move(0.0, 0.0, 0.0)
            return True
        except Exception:
            return False

    def set_read_only(self, ro: bool) -> bool:
        """上锁：先发零速度再置位，避免锁住运动中的狗。
        解锁：需 SDK 已连接。
        返回 True 表示状态变更成功。"""
        if ro:
            stopped = self.stop_force()
            # 无论停车是否成功，先阻止后续普通 move
            self._read_only = True
            if not stopped:
                return False
            return True
        if not self.connected:
            return False
        self._read_only = False
        return True

    def emergency_stop(self) -> bool:
        """急停：立即零速 + passive，不允许 sleep/crawl。"""
        if not self._app:
            return False
        try:
            self._app.move(0.0, 0.0, 0.0)
        except Exception:
            pass
        try:
            self._app.passive()
            return True
        except Exception as exc:
            print(f"[SdkWrapper] emergency_stop failed: {exc}")
            return False

    # =========================================================================
    # 控制权（ZSL-1W 无此概念，保留接口兼容）
    # =========================================================================

    def take_control(self) -> bool:
        """ZSL-1W 无需抢控制权。"""
        return True

    def release_control(self) -> bool:
        """ZSL-1W 无需释放控制权。"""
        return True

    # =========================================================================
    # 状态快照（对标铜锤 SnapshotData）
    # =========================================================================

    def snapshot(self) -> RobotDataCache:
        """
        线程安全地获取最新状态快照。
        对标铜锤的 RobotDriver::SnapshotData()。
        """
        if not self._app:
            with self._cache.lock:
                self._cache.connected = False
            return self._cache

        try:
            mode = self._app.getCurrentCtrlmode()
            battery = self._app.getBatteryPower()
        except Exception:
            mode = -1
            battery = 0.0

        with self._cache.lock:
            self._cache.mode = mode
            self._cache.battery = battery
            self._cache.mode_updated = True
            self._cache.connected = self._app.checkConnect() if self._app else False

        return self._cache
