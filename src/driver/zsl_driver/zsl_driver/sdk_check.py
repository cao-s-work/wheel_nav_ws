#!/usr/bin/env python3
"""
sdk_check.py — ZSL-1W SDK 诊断节点。

检查项：
  1. SDK .so 是否存在 → diagnostics "sdk_lib"
  2. Python binding 可导入 → diagnostics "sdk_import"
  3. ping 狗端 IP → diagnostics "network"
  4. SDK 连接 (checkConnect) → diagnostics "connection"
  5. 汇总 → /diagnostics

用法:
  ros2 run zsl_driver sdk_check --ros-args \
    -p sdk_dog_ip:=192.168.168.168 \
    -p sdk_local_ip:=192.168.168.216 \
    -p sdk_lib_dir:=/path/to/sdk
"""

import os
import sys
import subprocess
import time

import rclpy
from rclpy.node import Node
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

# 设置 SDK 路径（优先环境变量 ZSL_SDK_LIB_DIR，否则用默认路径）
_SDK_LIB_DIR = os.environ.get(
    "ZSL_SDK_LIB_DIR",
    os.path.join(os.path.dirname(os.path.abspath(__file__)), "sdk_lib"),
)
if _SDK_LIB_DIR not in sys.path:
    sys.path.insert(0, _SDK_LIB_DIR)


class SdkChecker(Node):
    def __init__(self):
        super().__init__("sdk_check")

        self._sdk_local_ip = self.declare_parameter("sdk_local_ip", "192.168.168.216").value
        self._sdk_dog_ip = self.declare_parameter("sdk_dog_ip", "192.168.168.168").value
        self._sdk_lib_dir = self.declare_parameter("sdk_lib_dir", _SDK_LIB_DIR).value

        self._pub = self.create_publisher(DiagnosticArray, "/diagnostics", 10)
        self._timer = self.create_timer(2.0, self._run_checks)

        self.get_logger().info(f"sdk_check ready. lib_dir={self._sdk_lib_dir}")
        self.get_logger().info(f"local={self._sdk_local_ip} dog={self._sdk_dog_ip}")

    def _run_checks(self):
        diag = DiagnosticArray()
        diag.header.stamp = self.get_clock().now().to_msg()

        # 1) SDK .so 检查
        so_ok, so_msg = self._check_so()
        diag.status.append(self._make_status("sdk_lib", so_ok, so_msg))

        # 2) Python binding 检查
        imp_ok, imp_msg = self._check_import()
        diag.status.append(self._make_status("sdk_import", imp_ok, imp_msg))

        # 3) 网络 ping 检查
        net_ok, net_msg = self._check_ping()
        diag.status.append(self._make_status("network", net_ok, net_msg))

        # 4) SDK 连接检查
        conn_ok, conn_msg = self._check_connection()
        diag.status.append(self._make_status("connection", conn_ok, conn_msg))

        # 5) 汇总
        all_ok = so_ok and imp_ok and net_ok and conn_ok
        diag.status.append(self._make_status("summary", all_ok, "ALL OK" if all_ok else "FAILURES"))

        self._pub.publish(diag)
        self.get_logger().info(
            f"so={so_ok} imp={imp_ok} net={net_ok} conn={conn_ok} → {'OK' if all_ok else 'FAIL'}"
        )

    def _make_status(self, name: str, ok: bool, msg: str) -> DiagnosticStatus:
        s = DiagnosticStatus()
        s.name = name
        s.level = DiagnosticStatus.OK if ok else DiagnosticStatus.ERROR
        s.message = msg
        s.values = [KeyValue(key="lib_dir", value=str(self._sdk_lib_dir))]
        return s

    def _check_so(self) -> tuple[bool, str]:
        try:
            files = [f for f in os.listdir(self._sdk_lib_dir) if f.endswith(".so")]
            if not files:
                return False, f"no .so in {self._sdk_lib_dir}"
            binding = [f for f in files if "mc_sdk_zsl_1w_py" in f]
            if not binding:
                return False, f"no Python binding .so in {self._sdk_lib_dir}"
            return True, f"{len(files)} .so files found"
        except FileNotFoundError:
            return False, f"sdk_lib_dir not found: {self._sdk_lib_dir}"
        except Exception as e:
            return False, str(e)

    def _check_import(self) -> tuple[bool, str]:
        try:
            # Force fresh import
            if "mc_sdk_zsl_1w_py" in sys.modules:
                del sys.modules["mc_sdk_zsl_1w_py"]
            import mc_sdk_zsl_1w_py
            return True, f"import OK: {mc_sdk_zsl_1w_py.__file__}"
        except Exception as e:
            return False, str(e)

    def _check_ping(self) -> tuple[bool, str]:
        try:
            r = subprocess.run(
                ["ping", "-c", "1", "-W", "2", self._sdk_dog_ip],
                capture_output=True, text=True, timeout=3,
            )
            ok = r.returncode == 0
            return ok, "reachable" if ok else r.stderr.strip() or "timeout"
        except subprocess.TimeoutExpired:
            return False, "ping timeout"
        except Exception as e:
            return False, str(e)

    def _check_connection(self) -> tuple[bool, str]:
        try:
            import mc_sdk_zsl_1w_py
            app = mc_sdk_zsl_1w_py.HighLevel()
            app.initRobot(self._sdk_local_ip, 43988, self._sdk_dog_ip)
            time.sleep(2)
            if app.checkConnect():
                mode = app.getCurrentCtrlmode()
                bat = app.getBatteryPower()
                return True, f"connected, mode={mode}, battery={bat}%"
            return False, "checkConnect() False"
        except Exception as e:
            return False, str(e)


def main(args=None):
    rclpy.init(args=args)
    node = SdkChecker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        node.get_logger().error(f"Crashed: {e}")
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass
