"""
zsl_driver_node — ZSL-1W 轮足钢镚 ROS 2 驱动节点。

架构对标铜锤 M1 的 robot_driver_node.cpp：
  cmd_vel → 定时 watchdog 发 move
  状态 → 10Hz 定时轮询发布
  姿态 → ROS 2 services

与铜锤的差异：
  - ZSL-1W SDK 无 IMU/Motion/Joint/Fault 推送 → 不发布这些
  - 无灯/头/摄像头控制
  - 无速度等级
  - 有 read_only 模式（安全闸门）

用法:
  ros2 run zsl_driver zsl_driver_node --ros-args \
    -p sdk_local_ip:=192.168.168.216 \
    -p sdk_local_port:=43988 \
    -p sdk_dog_ip:=192.168.168.168 \
    -p read_only:=true
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import BatteryState
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from std_msgs.msg import Bool, UInt32, Float32
from std_srvs.srv import Trigger, SetBool

from zsl_driver.sdk_wrapper import SdkWrapper, MODE_STAND, MODE_PASSIVE


class ZslDriverNode(Node):
    """ZSL-1W 驱动 ROS 2 节点，对标铜锤 RobotDriverNode。"""

    def __init__(self):
        super().__init__("zsl_driver_node")

        # ——— 参数 ———
        sdk_local_ip = self.declare_parameter("sdk_local_ip", "192.168.168.216").value
        sdk_local_port = self.declare_parameter("sdk_local_port", 43988).value
        sdk_dog_ip = self.declare_parameter("sdk_dog_ip", "192.168.168.168").value
        sdk_lib_dir = self.declare_parameter("sdk_lib_dir", "").value or None
        read_only = self.declare_parameter("read_only", True).value
        self._cmd_vel_timeout_ms = self.declare_parameter("cmd_vel_timeout_ms", 500).value
        self._speed_scale = self.declare_parameter("speed_scale", 1.0).value
        self._angular_scale = self.declare_parameter("angular_scale", 1.0).value
        cmd_rate = self.declare_parameter("cmd_vel_publish_rate", 50).value
        cmd_rate = max(10, min(cmd_rate, 100))
        state_rate = self.declare_parameter("state_publish_rate", 10.0).value

        # ——— SDK 封装 ———
        self._sdk = SdkWrapper(read_only=read_only, sdk_lib_dir=sdk_lib_dir)

        # ——— Service 工厂（对标铜锤 CreateServices） ———
        self._create_services()

        # ——— 连接 SDK ———
        self.get_logger().info(
            f"Connecting to {sdk_dog_ip}:{sdk_local_port} (local {sdk_local_ip}) ..."
        )
        if not self._sdk.connect(sdk_local_ip, sdk_local_port, sdk_dog_ip):
            self.get_logger().error("SDK connect failed. 节点继续运行，可 retry。")
        else:
            self.get_logger().info("SDK connected.")

        # ——— cmd_vel 订阅（对标铜锤 cmd_vel_sub_） ———
        self._cmd_vel_sub = self.create_subscription(
            Twist, "cmd_vel_safe", self._cmd_vel_cb, 10
        )
        self._last_cmd = Twist()
        self._cmd_received = False
        self._last_cmd_time = self.get_clock().now()

        # ——— 定时器 ———
        self._cmd_timer = self.create_timer(1.0 / cmd_rate, self._cmd_tick)
        self._state_timer = self.create_timer(1.0 / state_rate, self._publish_state)

        # ——— 状态 Topic 发布 ———
        self._pub_connection = self.create_publisher(Bool, "~/connection", 10)
        self._pub_read_only = self.create_publisher(Bool, "~/read_only", 10)
        self._pub_ctrl_mode = self.create_publisher(UInt32, "~/ctrl_mode", 10)
        self._pub_cmd_watchdog = self.create_publisher(Float32, "~/cmd_watchdog", 10)
        self._pub_battery = self.create_publisher(BatteryState, "~/battery", 10)
        self._pub_diag = self.create_publisher(DiagnosticArray, "~/status", 10)

        self.get_logger().info("ZslDriverNode ready.")
        if read_only:
            self.get_logger().warn("read_only=true — 运动指令被静默拦截！")

    # =========================================================================
    # Service 创建
    # =========================================================================

    def _create_services(self):
        """创建所有服务，对标铜锤的 CreateServices()。"""
        # 姿态
        self.create_service(Trigger, "~/stand_up", self._srv_stand_up)
        self.create_service(Trigger, "~/lie_down", self._srv_lie_down)
        self.create_service(Trigger, "~/crawl", self._srv_crawl)

        # 紧急
        self.create_service(Trigger, "~/emergency_stop", self._srv_emergency_stop)

        # 控制权（兼容铜锤接口，ZSL-1W 无实际操作）
        self.create_service(Trigger, "~/take_control", self._srv_take_control)
        self.create_service(Trigger, "~/release_control", self._srv_release_control)

        # read_only 开关（热切换保护，但需注意 adapter 本身不回调）
        self.create_service(SetBool, "~/set_read_only", self._srv_set_read_only)

    # ——— Service 回调 ———

    def _srv_stand_up(self, req, resp):
        ok = self._sdk.stand_up()
        resp.success = ok
        resp.message = "ok" if ok else "stand_up failed"
        return resp

    def _srv_lie_down(self, req, resp):
        ok = self._sdk.lie_down()
        resp.success = ok
        resp.message = "ok" if ok else "lie_down failed"
        return resp

    def _srv_crawl(self, req, resp):
        ok = self._sdk.crawl()
        resp.success = ok
        resp.message = "ok" if ok else "crawl failed"
        return resp

    def _srv_emergency_stop(self, req, resp):
        ok = self._sdk.emergency_stop()
        resp.success = ok
        resp.message = "ok" if ok else "emergency_stop failed"
        return resp

    def _srv_take_control(self, req, resp):
        resp.success = True
        resp.message = "no-op for ZSL-1W"
        return resp

    def _srv_release_control(self, req, resp):
        resp.success = True
        resp.message = "no-op for ZSL-1W"
        return resp

    def _srv_set_read_only(self, req, resp):
        self._sdk.set_read_only(req.data)
        resp.success = True
        resp.message = f"read_only={'true' if req.data else 'false'}"
        return resp

    # =========================================================================
    # cmd_vel 订阅 + watchdog（对标铜锤 CmdVelTick）
    # =========================================================================

    def _cmd_vel_cb(self, msg: Twist):
        self._last_cmd = msg
        self._cmd_received = True
        self._last_cmd_time = self.get_clock().now()

    def _cmd_tick(self):
        """
        定时发送速度指令。对标铜锤 CmdVelTick():
          age_ms < timeout → 发 move; 否则发零速。
        """
        if not self._sdk.connected:
            return
        if not self._cmd_received:
            return

        age_ms = (self.get_clock().now() - self._last_cmd_time).nanoseconds / 1e6
        if age_ms < self._cmd_vel_timeout_ms:
            vx = max(-1.0, min(1.0, self._last_cmd.linear.x * self._speed_scale))
            vy = max(-1.0, min(1.0, self._last_cmd.linear.y * self._speed_scale))
            wz = max(-1.0, min(1.0, self._last_cmd.angular.z * self._angular_scale))
        else:
            vx = vy = wz = 0.0

        self._sdk.move(vx, vy, wz)

    # =========================================================================
    # 状态发布（对标铜锤 PublishState）
    # =========================================================================

    def _publish_state(self):
        """10Hz 发布状态 Topic。"""
        snap = self._sdk.snapshot()
        now = self.get_clock().now()

        # connection (Bool)
        msg_conn = Bool(data=snap.connected)
        self._pub_connection.publish(msg_conn)

        # read_only (Bool)
        msg_ro = Bool(data=self._sdk.read_only)
        self._pub_read_only.publish(msg_ro)

        # ctrl_mode (UInt32)
        msg_mode = UInt32(data=snap.mode if snap.mode >= 0 else 0)
        self._pub_ctrl_mode.publish(msg_mode)

        # cmd_watchdog: cmd_vel age in seconds
        age_s = (now - self._last_cmd_time).nanoseconds / 1e9 if self._cmd_received else -1.0
        msg_wd = Float32(data=float(age_s))
        self._pub_cmd_watchdog.publish(msg_wd)

        # battery (BatteryState)
        msg_bat = BatteryState()
        msg_bat.header.stamp = now.to_msg()
        msg_bat.percentage = float(snap.battery)
        msg_bat.present = True
        self._pub_battery.publish(msg_bat)

        # status (DiagnosticArray)
        diag = DiagnosticArray()
        diag.header.stamp = now.to_msg()

        sdk_status = DiagnosticStatus()
        sdk_status.name = "zsl_driver/sdk"
        sdk_status.level = DiagnosticStatus.OK if snap.connected else DiagnosticStatus.ERROR
        sdk_status.message = "connected" if snap.connected else "disconnected"
        sdk_status.values = [
            KeyValue(key="connected", value=str(snap.connected)),
            KeyValue(key="read_only", value=str(self._sdk.read_only)),
            KeyValue(key="ctrl_mode", value=str(snap.mode)),
            KeyValue(key="battery_percent", value=f"{snap.battery:.1f}"),
            KeyValue(key="cmd_watchdog_s", value=f"{age_s:.3f}"),
        ]

        cmd_status = DiagnosticStatus()
        cmd_status.name = "zsl_driver/cmd_vel"
        if age_s < 0:
            cmd_status.level = DiagnosticStatus.WARN
            cmd_status.message = "no cmd_vel received yet"
        elif age_s > self._cmd_vel_timeout_ms / 1000.0:
            cmd_status.level = DiagnosticStatus.WARN
            cmd_status.message = f"cmd_vel timeout ({age_s:.2f}s > {self._cmd_vel_timeout_ms}ms)"
        else:
            cmd_status.level = DiagnosticStatus.OK
            cmd_status.message = f"cmd_vel active (age={age_s:.3f}s)"
        cmd_status.values = [
            KeyValue(key="last_cmd_age_s", value=f"{age_s:.3f}"),
            KeyValue(key="timeout_ms", value=str(self._cmd_vel_timeout_ms)),
        ]

        diag.status = [sdk_status, cmd_status]
        self._pub_diag.publish(diag)

    # =========================================================================
    # 生命周期
    # =========================================================================

    def destroy_node(self):
        self._sdk.disconnect()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ZslDriverNode()
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
