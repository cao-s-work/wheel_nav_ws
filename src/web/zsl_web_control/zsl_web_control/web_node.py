"""
web_node.py — ZSL-1W Web 控制网关主节点。

架构:
  Browser UI → WebSocket (实时) + REST (按钮)
  → zsl_web_control_node (本节点)
  → ros_api / nav2_client / safety_gate
  → ROS 2 topics/services/actions

安全:
  - deadman: 300ms 无心跳 → 零速
  - WebSocket 断开 → 零速
  - read_only 闸门
  - 所有速度经过 safety_gate.filter()
"""
import os
import json
import threading
import time
import asyncio

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, UInt32, Float32

# Web 框架
try:
    from aiohttp import web
    HAS_AIOHTTP = True
except ImportError:
    HAS_AIOHTTP = False

from zsl_web_control.ros_api import RosApi
from zsl_web_control.nav2_client import Nav2Client
from zsl_web_control.mapping_manager import MappingManager
from zsl_web_control.safety_gate import SafetyGate

# =============================================================================
# WebSocket 管理
# =============================================================================

class WebSocketManager:
    """管理所有活跃 WebSocket 连接，广播状态。"""

    def __init__(self):
        self._lock = threading.Lock()
        self._sockets: set = set()

    def add(self, ws):
        with self._lock:
            self._sockets.add(ws)

    def remove(self, ws):
        with self._lock:
            self._sockets.discard(ws)

    @property
    def count(self) -> int:
        with self._lock:
            return len(self._sockets)

    async def broadcast(self, data: dict):
        msg = json.dumps(data)
        dead = set()
        with self._lock:
            sockets = list(self._sockets)
        for ws in sockets:
            try:
                await ws.send_str(msg)
            except Exception:
                dead.add(ws)
        with self._lock:
            self._sockets -= dead

# =============================================================================
# 主节点
# =============================================================================

class WebControlNode(Node):
    def __init__(self):
        super().__init__("zsl_web_control_node")

        # 参数
        self.declare_parameter("port", 8080)
        self.declare_parameter("host", "127.0.0.1")
        self.declare_parameter("static_dir", "")
        port = self.get_parameter("port").value
        host = self.get_parameter("host").value
        static_dir = self.get_parameter("static_dir").value or os.path.join(
            os.path.dirname(os.path.abspath(__file__)), "..", "static"
        )
        static_dir = os.path.abspath(static_dir)

        # 子模块
        self._ros_api = RosApi(self)
        self._nav2_client = Nav2Client(self)
        self._mapping = MappingManager(self)
        self._safety = SafetyGate()

        # WebSocket 管理
        self._ws_mgr = WebSocketManager()

        # cmd_vel publisher (到 cmd_vel_mux)
        self._cmd_pub = self.create_publisher(Twist, "cmd_vel_teleop", 10)

        # 状态发布定时器 10Hz
        self._state_timer = self.create_timer(0.1, self._publish_state)

        # 死区心跳定时器
        self._heartbeat_timer = self.create_timer(0.05, self._check_deadman)

        # 当前 teleop 指令
        self._teleop_vx = 0.0
        self._teleop_vy = 0.0
        self._teleop_wz = 0.0
        self._teleop_lock = threading.Lock()

        # Web 服务器线程
        self._web_thread = None
        self._app = None
        self._runner = None

        self.get_logger().info(f"WebControlNode starting on {host}:{port}")

        if not HAS_AIOHTTP:
            self.get_logger().warn("aiohttp not installed. Web server disabled. "
                                   "Install: /usr/bin/pip3.10 install aiohttp")
            return

        # 启动 aiohttp 在独立线程
        self._web_thread = threading.Thread(
            target=self._run_web, args=(host, port, static_dir), daemon=True)
        self._web_thread.start()

    # =========================================================================
    # Web 服务器 (aiohttp)
    # =========================================================================

    def _run_web(self, host, port, static_dir):
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)

        app = web.Application()
        app["node"] = self

        # WebSocket
        app.router.add_get("/ws", self._ws_handler)

        # REST API
        app.router.add_post("/api/stand", self._api_stand)
        app.router.add_post("/api/lie", self._api_lie)
        app.router.add_post("/api/crawl", self._api_crawl)
        app.router.add_post("/api/estop", self._api_estop)
        app.router.add_post("/api/read_only", self._api_read_only)
        app.router.add_post("/api/heartbeat", self._api_heartbeat)
        app.router.add_post("/api/teleop", self._api_teleop)
        app.router.add_get("/api/state", self._api_state)
        app.router.add_post("/api/nav/goal", self._api_nav_goal)
        app.router.add_post("/api/nav/cancel", self._api_nav_cancel)
        app.router.add_post("/api/nav/clear_costmaps", self._api_nav_clear)
        app.router.add_get("/api/mapping/status", self._api_mapping_status)

        # 静态文件
        if os.path.isdir(static_dir):
            app.router.add_static("/", static_dir, show_index=True)

        # 用 AppRunner + TCPSite 代替 web.run_app（避免主线程信号限制）
        runner = web.AppRunner(app)
        loop.run_until_complete(runner.setup())
        site = web.TCPSite(runner, host, port)
        loop.run_until_complete(site.start())
        self.get_logger().info(f"Web server started on http://{host}:{port}")
        loop.run_forever()

    # ---- WebSocket ----

    async def _ws_handler(self, request):
        ws = web.WebSocketResponse()
        await ws.prepare(request)
        self._ws_mgr.add(ws)
        try:
            async for msg in ws:
                if msg.type == web.WSMsgType.TEXT:
                    try:
                        data = json.loads(msg.data)
                        await self._handle_ws_msg(ws, data)
                    except Exception:
                        pass
                elif msg.type == web.WSMsgType.ERROR:
                    break
        finally:
            self._ws_mgr.remove(ws)
            # 连接断开 → 清零该客户端的 teleop 指令
            with self._teleop_lock:
                self._teleop_vx = 0.0
                self._teleop_vy = 0.0
                self._teleop_wz = 0.0
        return ws

    async def _handle_ws_msg(self, ws, data):
        msg_type = data.get("type", "")
        if msg_type == "heartbeat":
            self._safety.heartbeat()
        elif msg_type == "teleop":
            vx = float(data.get("vx", 0))
            vy = float(data.get("vy", 0))
            wz = float(data.get("wz", 0))
            with self._teleop_lock:
                self._teleop_vx = vx
                self._teleop_vy = vy
                self._teleop_wz = wz
            self._safety.teleop_heartbeat()

    # ---- REST API ----

    async def _api_stand(self, request):
        return web.json_response(self._ros_api.stand_up())

    async def _api_lie(self, request):
        return web.json_response(self._ros_api.lie_down())

    async def _api_crawl(self, request):
        return web.json_response(self._ros_api.crawl())

    async def _api_estop(self, request):
        return web.json_response(self._ros_api.emergency_stop())

    async def _api_read_only(self, request):
        body = await request.json()
        ro = bool(body.get("read_only", True))
        self._safety.read_only = ro
        return web.json_response(self._ros_api.set_read_only(ro))

    async def _api_heartbeat(self, request):
        self._safety.heartbeat()
        return web.json_response({"ok": True})

    async def _api_teleop(self, request):
        body = await request.json()
        vx = float(body.get("vx", 0))
        vy = float(body.get("vy", 0))
        wz = float(body.get("wz", 0))
        with self._teleop_lock:
            self._teleop_vx = vx
            self._teleop_vy = vy
            self._teleop_wz = wz
        self._safety.teleop_heartbeat()
        return web.json_response({"ok": True})

    async def _api_state(self, request):
        return web.json_response(self._ros_api.summary())

    async def _api_nav_goal(self, request):
        body = await request.json()
        x = float(body.get("x", 0))
        y = float(body.get("y", 0))
        yaw = float(body.get("yaw", 0))
        ok = self._nav2_client.send_goal(x, y, yaw)
        return web.json_response({"ok": ok})

    async def _api_nav_cancel(self, request):
        ok = self._nav2_client.cancel_goal()
        return web.json_response({"ok": ok})

    async def _api_nav_clear(self, request):
        ok = self._nav2_client.clear_costmaps()
        return web.json_response({"ok": ok})

    async def _api_mapping_status(self, request):
        return web.json_response(self._mapping.get_status())

    # =========================================================================
    # 定时器回调
    # =========================================================================

    def _publish_state(self):
        """10Hz 状态广播 + cmd_vel 发布。"""
        # 安全过滤
        with self._teleop_lock:
            vx, vy, wz = self._safety.filter(
                self._teleop_vx, self._teleop_vy, self._teleop_wz)
        # 发布 cmd_vel
        twist = Twist()
        twist.linear.x = vx
        twist.linear.y = vy
        twist.angular.z = wz
        self._cmd_pub.publish(twist)

    def _check_deadman(self):
        """20Hz deadman 检查 — 速度心跳超时归零 teleop。"""
        if not self._safety.teleop_alive:
            with self._teleop_lock:
                self._teleop_vx = 0.0
                self._teleop_vy = 0.0
                self._teleop_wz = 0.0

    # =========================================================================
    # 生命周期
    # =========================================================================

    def destroy_node(self):
        self._cmd_pub.publish(Twist())  # 零速
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = WebControlNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
