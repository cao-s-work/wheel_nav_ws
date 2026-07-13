"""Commercial ZSL-1W ROS 2 Web gateway.

The browser never receives a generic ROS shell. Every operation is mapped to a
fixed ROS topic, service, action, or an allow-listed command configured as a ROS
parameter.
"""
from __future__ import annotations

import asyncio
import json
import os
import threading
import time
import uuid
from pathlib import Path
from typing import Any

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool

try:
    from aiohttp import WSMsgType, web
except ImportError:  # pragma: no cover - deployment dependency
    web = None
    WSMsgType = None

from .live_map_bridge import LiveMapBridge
from .mapping_manager import MappingManager
from .nav2_client import Nav2Client
from .process_manager import ProcessManager
from .ros_api import RosApi
from .safety_gate import SafetyGate
from .utils import EventJournal


class WebSocketHub:
    def __init__(self):
        self._sockets: dict[str, Any] = {}
        self._lock = threading.Lock()

    def add(self, client_id: str, socket: Any) -> None:
        with self._lock:
            self._sockets[client_id] = socket

    def remove(self, client_id: str) -> None:
        with self._lock:
            self._sockets.pop(client_id, None)

    def count(self) -> int:
        with self._lock:
            return len(self._sockets)

    async def broadcast(self, payload: dict[str, Any]) -> None:
        message = json.dumps(payload, ensure_ascii=False, separators=(",", ":"))
        with self._lock:
            sockets = list(self._sockets.items())
        dead: list[str] = []
        for client_id, socket in sockets:
            try:
                await socket.send_str(message)
            except Exception:
                dead.append(client_id)
        for client_id in dead:
            self.remove(client_id)


class WebControlNode(Node):
    def __init__(self):
        super().__init__("zsl_web_control_node")
        self._declare_parameters()

        self._journal = EventJournal(max_items=240)
        self._hub = WebSocketHub()
        self._state_lock = threading.RLock()
        self._teleop_lock = threading.Lock()

        log_root = str(self.get_parameter("managed_log_root").value)
        self._processes = ProcessManager(self._journal, log_root)
        self._ros_api = RosApi(self, self._journal)
        self._nav2 = Nav2Client(self, self._journal)
        self._mapping = MappingManager(self, self._journal, self._processes, self._nav2)
        self._live_map = LiveMapBridge(self)
        self._safety = SafetyGate(
            deadman_timeout_s=float(self.get_parameter("teleop_deadman_s").value),
            max_vx=float(self.get_parameter("web_max_vx").value),
            max_reverse=float(self.get_parameter("web_max_reverse").value),
            max_vy=float(self.get_parameter("web_max_vy").value),
            max_wz=float(self.get_parameter("web_max_wz").value),
        )

        self._manual_mode = False
        self._controller_id: str | None = None
        self._controller_expire = 0.0
        self._controller_lease_s = float(self.get_parameter("controller_lease_s").value)
        self._teleop = [0.0, 0.0, 0.0]

        self._cmd_pub = self.create_publisher(Twist, str(self.get_parameter("teleop_topic").value), 10)
        active_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._teleop_active_pub = self.create_publisher(
            Bool,
            str(self.get_parameter("teleop_active_topic").value),
            active_qos,
        )
        self._publish_control_mode()

        publish_rate = max(10.0, float(self.get_parameter("teleop_publish_rate").value))
        self.create_timer(1.0 / publish_rate, self._publish_teleop)
        self.create_timer(0.1, self._watch_control_lease)
        self.create_timer(0.5, self._broadcast_state_from_ros)

        self._web_loop: asyncio.AbstractEventLoop | None = None
        self._web_thread: threading.Thread | None = None
        self._runner = None
        self._start_web_server()
        self._journal.add("Commercial web gateway started", "success", "system")

    def _declare_parameters(self) -> None:
        defaults = {
            "host": "0.0.0.0",
            "port": 8080,
            "static_dir": "",
            "api_token": "",
            "driver_ns": "zsl_driver_node",
            "teleop_topic": "/cmd_vel_teleop",
            "teleop_active_topic": "/web/teleop_active",
            "initialpose_topic": "/initialpose",
            "amcl_pose_topic": "/amcl_pose",
            "odom_topic": "/Odometry",
            "map_topic": "/map",
            "lidar_topic": "/livox/lidar",
            "scan_topic": "/scan",
            "topic_rates_topic": "/system/topic_rates",
            "live_map_enabled": True,
            "live_map_frame": "map",
            "live_robot_frame": "base_link",
            "live_scan_max_points": 420,
            "live_path_max_points": 2400,
            "live_path_min_distance": 0.03,
            "live_map_image_rate_hz": 1.0,
            "global_localization_service": "/reinitialize_global_localization",
            "nomotion_update_service": "/request_nomotion_update",
            "map_save_service": "/map_saver/save_map",
            "map_load_service": "/map_server/load_map",
            "map_root": "~/gb_maps",
            "map_image_format": "png",
            "map_save_cli_fallback": True,
            "mapping_command": "ros2 launch zsl_bringup mapping.launch.py",
            "mapping_script_enabled": True,
            "mapping_script_path": "",
            "workspace_root": "",
            "mapping_ready_timeout_s": 60.0,
            "navigation_command": "ros2 launch zsl_bringup managed_navigation.launch.py map_file:={map}",
            "managed_log_root": "~/.ros/zsl_web_control/logs",
            "controller_lease_s": 5.0,
            "teleop_deadman_s": 0.35,
            "teleop_publish_rate": 20.0,
            "web_max_vx": 0.30,
            "web_max_reverse": 0.15,
            "web_max_vy": 0.0,
            "web_max_wz": 0.50,
            "stop_managed_processes_on_exit": True,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

    def _static_root(self) -> Path:
        configured = str(self.get_parameter("static_dir").value).strip()
        if configured:
            return Path(os.path.expanduser(configured)).resolve()
        return Path(get_package_share_directory("zsl_web_control")) / "static"

    def _start_web_server(self) -> None:
        if web is None:
            self.get_logger().error("python3-aiohttp is not installed; web server disabled")
            return
        host = str(self.get_parameter("host").value)
        port = int(self.get_parameter("port").value)
        token = str(self.get_parameter("api_token").value)
        if host not in {"127.0.0.1", "localhost", "::1"} and not token:
            self.get_logger().warning("Web server is exposed beyond localhost without api_token")
        self._web_thread = threading.Thread(
            target=self._run_web,
            args=(host, port),
            daemon=True,
            name="zsl-web-server",
        )
        self._web_thread.start()

    def _run_web(self, host: str, port: int) -> None:
        loop = asyncio.new_event_loop()
        self._web_loop = loop
        asyncio.set_event_loop(loop)
        token = str(self.get_parameter("api_token").value)

        @web.middleware
        async def auth_middleware(request, handler):
            if not token or request.path in {"/", "/healthz", "/style.css", "/app.js"}:
                return await handler(request)
            supplied = request.headers.get("Authorization", "")
            bearer = supplied[7:] if supplied.startswith("Bearer ") else ""
            query_token = request.query.get("token", "")
            if bearer != token and query_token != token:
                if request.path == "/ws":
                    return web.Response(status=401, text="unauthorized")
                return web.json_response({"success": False, "message": "unauthorized"}, status=401)
            return await handler(request)

        app = web.Application(middlewares=[auth_middleware], client_max_size=1024 * 1024)
        self._register_routes(app)
        static_root = self._static_root()
        app.router.add_get("/", lambda _: web.FileResponse(static_root / "index.html"))
        app.router.add_get("/style.css", lambda _: web.FileResponse(static_root / "style.css"))
        app.router.add_get("/app.js", lambda _: web.FileResponse(static_root / "app.js"))

        runner = web.AppRunner(app, access_log=None)
        self._runner = runner
        loop.run_until_complete(runner.setup())
        site = web.TCPSite(runner, host, port)
        loop.run_until_complete(site.start())
        self.get_logger().info(f"Web console listening on http://{host}:{port}")
        loop.run_forever()

    def _register_routes(self, app) -> None:
        app.router.add_get("/healthz", self._api_health)
        app.router.add_get("/ws", self._ws_handler)
        app.router.add_get("/api/v1/state", self._api_state)
        app.router.add_get("/api/v1/diagnostics", self._api_diagnostics)
        app.router.add_get("/api/v1/events", self._api_events)

        app.router.add_post("/api/v1/robot/stand", self._api_stand)
        app.router.add_post("/api/v1/robot/lie", self._api_lie)
        app.router.add_post("/api/v1/robot/crawl", self._api_crawl)
        app.router.add_post("/api/v1/robot/estop", self._api_estop)
        app.router.add_post("/api/v1/robot/reset_estop", self._api_reset_estop)
        app.router.add_post("/api/v1/robot/read_only", self._api_read_only)
        app.router.add_post("/api/v1/control/mode", self._api_control_mode)

        app.router.add_get("/api/v1/maps", self._api_maps)
        app.router.add_post("/api/v1/maps/save", self._api_map_save)
        app.router.add_post("/api/v1/maps/load", self._api_map_load)
        app.router.add_delete("/api/v1/maps/{name}", self._api_map_delete)
        app.router.add_get("/api/v1/maps/{name}/preview", self._api_map_preview)

        app.router.add_get("/api/v1/mapping/status", self._api_mapping_status)
        app.router.add_post("/api/v1/mapping/start", self._api_mapping_start)
        app.router.add_post("/api/v1/mapping/stop", self._api_mapping_stop)
        app.router.add_get("/api/v1/live_mapping", self._api_live_mapping)
        app.router.add_get("/api/v1/live_mapping/map.png", self._api_live_map_png)
        app.router.add_post("/api/v1/live_mapping/reset_path", self._api_live_reset_path)

        app.router.add_get("/api/v1/navigation/status", self._api_nav_status)
        app.router.add_post("/api/v1/navigation/start", self._api_nav_start)
        app.router.add_post("/api/v1/navigation/stop", self._api_nav_stop)
        app.router.add_post("/api/v1/navigation/goal", self._api_nav_goal)
        app.router.add_post("/api/v1/navigation/waypoints", self._api_nav_waypoints)
        app.router.add_post("/api/v1/navigation/cancel", self._api_nav_cancel)
        app.router.add_post("/api/v1/navigation/clear_costmaps", self._api_nav_clear)
        app.router.add_post("/api/v1/localization/initial_pose", self._api_initial_pose)
        app.router.add_post("/api/v1/localization/global", self._api_global_localization)
        app.router.add_post("/api/v1/localization/nomotion_update", self._api_nomotion_update)

    def _publish_control_mode(self) -> None:
        self._teleop_active_pub.publish(Bool(data=self._manual_mode))

    def _set_manual_mode(self, active: bool, controller_id: str | None = None, cancel_nav: bool = True) -> None:
        changed = active != self._manual_mode
        self._manual_mode = bool(active)
        if active and controller_id:
            self._controller_id = controller_id
            self._controller_expire = time.monotonic() + self._controller_lease_s
        if not active:
            self._controller_id = None
            self._controller_expire = 0.0
        with self._teleop_lock:
            self._teleop = [0.0, 0.0, 0.0]
        self._cmd_pub.publish(Twist())
        self._publish_control_mode()
        if changed:
            self._journal.add(
                "Manual control hold enabled" if active else "Automatic navigation mode enabled",
                "warning" if active else "success",
                "control",
            )
        if active and cancel_nav:
            self._nav2.cancel_goal()

    def _claim_control(self, client_id: str) -> tuple[bool, str]:
        now = time.monotonic()
        with self._state_lock:
            if self._controller_id and self._controller_id != client_id and now <= self._controller_expire:
                return False, "another operator currently owns manual control"
            self._set_manual_mode(True, controller_id=client_id, cancel_nav=True)
        return True, "manual control acquired"

    def _release_control(self, client_id: str | None, switch_auto: bool) -> tuple[bool, str]:
        with self._state_lock:
            if self._controller_id and client_id and self._controller_id != client_id:
                return False, "manual control is owned by another operator"
            if switch_auto:
                self._set_manual_mode(False, cancel_nav=False)
            else:
                self._controller_id = None
                self._controller_expire = 0.0
                with self._teleop_lock:
                    self._teleop = [0.0, 0.0, 0.0]
                self._cmd_pub.publish(Twist())
        return True, "automatic mode enabled" if switch_auto else "manual control released; robot remains held"

    async def _ws_handler(self, request):
        socket = web.WebSocketResponse(heartbeat=15.0, receive_timeout=40.0)
        await socket.prepare(request)
        client_id = uuid.uuid4().hex
        self._hub.add(client_id, socket)
        await socket.send_json({"type": "hello", "client_id": client_id, "state": self._state_snapshot()})
        try:
            async for message in socket:
                if message.type == WSMsgType.TEXT:
                    try:
                        payload = json.loads(message.data)
                        await self._handle_ws_message(socket, client_id, payload)
                    except (ValueError, TypeError, json.JSONDecodeError) as exc:
                        await socket.send_json({"type": "error", "message": f"invalid message: {exc}"})
                elif message.type in {WSMsgType.CLOSE, WSMsgType.ERROR}:
                    break
        finally:
            self._hub.remove(client_id)
            with self._state_lock:
                if self._controller_id == client_id:
                    self._release_control(client_id, switch_auto=False)
                    self._journal.add("Operator disconnected; robot held in manual mode", "warning", "control")
        return socket

    async def _handle_ws_message(self, socket, client_id: str, payload: dict[str, Any]) -> None:
        message_type = str(payload.get("type", ""))
        if message_type == "heartbeat":
            await socket.send_json({"type": "heartbeat_ack", "timestamp": time.time()})
            return
        if message_type == "control_mode":
            mode = str(payload.get("mode", ""))
            if mode == "manual":
                success, message = self._claim_control(client_id)
            elif mode == "auto":
                success, message = self._release_control(client_id, switch_auto=True)
            else:
                success, message = False, "mode must be manual or auto"
            await socket.send_json({"type": "control_mode_result", "success": success, "message": message})
            return
        if message_type == "teleop":
            with self._state_lock:
                allowed = self._manual_mode and self._controller_id == client_id
                if allowed:
                    self._controller_expire = time.monotonic() + self._controller_lease_s
            if not allowed:
                await socket.send_json({"type": "error", "message": "manual control lease is not held"})
                return
            values = [float(payload.get("vx", 0.0)), float(payload.get("vy", 0.0)), float(payload.get("wz", 0.0))]
            with self._teleop_lock:
                self._teleop = values
            self._safety.teleop_heartbeat()
            return
        await socket.send_json({"type": "error", "message": f"unsupported message type: {message_type}"})

    def _publish_teleop(self) -> None:
        self._safety.read_only = self._ros_api.read_only or self._ros_api.estop_latched
        with self._teleop_lock:
            vx, vy, wz = self._teleop
        if self._manual_mode:
            vx, vy, wz = self._safety.filter(vx, vy, wz)
        else:
            vx = vy = wz = 0.0
        command = Twist()
        command.linear.x = vx
        command.linear.y = vy
        command.angular.z = wz
        self._cmd_pub.publish(command)

    def _watch_control_lease(self) -> None:
        now = time.monotonic()
        with self._state_lock:
            expired = bool(self._controller_id and now > self._controller_expire)
            if expired:
                self._controller_id = None
                self._controller_expire = 0.0
        if expired:
            with self._teleop_lock:
                self._teleop = [0.0, 0.0, 0.0]
            self._cmd_pub.publish(Twist())
            # Fail-safe: keep manual mode active so Nav2 cannot resume unexpectedly.
            self._journal.add("Manual control lease expired; robot remains held", "warning", "control")

    def _state_snapshot(self) -> dict[str, Any]:
        robot = self._ros_api.summary()
        mapping = self._mapping.status()
        navigation = self._nav2.status()
        with self._state_lock:
            controller_present = self._controller_id is not None
            lease_remaining = max(0.0, self._controller_expire - time.monotonic()) if controller_present else 0.0
            control = {
                "mode": "manual" if self._manual_mode else "auto",
                "manual_hold": self._manual_mode,
                "controller_present": controller_present,
                "controller_lease_remaining_s": round(lease_remaining, 2),
                "websocket_clients": self._hub.count(),
                "teleop_alive": self._safety.teleop_alive,
            }
        return {
            "timestamp": time.time(),
            "robot": robot,
            "control": control,
            "mapping": mapping,
            "navigation": navigation,
            "maps": {
                "active": mapping.get("active_map"),
                "count": mapping.get("map_count", 0),
                "root": mapping.get("map_root"),
            },
            "events": self._journal.list(30),
        }

    def _broadcast_state_from_ros(self) -> None:
        if self._web_loop is None or not self._web_loop.is_running() or self._hub.count() == 0:
            return
        payload = {"type": "state", "data": self._state_snapshot()}
        asyncio.run_coroutine_threadsafe(self._hub.broadcast(payload), self._web_loop)

    async def _json_body(self, request) -> dict[str, Any]:
        try:
            body = await request.json()
            return body if isinstance(body, dict) else {}
        except Exception:
            return {}

    @staticmethod
    def _response(result: dict[str, Any], status: int | None = None):
        code = status if status is not None else (200 if result.get("success", False) else 400)
        return web.json_response(result, status=code)

    async def _run(self, function, *args):
        return await asyncio.to_thread(function, *args)

    async def _api_health(self, _):
        return web.json_response({"success": True, "message": "ok", "timestamp": time.time()})

    async def _api_state(self, _):
        return web.json_response({"success": True, "data": self._state_snapshot()})

    async def _api_events(self, request):
        limit = int(request.query.get("limit", "80"))
        return web.json_response({"success": True, "data": self._journal.list(limit)})

    async def _api_diagnostics(self, _):
        try:
            nodes = sorted(self.get_node_names())
            services = sorted(name for name, _ in self.get_service_names_and_types())
            topics = sorted(name for name, _ in self.get_topic_names_and_types())
        except Exception:
            nodes, services, topics = [], [], []
        return web.json_response(
            {
                "success": True,
                "data": {
                    "nodes": nodes,
                    "services": services,
                    "topics": topics,
                    "processes": self._processes.status(),
                    "driver_diagnostics": self._ros_api.summary().get("diagnostics", []),
                },
            }
        )

    async def _api_stand(self, _):
        return self._response(await self._run(self._ros_api.stand_up))

    async def _api_lie(self, _):
        return self._response(await self._run(self._ros_api.lie_down))

    async def _api_crawl(self, _):
        return self._response(await self._run(self._ros_api.crawl))

    async def _api_estop(self, _):
        self._set_manual_mode(True, cancel_nav=True)
        return self._response(await self._run(self._ros_api.emergency_stop))

    async def _api_reset_estop(self, _):
        return self._response(await self._run(self._ros_api.reset_estop))

    async def _api_read_only(self, request):
        body = await self._json_body(request)
        read_only = bool(body.get("read_only", True))
        return self._response(await self._run(self._ros_api.set_read_only, read_only))

    async def _api_control_mode(self, request):
        body = await self._json_body(request)
        mode = str(body.get("mode", "manual"))
        if mode == "manual":
            self._set_manual_mode(True, cancel_nav=True)
            return self._response({"success": True, "message": "manual hold enabled"})
        if mode == "auto":
            self._set_manual_mode(False, cancel_nav=False)
            return self._response({"success": True, "message": "automatic mode enabled"})
        return self._response({"success": False, "message": "mode must be manual or auto"})

    async def _api_maps(self, _):
        maps = await self._run(self._mapping.list_maps)
        return web.json_response({"success": True, "data": maps})

    async def _api_map_save(self, request):
        body = await self._json_body(request)
        return self._response(await self._run(self._mapping.save_map, str(body.get("name", ""))))

    async def _api_map_load(self, request):
        body = await self._json_body(request)
        name = str(body.get("name", ""))
        self._set_manual_mode(True, cancel_nav=True)
        result = await self._run(self._mapping.load_map, name)
        if result.get("success"):
            await self._run(self._nav2.clear_costmaps)
        return self._response(result)

    async def _api_map_delete(self, request):
        return self._response(await self._run(self._mapping.delete_map, request.match_info["name"]))

    async def _api_map_preview(self, request):
        path = await self._run(self._mapping.preview_path, request.match_info["name"])
        if path is None:
            return web.Response(status=404, text="preview unavailable")
        return web.FileResponse(path)

    async def _api_mapping_status(self, _):
        return web.json_response({"success": True, "data": self._mapping.status()})

    async def _api_mapping_start(self, _):
        self._set_manual_mode(True, cancel_nav=True)
        self._live_map.reset_path()
        return self._response(await self._run(self._mapping.start_mapping))

    async def _api_mapping_stop(self, _):
        return self._response(await self._run(self._mapping.stop_mapping))

    async def _api_live_mapping(self, _):
        return web.json_response({"success": True, "data": self._live_map.snapshot()})

    async def _api_live_map_png(self, _):
        image, version = await self._run(self._live_map.map_png)
        if image is None:
            return web.Response(status=404, text="live map unavailable")
        return web.Response(
            body=image,
            content_type="image/png",
            headers={
                "Cache-Control": "no-store, no-cache, must-revalidate, max-age=0",
                "X-Live-Map-Version": str(version),
            },
        )

    async def _api_live_reset_path(self, _):
        return self._response(await self._run(self._live_map.reset_path))

    async def _api_nav_status(self, _):
        return web.json_response({"success": True, "data": self._nav2.status()})

    async def _api_nav_start(self, request):
        body = await self._json_body(request)
        result = await self._run(self._mapping.start_navigation, str(body.get("map", "")))
        if result.get("success"):
            self._set_manual_mode(False, cancel_nav=False)
        return self._response(result)

    async def _api_nav_stop(self, _):
        self._set_manual_mode(True, cancel_nav=True)
        return self._response(await self._run(self._mapping.stop_navigation))

    async def _api_nav_goal(self, request):
        body = await self._json_body(request)
        self._set_manual_mode(False, cancel_nav=False)
        result = await self._run(
            self._nav2.send_goal,
            float(body.get("x", 0.0)),
            float(body.get("y", 0.0)),
            float(body.get("yaw_deg", body.get("yaw", 0.0))),
            str(body.get("frame_id", "map")),
            str(body.get("behavior_tree", "")),
        )
        return self._response(result)

    async def _api_nav_waypoints(self, request):
        body = await self._json_body(request)
        poses = body.get("poses", [])
        if not isinstance(poses, list) or len(poses) > 100:
            return self._response({"success": False, "message": "poses must be a list with at most 100 items"})
        self._set_manual_mode(False, cancel_nav=False)
        return self._response(await self._run(self._nav2.send_waypoints, poses, str(body.get("frame_id", "map"))))

    async def _api_nav_cancel(self, _):
        self._set_manual_mode(True, cancel_nav=False)
        return self._response(await self._run(self._nav2.cancel_goal))

    async def _api_nav_clear(self, _):
        return self._response(await self._run(self._nav2.clear_costmaps))

    async def _api_initial_pose(self, request):
        body = await self._json_body(request)
        self._set_manual_mode(True, cancel_nav=True)
        result = await self._run(
            self._ros_api.set_initial_pose,
            float(body.get("x", 0.0)),
            float(body.get("y", 0.0)),
            float(body.get("yaw_deg", body.get("yaw", 0.0))),
            float(body.get("covariance_xy", 0.25)),
            float(body.get("covariance_yaw", 0.0685)),
        )
        if result.get("success"):
            await asyncio.sleep(0.15)
            await self._run(self._nav2.clear_costmaps)
            await self._run(self._ros_api.nomotion_update)
        return self._response(result)

    async def _api_global_localization(self, _):
        self._set_manual_mode(True, cancel_nav=True)
        return self._response(await self._run(self._ros_api.global_localization))

    async def _api_nomotion_update(self, _):
        return self._response(await self._run(self._ros_api.nomotion_update))

    def destroy_node(self):
        self._cmd_pub.publish(Twist())
        self._manual_mode = True
        self._publish_control_mode()
        if bool(self.get_parameter("stop_managed_processes_on_exit").value):
            self._processes.stop_all()
        if self._web_loop and self._web_loop.is_running():
            self._web_loop.call_soon_threadsafe(self._web_loop.stop)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = WebControlNode()
    executor = MultiThreadedExecutor(num_threads=6)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.remove_node(node)
        node.destroy_node()
        rclpy.shutdown()
