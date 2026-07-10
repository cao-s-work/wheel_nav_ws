"""Nav2 action and service facade used by the web gateway."""
from __future__ import annotations

import math
import threading
import time
from typing import Any

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateThroughPoses, NavigateToPose
from nav2_msgs.srv import ClearEntireCostmap
from rclpy.action import ActionClient
from rclpy.node import Node

from .utils import EventJournal, duration_to_seconds, wait_future, yaw_to_quaternion


class Nav2Client:
    def __init__(self, node: Node, journal: EventJournal):
        self._node = node
        self._journal = journal
        self._lock = threading.RLock()
        self._navigate = ActionClient(node, NavigateToPose, "navigate_to_pose")
        self._navigate_through = ActionClient(node, NavigateThroughPoses, "navigate_through_poses")
        self._global_clear = node.create_client(
            ClearEntireCostmap,
            "/global_costmap/clear_entirely_global_costmap",
        )
        self._local_clear = node.create_client(
            ClearEntireCostmap,
            "/local_costmap/clear_entirely_local_costmap",
        )
        self._goal_handle = None
        self._goal_future = None
        self._cancel_requested = False
        self._status: dict[str, Any] = {
            "state": "idle",
            "message": "No navigation task",
            "goal": None,
            "distance_remaining": None,
            "estimated_time_remaining_s": None,
            "navigation_time_s": 0.0,
            "recoveries": 0,
            "poses_remaining": None,
            "updated_at": time.time(),
        }

    @staticmethod
    def _pose(x: float, y: float, yaw_deg: float, frame_id: str, stamp) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = frame_id
        pose.header.stamp = stamp
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        qx, qy, qz, qw = yaw_to_quaternion(math.radians(float(yaw_deg)))
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        return pose

    def available(self, timeout_s: float = 0.0) -> bool:
        try:
            return bool(self._navigate.wait_for_server(timeout_sec=max(0.0, timeout_s)))
        except Exception:
            return False

    def _set_status(self, **values: Any) -> None:
        with self._lock:
            self._status.update(values)
            self._status["updated_at"] = time.time()

    def status(self) -> dict[str, Any]:
        with self._lock:
            result = dict(self._status)
        result["server_available"] = self.available(0.0)
        return result

    def send_goal(
        self,
        x: float,
        y: float,
        yaw_deg: float,
        frame_id: str = "map",
        behavior_tree: str = "",
    ) -> dict[str, Any]:
        if not self._navigate.wait_for_server(timeout_sec=2.0):
            return {"success": False, "message": "navigate_to_pose action server unavailable"}

        goal = NavigateToPose.Goal()
        goal.pose = self._pose(x, y, yaw_deg, frame_id, self._node.get_clock().now().to_msg())
        goal.behavior_tree = behavior_tree

        with self._lock:
            self._cancel_requested = False
            self._goal_handle = None
            self._status = {
                "state": "sending",
                "message": "Sending navigation goal",
                "goal": {"x": x, "y": y, "yaw_deg": yaw_deg, "frame_id": frame_id},
                "distance_remaining": None,
                "estimated_time_remaining_s": None,
                "navigation_time_s": 0.0,
                "recoveries": 0,
                "poses_remaining": None,
                "updated_at": time.time(),
            }

        future = self._navigate.send_goal_async(goal, feedback_callback=self._feedback_cb)
        self._goal_future = future
        future.add_done_callback(self._goal_response_cb)
        self._journal.add(
            f"Navigation goal sent: ({x:.2f}, {y:.2f}, {yaw_deg:.1f}°)",
            "info",
            "navigation",
        )
        return {"success": True, "message": "navigation goal submitted"}

    def send_waypoints(self, poses: list[dict[str, float]], frame_id: str = "map") -> dict[str, Any]:
        if not poses:
            return {"success": False, "message": "waypoint list is empty"}
        if not self._navigate_through.wait_for_server(timeout_sec=2.0):
            return {"success": False, "message": "navigate_through_poses action server unavailable"}

        goal = NavigateThroughPoses.Goal()
        stamp = self._node.get_clock().now().to_msg()
        goal.poses = [
            self._pose(float(item["x"]), float(item["y"]), float(item.get("yaw_deg", 0.0)), frame_id, stamp)
            for item in poses
        ]
        goal.behavior_tree = ""
        with self._lock:
            self._cancel_requested = False
            self._goal_handle = None
            self._status = {
                "state": "sending",
                "message": "Sending waypoint route",
                "goal": {"waypoints": poses, "frame_id": frame_id},
                "distance_remaining": None,
                "estimated_time_remaining_s": None,
                "navigation_time_s": 0.0,
                "recoveries": 0,
                "poses_remaining": len(poses),
                "updated_at": time.time(),
            }
        future = self._navigate_through.send_goal_async(goal, feedback_callback=self._feedback_cb)
        self._goal_future = future
        future.add_done_callback(self._goal_response_cb)
        self._journal.add(f"Waypoint route submitted ({len(poses)} poses)", "info", "navigation")
        return {"success": True, "message": "waypoint route submitted"}

    def _goal_response_cb(self, future) -> None:
        try:
            handle = future.result()
        except Exception as exc:
            self._set_status(state="error", message=f"Goal request failed: {exc}")
            self._journal.add(f"Navigation goal request failed: {exc}", "error", "navigation")
            return

        if handle is None or not handle.accepted:
            self._set_status(state="rejected", message="Navigation goal rejected")
            self._journal.add("Navigation goal rejected", "error", "navigation")
            return

        with self._lock:
            self._goal_handle = handle
            cancel_requested = self._cancel_requested
        self._set_status(state="active", message="Navigation active")
        self._journal.add("Navigation goal accepted", "success", "navigation")

        if cancel_requested:
            cancel_future = handle.cancel_goal_async()
            cancel_future.add_done_callback(self._cancel_response_cb)

        result_future = handle.get_result_async()
        result_future.add_done_callback(self._result_cb)

    def _feedback_cb(self, message) -> None:
        feedback = message.feedback
        values: dict[str, Any] = {
            "state": "active",
            "message": "Navigation active",
            "distance_remaining": round(float(feedback.distance_remaining), 3),
            "estimated_time_remaining_s": round(duration_to_seconds(feedback.estimated_time_remaining), 1),
            "navigation_time_s": round(duration_to_seconds(feedback.navigation_time), 1),
            "recoveries": int(feedback.number_of_recoveries),
        }
        if hasattr(feedback, "number_of_poses_remaining"):
            values["poses_remaining"] = int(feedback.number_of_poses_remaining)
        self._set_status(**values)

    def _result_cb(self, future) -> None:
        try:
            wrapped = future.result()
            status = int(wrapped.status)
        except Exception as exc:
            self._set_status(state="error", message=f"Navigation result failed: {exc}")
            return

        state_map = {
            GoalStatus.STATUS_SUCCEEDED: ("succeeded", "Navigation completed"),
            GoalStatus.STATUS_CANCELED: ("canceled", "Navigation canceled"),
            GoalStatus.STATUS_ABORTED: ("aborted", "Navigation aborted"),
        }
        state, message = state_map.get(status, ("finished", f"Navigation finished with status {status}"))
        self._set_status(state=state, message=message, distance_remaining=0.0 if state == "succeeded" else None)
        self._journal.add(message, "success" if state == "succeeded" else "warning", "navigation")
        with self._lock:
            self._goal_handle = None

    def cancel_goal(self) -> dict[str, Any]:
        with self._lock:
            self._cancel_requested = True
            handle = self._goal_handle
            state = self._status.get("state")
        if handle is None:
            if state in {"idle", "succeeded", "canceled", "aborted", "rejected", "error"}:
                return {"success": True, "message": "no active navigation goal"}
            self._set_status(message="Cancellation queued")
            return {"success": True, "message": "cancellation queued until goal is accepted"}
        future = handle.cancel_goal_async()
        future.add_done_callback(self._cancel_response_cb)
        self._set_status(state="canceling", message="Canceling navigation")
        return {"success": True, "message": "navigation cancellation requested"}

    def _cancel_response_cb(self, future) -> None:
        try:
            response = future.result()
            accepted = bool(response and len(response.goals_canceling) > 0)
        except Exception as exc:
            self._set_status(state="error", message=f"Cancel request failed: {exc}")
            return
        if accepted:
            self._set_status(state="canceling", message="Navigation cancellation accepted")
            self._journal.add("Navigation cancellation accepted", "warning", "navigation")
        else:
            self._set_status(message="Navigation cancellation was not accepted")
            self._journal.add("Navigation cancellation was not accepted", "error", "navigation")

    def _clear_one(self, client, name: str) -> tuple[bool, str]:
        if not client.wait_for_service(timeout_sec=1.0):
            return False, f"{name} service unavailable"
        future = client.call_async(ClearEntireCostmap.Request())
        ok, _, error = wait_future(future, 4.0)
        return (True, "") if ok else (False, f"{name}: {error}")

    def clear_costmaps(self) -> dict[str, Any]:
        global_ok, global_error = self._clear_one(self._global_clear, "global costmap clear")
        local_ok, local_error = self._clear_one(self._local_clear, "local costmap clear")
        success = global_ok and local_ok
        message = "global and local costmaps cleared" if success else "; ".join(
            item for item in (global_error, local_error) if item
        )
        self._journal.add(message, "success" if success else "error", "navigation")
        return {"success": success, "message": message}
