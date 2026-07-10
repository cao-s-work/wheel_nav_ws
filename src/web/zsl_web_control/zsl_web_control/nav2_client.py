"""
nav2_client.py — Nav2 Action 客户端封装。

- NavigateToPose
- cancel goal（支持 handle 未返回时排队取消）
- clear costmaps
- 状态查询
"""
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import ClearEntireCostmap
from geometry_msgs.msg import PoseStamped


class Nav2Client:
    """非 ROS 节点，持有外部 node 的 action/service client 引用。"""

    def __init__(self, node: Node):
        self._node = node
        self._goal_client = ActionClient(node, NavigateToPose, "navigate_to_pose")
        self._goal_handle = None
        self._cancel_requested = False

    def send_goal(self, x: float, y: float, yaw_deg: float,
                  frame_id: str = "map") -> bool:
        """发送 NavigateToPose goal。yaw_deg 是度。"""
        import math
        self._cancel_requested = False  # 新 goal 重置取消标记
        yaw = math.radians(yaw_deg)
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = frame_id
        goal.pose.header.stamp = self._node.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.z = math.sin(yaw / 2)
        goal.pose.pose.orientation.w = math.cos(yaw / 2)

        if not self._goal_client.wait_for_server(timeout_sec=2.0):
            self._node.get_logger().error("navigate_to_pose action server unavailable")
            return False

        future = self._goal_client.send_goal_async(goal)
        future.add_done_callback(self._goal_response_cb)
        return True

    def _goal_response_cb(self, future):
        try:
            goal_handle = future.result()
        except Exception as exc:
            self._node.get_logger().error(f"Goal request failed: {exc}")
            self._goal_handle = None
            return

        if goal_handle is None or not goal_handle.accepted:
            self._node.get_logger().error("Goal rejected by server")
            self._goal_handle = None
            return

        self._goal_handle = goal_handle
        self._node.get_logger().info("Goal accepted")

        # 如果在 goal 等待期间已经请求取消，立即执行
        if self._cancel_requested:
            cancel_future = goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(self._cancel_response_cb)

    def cancel_goal(self) -> bool:
        """取消当前导航任务。如 goal handle 尚未返回则排队。"""
        self._cancel_requested = True
        if self._goal_handle is None:
            self._node.get_logger().info(
                "Nav2 cancel queued; waiting for goal handle"
            )
            return True
        if not self._goal_handle.accepted:
            return False
        future = self._goal_handle.cancel_goal_async()
        future.add_done_callback(self._cancel_response_cb)
        return True

    def _cancel_response_cb(self, future):
        try:
            response = future.result()
            accepted = len(response.goals_canceling) > 0
            if accepted:
                self._node.get_logger().info("Nav2 goal cancellation accepted")
            else:
                self._node.get_logger().warn("Nav2 goal cancellation was not accepted")
        except Exception as exc:
            self._node.get_logger().error(f"Nav2 cancel request failed: {exc}")

    def clear_costmaps(self) -> bool:
        """清除 local 和 global costmap。"""
        cli = self._node.create_client(ClearEntireCostmap,
                                        "/global_costmap/clear_entirely_global_costmap")
        if cli.wait_for_service(timeout_sec=1.0):
            cli.call_async(ClearEntireCostmap.Request())
        cli2 = self._node.create_client(ClearEntireCostmap,
                                         "/local_costmap/clear_entirely_local_costmap")
        if cli2.wait_for_service(timeout_sec=1.0):
            cli2.call_async(ClearEntireCostmap.Request())
        return True
