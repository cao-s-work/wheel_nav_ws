"""
nav2_client.py — Nav2 Action 客户端封装。

- NavigateToPose
- cancel goal
- clear costmaps
- 状态查询
"""
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import ClearEntireCostmap
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Trigger


class Nav2Client:
    """非 ROS 节点，持有外部 node 的 action/service client 引用。"""

    def __init__(self, node: Node):
        self._node = node
        self._goal_client = ActionClient(node, NavigateToPose, "navigate_to_pose")
        self._goal_handle = None

    def send_goal(self, x: float, y: float, yaw_deg: float,
                  frame_id: str = "map") -> bool:
        """发送 NavigateToPose goal。yaw_deg 是度。"""
        import math
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
        self._goal_handle = future.result()
        if self._goal_handle is None:
            self._node.get_logger().error("Goal rejected by server")
            return
        self._node.get_logger().info("Goal accepted")

    def cancel_goal(self) -> bool:
        if self._goal_handle is not None:
            future = self._goal_handle.cancel_goal_async()
            return True
        return False

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
