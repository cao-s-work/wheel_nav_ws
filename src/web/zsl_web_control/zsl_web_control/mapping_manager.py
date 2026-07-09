"""mapping_manager.py — 建图操作封装。"""
import rclpy
from rclpy.node import Node


class MappingManager:
    """SLAM 建图相关的高层操作。第一阶段仅提供状态查询接口。"""

    def __init__(self, node: Node):
        self._node = node
        self._map_metadata = None

    # ---- 状态 ----

    def get_status(self) -> dict:
        """返回建图子系统状态摘要。"""
        return {
            "slam_active": False,      # TODO: 检测 slam_toolbox lifecycle
            "lidar_hz": 0.0,           # TODO: 订阅 /livox/lidar 计算 hz
            "scan_hz": 0.0,            # TODO: 订阅 /scan 计算 hz
            "map_available": self._map_metadata is not None,
        }

    def update_map_info(self, info):
        """由 web_node 回调更新地图元数据。"""
        self._map_metadata = info
