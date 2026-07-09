# 建图调试记录

## 问题 1: colcon build 失败

### 现象
```
CMake Error: package.xml does not exist → livox_ros_driver2
CMake Error: Findoctomap.cmake not found → octomap_server2
CMake Error: Findoctomap_msgs.cmake not found → octomap_server2
```

### 根因

1. **livox_ros_driver2 缺少 package.xml**：该包是双 ROS1/ROS2 设计，源码只有 `package_ROS1.xml` 和 `package_ROS2.xml`。编译前需手动复制：
   ```
   cd src/driver/livox_ros_driver2
   cp package_ROS2.xml package.xml
   cp -rf launch_ROS2/ launch/
   ```

2. **octomap 依赖缺失**：
   ```
   sudo apt-get install -y ros-humble-octomap ros-humble-octomap-msgs liboctomap-dev
   ```

   （如果不需要 octomap_server2，可跳过：`colcon build --packages-skip octomap_server2`）

---

## 问题 2: mapping.sh 无 TF 树 / 无里程计

### 现象
- `ros2 topic hz /livox/imu` → 200Hz（正常）
- `ros2 topic hz /livox/lidar` → 无数据
- `/Odometry` 无消息
- TF 树为空

### 排查链路

1. **ping 192.168.168.4** → 通（lidar 可达）
2. **UDP 端口 56301** → 收到 1380 字节（点云数据已到达本机）
3. **ros2 node info /livox_lidar_publisher** → 发布 `/livox/lidar` 但类型是 `livox_ros_driver2/msg/CustomMsg`
4. **FAST-LIO 配置** `lidar_type: 4` → 订阅 `sensor_msgs/msg/PointCloud2`

### 根因

**消息类型不匹配。**

| 组件 | 期望类型 | 实际类型 |
|------|----------|----------|
| livox 驱动 (xfer_format=1) | — | `CustomMsg` |
| FAST-LIO (lidar_type=4) | `PointCloud2` | — |

`msg_MID360_launch.py` 中 `xfer_format = 1` 导致驱动发布自定义格式，FAST-LIO 无法订阅 → 无点云输入 → 不收敛 → 无里程计 → TF 树为空。

### 修复

```python
# msg_MID360_launch.py
xfer_format = 0  # 0: PointCloud2(PointXYZRTL), 1: CustomMsg
```

涉及文件：
- `src/driver/livox_ros_driver2/launch/msg_MID360_launch.py`
- `src/driver/livox_ros_driver2/launch_ROS2/msg_MID360_launch.py`

重启 livox 驱动生效。
