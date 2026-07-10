# ZSL-1W 商业化 Web 运维控制台

这是一套针对 `cao-s-work/wheel_nav_ws` 的前后端替换包，包含：

- 机器人状态、SDK、电量、急停和 read-only 状态；
- 单控制者人工遥控、WebSocket deadman、手动保持和自动导航切换；
- 站立、趴下、匍匐、上锁、长按解锁、急停、急停复位；
- 建图栈启动/停止、传感器频率、地图尺寸与地图保存；
- 地图库扫描、地图预览、地图加载、地图删除；
- Nav2 启动/停止、目标点导航、任务取消、剩余距离、清除 costmap；
- `/initialpose` 重定位、AMCL 全局重定位、静止更新；
- 受控进程日志、ROS 节点/服务/话题和 driver diagnostics；
- 可选 API Token；网页不能提交任意 Shell 命令。

## 1. 安装

```bash
unzip zsl_commercial_web_bundle.zip
cd zsl_commercial_web_bundle
bash install.sh /home/nvidia/wheel_nav_ws --build
source /home/nvidia/wheel_nav_ws/install/setup.bash
```

若你的真实工作空间是 `/home/nvidia/gb_ws`：

```bash
bash install.sh /home/nvidia/gb_ws --build
source /home/nvidia/gb_ws/install/setup.bash
```

依赖：

```bash
sudo apt update
sudo apt install -y python3-aiohttp python3-yaml
```

## 2. 推荐启动方式

先启动底盘、LiDAR、LIO、速度 mux、安全节点和 driver：

```bash
ros2 launch zsl_bringup robot_base.launch.py read_only:=true
```

再单独启动 Web：

```bash
ros2 launch zsl_web_control zsl_web_control.launch.py
```

浏览器默认只能从机器人本机访问。开发电脑建立 SSH 隧道：

```bash
ssh -L 8080:127.0.0.1:8080 nvidia@机器人IP
```

然后打开：

```text
http://127.0.0.1:8080
```

网页中的“开始建图”和“启动导航”会分别执行配置文件中的白名单命令：

```yaml
mapping_command: "ros2 launch zsl_bringup mapping.launch.py"
navigation_command: "ros2 launch zsl_bringup managed_navigation.launch.py map_file:={map}"
```

这些子栈不会重复启动 robot_base 或 Web。

## 3. 建图工作流

1. 确认 LiDAR、Scan、Odometry 均有频率；
2. 点击“开始建图”；
3. 进入人工控制，低速覆盖整个场景；
4. 输入地图名称；
5. 点击“保存当前地图”；
6. 后端优先调用 `/map_saver/save_map`；服务不可用时可回退到 `map_saver_cli`；
7. 地图保存到 `~/gb_maps`，可在 `web_control.yaml` 中修改。

本包修改了 `mapping.launch.py`，会启动 `map_saver_server`，因此正常情况下无需 CLI 回退。

## 4. 地图切换与导航

- “仅切换地图”：取消当前任务、保持停车、调用 `/map_server/load_map`、清除 local/global costmap；
- “启动导航”：停止网页启动的建图子栈，使用选中地图启动 AMCL + Nav2；
- “停止导航栈”：取消任务并停止网页启动的导航子进程；
- 若 Nav2 已由外部启动，后端不会重复启动，而是尝试直接加载地图。

## 5. 重定位

“设置初始位姿”会按以下顺序执行：

```text
人工保持 → 取消导航 → 发布 /initialpose → 清除代价地图 → request_nomotion_update
```

“全局重定位”调用：

```text
/reinitialize_global_localization
```

“静止更新”调用：

```text
/request_nomotion_update
```

服务名称可在配置文件中调整。

## 6. 安全设计

- Web 遥控发布到 `/cmd_vel_teleop`；
- `/web/teleop_active` 使用 transient-local 状态；
- 同一时间只有一个 WebSocket 客户端拥有人工控制权；
- 控制者断开或 lease 超时：速度归零，仍保持手动阻断，不自动恢复 Nav2；
- 急停按钮会先进入手动保持并取消导航，再调用 driver 的急停服务；
- reset estop 不会自动解锁；
- Web 限速只是第一层，最终必须继续由 `cmd_vel_safety` 限速并由 driver watchdog 兜底。

## 7. 局域网部署与 Token

默认：

```yaml
host: "127.0.0.1"
api_token: ""
```

局域网访问时改为：

```yaml
host: "0.0.0.0"
api_token: "请替换为足够长的随机字符串"
```

浏览器使用：

```text
http://机器人IP:8080/?token=你的Token
```

Token 会保存在浏览器 `localStorage`，API 与 WebSocket 都会携带。

## 8. 配置文件

```text
src/web/zsl_web_control/config/web_control.yaml
```

重点检查：

- `map_root`
- `odom_topic`
- `lidar_topic`
- `map_save_service`
- `map_load_service`
- `mapping_command`
- `navigation_command`

你的工程目前常用 LIO odometry 是 `/Odometry`；若最终改成 `/odom`，需要同步修改配置。

## 9. 验证

```bash
bash smoke_test.sh
```

再检查：

```bash
ros2 topic info -v /cmd_vel
ros2 topic info -v /cmd_vel_teleop
ros2 topic info -v /cmd_vel_selected
ros2 topic info -v /cmd_vel_safe
ros2 topic echo /web/teleop_active
```

正确链路：

```text
Nav2 velocity_smoother -> /cmd_vel
Web -> /cmd_vel_teleop
cmd_vel_mux -> /cmd_vel_selected
cmd_vel_safety -> /cmd_vel_safe
zsl_driver <- /cmd_vel_safe
```

## 10. 实机前必须验证

1. read-only 状态下所有速度均被拦截；
2. 急停锁存后，即使 Nav2 持续发速度，`/cmd_vel_safe` 仍保持零；
3. reset estop 后仍为 read-only；
4. WebSocket 断开后不恢复旧导航；
5. 切换地图和重定位期间保持停车；
6. 地图加载成功后再允许发送目标点。

本包无法替代实体机器人的悬空测试、低速落地测试和厂商 SDK 急停验证。
