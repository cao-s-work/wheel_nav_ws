# zsl_driver — ZSL-1W 轮足钢镚 ROS 2 驱动

封装 `mc_sdk_zsl_1w_py`，提供 `cmd_vel` + `service` 接口。

## 快速启动

```bash
# 1) 编译
cd wheel_nav_ws
colcon build --symlink-install --packages-select zsl_driver

# 2) 诊断（可选，先确认 SDK 就绪）
ros2 run zsl_driver sdk_check --ros-args -p sdk_dog_ip:=192.168.168.168

# 3) 启动驱动（只读模式）
ros2 run zsl_driver zsl_driver_node --ros-args -p read_only:=true

# 4) 或 launch
ros2 launch zsl_driver zsl_driver.launch.py read_only:=true
```

## SDK 路径配置（三级 fallback）

优先级：**参数 > 环境变量 > 自动识别 > 包内**

| 方式 | 说明 |
|------|------|
| `-p sdk_lib_dir:=/path` | 显式指定 .so 目录 |
| `export ZSL_SDK_LIB_DIR=/path` | 环境变量 |
| 自动 `~/gb_ws2/sdk/.../lib/zsl-1w/{arch}/` | 按 `aarch64`/`x86_64` 自动查找 |
| 包内 `zsl_driver/sdk_lib/` | 预打包 fallback |

## 网络配置

| 场景 | 狗端 IP | Jetson 端操作 |
|------|--------|-------------|
| **有线（默认）** | `192.168.168.168` | `sudo ip addr add 192.168.168.216/24 dev enP8p1s0` |
| **无线** | `192.168.234.1` | 连接狗 WiFi，设同网段 IP |

> ⚠️ 狗端 `sdk_config.yaml` 的 `target_ip` / `target_port` 必须和 ROS 主机一致，修改后需重启机器人：
> ```bash
> ssh firefly@192.168.168.168
> sudo vim /opt/export/config/sdk_config.yaml
> sudo systemctl restart robot-launch
> ```

## Service 接口

| Service | 说明 |
|---------|------|
| `~/stand_up` | 站立 |
| `~/lie_down` | 安全趴下（匍匐 → 趴下） |
| `~/crawl` | 匍匐 |
| `~/emergency_stop` | 急停 → 趴下 |
| `~/take_control` | （兼容接口，ZSL-1W no-op） |
| `~/release_control` | （兼容接口，no-op） |
| `~/set_read_only` | 热切 read_only |

## 参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `sdk_local_ip` | `192.168.168.216` | Jetson 本机 IP |
| `sdk_local_port` | `43988` | 本地 UDP 端口 |
| `sdk_dog_ip` | `192.168.168.168` | 狗端 IP |
| `sdk_lib_dir` | `""` | SDK .so 目录（留空用 fallback） |
| `read_only` | `true` | 只读模式（安全闸门） |
| `cmd_vel_timeout_ms` | `500` | cmd_vel watchdog 超时 |
| `speed_scale` | `1.0` | 线速度倍率 |
| `angular_scale` | `1.0` | 角速度倍率 |
