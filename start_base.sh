#!/bin/bash
# ZSL-1W 统一启动脚本 (base mode)
set -e
export HOME=/home/nvidia

# ——— 狗端 mc_ctrl 重启（确保 SDK 连接可用） ———
DOG_IP="192.168.168.168"
DOG_PASS="${DOG_PASS:-firefly}"
echo "[bringup] 重启狗端 mc_ctrl..."
if sshpass -p "$DOG_PASS" ssh -o StrictHostKeyChecking=no -o ConnectTimeout=5 \
    "firefly@$DOG_IP" \
    "sudo killall mc_ctrl 2>/dev/null; sleep 1; \
     nohup /opt/app_launch/start_motion_control_xgw.sh > /tmp/mc_ctrl.log 2>&1 &" 2>/dev/null; then
    sleep 3
    sshpass -p "$DOG_PASS" ssh -o ConnectTimeout=3 "firefly@$DOG_IP" \
        "pgrep mc_ctrl" 2>/dev/null && echo "[bringup] mc_ctrl ✅" || echo "[bringup] mc_ctrl ❌"
else
    echo "[bringup] ⚠️ 狗端不可达，跳过 mc_ctrl 重启"
fi

source /opt/ros/humble/setup.bash
source /home/nvidia/wheel_nav_ws/install/setup.bash
# HACK: Hermes 后台进程 local_setup 不包含 zsl_web_control，手动注入
export AMENT_PREFIX_PATH="/home/nvidia/wheel_nav_ws/install/zsl_web_control:$AMENT_PREFIX_PATH"
unset RMW_IMPLEMENTATION ZENOH_CONFIG_OVERRIDE
export ROS_DOMAIN_ID=0

ros2 launch zsl_bringup bringup.launch.py mode:=base web:=true read_only:=true "$@"
