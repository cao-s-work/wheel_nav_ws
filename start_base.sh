#!/bin/bash
# ZSL-1W 统一启动脚本 (base mode)
set -e
export HOME=/home/nvidia
source /opt/ros/humble/setup.bash
source /home/nvidia/wheel_nav_ws/install/setup.bash
# HACK: Hermes 后台进程 local_setup 不包含 zsl_web_control，手动注入
export AMENT_PREFIX_PATH="/home/nvidia/wheel_nav_ws/install/zsl_web_control:$AMENT_PREFIX_PATH"
unset RMW_IMPLEMENTATION ZENOH_CONFIG_OVERRIDE
export ROS_DOMAIN_ID=0

ros2 launch zsl_bringup bringup.launch.py mode:=base web:=true read_only:=true "$@"
