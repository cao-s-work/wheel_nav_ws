#!/usr/bin/env bash
# ZSL-1W ROS2 建图管理脚本
# 命令：start | save [name] | status | stop-mapping | stop | run [name] | logs

set -Eeuo pipefail

ROS_DISTRO_NAME="${ROS_DISTRO:-humble}"
ROS_SETUP="/opt/ros/${ROS_DISTRO_NAME}/setup.bash"
WORKSPACE="${ZSL_WS:-}"
MAP_DIR="${ZSL_MAP_DIR:-$HOME/gb_maps}"
READ_ONLY="${ZSL_READ_ONLY:-true}"
RVIZ="${ZSL_RVIZ:-false}"
START_BASE="${ZSL_START_BASE:-true}"
NO_WAIT="${ZSL_NO_WAIT:-false}"
STATE_DIR="${XDG_RUNTIME_DIR:-/tmp}/zsl_mapping_${USER}"
BASE_PID_FILE="$STATE_DIR/robot_base.pid"
MAPPING_PID_FILE="$STATE_DIR/mapping.pid"
BASE_LOG="$STATE_DIR/robot_base.log"
MAPPING_LOG="$STATE_DIR/mapping.log"

mkdir -p "$STATE_DIR" "$MAP_DIR"

log()  { printf '\033[1;34m[INFO]\033[0m %s\n' "$*"; }
ok()   { printf '\033[1;32m[ OK ]\033[0m %s\n' "$*"; }
warn() { printf '\033[1;33m[WARN]\033[0m %s\n' "$*" >&2; }
err()  { printf '\033[1;31m[ERR ]\033[0m %s\n' "$*" >&2; }
die()  { err "$*"; exit 1; }

usage() {
  cat <<USAGE
用法：
  $0 start
  $0 save [地图名称]
  $0 status
  $0 stop-mapping
  $0 stop
  $0 stop --save [地图名称]
  $0 run [地图名称]
  $0 logs

环境变量：
  ZSL_WS=/home/nvidia/wheel_nav_ws
  ZSL_MAP_DIR=/home/nvidia/gb_maps
  ZSL_READ_ONLY=true|false   默认 true
  ZSL_RVIZ=true|false        默认 false
  ZSL_START_BASE=true|false  默认 true
  ZSL_NO_WAIT=true|false     Web 后台启动时设为 true
USAGE
}

detect_workspace() {
  if [[ -n "$WORKSPACE" ]]; then
    [[ -f "$WORKSPACE/install/setup.bash" ]] || \
      die "$WORKSPACE/install/setup.bash 不存在，请先 colcon build。"
    return
  fi

  local candidate
  for candidate in \
    "$HOME/wheel_nav_ws" \
    "$HOME/gb_ws" \
    /home/nvidia/wheel_nav_ws \
    /home/nvidia/gb_ws; do
    if [[ -f "$candidate/install/setup.bash" ]]; then
      WORKSPACE="$candidate"
      return
    fi
  done

  die "未找到已编译工作空间，请设置 ZSL_WS。"
}

source_ros() {
  [[ -f "$ROS_SETUP" ]] || die "$ROS_SETUP 不存在。"
  detect_workspace
  set +u
  # shellcheck disable=SC1090
  source "$ROS_SETUP"
  # shellcheck disable=SC1090
  source "$WORKSPACE/install/setup.bash"
  set -u
}

pid_alive() {
  local file="$1"
  [[ -f "$file" ]] || return 1
  local pid
  pid="$(cat "$file" 2>/dev/null || true)"
  [[ "$pid" =~ ^[0-9]+$ ]] || return 1
  kill -0 "$pid" 2>/dev/null
}

cleanup_pid() {
  local file="$1"
  if [[ -f "$file" ]] && ! pid_alive "$file"; then
    rm -f "$file"
  fi
}

topic_has_publisher() {
  local topic="$1"
  timeout 4 ros2 topic info "$topic" 2>/dev/null | \
    grep -Eq 'Publisher count: [1-9][0-9]*'
}

node_exists() {
  local node="$1"
  timeout 4 ros2 node list 2>/dev/null | grep -Fxq "$node"
}

wait_topic() {
  local topic="$1" timeout_s="$2" label="$3"
  local start now
  start="$(date +%s)"
  while true; do
    if topic_has_publisher "$topic"; then
      ok "$label 已就绪：$topic"
      return 0
    fi
    now="$(date +%s)"
    if (( now - start >= timeout_s )); then
      warn "等待 $label 超时：$topic"
      return 1
    fi
    sleep 1
  done
}

start_group() {
  local pid_file="$1" log_file="$2"
  shift 2

  cleanup_pid "$pid_file"
  if pid_alive "$pid_file"; then
    warn "进程已运行，PID=$(cat "$pid_file")"
    return 0
  fi

  setsid "$@" >>"$log_file" 2>&1 &
  local pid=$!
  echo "$pid" >"$pid_file"
  sleep 1

  if ! kill -0 "$pid" 2>/dev/null; then
    rm -f "$pid_file"
    err "启动失败，日志：$log_file"
    tail -n 50 "$log_file" 2>/dev/null || true
    return 1
  fi

  ok "启动成功，PID=$pid，日志=$log_file"
}

stop_group() {
  local pid_file="$1" label="$2"
  cleanup_pid "$pid_file"
  if ! pid_alive "$pid_file"; then
    log "$label 未由本脚本启动或已经停止。"
    rm -f "$pid_file"
    return 0
  fi

  local pid
  pid="$(cat "$pid_file")"
  log "停止 $label，PID=$pid"
  kill -INT -- "-$pid" 2>/dev/null || kill -INT "$pid" 2>/dev/null || true

  for _ in {1..30}; do
    if ! kill -0 "$pid" 2>/dev/null; then
      rm -f "$pid_file"
      ok "$label 已停止。"
      return 0
    fi
    sleep 0.2
  done

  warn "$label 未及时退出，发送 TERM。"
  kill -TERM -- "-$pid" 2>/dev/null || kill -TERM "$pid" 2>/dev/null || true
  sleep 1
  kill -KILL -- "-$pid" 2>/dev/null || kill -KILL "$pid" 2>/dev/null || true
  rm -f "$pid_file"
}

sanitize_name() {
  local name="${1:-map_$(date +%Y%m%d_%H%M%S)}"
  name="${name##*/}"
  name="$(sed 's/[^A-Za-z0-9._-]/_/g' <<<"$name")"
  name="${name#.}"
  [[ -n "$name" ]] || name="map_$(date +%Y%m%d_%H%M%S)"
  printf '%s' "$name"
}

start_stack() {
  source_ros
  mkdir -p "$MAP_DIR"

  log "工作空间：$WORKSPACE"
  log "地图目录：$MAP_DIR"
  log "read_only=$READ_ONLY, rviz=$RVIZ"

  if [[ "$START_BASE" == "true" ]]; then
    if node_exists /zsl_driver_node || topic_has_publisher /cloud_registered_body; then
      log "基础链路已在运行，不重复启动。"
    else
      log "启动 Livox + FAST-LIO + pointcloud_to_laserscan + 底盘安全链……"
      start_group \
        "$BASE_PID_FILE" "$BASE_LOG" \
        ros2 launch zsl_bringup robot_base.launch.py \
        "read_only:=$READ_ONLY" "rviz:=$RVIZ"
    fi
  fi

  if [[ "$NO_WAIT" != "true" ]]; then
    wait_topic /cloud_registered_body 40 "FAST-LIO 点云" || true
    wait_topic /scan 20 "二维激光" || true
  fi

  if node_exists /slam_toolbox; then
    log "slam_toolbox 已运行，不重复启动。"
  else
    log "启动 SLAM Toolbox……"
    start_group \
      "$MAPPING_PID_FILE" "$MAPPING_LOG" \
      ros2 launch zsl_bringup mapping.launch.py
  fi

  if [[ "$NO_WAIT" != "true" ]]; then
    wait_topic /map 45 "二维栅格地图" || {
      warn "没有检测到 /map，请检查："
      warn "  tail -f $MAPPING_LOG"
      warn "  ros2 topic hz /scan"
      warn "  ros2 run tf2_ros tf2_echo camera_init base_link"
    }
  fi

  ok "建图启动命令已执行。"
  echo "保存地图：$0 save floor_1"
  echo "只停止建图：$0 stop-mapping"
  echo "停止全部：$0 stop"
}

save_posegraph() {
  local prefix="$1"
  local type
  type="$(timeout 3 ros2 service type /slam_toolbox/serialize_map 2>/dev/null || true)"
  [[ "$type" == "slam_toolbox/srv/SerializePoseGraph" ]] || return 0

  log "保存 SLAM Toolbox 位姿图……"
  timeout 30 ros2 service call \
    /slam_toolbox/serialize_map \
    slam_toolbox/srv/SerializePoseGraph \
    "{filename: '${prefix}.posegraph'}" \
    >/tmp/zsl_posegraph_save.log 2>&1 || \
    warn "位姿图保存失败，但不影响 yaml/pgm 地图。"
}

save_map() {
  source_ros
  mkdir -p "$MAP_DIR"

  local name prefix backup
  name="$(sanitize_name "${1:-}")"
  prefix="$MAP_DIR/$name"

  topic_has_publisher /map || die "/map 没有发布者，无法保存。"
  ros2 pkg prefix nav2_map_server >/dev/null 2>&1 || \
    die "未安装 nav2_map_server：sudo apt install ros-$ROS_DISTRO_NAME-nav2-map-server"

  if [[ -e "$prefix.yaml" || -e "$prefix.pgm" || -e "$prefix.png" ]]; then
    backup="${prefix}_backup_$(date +%Y%m%d_%H%M%S)"
    warn "地图已存在，备份旧文件为 ${backup}_*"
    [[ -e "$prefix.yaml" ]] && mv "$prefix.yaml" "$backup.yaml"
    [[ -e "$prefix.pgm"  ]] && mv "$prefix.pgm"  "$backup.pgm"
    [[ -e "$prefix.png"  ]] && mv "$prefix.png"  "$backup.png"
  fi

  log "保存地图：$prefix.yaml + $prefix.pgm"
  if ! timeout 40 ros2 run nav2_map_server map_saver_cli -f "$prefix" \
      >/tmp/zsl_map_save.log 2>&1; then
    cat /tmp/zsl_map_save.log >&2 || true
    die "地图保存失败。"
  fi

  [[ -s "$prefix.yaml" ]] || die "没有生成 $prefix.yaml"
  [[ -s "$prefix.pgm" || -s "$prefix.png" ]] || die "没有生成地图图像。"

  save_posegraph "$prefix" || true
  ok "地图保存成功：$prefix.yaml"
  ls -lh "$prefix.yaml" "$prefix.pgm" "$prefix.png" 2>/dev/null || true
}

status() {
  source_ros
  echo "========================================"
  echo " ZSL-1W 建图状态"
  echo "========================================"
  echo "工作空间：$WORKSPACE"
  echo "地图目录：$MAP_DIR"
  echo

  cleanup_pid "$BASE_PID_FILE"
  cleanup_pid "$MAPPING_PID_FILE"
  pid_alive "$BASE_PID_FILE" && echo "robot_base: RUNNING PID=$(cat "$BASE_PID_FILE")" || echo "robot_base: STOPPED/EXTERNAL"
  pid_alive "$MAPPING_PID_FILE" && echo "mapping:    RUNNING PID=$(cat "$MAPPING_PID_FILE")" || echo "mapping:    STOPPED/EXTERNAL"
  echo

  for topic in /livox/lidar /livox/imu /cloud_registered_body /Odometry /scan /map; do
    if topic_has_publisher "$topic"; then
      echo "$topic: OK"
    else
      echo "$topic: NO PUBLISHER"
    fi
  done
  echo
  echo "日志："
  echo "  $BASE_LOG"
  echo "  $MAPPING_LOG"
}

stop_mapping_only() {
  stop_group "$MAPPING_PID_FILE" mapping
  ok "建图子栈已停止，基础驱动与传感器保持运行。"
}

stop_stack() {
  if [[ "${1:-}" == "--save" ]]; then
    save_map "${2:-}"
  fi
  stop_group "$MAPPING_PID_FILE" mapping
  stop_group "$BASE_PID_FILE" robot_base
  ok "本脚本启动的进程已停止。"
}

logs() {
  echo "===== robot_base ====="
  tail -n 100 "$BASE_LOG" 2>/dev/null || true
  echo "===== mapping ====="
  tail -n 100 "$MAPPING_LOG" 2>/dev/null || true
}

interactive_run() {
  local default_name
  default_name="$(sanitize_name "${1:-}")"
  start_stack

  trap 'echo; warn "收到中断，停止进程（未自动保存）"; stop_stack; exit 130' INT TERM

  cat <<HELP
交互命令：
  s  保存地图（名称：$default_name）
  t  查看状态
  q  保存并退出
  x  不保存直接退出
HELP

  while true; do
    printf '\n请输入 [s/t/q/x]: '
    IFS= read -r -n 1 key || key=x
    echo
    case "$key" in
      s|S) save_map "$default_name" ;;
      t|T) status ;;
      q|Q) save_map "$default_name"; stop_stack; break ;;
      x|X) stop_stack; break ;;
      *) warn "未知命令：$key" ;;
    esac
  done
}

command="${1:-help}"
shift || true
case "$command" in
  start)  start_stack ;;
  save)   save_map "${1:-}" ;;
  status) status ;;
  stop-mapping) stop_mapping_only ;;
  stop)   stop_stack "$@" ;;
  run)    interactive_run "${1:-}" ;;
  logs)   logs ;;
  help|-h|--help) usage ;;
  *) err "未知命令：$command"; usage; exit 2 ;;
esac
