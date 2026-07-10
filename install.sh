#!/usr/bin/env bash
set -euo pipefail

WORKSPACE="${1:-$HOME/wheel_nav_ws}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
STAMP="$(date +%Y%m%d_%H%M%S)"
BACKUP_DIR="$WORKSPACE/.zsl_web_backup/$STAMP"

if [[ ! -d "$WORKSPACE/src" ]]; then
  echo "[ERROR] ROS 2 workspace not found: $WORKSPACE"
  echo "Usage: bash install.sh /home/nvidia/wheel_nav_ws"
  exit 1
fi

mkdir -p "$BACKUP_DIR"

WEB_DST="$WORKSPACE/src/web/zsl_web_control"
BRINGUP_DST="$WORKSPACE/src/bringup/zsl_bringup"

if [[ -d "$WEB_DST" ]]; then
  mkdir -p "$BACKUP_DIR/src/web"
  cp -a "$WEB_DST" "$BACKUP_DIR/src/web/"
fi

for file in navigation.launch.py mapping.launch.py managed_navigation.launch.py; do
  if [[ -f "$BRINGUP_DST/launch/$file" ]]; then
    mkdir -p "$BACKUP_DIR/src/bringup/zsl_bringup/launch"
    cp -a "$BRINGUP_DST/launch/$file" "$BACKUP_DIR/src/bringup/zsl_bringup/launch/"
  fi
done
for file in CMakeLists.txt package.xml; do
  if [[ -f "$BRINGUP_DST/$file" ]]; then
    mkdir -p "$BACKUP_DIR/src/bringup/zsl_bringup"
    cp -a "$BRINGUP_DST/$file" "$BACKUP_DIR/src/bringup/zsl_bringup/"
  fi
done

mkdir -p "$WORKSPACE/src/web" "$BRINGUP_DST/launch"
rm -rf "$WEB_DST"
cp -a "$SCRIPT_DIR/src/web/zsl_web_control" "$WEB_DST"
cp -a "$SCRIPT_DIR/src/bringup/zsl_bringup/launch/." "$BRINGUP_DST/launch/"
cp -a "$SCRIPT_DIR/src/bringup/zsl_bringup/CMakeLists.txt" "$BRINGUP_DST/CMakeLists.txt"
cp -a "$SCRIPT_DIR/src/bringup/zsl_bringup/package.xml" "$BRINGUP_DST/package.xml"

mkdir -p "$HOME/gb_maps"

echo "[OK] Files installed"
echo "[OK] Backup: $BACKUP_DIR"

if ! python3 -c 'import aiohttp, yaml' >/dev/null 2>&1; then
  echo "[WARN] Missing Python runtime dependencies. Install with:"
  echo "       sudo apt update && sudo apt install -y python3-aiohttp python3-yaml"
fi

if [[ "${2:-}" == "--build" ]]; then
  source /opt/ros/humble/setup.bash
  cd "$WORKSPACE"
  colcon build --symlink-install --packages-select zsl_web_control zsl_bringup
  echo "[OK] Build completed"
  echo "Run: source $WORKSPACE/install/setup.bash"
else
  echo "Next:"
  echo "  cd $WORKSPACE"
  echo "  source /opt/ros/humble/setup.bash"
  echo "  colcon build --symlink-install --packages-select zsl_web_control zsl_bringup"
  echo "  source install/setup.bash"
fi
