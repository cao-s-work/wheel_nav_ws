#!/usr/bin/env bash
set -euo pipefail

BASE_URL="${1:-http://127.0.0.1:8080}"
TOKEN="${ZSL_API_TOKEN:-}"
AUTH=()
if [[ -n "$TOKEN" ]]; then AUTH=(-H "Authorization: Bearer $TOKEN"); fi

echo "== HTTP health =="
curl -fsS "${AUTH[@]}" "$BASE_URL/healthz" | python3 -m json.tool

echo "== API state =="
curl -fsS "${AUTH[@]}" "$BASE_URL/api/v1/state" | python3 -m json.tool | head -80

echo "== ROS velocity chain =="
for topic in /cmd_vel /cmd_vel_teleop /cmd_vel_selected /cmd_vel_safe /web/teleop_active; do
  echo "--- $topic"
  timeout 4 ros2 topic info -v "$topic" || true
done

echo "== Web / Nav2 services =="
ros2 service list | grep -E 'save_map|load_map|clear_entirely|global_localization|nomotion|zsl_driver' || true
