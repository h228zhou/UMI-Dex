#!/usr/bin/env bash
# Interactive recorder wrapper — runs via `ros2 run` so stdin stays connected.
# Usage:
#   ros2 run umi_dex_bringup record.sh [--protocol can|usart]
#                                      [--bag-dir DIR] [--warmup SECS]
#                                      [--base-name NAME]
set -euo pipefail

protocol="can"
bag_dir="outputs"
warmup="15.0"
base_name="capture"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --protocol)   protocol="$2"; shift 2 ;;
    --bag-dir)    bag_dir="$2"; shift 2 ;;
    --warmup)     warmup="$2"; shift 2 ;;
    --base-name)  base_name="$2"; shift 2 ;;
    -h|--help)    sed -n '2,7p' "$0"; exit 0 ;;
    *) echo "unknown arg: $1" >&2; exit 2 ;;
  esac
done

case "$protocol" in
  can)   hand_topic="/hand/can_raw" ;;
  usart) hand_topic="/hand/usart_raw" ;;
  *) echo "--protocol must be 'can' or 'usart'" >&2; exit 2 ;;
esac

topics="['/camera/infra1/image_rect_raw','/camera/infra1/camera_info',\
'/camera/infra2/image_rect_raw','/camera/infra2/camera_info',\
'/camera/imu','/camera_d405/color/image_raw','/camera_d405/color/camera_info',\
'${hand_topic}','/session/episode']"

exec ros2 run umi_dex_bringup interactive_capture_node --ros-args \
  -p "bag_dir:=${bag_dir}" \
  -p "base_name:=${base_name}" \
  -p "warmup_duration_s:=${warmup}" \
  -p "episode_topic:=/session/episode" \
  -p "topics:=${topics}"
