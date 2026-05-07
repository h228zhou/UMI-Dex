#!/usr/bin/env bash
# Interactive recorder wrapper — runs via `ros2 run` so stdin stays connected.
# Usage:
#   ros2 run umi_dex_bringup record.sh [--protocol can|usart]
#                                      [--bag-dir DIR] [--warmup SECS]
#                                      [--base-name NAME]
#                                      [--no-slam]
#                                      [--slam-vocab PATH] [--slam-settings PATH]
#
# Repo-root detection: walks up from $PWD (or $UMI_DEX_ROOT) looking for the
# umi-dex pyproject.toml, then adds its src/ and .venv/ site-packages to
# PYTHONPATH so the colcon-installed ROS2 node can import umi_dex + orbslam3.
set -euo pipefail

find_repo_root() {
  local d="${UMI_DEX_ROOT:-$PWD}"
  d="$(cd "$d" 2>/dev/null && pwd)" || return 1
  while [[ "$d" != "/" ]]; do
    if [[ -f "$d/pyproject.toml" ]] \
       && grep -q '^name = "umi-dex"' "$d/pyproject.toml" 2>/dev/null; then
      echo "$d"; return 0
    fi
    d="$(dirname "$d")"
  done
  return 1
}

repo_root="$(find_repo_root || true)"
if [[ -n "$repo_root" ]]; then
  for site in "$repo_root"/.venv/lib/python*/site-packages; do
    if [[ -d "$site" ]]; then
      export PYTHONPATH="$site${PYTHONPATH:+:$PYTHONPATH}"
      break
    fi
  done
  export PYTHONPATH="$repo_root/src${PYTHONPATH:+:$PYTHONPATH}"
fi

protocol="can"
bag_dir="outputs"
warmup="60.0"
base_name="capture"
enable_slam="true"
slam_vocab="${repo_root:-$PWD}/config/ORBvoc.txt"
slam_settings="${repo_root:-$PWD}/config/intel_d455.yaml"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --protocol)       protocol="$2"; shift 2 ;;
    --bag-dir)        bag_dir="$2"; shift 2 ;;
    --warmup)         warmup="$2"; shift 2 ;;
    --base-name)      base_name="$2"; shift 2 ;;
    --no-slam)        enable_slam="false"; shift 1 ;;
    --slam-vocab)     slam_vocab="$2"; shift 2 ;;
    --slam-settings)  slam_settings="$2"; shift 2 ;;
    -h|--help)        sed -n '2,10p' "$0"; exit 0 ;;
    *) echo "unknown arg: $1" >&2; exit 2 ;;
  esac
done

case "$protocol" in
  can)   hand_topic="/hand/can_raw" ;;
  usart) hand_topic="/hand/usart_raw" ;;
  *) echo "--protocol must be 'can' or 'usart'" >&2; exit 2 ;;
esac

[[ "$warmup" == *.* ]] || warmup="${warmup}.0"

topics="['/camera/infra1/image_rect_raw','/camera/infra1/camera_info',\
'/camera/infra2/image_rect_raw','/camera/infra2/camera_info',\
'/camera/imu','/camera_d405/color/image_raw','/camera_d405/color/camera_info',\
'${hand_topic}','/session/episode']"

exec ros2 run umi_dex_bringup interactive_capture_node --ros-args \
  -p "bag_dir:=${bag_dir}" \
  -p "base_name:=${base_name}" \
  -p "warmup_timeout_s:=${warmup}" \
  -p "enable_slam_probe:=${enable_slam}" \
  -p "slam_vocab_path:=${slam_vocab}" \
  -p "slam_settings_path:=${slam_settings}" \
  -p "episode_topic:=/session/episode" \
  -p "topics:=${topics}"
