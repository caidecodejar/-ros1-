#!/usr/bin/env bash
set -euo pipefail

source "$HOME/eprobot_env.sh" 2>/dev/null || true
source "$HOME/robot_ws/devel/setup.bash"

export ROBOT_TYPE="${ROBOT_TYPE:-EPRobotV2.2}"
export ROS_MASTER_URI="${ROS_MASTER_URI:-http://192.168.12.1:11311}"
export ROS_IP="${ROS_IP:-192.168.12.1}"
unset ROS_HOSTNAME

WAYPOINTS="${WAYPOINTS:-$HOME/robot_ws/src/ros1_smart_pharmacy_patrol/config/waypoints_real.yaml}"

exec roslaunch ros1_smart_pharmacy_patrol patrol.launch \
  send_goals:="${SEND_GOALS:-true}" \
  waypoints:="$WAYPOINTS" \
  board1:="${BOARD1:-A:1}" \
  board2:="${BOARD2:-FREE}" \
  timeout:="${PATROL_TIMEOUT:-120.0}" \
  use_live_board1:="${USE_LIVE_BOARD1:-false}" \
  use_live_board2:="${USE_LIVE_BOARD2:-false}" \
  execute_all_board1_tasks:="${EXECUTE_ALL_BOARD1_TASKS:-true}" \
  max_board1_tasks:="${MAX_BOARD1_TASKS:-2}" \
  board2_apply_only_matching_window:="${BOARD2_APPLY_ONLY_MATCHING_WINDOW:-true}" \
  "$@"
