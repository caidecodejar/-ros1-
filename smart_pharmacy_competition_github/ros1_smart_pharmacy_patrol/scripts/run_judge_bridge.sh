#!/usr/bin/env bash
set -euo pipefail

source "$HOME/eprobot_env.sh" 2>/dev/null || true
source "$HOME/robot_ws/devel/setup.bash"

export ROBOT_TYPE="${ROBOT_TYPE:-EPRobotV2.2}"
export ROS_MASTER_URI="${ROS_MASTER_URI:-http://192.168.12.1:11311}"
export ROS_IP="${ROS_IP:-192.168.12.1}"
unset ROS_HOSTNAME

exec roslaunch ros1_smart_pharmacy_patrol judge_bridge.launch \
  judge_host:="${JUDGE_HOST:-192.168.12.248}" \
  judge_port:="${JUDGE_PORT:-8888}" \
  source_ip:="${JUDGE_SOURCE_IP:-}" \
  send_hz:="${JUDGE_SEND_HZ:-5.0}" \
  initial_task:="${JUDGE_INITIAL_TASK:-start}" \
  initial_cv1:="${JUDGE_INITIAL_CV1:-}" \
  initial_cv2:="${JUDGE_INITIAL_CV2:-}" \
  "$@"
