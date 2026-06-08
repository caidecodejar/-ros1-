#!/usr/bin/env bash
set -e

source ~/eprobot_env.sh 2>/dev/null || true
source ~/robot_ws/devel/setup.bash

export ROBOT_TYPE="${ROBOT_TYPE:-EPRobotV2.2}"
export ROS_MASTER_URI="${ROS_MASTER_URI:-http://192.168.12.1:11311}"
export ROS_IP="${ROS_IP:-192.168.12.1}"
unset ROS_HOSTNAME

JUDGE_HOST="${JUDGE_HOST:-192.168.12.248}"
JUDGE_PORT="${JUDGE_PORT:-8888}"
JUDGE_SOURCE_IP="${JUDGE_SOURCE_IP:-192.168.12.1}"
START_BOARD2_OCR="${START_BOARD2_OCR:-true}"
START_AUTO_ARRIVAL_DETECTOR="${START_AUTO_ARRIVAL_DETECTOR:-false}"
INITIAL_CV2="${INITIAL_CV2:-FREE}"

exec roslaunch ros1_smart_pharmacy_patrol manual_competition.launch \
  judge_host:="${JUDGE_HOST}" \
  judge_port:="${JUDGE_PORT}" \
  judge_source_ip:="${JUDGE_SOURCE_IP}" \
  start_board2_ocr:="${START_BOARD2_OCR}" \
  start_auto_arrival_detector:="${START_AUTO_ARRIVAL_DETECTOR}" \
  initial_cv2:="${INITIAL_CV2}"
