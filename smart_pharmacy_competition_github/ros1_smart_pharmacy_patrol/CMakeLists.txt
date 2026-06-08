cmake_minimum_required(VERSION 3.0.2)
project(ros1_smart_pharmacy_patrol)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  geometry_msgs
  move_base_msgs
  nav_msgs
  rospy
  sensor_msgs
  std_msgs
  tf
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/auto_arrival_detector.py
  scripts/fake_judge_server.py
  scripts/judge_bridge.py
  scripts/judge_smoke_test.py
  scripts/manual_arrival_voice_node.py
  scripts/board2_ocr_node.py
  scripts/patrol_mission.py
  scripts/qrcode_voice_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS
  scripts/run_judge_bridge.sh
  scripts/run_manual_competition.sh
  scripts/run_patrol_real.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch sounds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
