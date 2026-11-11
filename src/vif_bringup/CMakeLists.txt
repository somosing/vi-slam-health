cmake_minimum_required(VERSION 3.8)
project(vif_bringup)

find_package(ament_cmake REQUIRED)
install(DIRECTORY launch config params DESTINATION share/${PROJECT_NAME})
install(PROGRAMS
  scripts/slam_map_odom.py
  scripts/slam_tracking_cmd_gate.py
  DESTINATION lib/${PROJECT_NAME})
ament_package()
