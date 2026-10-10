cmake_minimum_required(VERSION 3.8)
project(atlas_slam)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch maps rviz
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/collision_alarm_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
