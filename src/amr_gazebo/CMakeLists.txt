cmake_minimum_required(VERSION 3.8)
project(amr_gazebo)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch config worlds
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS scripts/lift_cmd.py scripts/lift_stage_relay.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
