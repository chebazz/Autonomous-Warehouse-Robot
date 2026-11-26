cmake_minimum_required(VERSION 3.8)
project(amr_navigation)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch maps rviz DESTINATION share/${PROJECT_NAME})
install(PROGRAMS
  scripts/scan_merger.py
  scripts/mapping_route.py
  scripts/save_map.sh
  scripts/make_keepout_map.py
  DESTINATION lib/${PROJECT_NAME})

ament_package()
