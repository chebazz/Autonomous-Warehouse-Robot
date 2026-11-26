cmake_minimum_required(VERSION 3.8)
project(amr_tasks)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/task_manager.py DESTINATION lib/${PROJECT_NAME})

ament_package()
