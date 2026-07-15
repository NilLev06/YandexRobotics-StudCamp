cmake_minimum_required(VERSION 3.8)
project(main)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})


install(PROGRAMS scripts/goal_proxy.py DESTINATION lib/${PROJECT_NAME})

ament_package()
