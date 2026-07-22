cmake_minimum_required(VERSION 3.8)
project(main)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/goal_proxy.py
    scripts/save_cartographer_map.sh
    scripts/check_cartographer.sh
    scripts/stop_maze.sh
    scripts/start_maze.sh
    scripts/check_maze.sh
    scripts/frontier_core.py
    scripts/maze_explorer.py
    scripts/odom_sanitizer.py
    scripts/battery_guard.py
    scripts/object_search_manager.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
