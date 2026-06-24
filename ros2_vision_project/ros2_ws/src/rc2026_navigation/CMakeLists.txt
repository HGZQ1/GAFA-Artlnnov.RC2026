cmake_minimum_required(VERSION 3.8)
project(rc2026_navigation)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch map
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/map_relocalizer.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME map_relocalizer
)

ament_package()
