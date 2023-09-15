cmake_minimum_required(VERSION 3.5)
project(costmap_filters)

set(CMAKE_CXX_STANDARD 14)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch maps params
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
