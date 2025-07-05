cmake_minimum_required(VERSION 3.5)
project(boundary_detection)

# Default to C++20
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -g)
endif()

# Node executable
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(boundary_detection_node src/node.cpp)
ament_target_dependencies(boundary_detection_node
  rclcpp
  sensor_msgs
  geometry_msgs
  visualization_msgs
  std_msgs
)

install(TARGETS boundary_detection_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files if present
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
  OPTIONAL
)

# Package setup
ament_package()
