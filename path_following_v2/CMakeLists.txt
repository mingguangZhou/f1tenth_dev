cmake_minimum_required(VERSION 3.5)
project(path_following_v2)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(path_following_v2_node src/path_following_v2_node.cpp)
add_executable(path_generator_node src/path_generator_node.cpp)

ament_target_dependencies(path_following_v2_node
  rclcpp
  nav_msgs
  geometry_msgs
  visualization_msgs
  ackermann_msgs
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

ament_target_dependencies(path_generator_node
  rclcpp
  nav_msgs
  geometry_msgs
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

install(TARGETS
  path_following_v2_node
  path_generator_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()