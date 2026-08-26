cmake_minimum_required(VERSION 3.5)
project(path_following_v2)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "Build type" FORCE)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(path_following_v2_node src/path_following_v2_node.cpp)
add_executable(path_generator_node src/path_generator_node.cpp)
add_executable(local_trajectory_planner_node src/local_trajectory_planner_node.cpp)
target_include_directories(local_trajectory_planner_node PRIVATE include)

ament_target_dependencies(path_following_v2_node
  rclcpp
  diagnostic_msgs
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
  diagnostic_msgs
  nav_msgs
  geometry_msgs
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

ament_target_dependencies(local_trajectory_planner_node
  rclcpp
  diagnostic_msgs
  nav_msgs
  geometry_msgs
  sensor_msgs
  std_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

install(TARGETS
  path_following_v2_node
  path_generator_node
  local_trajectory_planner_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(FILES README.md
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(lightweight_frenet_lattice_test
    test/lightweight_frenet_lattice_test.cpp
  )
  target_include_directories(lightweight_frenet_lattice_test PRIVATE include)

  ament_add_gtest(lattice_clearance_test
    test/lattice_clearance_test.cpp
  )
  target_include_directories(lattice_clearance_test PRIVATE include)

  ament_add_gtest(bounded_corridor_smoother_test
    test/bounded_corridor_smoother_test.cpp
  )
  target_include_directories(bounded_corridor_smoother_test PRIVATE include)

  ament_add_gtest(candidate_selection_test
    test/candidate_selection_test.cpp
  )
  target_include_directories(candidate_selection_test PRIVATE include)

  ament_add_gtest(active_path_safety_test
    test/active_path_safety_test.cpp
  )
  target_include_directories(active_path_safety_test PRIVATE include)

  ament_add_gtest(maneuver_speed_policy_test
    test/maneuver_speed_policy_test.cpp
  )
  target_include_directories(maneuver_speed_policy_test PRIVATE include)

  ament_add_gtest(path_splice_test
    test/path_splice_test.cpp
  )
  target_include_directories(path_splice_test PRIVATE include)

  ament_add_gtest(polyline_query_test
    test/polyline_query_test.cpp
  )
  target_include_directories(polyline_query_test PRIVATE include)

  ament_add_gtest(scan_pooling_test
    test/scan_pooling_test.cpp
  )
  target_include_directories(scan_pooling_test PRIVATE include)
endif()

ament_package()
