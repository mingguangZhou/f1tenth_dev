cmake_minimum_required(VERSION 3.5)
project(reactive_control_v2)

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
find_package(ackermann_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

add_executable(upper_corridor_follower src/upper_corridor_follower.cpp)
ament_target_dependencies(upper_corridor_follower
  ackermann_msgs
  builtin_interfaces
  diagnostic_msgs
  geometry_msgs
  nav_msgs
  rclcpp
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
  visualization_msgs
)

add_executable(lower_safety_controller src/lower_safety_controller.cpp)
target_include_directories(lower_safety_controller PRIVATE include)
ament_target_dependencies(lower_safety_controller
  ackermann_msgs
  builtin_interfaces
  diagnostic_msgs
  geometry_msgs
  nav_msgs
  rclcpp
  sensor_msgs
  std_msgs
)

install(TARGETS upper_corridor_follower lower_safety_controller
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch rviz
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(FILES README.md LICENSE
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(raceline_stall_handoff_test
    test/raceline_stall_handoff_test.cpp
  )
  target_include_directories(raceline_stall_handoff_test PRIVATE include)

  ament_add_gtest(wrong_way_recovery_test
    test/wrong_way_recovery_test.cpp
  )
  target_include_directories(wrong_way_recovery_test PRIVATE include)

  ament_add_gtest(reverse_swept_safety_test
    test/reverse_swept_safety_test.cpp
  )
  target_include_directories(reverse_swept_safety_test PRIVATE include)
endif()

ament_package()
