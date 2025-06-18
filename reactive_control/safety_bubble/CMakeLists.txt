cmake_minimum_required(VERSION 3.5)
project(safety_bubble)

# Default to C++20
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -g)
endif()

# Find the package before foreach
find_package(rosidl_default_generators REQUIRED)
find_package(visualization_msgs REQUIRED)

# Find dependencies
foreach(PACKAGE 
  ament_cmake 
  ament_cmake_python
  rclcpp
  rclpy 
  geometry_msgs
  nav_msgs
  ackermann_msgs
  sensor_msgs
  std_msgs)

  find_package(${PACKAGE} REQUIRED)

endforeach()

# Add message definition section here
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/BubbleInfo.msg"
  DEPENDENCIES std_msgs  # Add others if needed
)

# Include Cpp "include" directory
include_directories(include)

# Create Cpp executable
add_executable(safety_bubble_node src/safety_bubble_node.cpp)
ament_target_dependencies(safety_bubble_node 
  rclcpp geometry_msgs ackermann_msgs nav_msgs sensor_msgs std_msgs
)
add_executable(visualization_node src/visualization_node.cpp)
ament_target_dependencies(visualization_node
  rclcpp std_msgs geometry_msgs visualization_msgs
)

rosidl_target_interfaces(safety_bubble_node  # Add this block after ament_target_dependencies
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

# Install Cpp executables
install(TARGETS
  safety_bubble_node
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS
  visualization_node
  DESTINATION lib/${PROJECT_NAME})

# Install Launch
install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install the config directory
install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Export message runtime dependency
ament_export_dependencies(rosidl_default_runtime)

ament_package()
