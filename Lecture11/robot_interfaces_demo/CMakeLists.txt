cmake_minimum_required(VERSION 3.8)
project(robot_interfaces_demo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(robot_custom_interfaces REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(include)

add_executable(robot_status_demo
  src/robot_status.cpp
)

ament_target_dependencies(robot_status_demo
  rclcpp
  robot_custom_interfaces
  std_msgs
)

install(TARGETS
  robot_status_demo
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_dependencies(
  rclcpp
  robot_custom_interfaces
  std_msgs
)

ament_package()