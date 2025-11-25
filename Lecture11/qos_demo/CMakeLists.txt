cmake_minimum_required(VERSION 3.8)
project(qos_demo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

# Include directories
include_directories(include)

# Sensor Monitor executable
add_executable(sensor_monitor 
  src/sensor_monitor.cpp
)
ament_target_dependencies(sensor_monitor
  rclcpp
  sensor_msgs
)

# Install executables
install(TARGETS
  sensor_monitor
  DESTINATION lib/${PROJECT_NAME}
)

# Install include directory
install(DIRECTORY include/
  DESTINATION include/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()