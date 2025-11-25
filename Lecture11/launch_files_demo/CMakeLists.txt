cmake_minimum_required(VERSION 3.8)
project(launch_files_demo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

# Include directories
include_directories(include)

# Camera Demo Node
add_executable(camera_demo_node 
  src/camera_demo_node.cpp 
)
ament_target_dependencies(camera_demo_node
  rclcpp
  sensor_msgs
)

# LiDAR Demo Node
add_executable(lidar_demo_node 
  src/lidar_demo_node.cpp 
)
ament_target_dependencies(lidar_demo_node
  rclcpp
  sensor_msgs
)

# Temperature Demo Node
add_executable(temperature_demo_node 
  src/temperature_demo_node.cpp 
)
ament_target_dependencies(temperature_demo_node
  rclcpp
  sensor_msgs
)

# Processing Demo Node
add_executable(processing_demo_node 
  src/processing_demo_node.cpp 
)
ament_target_dependencies(processing_demo_node
  rclcpp
  sensor_msgs
)

# Install targets
install(TARGETS
  camera_demo_node
  lidar_demo_node
  temperature_demo_node
  processing_demo_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install header files
install(DIRECTORY include/
  DESTINATION include/
)
# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
