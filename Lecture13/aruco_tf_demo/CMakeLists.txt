cmake_minimum_required(VERSION 3.8)
project(aruco_tf_demo)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(orocos_kdl_vendor REQUIRED)

# Include directories
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

# ==============================================================================
# Executables
# ==============================================================================

# ArUco detector node
add_executable(aruco_detector_node src/aruco_detector_node.cpp)
ament_target_dependencies(aruco_detector_node
  rclcpp
  sensor_msgs
  geometry_msgs
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  image_transport
  cv_bridge
  orocos_kdl
  kdl_parser
)
target_link_libraries(aruco_detector_node ${OpenCV_LIBRARIES})

# ==============================================================================
# Install
# ==============================================================================

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install world files
install(DIRECTORY worlds/
  DESTINATION share/${PROJECT_NAME}/worlds
)

# Install model files
install(DIRECTORY models/
  DESTINATION share/${PROJECT_NAME}/models
)

# Install config files
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# Install executables
install(TARGETS
  aruco_detector_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
