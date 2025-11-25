cmake_minimum_required(VERSION 3.8)
project(executors_demo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# Add include directories
include_directories(include)

# Create executable
add_executable(single_threaded src/single_threaded.cpp)
add_executable(two_mutex src/two_mutex.cpp)
add_executable(mutex_reentrant src/mutex_reentrant.cpp)

# Specify dependencies
ament_target_dependencies(single_threaded
  rclcpp
)

ament_target_dependencies(two_mutex
  rclcpp
)

ament_target_dependencies(mutex_reentrant
  rclcpp
)

# Install targets
install(TARGETS
  single_threaded
  two_mutex
  mutex_reentrant
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()
