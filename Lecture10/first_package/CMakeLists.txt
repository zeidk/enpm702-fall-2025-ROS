cmake_minimum_required(VERSION 3.8)
project(first_package)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Add include directories
include_directories(include)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# add_executable(hello_demo src/main.cpp)
# ament_target_dependencies(hello_demo rclcpp)

add_executable(hello_demo src/main.cpp src/hello_node.cpp)
ament_target_dependencies(hello_demo rclcpp)

# Add this before ament_package()
install(TARGETS
  hello_demo
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
