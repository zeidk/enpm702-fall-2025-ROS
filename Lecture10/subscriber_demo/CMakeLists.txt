cmake_minimum_required(VERSION 3.8)
project(subscriber_demo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Add include directories
include_directories(include)

add_executable(subscriber_demo src/main.cpp src/subscriber_demo.cpp)
ament_target_dependencies(subscriber_demo rclcpp std_msgs)

# Add this before ament_package()
install(TARGETS
  subscriber_demo
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
