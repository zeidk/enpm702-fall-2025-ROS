cmake_minimum_required(VERSION 3.8)
project(publisher_demo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Add include directories
include_directories(include)

add_executable(publisher_demo src/main.cpp src/publisher_demo.cpp)
ament_target_dependencies(publisher_demo rclcpp std_msgs)

# Add this before ament_package()
install(TARGETS
  publisher_demo
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
