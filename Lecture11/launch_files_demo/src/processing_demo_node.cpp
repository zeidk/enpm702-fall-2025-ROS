#include "launch_files_demo/processing_demo_node.hpp"
#include "launch_files_demo/color_utils.hpp"

namespace launch_demo_nodes {

ProcessingDemoNode::ProcessingDemoNode() : Node("processing_demo_node") {
  // Create subscribers for the three different data types
  data_lidar_subscription_ =
      this->create_subscription<sensor_msgs::msg::LaserScan>(
          "/data/lidar", 10,
          std::bind(&ProcessingDemoNode::data_lidar_callback, this,
                    std::placeholders::_1));

  data_camera_subscription_ =
      this->create_subscription<sensor_msgs::msg::Image>(
          "/data/camera", 10,
          std::bind(&ProcessingDemoNode::data_camera_callback, this,
                    std::placeholders::_1));

  data_temperature_subscription_ =
      this->create_subscription<sensor_msgs::msg::Temperature>(
          "/data/temperature", 10,
          std::bind(&ProcessingDemoNode::data_temperature_callback, this,
                    std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "Processing Demo Node started");
}

void ProcessingDemoNode::data_lidar_callback(
    const sensor_msgs::msg::LaserScan::SharedPtr msg) {
  if (!msg->ranges.empty()) {
    RCLCPP_INFO(this->get_logger(), "%sReceived LiDAR data: ranges[0]=%.2f%s",
                Color::GREEN, msg->ranges[0], Color::RESET);
  } else {
    RCLCPP_INFO(this->get_logger(),
                "%sReceived LiDAR data: no ranges available%s", Color::GREEN,
                Color::RESET);
  }
}

void ProcessingDemoNode::data_camera_callback(
    const sensor_msgs::msg::Image::SharedPtr msg) {
  RCLCPP_INFO(this->get_logger(),
              "%sReceived Camera data: width=%d, height=%d%s", Color::PURPLE,
              msg->width, msg->height, Color::RESET);
}

void ProcessingDemoNode::data_temperature_callback(
    const sensor_msgs::msg::Temperature::SharedPtr msg) {
  RCLCPP_INFO(this->get_logger(), "%sReceived Temperature data: %.2f°C%s",
              Color::YELLOW, msg->temperature, Color::RESET);
}

} // namespace launch_demo_nodes

int main(int argc, char **argv) {
  try {
    // Initialize ROS2 communication
    rclcpp::init(argc, argv);

    // Create the node
    auto node = std::make_shared<launch_demo_nodes::ProcessingDemoNode>();

    // Create a MultiThreadedExecutor
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);

    // Spin the executor
    executor.spin();

  } catch (const std::exception &e) {
    // Catch and report any exceptions
    std::cerr << "Error occurred: " << e.what() << std::endl;
    return 1;
  }

  // Cleanup resources properly
  rclcpp::shutdown();
}