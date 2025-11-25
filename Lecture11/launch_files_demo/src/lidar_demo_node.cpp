#include "launch_files_demo/lidar_demo_node.hpp"
#include "launch_files_demo/color_utils.hpp"
#include <cmath>
#include <random>

namespace launch_demo_nodes {

LidarDemoNode::LidarDemoNode() : Node("lidar_demo_node") {
  // Create publisher
  data_lidar_publisher_ =
      this->create_publisher<sensor_msgs::msg::LaserScan>("/data/lidar", 10);

  // Set up timer to publish at regular intervals
  auto timer_period = std::chrono::duration<double>(1.0 / PUBLISH_FREQUENCY);
  data_lidar_timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(timer_period),
      std::bind(&LidarDemoNode::data_lidar_pub_callback, this));

  RCLCPP_INFO(this->get_logger(), "LiDAR Demo Node started");
}

void LidarDemoNode::data_lidar_pub_callback() {
  // Set header information
  data_lidar_msg_.header.stamp = this->get_clock()->now();
  data_lidar_msg_.header.frame_id = "laser_frame";

  // Set the angle and range parameters
  data_lidar_msg_.angle_min = ANGLE_MIN;
  data_lidar_msg_.angle_max = ANGLE_MAX;
  data_lidar_msg_.angle_increment = ANGLE_INCREMENT;
  data_lidar_msg_.time_increment = 0.0;
  data_lidar_msg_.scan_time = SCAN_TIME;
  data_lidar_msg_.range_min = RANGE_MIN;
  data_lidar_msg_.range_max = RANGE_MAX;

  // Number of readings based on angle range
  int num_readings =
      static_cast<int>((ANGLE_MAX - ANGLE_MIN) / ANGLE_INCREMENT);

  // Resize vectors
  data_lidar_msg_.ranges.resize(num_readings);
  data_lidar_msg_.intensities.resize(num_readings);

  // Create random number generators
  static std::random_device rd;
  static std::mt19937 gen(rd());
  static std::uniform_real_distribution<float> range_dis(RANGE_MIN, RANGE_MAX);
  static std::uniform_real_distribution<float> intensity_dis(0.0f, 1.0f);

  // Generate random ranges and intensities
  for (int i = 0; i < num_readings; ++i) {
    data_lidar_msg_.ranges[i] = range_dis(gen);
    data_lidar_msg_.intensities[i] = intensity_dis(gen);
  }

  // Publish the message
  data_lidar_publisher_->publish(data_lidar_msg_);
  RCLCPP_INFO(this->get_logger(), "%sPublished lidar data%s", Color::GREEN,
              Color::RESET);
}

} // namespace launch_demo_nodes

int main(int argc, char **argv) {
  try {
    // Initialize ROS2 communication
    rclcpp::init(argc, argv);

    // Create and initialize the node
    auto node = std::make_shared<launch_demo_nodes::LidarDemoNode>();

    // Spin the node to process callbacks
    rclcpp::spin(node);

  } catch (const std::exception &e) {
    // Catch and report any exceptions
    std::cerr << "Error occurred: " << e.what() << std::endl;
    return 1;
  }

  // Cleanup resources properly
  rclcpp::shutdown();
}