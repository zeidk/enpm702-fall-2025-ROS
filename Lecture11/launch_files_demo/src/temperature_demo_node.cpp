#include "launch_files_demo/temperature_demo_node.hpp"
#include "launch_files_demo/color_utils.hpp"
#include <random>

namespace launch_demo_nodes {

TemperatureDemoNode::TemperatureDemoNode() : Node("temperature_demo_node") {
  // Create publisher
  data_temperature_publisher_ =
      this->create_publisher<sensor_msgs::msg::Temperature>("/data/temperature",
                                                            10);

  // Set up timer to publish at regular intervals
  auto timer_period = std::chrono::duration<double>(1.0 / PUBLISH_FREQUENCY);
  data_temperature_timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(timer_period),
      std::bind(&TemperatureDemoNode::data_temperature_pub_callback, this));

  RCLCPP_INFO(this->get_logger(), "Temperature Demo Node started");
}

void TemperatureDemoNode::data_temperature_pub_callback() {
  // Set header information
  data_temperature_msg_.header.stamp = this->get_clock()->now();
  data_temperature_msg_.header.frame_id = "temperature_frame";

  // Create random number generator
  static std::random_device rd;
  static std::mt19937 gen(rd());
  static std::uniform_real_distribution<double> temp_dis(TEMP_MIN, TEMP_MAX);

  // Generate random temperature data between -10°C and 40°C
  data_temperature_msg_.temperature = temp_dis(gen);
  data_temperature_msg_.variance = TEMP_VARIANCE; // Variance in the measurement

  // Publish the message
  data_temperature_publisher_->publish(data_temperature_msg_);
  RCLCPP_INFO(this->get_logger(), "%sPublished temperature: %.2f°C%s",
              Color::YELLOW, data_temperature_msg_.temperature, Color::RESET);
}

} // namespace launch_demo_nodes

int main(int argc, char **argv) {
  try {
    // Initialize ROS2 communication
    rclcpp::init(argc, argv);

    // Create and initialize the node
    auto node = std::make_shared<launch_demo_nodes::TemperatureDemoNode>();

    // Spin the node to process callbacks
    rclcpp::spin(node);

  } catch (const std::exception &e) {
    // Catch and report any exceptions
    std::cerr << "Error occurred: " << e.what() << std::endl;
    return 1;
  }

  // Cleanup resources properly
  rclcpp::shutdown();
  return 0;
}