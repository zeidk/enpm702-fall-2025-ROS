#include "launch_files_demo/camera_demo_node.hpp"
#include "launch_files_demo/color_utils.hpp"
#include <random>
#include <rclcpp/rclcpp.hpp>

namespace launch_demo_nodes {

CameraDemoNode::CameraDemoNode() : Node("camera_demo_node") {
  // Create publisher
  data_camera_publisher_ =
      this->create_publisher<sensor_msgs::msg::Image>("/data/camera", 10);

  // Set up timer to publish at regular intervals
  auto timer_period = std::chrono::duration<double>(1.0 / PUBLISH_FREQUENCY);
  data_camera_timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(timer_period),
      std::bind(&CameraDemoNode::data_camera_pub_callback, this));

  RCLCPP_INFO(this->get_logger(), "Camera Demo Node started");
}

void CameraDemoNode::data_camera_pub_callback() {
  // Set header information
  data_camera_msg_.header.stamp = this->get_clock()->now();
  data_camera_msg_.header.frame_id = "camera_frame";

  // Set image parameters
  const int step = IMAGE_WIDTH * CHANNELS;

  data_camera_msg_.height = IMAGE_HEIGHT;
  data_camera_msg_.width = IMAGE_WIDTH;
  data_camera_msg_.encoding = "rgb8";
  data_camera_msg_.is_bigendian = false;
  data_camera_msg_.step = step;

  // Generate random pixel data
  const size_t data_size = IMAGE_HEIGHT * step;
  data_camera_msg_.data.resize(data_size);

  // Create random number generator
  static std::random_device rd;
  static std::mt19937 gen(rd());
  static std::uniform_int_distribution<uint8_t> dis(0, 255);

  // Fill with random data
  for (size_t i = 0; i < data_size; ++i) {
    data_camera_msg_.data[i] = dis(gen);
  }

  // Publish the message
  data_camera_publisher_->publish(data_camera_msg_);
  RCLCPP_INFO(this->get_logger(), "%sPublished random camera image%s",
              Color::PURPLE, Color::RESET);
}

} // namespace launch_demo_nodes

int main(int argc, char **argv) {
  try {
    // Initialize ROS2 communication
    rclcpp::init(argc, argv);

    // Create and initialize the node
    auto node = std::make_shared<launch_demo_nodes::CameraDemoNode>();

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