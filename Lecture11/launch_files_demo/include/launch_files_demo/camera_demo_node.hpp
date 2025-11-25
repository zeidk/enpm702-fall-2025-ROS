#pragma once
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace launch_demo_nodes {

class CameraDemoNode : public rclcpp::Node {
public:
  CameraDemoNode();

private:
  void data_camera_pub_callback();

  // Publishers
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr data_camera_publisher_;

  // Timers
  rclcpp::TimerBase::SharedPtr data_camera_timer_;

  // Messages
  sensor_msgs::msg::Image data_camera_msg_;

  // Constants
  static constexpr int IMAGE_WIDTH = 640;
  static constexpr int IMAGE_HEIGHT = 480;
  static constexpr int CHANNELS = 3;
  static constexpr double PUBLISH_FREQUENCY = 1.0; // Hz
};

} // namespace launch_demo_nodes
