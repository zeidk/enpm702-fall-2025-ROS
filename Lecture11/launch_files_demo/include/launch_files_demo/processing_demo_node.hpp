#pragma once
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/temperature.hpp>

namespace launch_demo_nodes {

class ProcessingDemoNode : public rclcpp::Node {
public:
  ProcessingDemoNode();

private:
  void data_lidar_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void data_camera_callback(const sensor_msgs::msg::Image::SharedPtr msg);
  void
  data_temperature_callback(const sensor_msgs::msg::Temperature::SharedPtr msg);

  // Subscribers
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr
      data_lidar_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr
      data_camera_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::Temperature>::SharedPtr
      data_temperature_subscription_;
};

} // namespace launch_demo_nodes
