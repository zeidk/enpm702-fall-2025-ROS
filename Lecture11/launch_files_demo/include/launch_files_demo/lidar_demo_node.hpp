#pragma once
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

namespace launch_demo_nodes {

class LidarDemoNode : public rclcpp::Node {
public:
  LidarDemoNode();

private:
  void data_lidar_pub_callback();

  // Publishers
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr
      data_lidar_publisher_;

  // Timers
  rclcpp::TimerBase::SharedPtr data_lidar_timer_;

  // Messages
  sensor_msgs::msg::LaserScan data_lidar_msg_;

  // Constants
  static constexpr double PUBLISH_FREQUENCY = 10.0; // Hz
  static constexpr double ANGLE_MIN = -M_PI;
  static constexpr double ANGLE_MAX = M_PI;
  static constexpr double ANGLE_INCREMENT = M_PI / 180.0; // 1 degree
  static constexpr double RANGE_MIN = 0.2;
  static constexpr double RANGE_MAX = 10.0;
  static constexpr double SCAN_TIME = 0.1;
};

} // namespace launch_demo_nodes
