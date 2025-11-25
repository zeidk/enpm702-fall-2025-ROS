// include/qos_demo/sensor_monitor.hpp
#pragma once

#include "rclcpp/qos.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/temperature.hpp"

class SensorMonitor : public rclcpp::Node {
public:
  SensorMonitor();

private:
  void timer_callback();
  void reliable_callback(const sensor_msgs::msg::Temperature::SharedPtr msg);
  void best_effort_callback(const sensor_msgs::msg::Temperature::SharedPtr msg);

  // Publisher
  rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr sensor_publisher_;

  // Subscribers with different QoS
  rclcpp::Subscription<sensor_msgs::msg::Temperature>::SharedPtr
      reliable_subscriber_;
  rclcpp::Subscription<sensor_msgs::msg::Temperature>::SharedPtr
      best_effort_subscriber_;
  rclcpp::Subscription<sensor_msgs::msg::Temperature>::SharedPtr
      custom_subscriber_;

  // Timer for publishing
  rclcpp::TimerBase::SharedPtr timer_;
};