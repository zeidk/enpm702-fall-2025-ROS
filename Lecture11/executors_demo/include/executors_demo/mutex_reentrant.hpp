#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>

namespace executors_demo {

class ReentrantDemoNode : public rclcpp::Node {
public:
  explicit ReentrantDemoNode(const std::string &node_name);

private:
  void timer1_callback();
  void timer2_callback();
  void timer3_callback();
  void timer4_callback();
  void timer5_callback();

  // Callback groups
  rclcpp::CallbackGroup::SharedPtr reentrant_group_;
  rclcpp::CallbackGroup::SharedPtr mutually_exclusive_group_;

  // Timers
  rclcpp::TimerBase::SharedPtr timer1_;
  rclcpp::TimerBase::SharedPtr timer2_;
  rclcpp::TimerBase::SharedPtr timer3_;
  rclcpp::TimerBase::SharedPtr timer4_;
  rclcpp::TimerBase::SharedPtr timer5_;
};
} // namespace executors_demo