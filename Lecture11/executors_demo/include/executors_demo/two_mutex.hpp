#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>

namespace executors_demo {

class TwoMutexDemoNode : public rclcpp::Node {
public:
  explicit TwoMutexDemoNode(const std::string &node_name);

private:
  void timer1_callback();
  void timer2_callback();
  void timer3_callback();
  void timer4_callback();

  // Callback groups
  rclcpp::CallbackGroup::SharedPtr mutex_group1_;
  rclcpp::CallbackGroup::SharedPtr mutex_group2_;

  // Timers
  rclcpp::TimerBase::SharedPtr timer1_;
  rclcpp::TimerBase::SharedPtr timer2_;
  rclcpp::TimerBase::SharedPtr timer3_;
  rclcpp::TimerBase::SharedPtr timer4_;
};

} // namespace executors_demo