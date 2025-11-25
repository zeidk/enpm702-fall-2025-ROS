#pragma once

#include <rclcpp/rclcpp.hpp>
#include <memory>

namespace executors_demo {

class SingleThreadedDemoNode : public rclcpp::Node
{
public:
    explicit SingleThreadedDemoNode(const std::string& node_name);

private:
    void timer1_callback();
    void timer2_callback();
    void timer3_callback();
    void timer4_callback();

    // Member variables
    int test_;
    rclcpp::TimerBase::SharedPtr timer1_;
    rclcpp::TimerBase::SharedPtr timer2_;
    rclcpp::TimerBase::SharedPtr timer3_;
    rclcpp::TimerBase::SharedPtr timer4_;
};
} // namespace executors_demo