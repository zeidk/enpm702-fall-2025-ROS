#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <memory>

class SubscriberDemo : public rclcpp::Node{
public:
    explicit SubscriberDemo(const std::string& node_name);

private:
    void topic_callback(const std_msgs::msg::String::SharedPtr msg);
    
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};