#pragma once

#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <string>
#include <vector>
#include <map>
#include "robot_custom_interfaces/msg/robot_status.hpp"

namespace robot_interfaces_demo {

class RobotStatusDemo : public rclcpp::Node {
public:
    explicit RobotStatusDemo(const std::string& robot_name = "robot_01");

private:
    void publish_status();
    std::vector<float> get_sensor_readings();
    void status_callback(const robot_custom_interfaces::msg::RobotStatus::SharedPtr msg);
    std::string mode_to_string(uint8_t mode) const;
    void process_other_robot_status(const robot_custom_interfaces::msg::RobotStatus::SharedPtr msg);

    rclcpp::Publisher<robot_custom_interfaces::msg::RobotStatus>::SharedPtr status_publisher_;
    rclcpp::Subscription<robot_custom_interfaces::msg::RobotStatus>::SharedPtr status_subscriber_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    std::string robot_name_;
    uint8_t current_mode_;
    float battery_voltage_;
    float temperature_;
    float humidity_;
    int status_counter_;
    
    std::map<uint8_t, std::string> mode_names_;
    std::map<std::string, rclcpp::Time> last_seen_robots_;
};

}  // namespace robot_interfaces_demo