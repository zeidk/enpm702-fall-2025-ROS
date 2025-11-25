#pragma once
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/temperature.hpp>
#include <memory>

namespace launch_demo_nodes
{

class TemperatureDemoNode : public rclcpp::Node
{
public:
    TemperatureDemoNode();

private:
    void data_temperature_pub_callback();

    // Publishers
    rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr data_temperature_publisher_;
    
    // Timers
    rclcpp::TimerBase::SharedPtr data_temperature_timer_;
    
    // Messages
    sensor_msgs::msg::Temperature data_temperature_msg_;
    
    // Constants
    static constexpr double PUBLISH_FREQUENCY = 1.0;  // Hz
    static constexpr double TEMP_MIN = -10.0;  // °C
    static constexpr double TEMP_MAX = 40.0;   // °C
    static constexpr double TEMP_VARIANCE = 0.5;
};

}  // namespace launch_demo_nodes
