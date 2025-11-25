#include "robot_interfaces_demo/robot_status.hpp"
#include <rclcpp/rclcpp.hpp>
#include <random>

using namespace std::chrono_literals;

namespace robot_interfaces_demo {

RobotStatusDemo::RobotStatusDemo(const std::string& robot_name)
    : Node("robot_status_demo"), 
      robot_name_(robot_name),
      current_mode_(robot_custom_interfaces::msg::RobotStatus::MODE_IDLE),
      battery_voltage_(12.5f),
      temperature_(25.0f),
      humidity_(45.0f),
      status_counter_(0) {
    
    this->set_parameter(rclcpp::Parameter("use_sim_time", true));
    
    mode_names_ = {
        {robot_custom_interfaces::msg::RobotStatus::MODE_IDLE, "IDLE"},
        {robot_custom_interfaces::msg::RobotStatus::MODE_MOVING, "MOVING"},
        {robot_custom_interfaces::msg::RobotStatus::MODE_WORKING, "WORKING"},
        {robot_custom_interfaces::msg::RobotStatus::MODE_CHARGING, "CHARGING"},
        {robot_custom_interfaces::msg::RobotStatus::MODE_ERROR, "ERROR"}
    };
    
    status_publisher_ = this->create_publisher<robot_custom_interfaces::msg::RobotStatus>(
        "/robot_status", 10);
    
    status_subscriber_ = this->create_subscription<robot_custom_interfaces::msg::RobotStatus>(
        "/robot_status", 10, 
        std::bind(&RobotStatusDemo::status_callback, this, std::placeholders::_1));
    
    timer_ = this->create_wall_timer(
        2000ms, std::bind(&RobotStatusDemo::publish_status, this));
    
    RCLCPP_INFO(this->get_logger(), 
        "RobotStatusDemo initialized for robot: %s (Publishing & Subscribing)", 
        robot_name_.c_str());
}

void RobotStatusDemo::publish_status() {
    auto msg = robot_custom_interfaces::msg::RobotStatus();
    
    msg.header.stamp = this->get_clock()->now();
    msg.header.frame_id = "base_link";
    
    msg.robot_name = robot_name_;
    
    status_counter_++;
    if (status_counter_ % 15 == 0) {
        current_mode_ = (current_mode_ + 1) % 5;
    }
    msg.mode = current_mode_;
    
    msg.sensor_readings = get_sensor_readings();
    
    status_publisher_->publish(msg);
    
    RCLCPP_INFO(this->get_logger(), 
        "[PUB] %s: Mode=%s, Battery=%.1fV, Temp=%.1f°C, Humidity=%.1f%%", 
        robot_name_.c_str(), 
        mode_to_string(msg.mode).c_str(),
        msg.sensor_readings.size() > 0 ? msg.sensor_readings[0] : 0.0f,
        msg.sensor_readings.size() > 1 ? msg.sensor_readings[1] : 0.0f,
        msg.sensor_readings.size() > 2 ? msg.sensor_readings[2] : 0.0f);
}

std::vector<float> RobotStatusDemo::get_sensor_readings() {
    static std::random_device rd;
    static std::mt19937 gen(rd());
    static std::uniform_real_distribution<float> battery_noise(-0.2f, 0.2f);
    static std::uniform_real_distribution<float> temp_noise(-1.0f, 1.0f);
    static std::uniform_real_distribution<float> humidity_noise(-3.0f, 3.0f);
    
    if (current_mode_ == robot_custom_interfaces::msg::RobotStatus::MODE_CHARGING) {
        battery_voltage_ += 0.03f;
        if (battery_voltage_ > 12.6f) {
            battery_voltage_ = 12.6f;
        }
    } else if (current_mode_ != robot_custom_interfaces::msg::RobotStatus::MODE_IDLE) {
        battery_voltage_ -= 0.005f;
        if (battery_voltage_ < 10.0f) {
            battery_voltage_ = 10.0f;
        }
    }
    
    if (current_mode_ == robot_custom_interfaces::msg::RobotStatus::MODE_WORKING ||
        current_mode_ == robot_custom_interfaces::msg::RobotStatus::MODE_MOVING) {
        temperature_ += 0.1f;
        if (temperature_ > 45.0f) {
            temperature_ = 45.0f;
        }
    } else {
        temperature_ -= 0.05f;
        if (temperature_ < 20.0f) {
            temperature_ = 20.0f;
        }
    }
    
    humidity_ += (gen() % 3 - 1) * 0.5f;
    if (humidity_ > 80.0f) humidity_ = 80.0f;
    if (humidity_ < 20.0f) humidity_ = 20.0f;
    
    return {
        battery_voltage_ + battery_noise(gen),
        temperature_ + temp_noise(gen),
        humidity_ + humidity_noise(gen)
    };
}

void RobotStatusDemo::status_callback(const robot_custom_interfaces::msg::RobotStatus::SharedPtr msg) {
    
    last_seen_robots_[msg->robot_name] = this->get_clock()->now();
    
    process_other_robot_status(msg);
}

void RobotStatusDemo::process_other_robot_status(const robot_custom_interfaces::msg::RobotStatus::SharedPtr msg) {
    std::string mode_str = mode_to_string(msg->mode);
    
    float battery_voltage{0.0f};
    float temperature{0.0f};
    float humidity{0.0f};
    
    if (msg->sensor_readings.size() >= 1) {
        battery_voltage = msg->sensor_readings[0];
    }
    if (msg->sensor_readings.size() >= 2) {
        temperature = msg->sensor_readings[1];
    }
    if (msg->sensor_readings.size() >= 3) {
        humidity = msg->sensor_readings[2];
    }
    
    RCLCPP_INFO(this->get_logger(), 
        "[SUB] Received from %s: Mode=%s, Battery=%.1fV, Temp=%.1f°C, Humidity=%.1f%%", 
        msg->robot_name.c_str(), 
        mode_str.c_str(), 
        battery_voltage, 
        temperature,
        humidity);
    
    if (battery_voltage < 11.0f && battery_voltage > 0.0f) {
        RCLCPP_WARN(this->get_logger(), 
            "[ALERT] %s has low battery: %.1fV", 
            msg->robot_name.c_str(), battery_voltage);
    }
    
    if (temperature > 40.0f) {
        RCLCPP_WARN(this->get_logger(), 
            "[ALERT] %s is overheating: %.1f°C", 
            msg->robot_name.c_str(), temperature);
    }
    
    if (msg->mode == robot_custom_interfaces::msg::RobotStatus::MODE_ERROR) {
        RCLCPP_ERROR(this->get_logger(), 
            "[ALERT] %s is in ERROR mode - requires attention!", 
            msg->robot_name.c_str());
    }
    
    if (status_counter_ % 10 == 0) {
        RCLCPP_INFO(this->get_logger(), 
            "[FLEET] Currently monitoring %zu other robots", 
            last_seen_robots_.size());
    }
}

std::string RobotStatusDemo::mode_to_string(uint8_t mode) const {
    auto it = mode_names_.find(mode);
    return (it != mode_names_.end()) ? it->second : "UNKNOWN";
}

}  // namespace robot_interfaces_demo



int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    
    std::string robot_name = "robot_01";
    
    if (argc > 1) {
        robot_name = argv[1];
    }
    
    try {
        auto node = std::make_shared<robot_interfaces_demo::RobotStatusDemo>(robot_name);
        
        RCLCPP_INFO(rclcpp::get_logger("main"), 
            "Starting RobotStatusDemo for: %s (Publisher + Subscriber)", 
            robot_name.c_str());
        
        rclcpp::spin(node);
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("main"), 
            "Exception in RobotStatusDemo: %s", e.what());
        return 1;
    }
    
    rclcpp::shutdown();
}