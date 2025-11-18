#include "publisher_demo/publisher_demo.hpp"

PublisherDemo::PublisherDemo(const std::string& node_name)
    : Node(node_name), count_{0}{
    // Create publisher with message type, topic name, and QoS depth
    publisher_ = this->create_publisher<std_msgs::msg::String>("leia", 10);
    
    // Create timer for periodic publishing (2Hz = 500ms)
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(500),
        std::bind(&PublisherDemo::timer_callback, this));
        
    RCLCPP_INFO(this->get_logger(), "Publisher initialized");
}

void PublisherDemo::timer_callback(){
    // Create message
    auto message = std_msgs::msg::String();
    
    // Populate message data
    message.data = "Help me Obi-Wan Kenobi, you're my only hope #" + 
                   std::to_string(count_++);
    
    // Log what we're publishing (optional)
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    
    // Publish the message
    publisher_->publish(message);
}