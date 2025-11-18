#include "subscriber_demo/subscriber_demo.hpp"

SubscriberDemo::SubscriberDemo(const std::string& node_name)
    : Node(node_name){
    subscription_ = this->create_subscription<std_msgs::msg::String>(
        "leia",  // Topic name
        10,      // QoS queue depth
        std::bind(&SubscriberDemo::topic_callback, this, std::placeholders::_1));
        
    RCLCPP_INFO(this->get_logger(), "Subscriber initialized, listening to 'leia'");
}

void SubscriberDemo::topic_callback(const std_msgs::msg::String::SharedPtr msg){
    RCLCPP_INFO(this->get_logger(), "Received: '%s'", msg->data.c_str());
    
    // Process the message as needed:
    // - Store data for later use
    // - Trigger other actions
    // - Update internal state
    // - Publish transformed data
}