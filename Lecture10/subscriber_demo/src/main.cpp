#include "subscriber_demo/subscriber_demo.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SubscriberDemo>("subscriber_demo");
    rclcpp::spin(node);
    rclcpp::shutdown();
}