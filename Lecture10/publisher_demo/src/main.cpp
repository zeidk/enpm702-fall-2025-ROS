#include "publisher_demo/publisher_demo.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PublisherDemo>("publisher_demo");
    rclcpp::spin(node);
    rclcpp::shutdown();
}