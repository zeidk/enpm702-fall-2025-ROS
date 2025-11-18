//-- version NOT using OOP
#include <rclcpp/rclcpp.hpp>

int main(int argc, char **argv) {
    // Initialize ROS 2
    rclcpp::init(argc, argv);
    
    // Create a node
    auto node = std::make_shared<rclcpp::Node>("hello");
    
    // Log a message
    RCLCPP_INFO_STREAM(node->get_logger(), "Hello from ROS 2!");
    
    // Shutdown ROS 2
    rclcpp::shutdown();
}

// // //-- version using OOP
// #include "first_package/hello_node.hpp"
// #include <rclcpp/rclcpp.hpp>

// int main(int argc, char **argv){
//     rclcpp::init(argc, argv);
//     auto node = std::make_shared<first_package::HelloNode>("hello_oop");
//     rclcpp::shutdown();
// }