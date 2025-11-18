#pragma once

#include <rclcpp/rclcpp.hpp>

namespace first_package
{

class HelloNode : public rclcpp::Node{
public:
    explicit HelloNode(const std::string& node_name);
    
private:
    void print_hello();
};

}  // namespace first_package