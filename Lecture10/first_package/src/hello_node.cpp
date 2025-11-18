

#include "first_package/hello_node.hpp"

namespace first_package
{

HelloNode::HelloNode(const std::string& node_name) : Node(node_name){
    RCLCPP_INFO(this->get_logger(), "HelloNode constructor called");
    print_hello();
}

void HelloNode::print_hello(){
    RCLCPP_INFO_STREAM(this->get_logger(), 
                      "Hello from " << this->get_name() << "!");
}

}  // namespace first_package