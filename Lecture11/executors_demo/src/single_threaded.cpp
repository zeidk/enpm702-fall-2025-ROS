#include "executors_demo/single_threaded.hpp"
#include "executors_demo/color_utils.hpp"
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <chrono>
#include <csignal>


namespace executors_demo {

SingleThreadedDemoNode::SingleThreadedDemoNode(const std::string& node_name)
    : Node(node_name), test_(0)
{
    // Timer1 - 1 second period
    timer1_ = this->create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&SingleThreadedDemoNode::timer1_callback, this));
    
    // Timer2 - 1 second period
    timer2_ = this->create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&SingleThreadedDemoNode::timer2_callback, this));
    
    // Timer3 - 1 second period
    timer3_ = this->create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&SingleThreadedDemoNode::timer3_callback, this));
    
    // Timer4 - 1 second period
    timer4_ = this->create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&SingleThreadedDemoNode::timer4_callback, this));
    
    RCLCPP_INFO(this->get_logger(), "%s initialized", node_name.c_str());
}

void SingleThreadedDemoNode::timer1_callback()
{
    RCLCPP_INFO(this->get_logger(), "%sTimer1 callback%s", Color::YELLOW, Color::RESET);
}

void SingleThreadedDemoNode::timer2_callback()
{
    RCLCPP_INFO(this->get_logger(), "%sTimer2 callback%s", Color::BLUE, Color::RESET);
    // Uncomment the lines below to test blocking behavior
    // std::this_thread::sleep_for(std::chrono::seconds(5));
}

void SingleThreadedDemoNode::timer3_callback()
{
    RCLCPP_INFO(this->get_logger(), "%sTimer3 callback%s", Color::GREEN, Color::RESET);
}

void SingleThreadedDemoNode::timer4_callback()
{
    RCLCPP_INFO(this->get_logger(), "%sTimer4 callback%s", Color::RED, Color::RESET);
}

} // namespace executors_demo

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<executors_demo::SingleThreadedDemoNode>("single_threaded_demo");
    
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    
    try {
        executor.spin();
    }
    catch (const std::exception& e) {
        RCLCPP_ERROR(node->get_logger(), "Exception caught: %s", e.what());
    }
    
    RCLCPP_WARN(node->get_logger(), "Shutting down single-threaded executor demo");
    
    rclcpp::shutdown();
}