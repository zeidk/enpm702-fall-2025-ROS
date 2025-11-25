#include "executors_demo/mutex_reentrant.hpp"
#include "executors_demo/color_utils.hpp"
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <chrono>
#include <thread>

namespace executors_demo {

ReentrantDemoNode::ReentrantDemoNode(const std::string& node_name)
    : Node(node_name)
{
    // Create callback groups
    reentrant_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    mutually_exclusive_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    // Timer1 and Timer2 in reentrant group (can run in parallel)
    timer1_ = this->create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&ReentrantDemoNode::timer1_callback, this),
        reentrant_group_);
    
    timer2_ = this->create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&ReentrantDemoNode::timer2_callback, this),
        reentrant_group_);
    
    // Timer3 and Timer4 in mutually exclusive group (cannot run in parallel)
    timer3_ = this->create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&ReentrantDemoNode::timer3_callback, this),
        reentrant_group_);
    
    timer4_ = this->create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&ReentrantDemoNode::timer4_callback, this),
        mutually_exclusive_group_);

    timer5_ = this->create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&ReentrantDemoNode::timer5_callback, this),
        mutually_exclusive_group_);
    
    RCLCPP_INFO(this->get_logger(), "%s initialized", node_name.c_str());
    RCLCPP_INFO(this->get_logger(), "Timer1,2: Reentrant group | Timer3,4: Mutually exclusive group");
}

void ReentrantDemoNode::timer1_callback()
{
    RCLCPP_INFO(this->get_logger(), "%s[REENTRANT] Timer1 start - Thread ID: %zu%s", 
                Color::YELLOW, std::hash<std::thread::id>{}(std::this_thread::get_id()), Color::RESET);
    
    // Simulate some work
    // std::this_thread::sleep_for(std::chrono::milliseconds(5000));
    
    RCLCPP_INFO(this->get_logger(), "%s[REENTRANT] Timer1 end%s", Color::YELLOW, Color::RESET);
}

void ReentrantDemoNode::timer2_callback()
{
    RCLCPP_INFO(this->get_logger(), "%s[REENTRANT] Timer2 start - Thread ID: %zu%s", 
                Color::BLUE, std::hash<std::thread::id>{}(std::this_thread::get_id()), Color::RESET);
    
    // Simulate some work
    // std::this_thread::sleep_for(std::chrono::milliseconds(5000));
    
    RCLCPP_INFO(this->get_logger(), "%s[REENTRANT] Timer2 end%s", Color::BLUE, Color::RESET);
}

void ReentrantDemoNode::timer3_callback()
{
    RCLCPP_INFO(this->get_logger(), "%s[MUTEX] Timer3 start - Thread ID: %zu%s", 
                Color::GREEN, std::hash<std::thread::id>{}(std::this_thread::get_id()), Color::RESET);
    
    // Simulate some work
    // std::this_thread::sleep_for(std::chrono::milliseconds(5000));
    
    RCLCPP_INFO(this->get_logger(), "%s[MUTEX] Timer3 end%s", Color::GREEN, Color::RESET);
}

void ReentrantDemoNode::timer4_callback()
{
    RCLCPP_INFO(this->get_logger(), "%s[MUTEX] Timer4 start - Thread ID: %zu%s", 
                Color::RED, std::hash<std::thread::id>{}(std::this_thread::get_id()), Color::RESET);
    
    // Simulate some work
    // std::this_thread::sleep_for(std::chrono::milliseconds(5000));
    
    RCLCPP_INFO(this->get_logger(), "%s[MUTEX] Timer4 end%s", Color::RED, Color::RESET);
}

void ReentrantDemoNode::timer5_callback()
{
    RCLCPP_INFO(this->get_logger(), "%s[MUTEX] Timer5 start - Thread ID: %zu%s", 
                Color::RED, std::hash<std::thread::id>{}(std::this_thread::get_id()), Color::RESET);
    
    // Simulate some work
    // std::this_thread::sleep_for(std::chrono::milliseconds(5000));
    
    RCLCPP_INFO(this->get_logger(), "%s[MUTEX] Timer5 end%s", Color::RED, Color::RESET);
}

} // namespace executors_demo

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<executors_demo::ReentrantDemoNode>("reentrant_demo");
    
    // Use MultiThreadedExecutor to see the effect of callback groups
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
    executor.add_node(node);
    
    try {
        executor.spin();
    }
    catch (const std::exception& e) {
        RCLCPP_ERROR(node->get_logger(), "Exception caught: %s", e.what());
    }
    
    RCLCPP_WARN(node->get_logger(), "Shutting down reentrant demo");
    
    rclcpp::shutdown();
}