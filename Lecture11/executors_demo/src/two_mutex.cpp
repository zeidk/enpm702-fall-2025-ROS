#include "executors_demo/two_mutex.hpp"
#include "executors_demo/color_utils.hpp"
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <chrono>
#include <thread>

namespace executors_demo {

TwoMutexDemoNode::TwoMutexDemoNode(const std::string& node_name)
    : Node(node_name)
{
    // Create two mutually exclusive callback groups
    mutex_group1_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    mutex_group2_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    // Timer1 and Timer2 in first mutually exclusive group
    timer1_ = this->create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&TwoMutexDemoNode::timer1_callback, this),
        mutex_group1_);
    
    timer2_ = this->create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&TwoMutexDemoNode::timer2_callback, this),
        mutex_group1_);
    
    // Timer3 and Timer4 in second mutually exclusive group
    timer3_ = this->create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&TwoMutexDemoNode::timer3_callback, this),
        mutex_group2_);
    
    timer4_ = this->create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&TwoMutexDemoNode::timer4_callback, this),
        mutex_group2_);
    
    RCLCPP_INFO(this->get_logger(), "%s initialized", node_name.c_str());
    RCLCPP_INFO(this->get_logger(), "Timer1,2: Mutex Group 1 | Timer3,4: Mutex Group 2");
}

void TwoMutexDemoNode::timer1_callback()
{
    RCLCPP_INFO(this->get_logger(), "%s[MUTEX1] Timer1 start - Thread ID: %zu%s", 
                Color::YELLOW, std::hash<std::thread::id>{}(std::this_thread::get_id()), Color::RESET);
    
    // Simulate some work
    // std::this_thread::sleep_for(std::chrono::milliseconds(5000));
    
    RCLCPP_INFO(this->get_logger(), "%s[MUTEX1] Timer1 end%s", Color::YELLOW, Color::RESET);
}

void TwoMutexDemoNode::timer2_callback()
{
    RCLCPP_INFO(this->get_logger(), "%s[MUTEX1] Timer2 start - Thread ID: %zu%s", 
                Color::BLUE, std::hash<std::thread::id>{}(std::this_thread::get_id()), Color::RESET);
    
    // Simulate some work
    // std::this_thread::sleep_for(std::chrono::milliseconds(5000));
    
    RCLCPP_INFO(this->get_logger(), "%s[MUTEX1] Timer2 end%s", Color::BLUE, Color::RESET);
}

void TwoMutexDemoNode::timer3_callback()
{
    RCLCPP_INFO(this->get_logger(), "%s[MUTEX2] Timer3 start - Thread ID: %zu%s", 
                Color::GREEN, std::hash<std::thread::id>{}(std::this_thread::get_id()), Color::RESET);
    
    // Simulate some work
    // std::this_thread::sleep_for(std::chrono::milliseconds(5000));
    
    RCLCPP_INFO(this->get_logger(), "%s[MUTEX2] Timer3 end%s", Color::GREEN, Color::RESET);
}

void TwoMutexDemoNode::timer4_callback()
{
    RCLCPP_INFO(this->get_logger(), "%s[MUTEX2] Timer4 start - Thread ID: %zu%s", 
                Color::RED, std::hash<std::thread::id>{}(std::this_thread::get_id()), Color::RESET);
    
    // Simulate some work
    // std::this_thread::sleep_for(std::chrono::milliseconds(5000));
    
    RCLCPP_INFO(this->get_logger(), "%s[MUTEX2] Timer4 end%s", Color::RED, Color::RESET);
}

} // namespace executors_demo

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<executors_demo::TwoMutexDemoNode>("two_mutex_demo");
    
    // Use MultiThreadedExecutor to see the effect of callback groups
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
    executor.add_node(node);
    
    try {
        executor.spin();
    }
    catch (const std::exception& e) {
        RCLCPP_ERROR(node->get_logger(), "Exception caught: %s", e.what());
    }
    
    RCLCPP_WARN(node->get_logger(), "Shutting down two mutex demo");
    
    rclcpp::shutdown();
}