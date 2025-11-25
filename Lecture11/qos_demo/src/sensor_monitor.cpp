#include "qos_demo/sensor_monitor.hpp"
#include <cstdlib>
#include <ctime>
#include <rclcpp/rclcpp.hpp>

SensorMonitor::SensorMonitor() : Node("sensor_monitor")
{
  // Seed random number generator
  std::srand(std::time(nullptr));

  // Publisher with sensor data QoS profile
  sensor_publisher_ = create_publisher<sensor_msgs::msg::Temperature>(
    "temperature", 
    rclcpp::SensorDataQoS()  // Best-effort, volatile, keep last 5
  );

  // Reliable subscriber - will have issues with best-effort publisher
  auto reliable_qos = rclcpp::QoS(10)
    .reliability(rclcpp::ReliabilityPolicy::Reliable)
    .durability(rclcpp::DurabilityPolicy::Volatile);

  reliable_subscriber_ = create_subscription<sensor_msgs::msg::Temperature>(
    "temperature", reliable_qos,
    std::bind(&SensorMonitor::reliable_callback, this, std::placeholders::_1)
  );

  // Best-effort subscriber - compatible with sensor data QoS
  best_effort_subscriber_ = create_subscription<sensor_msgs::msg::Temperature>(
    "temperature", rclcpp::SensorDataQoS(),
    std::bind(&SensorMonitor::best_effort_callback, this, std::placeholders::_1)
  );

  // Custom QoS subscriber
  auto custom_qos = rclcpp::QoS(rclcpp::KeepLast(50))
    .reliability(rclcpp::ReliabilityPolicy::BestEffort)
    .durability(rclcpp::DurabilityPolicy::Volatile)
    .deadline(std::chrono::milliseconds(200))
    .lifespan(std::chrono::seconds(10));

  custom_subscriber_ = create_subscription<sensor_msgs::msg::Temperature>(
    "temperature", custom_qos,
    [this](const sensor_msgs::msg::Temperature::SharedPtr msg) {
      static int custom_count = 0;
      custom_count++;
      
      if (custom_count % 30 == 0) {
        RCLCPP_INFO(get_logger(), "Custom QoS received %d messages: %.2f°C", 
                    custom_count, msg->temperature);
      }
    }
    
  );

  // Timer for publishing at 10Hz
  timer_ = create_wall_timer(
    std::chrono::milliseconds(100),
    std::bind(&SensorMonitor::timer_callback, this)
  );

  RCLCPP_INFO(get_logger(), "Sensor monitor started with SensorDataQoS");
  RCLCPP_INFO(get_logger(), "Publisher: Best-effort, Subscribers: Reliable + Best-effort + Custom");
}

void SensorMonitor::timer_callback()
{
  auto msg = sensor_msgs::msg::Temperature();
  msg.header.stamp = now();
  msg.header.frame_id = "sensor_frame";
  
  // Simulate temperature readings with noise
  static double base_temp = 25.0;
  base_temp += (std::rand() % 100 - 50) * 0.01;  // ±0.5°C noise
  msg.temperature = base_temp;
  msg.variance = 0.1;
  
  sensor_publisher_->publish(msg);
  
  // Log every 50th message to avoid spam
  static int count = 0;
  if (++count % 50 == 0) {
    RCLCPP_INFO(get_logger(), "Published temperature: %.2f°C (message #%d)", 
                msg.temperature, count);
  }
}

void SensorMonitor::reliable_callback(
  const sensor_msgs::msg::Temperature::SharedPtr msg)
{
  static int reliable_count = 0;
  reliable_count++;
  
  // This subscriber may not receive many (or any) messages
  // because it requires reliable delivery but publisher is best-effort
  if (reliable_count % 10 == 0) {
    RCLCPP_WARN(get_logger(), 
                "Reliable subscriber: received %d messages (%.2f°C) - may miss many!", 
                reliable_count, msg->temperature);
  }
}

void SensorMonitor::best_effort_callback(
  const sensor_msgs::msg::Temperature::SharedPtr msg)
{
  static int best_effort_count = 0;
  best_effort_count++;
  
  // This subscriber should receive most messages
  if (best_effort_count % 40 == 0) {
    RCLCPP_INFO(get_logger(), 
                "Best-effort subscriber: received %d messages (%.2f°C)", 
                best_effort_count, msg->temperature);
  }
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  
  auto node = std::make_shared<SensorMonitor>();
  rclcpp::spin(node);
  
  rclcpp::shutdown();
}