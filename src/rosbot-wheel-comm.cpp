#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/float32_multi_array.hpp" // Required for the array command

using namespace std::chrono_literals;
using std::placeholders::_1;

class WheelController : public rclcpp::Node
{
public:
  WheelController()
  : Node("wheel_controller_node"), left_ticks_(0), right_ticks_(0)
  {
    // --- SUBSCRIBERS (Listen to the Pico) ---
    // Subscribe to Left Encoder
    left_sub_ = this->create_subscription<std_msgs::msg::Int32>(
      "left_wheel/raw_ticks", 10,
      std::bind(&WheelController::left_callback, this, _1));

    // Subscribe to Right Encoder
    right_sub_ = this->create_subscription<std_msgs::msg::Int32>(
      "right_wheel/raw_ticks", 10,
      std::bind(&WheelController::right_callback, this, _1));

    // --- PUBLISHER (Talk to the Pico) ---
    // Publisher for motor commands [left_speed, right_speed]
    cmd_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("wheel_cmd", 10);

    // --- TIMER (Control Loop) ---
    // Every 500ms, send a command and print status
    timer_ = this->create_wall_timer(
      500ms, std::bind(&WheelController::timer_callback, this));
      
    RCLCPP_INFO(this->get_logger(), "Wheel Controller Started. Sending commands...");
  }

private:
  // Update internal state when left ticks arrive
  void left_callback(const std_msgs::msg::Int32::SharedPtr msg)
  {
    left_ticks_ = msg->data;
  }

  // Update internal state when right ticks arrive
  void right_callback(const std_msgs::msg::Int32::SharedPtr msg)
  {
    right_ticks_ = msg->data;
  }

  // Main loop: Send motor commands and log data
  void timer_callback()
  {
    // 1. Create the message
    auto message = std_msgs::msg::Float32MultiArray();
    
    // 2. Set the data: [Left Speed, Right Speed]
    // Sending 0.5 (50% speed) to both motors
    message.data = {0.5, 0.5}; 

    // 3. Publish the command
    cmd_pub_->publish(message);

    // 4. Log the feedback
    RCLCPP_INFO(this->get_logger(), 
      "Sent: [0.5, 0.5] | Encoders - Left: %d, Right: %d", 
      left_ticks_, right_ticks_);
  }

  // Member variables
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr left_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr right_sub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr cmd_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  int32_t left_ticks_;
  int32_t right_ticks_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WheelController>());
  rclcpp::shutdown();
  return 0;
}