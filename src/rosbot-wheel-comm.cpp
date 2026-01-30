#include <chrono>
#include <cmath>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

using namespace std::chrono_literals;

class DiffDriveController : public rclcpp::Node
{
public:
  DiffDriveController()
  : Node("diff_drive_controller")
  {
    // ---------------- PARAMETERS ----------------
    this->declare_parameter<int>("pulses_per_revolution", 200);
    this->declare_parameter<double>("wheel_radius", 0.04);
    this->declare_parameter<double>("wheel_separation", 0.17);
    this->declare_parameter<double>("kp", 1.5);
    this->declare_parameter<double>("control_rate", 50.0);
    this->declare_parameter<double>("max_wheel_speed", 10.0);

    ppr_ = this->get_parameter("pulses_per_revolution").as_int();
    wheel_radius_ = this->get_parameter("wheel_radius").as_double();
    wheel_sep_ = this->get_parameter("wheel_separation").as_double();
    kp_ = this->get_parameter("kp").as_double();
    max_wheel_speed_ = this->get_parameter("max_wheel_speed").as_double();

    double rate = this->get_parameter("control_rate").as_double();
    control_period_ = std::chrono::duration<double>(1.0 / rate);

    // ---------------- SUBSCRIBERS ----------------
    left_ticks_sub_ = this->create_subscription<std_msgs::msg::Int32>(
      "/left_wheel/raw_ticks", 10,
      std::bind(&DiffDriveController::leftTicksCb, this, std::placeholders::_1));

    right_ticks_sub_ = this->create_subscription<std_msgs::msg::Int32>(
      "/right_wheel/raw_ticks", 10,
      std::bind(&DiffDriveController::rightTicksCb, this, std::placeholders::_1));

    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10,
      std::bind(&DiffDriveController::cmdVelCb, this, std::placeholders::_1));

    // ---------------- PUBLISHERS ----------------
    joint_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
      "/joint_states", 10);

    wheel_cmd_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
      "/wheel_cmd", 10);

    // ---------------- TIMER ----------------
    last_time_ = this->now();
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(control_period_),
      std::bind(&DiffDriveController::controlLoop, this));

    RCLCPP_INFO(this->get_logger(), "Differential Drive Controller (C++) started");
  }

private:
  // ---------------- CALLBACKS ----------------
  void leftTicksCb(const std_msgs::msg::Int32::SharedPtr msg)
  {
    left_ticks_ = msg->data;
  }

  void rightTicksCb(const std_msgs::msg::Int32::SharedPtr msg)
  {
    right_ticks_ = msg->data;
  }

  void cmdVelCb(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    cmd_v_ = msg->linear.x;
    cmd_w_ = msg->angular.z;
  }

  // ---------------- CONTROL LOOP ----------------
  void controlLoop()
  {
    rclcpp::Time now = this->now();
    double dt = (now - last_time_).seconds();
    if (dt <= 0.0) return;
    last_time_ = now;

    int d_left = left_ticks_ - prev_left_ticks_;
    int d_right = right_ticks_ - prev_right_ticks_;

    prev_left_ticks_ = left_ticks_;
    prev_right_ticks_ = right_ticks_;

    double ticks_to_rad = 2.0 * M_PI / static_cast<double>(ppr_);

    left_pos_ += d_left * ticks_to_rad;
    right_pos_ += d_right * ticks_to_rad;

    left_vel_ = (d_left * ticks_to_rad) / dt;
    right_vel_ = (d_right * ticks_to_rad) / dt;

    // ---- Publish JointState ----
    sensor_msgs::msg::JointState js;
    js.header.stamp = now;
    js.name = {"left_wheel_joint", "right_wheel_joint"};
    js.position = {left_pos_, right_pos_};
    js.velocity = {left_vel_, right_vel_};
    joint_pub_->publish(js);

    // ---- Inverse Kinematics ----
    double v_l_ref =
      (cmd_v_ - (cmd_w_ * wheel_sep_ / 2.0)) / wheel_radius_;
    double v_r_ref =
      (cmd_v_ + (cmd_w_ * wheel_sep_ / 2.0)) / wheel_radius_;

    // ---- P Controller ----
    double u_l = kp_ * (v_l_ref - left_vel_);
    double u_r = kp_ * (v_r_ref - right_vel_);

    // ---- Normalize to [-1, 1] ----
    u_l = std::clamp(u_l / max_wheel_speed_, -1.0, 1.0);
    u_r = std::clamp(u_r / max_wheel_speed_, -1.0, 1.0);

    // ---- Publish Wheel Command ----
    std_msgs::msg::Float32MultiArray cmd;
    cmd.data = {static_cast<float>(u_l), static_cast<float>(u_r)};
    wheel_cmd_pub_->publish(cmd);
  }

  // ---------------- VARIABLES ----------------
  int ppr_;
  double wheel_radius_, wheel_sep_, kp_, max_wheel_speed_;
  std::chrono::duration<double> control_period_;

  int left_ticks_{0}, right_ticks_{0};
  int prev_left_ticks_{0}, prev_right_ticks_{0};

  double left_pos_{0.0}, right_pos_{0.0};
  double left_vel_{0.0}, right_vel_{0.0};

  double cmd_v_{0.0}, cmd_w_{0.0};

  rclcpp::Time last_time_;

  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr left_ticks_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr right_ticks_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr wheel_cmd_pub_;

  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DiffDriveController>());
  rclcpp::shutdown();
  return 0;
}
