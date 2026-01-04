/**
 * Micro-ROS Arduino Code for Raspberry Pi Pico (Earle Philhower Core)
 * Differential Drive Robot with TB6612FNG and Single Output Encoders
 * * Target ROS 2 Distro: Jazzy Jalisco (compatible with Humble/Iron)
 * Transport: Native USB (Serial)
 */

#include <Arduino.h>
#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <geometry_msgs/msg/twist.h>
#include <nav_msgs/msg/odometry.h>

// ================================================================
// PIN DEFINITIONS
// ================================================================

// Motor Driver TB6612FNG
const int PIN_STBY = 12;
const int PIN_PWMB = 11; // Right Motor Speed
const int PIN_IN2B = 10; // Right Motor Direction 2
const int PIN_IN1B = 9;  // Right Motor Direction 1
const int PIN_IN1A = 8;  // Left Motor Direction 1
const int PIN_IN2A = 7;  // Left Motor Direction 2
const int PIN_PWMA = 6;  // Left Motor Speed

// Encoders (Single Output Hall Effect)
const int PIN_ENC_L = 14; // Left Encoder
const int PIN_ENC_R = 15; // Right Encoder

// ================================================================
// ROBOT CONFIGURATION (MEASURE THESE!)
// ================================================================

// Distance between wheels in meters
const float WHEEL_BASE = 0.20; 

// Wheel radius in meters
const float WHEEL_RADIUS = 0.04; 

// Ticks per revolution of the encoder output shaft
// Note: Calculate this manually: (Motor Gear Ratio) * (Encoder Pulses per Rev)
const float TICKS_PER_REV = 200.0; 

// Max PWM value (8-bit)
const int MAX_PWM = 255;

// ================================================================
// GLOBALS
// ================================================================

// Encoder Ticks
volatile long enc_left_ticks = 0;
volatile long enc_right_ticks = 0;

// Direction State for Single Channel Encoders
// 1 = Forward, -1 = Backward
int dir_left = 1;
int dir_right = 1;

// Micro-ROS Entities
rcl_node_t node;
rclc_support_t support;
rcl_allocator_t allocator;
rclc_executor_t executor;

rcl_publisher_t odom_publisher;
rcl_subscription_t twist_subscriber;

nav_msgs__msg__Odometry odom_msg;
geometry_msgs__msg__Twist twist_msg;

// Timer for control loop
unsigned long last_time = 0;
const unsigned long LOOP_TIME_MS = 50; // 20 Hz

// Odometry State
float x_pos = 0.0;
float y_pos = 0.0;
float theta = 0.0;

// ================================================================
// INTERRUPT SERVICE ROUTINES (ISRs)
// ================================================================

// Because these are single channel encoders, we rely on the
// commanded direction to decide if we are incrementing or decrementing.
void isr_enc_left() {
  if (dir_left > 0) enc_left_ticks++;
  else enc_left_ticks--;
}

void isr_enc_right() {
  if (dir_right > 0) enc_right_ticks++;
  else enc_right_ticks--;
}

// ================================================================
// MOTOR CONTROL FUNCTIONS
// ================================================================

void setMotor(int pwmPin, int in1Pin, int in2Pin, int speed, int &directionState) {
  // Clamp speed
  if (speed > MAX_PWM) speed = MAX_PWM;
  if (speed < -MAX_PWM) speed = -MAX_PWM;

  if (speed > 0) {
    // Forward
    digitalWrite(in1Pin, HIGH);
    digitalWrite(in2Pin, LOW);
    directionState = 1;
  } else if (speed < 0) {
    // Reverse
    digitalWrite(in1Pin, LOW);
    digitalWrite(in2Pin, HIGH);
    directionState = -1;
    speed = -speed; // Make PWM positive
  } else {
    // Stop
    digitalWrite(in1Pin, LOW);
    digitalWrite(in2Pin, LOW);
    speed = 0;
  }
  
  analogWrite(pwmPin, speed);
}

// ================================================================
// ROS CALLBACKS
// ================================================================

void subscription_callback(const void * msgin) {
  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;

  float linear = msg->linear.x;
  float angular = msg->angular.z;

  // Inverse Kinematics: Calculate target speed for each wheel (m/s)
  float v_left = linear - (angular * WHEEL_BASE / 2.0);
  float v_right = linear + (angular * WHEEL_BASE / 2.0);

  // Convert m/s to PWM
  // Needs calibration: Map Max m/s to 255 PWM. 
  // Simple linear approximation: PWM = (v / V_MAX) * 255
  // Assuming Max Speed approx 0.5 m/s for this example. ADJUST THIS CONSTANT!
  const float MAX_SPEED_MS = 0.5; 
  
  int pwm_left = (int)((v_left / MAX_SPEED_MS) * 255.0);
  int pwm_right = (int)((v_right / MAX_SPEED_MS) * 255.0);

  setMotor(PIN_PWMA, PIN_IN1A, PIN_IN2A, pwm_left, dir_left);
  setMotor(PIN_PWMB, PIN_IN1B, PIN_IN2B, pwm_right, dir_right);
}

// ================================================================
// SETUP
// ================================================================

void setup() {
  // 1. Hardware Init
  pinMode(PIN_STBY, OUTPUT);
  pinMode(PIN_PWMA, OUTPUT);
  pinMode(PIN_IN1A, OUTPUT);
  pinMode(PIN_IN2A, OUTPUT);
  pinMode(PIN_PWMB, OUTPUT);
  pinMode(PIN_IN1B, OUTPUT);
  pinMode(PIN_IN2B, OUTPUT);

  pinMode(PIN_ENC_L, INPUT_PULLUP);
  pinMode(PIN_ENC_R, INPUT_PULLUP);

  digitalWrite(PIN_STBY, HIGH); // Enable Motor Driver

  attachInterrupt(digitalPinToInterrupt(PIN_ENC_L), isr_enc_left, RISING);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_R), isr_enc_right, RISING);

  // 2. Transport Init
//  Serial.begin(115200);
  set_microros_transports();

  delay(2000); // Wait for agent connection

  // 3. Micro-ROS Init
  allocator = rcl_get_default_allocator();

  // Create init_options
  rclc_support_init(&support, 0, NULL, &allocator);

  // Create node
  rclc_node_init_default(&node, "pico_diff_drive", "", &support);

  // Create Subscriber (Twist)
  rclc_subscription_init_default(
    &twist_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "cmd_vel"
  );

  // Create Publisher (Odometry)
  rclc_publisher_init_default(
    &odom_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
    "odom"
  );

  // Create Executor
  rclc_executor_init(&executor, &support.context, 1, &allocator);
  rclc_executor_add_subscription(&executor, &twist_subscriber, &twist_msg, &subscription_callback, ON_NEW_DATA);
}

// ================================================================
// LOOP
// ================================================================

void loop() {
  // 1. Handle ROS Communication
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));

  // 2. Odometry Calculation (Run at 20Hz)
  unsigned long current_time = millis();
  if (current_time - last_time >= LOOP_TIME_MS) {
    double dt = (current_time - last_time) / 1000.0;
    last_time = current_time;

    // Snapshot encoder values (atomic read ideally, but long is usually fine on 32-bit)
    long current_enc_left = enc_left_ticks;
    long current_enc_right = enc_right_ticks;
    
    // Reset ticks for relative calculation (or keep cumulative and diff)
    // Here we use cumulative method
    static long prev_enc_left = 0;
    static long prev_enc_right = 0;

    long d_ticks_l = current_enc_left - prev_enc_left;
    long d_ticks_r = current_enc_right - prev_enc_right;

    prev_enc_left = current_enc_left;
    prev_enc_right = current_enc_right;

    // Calculate distance per wheel
    float dist_per_tick = (2.0 * PI * WHEEL_RADIUS) / TICKS_PER_REV;
    float d_left = d_ticks_l * dist_per_tick;
    float d_right = d_ticks_r * dist_per_tick;

    // Calculate robot movement
    float d_center = (d_left + d_right) / 2.0;
    float d_theta = (d_right - d_left) / WHEEL_BASE;

    // Update Odometry
    x_pos += d_center * cos(theta + d_theta / 2.0);
    y_pos += d_center * sin(theta + d_theta / 2.0);
    theta += d_theta;

    // Normalize Theta to -PI to PI
    if (theta > PI) theta -= 2 * PI;
    else if (theta < -PI) theta += 2 * PI;

    // 3. Populate and Publish Odometry Message
    odom_msg.header.stamp.sec = millis() / 1000;
    odom_msg.header.stamp.nanosec = (millis() % 1000) * 1000000;
    odom_msg.header.frame_id.data = (char*)"odom";
    odom_msg.header.frame_id.size = strlen(odom_msg.header.frame_id.data);
    odom_msg.child_frame_id.data = (char*)"base_link";
    odom_msg.child_frame_id.size = strlen(odom_msg.child_frame_id.data);

    // Position
    odom_msg.pose.pose.position.x = x_pos;
    odom_msg.pose.pose.position.y = y_pos;
    odom_msg.pose.pose.position.z = 0.0;

    // Orientation (Quaternion from Yaw)
    odom_msg.pose.pose.orientation.z = sin(theta / 2.0);
    odom_msg.pose.pose.orientation.w = cos(theta / 2.0);

    // Velocity (Linear and Angular)
    odom_msg.twist.twist.linear.x = d_center / dt;
    odom_msg.twist.twist.angular.z = d_theta / dt;

    rcl_publish(&odom_publisher, &odom_msg, NULL);
  }
}
