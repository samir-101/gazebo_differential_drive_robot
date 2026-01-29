#include <Arduino.h>
#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/float32_multi_array.h> // NEW: For sending [left, right] together

// --- PIN DEFINITIONS ---
#define STBY_PIN 12
#define PWMB_PIN 11
#define BIN2_PIN 10
#define BIN1_PIN 9
#define AIN1_PIN 8
#define AIN2_PIN 7
#define PWMA_PIN 6
#define ENC_LEFT_PIN 14
#define ENC_RIGHT_PIN 15
#define LED_PIN 25

// --- GLOBALS ---
rcl_publisher_t left_enc_pub;
rcl_publisher_t right_enc_pub;

// CHANGED: Single subscriber for both wheels
rcl_subscription_t wheels_cmd_sub; 

std_msgs__msg__Int32 left_enc_msg;
std_msgs__msg__Int32 right_enc_msg;

// CHANGED: MultiArray message to hold [left, right]
std_msgs__msg__Float32MultiArray wheels_cmd_msg; 

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

volatile long left_ticks = 0;
volatile long right_ticks = 0;
float target_left = 0.0;
float target_right = 0.0;

// --- ERROR CHECK ---
void RCCHECK(rcl_ret_t stat) {
  if (stat != RCL_RET_OK) {
    while (1) {
      digitalWrite(LED_PIN, !digitalRead(LED_PIN)); // Fast blink on error
      delay(100);
    }
  }
}

// --- ISRs ---
void isr_enc_left() {
  if (target_left >= 0) left_ticks++; else left_ticks--;
}
void isr_enc_right() {
  if (target_right >= 0) right_ticks++; else right_ticks--;
}

// --- MOTOR CONTROL ---
void setMotor(int pwmPin, int in1Pin, int in2Pin, float speed) {
  float duty = constrain(speed, -1.0, 1.0);
  if (duty > 0) {
    digitalWrite(in1Pin, HIGH); digitalWrite(in2Pin, LOW);
  } else if (duty < 0) {
    digitalWrite(in1Pin, LOW); digitalWrite(in2Pin, HIGH);
    duty = -duty;
  } else {
    digitalWrite(in1Pin, LOW); digitalWrite(in2Pin, LOW);
  }
  analogWrite(pwmPin, (int)(duty * 255));
}

// --- CALLBACK ---
// Expects an array of 2 floats: [left_speed, right_speed]
void wheels_cb(const void * msgin) {
  const std_msgs__msg__Float32MultiArray * msg = (const std_msgs__msg__Float32MultiArray *)msgin;
  
  // Safety check: ensure we received at least 2 numbers
  if (msg->data.size >= 2) {
    target_left = msg->data.data[0];
    target_right = msg->data.data[1];
    
    setMotor(PWMA_PIN, AIN1_PIN, AIN2_PIN, target_left);
    setMotor(PWMB_PIN, BIN1_PIN, BIN2_PIN, target_right);
  }
}

// --- SETUP ---
void setup() {
  pinMode(LED_PIN, OUTPUT);
  pinMode(STBY_PIN, OUTPUT);
  pinMode(PWMB_PIN, OUTPUT); pinMode(BIN2_PIN, OUTPUT); pinMode(BIN1_PIN, OUTPUT);
  pinMode(AIN1_PIN, OUTPUT); pinMode(AIN2_PIN, OUTPUT); pinMode(PWMA_PIN, OUTPUT);
  pinMode(ENC_LEFT_PIN, INPUT_PULLUP);
  pinMode(ENC_RIGHT_PIN, INPUT_PULLUP);
  digitalWrite(STBY_PIN, HIGH);

  attachInterrupt(digitalPinToInterrupt(ENC_LEFT_PIN), isr_enc_left, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_RIGHT_PIN), isr_enc_right, RISING);

  set_microros_transports();
  allocator = rcl_get_default_allocator();

  // 1. Init Support & Node
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "pico_hardware_node", "", &support));

  // 2. Init Publishers
  RCCHECK(rclc_publisher_init_default(&left_enc_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "left_wheel/raw_ticks"));
  RCCHECK(rclc_publisher_init_default(&right_enc_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "right_wheel/raw_ticks"));

  // 3. Init Subscriber (Single Topic)
  RCCHECK(rclc_subscription_init_default(
    &wheels_cmd_sub, 
    &node, 
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray), 
    "wheel_cmd"  // New Topic Name
  ));
  
  // --- MEMORY ALLOCATION FOR MULTIARRAY ---
  // We must reserve space for the incoming data array, or micro-ROS will crash.
  // We expect 2 floats (Left, Right).
  static float memory_buffer[2]; 
  wheels_cmd_msg.data.capacity = 2;
  wheels_cmd_msg.data.size = 0;
  wheels_cmd_msg.data.data = memory_buffer;

  // 4. Init Executor
  // We now only need 3 handles (2 pubs + 1 sub)
  RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &wheels_cmd_sub, &wheels_cmd_msg, &wheels_cb, ON_NEW_DATA));
}

// --- LOOP ---
void loop() {
  left_enc_msg.data = (int32_t)left_ticks;
  right_enc_msg.data = (int32_t)right_ticks;

  rcl_publish(&left_enc_pub, &left_enc_msg, NULL);
  rcl_publish(&right_enc_pub, &right_enc_msg, NULL);

  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
  delay(50);
}
