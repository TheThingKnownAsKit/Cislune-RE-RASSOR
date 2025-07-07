#include <Arduino.h>
#include <micro_ros_platformio.h>  // micro-ROS PlatformIO library
// micro-ROS and ROS 2 message headers
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>

// Motor control pin definitions (TODO ADJUST THESE)
#define LEFT_DIR_FWD_PIN   2    // left motor forward direction pin
#define LEFT_DIR_BCK_PIN   3    // left motor backward/reverse pin
#define LEFT_PWM_PIN       4    // left motor PWM speed pin
#define RIGHT_DIR_FWD_PIN  5    // right motor forward pin
#define RIGHT_DIR_BCK_PIN  6    // right motor backward pin
#define RIGHT_PWM_PIN      7    // right motor PWM pin

// Robot constants
const float WHEEL_BASE = 0.30;     // distance between wheels in meters (example)
const float MAX_LIN_SPEED = 1.0;   // m/s corresponding to full speed
const int MAX_PWM = 255;
const float PWM_PER_MPS = (MAX_PWM / MAX_LIN_SPEED);  // PWM increment per 1 m/s of speed

// micro-ROS core structures
rcl_subscription_t cmd_vel_sub;
geometry_msgs__msg__Twist cmd_vel_msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_node_t node;
rcl_allocator_t allocator;

// Simple error handler macro and LED for debug
#define RCCHECK(fn)  { rcl_ret_t temp_rc = (fn); if ((temp_rc != RCL_RET_OK)) { errorLoop(); } }
#define RCSOFTCHECK(fn)  { rcl_ret_t temp_rc = (fn); if ((temp_rc != RCL_RET_OK)) { /* non-fatal error */ } }
const int LED_PIN = 13;  // Teensy 4.1 on-board LED (pin 13)

void errorLoop() {
  // Indicate fatal error: blink LED rapidly
  pinMode(LED_PIN, OUTPUT);
  while (1) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

// Callback function to handle incoming Twist messages
void cmdVelCallback(const void * msgin) {
  // Cast the incoming ROS message to the Twist type
  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
  float linear_x = msg->linear.x;
  float angular_z = msg->angular.z;
  
  // Compute individual wheel speeds (m/s)
  float left_speed  = linear_x - (angular_z * WHEEL_BASE / 2.0f);
  float right_speed = linear_x + (angular_z * WHEEL_BASE / 2.0f);
  
  // Convert speeds (m/s) to PWM values (0-255)
  int left_pwm  = (int) (fabs(left_speed) * PWM_PER_MPS);
  int right_pwm = (int) (fabs(right_speed) * PWM_PER_MPS);
  if (left_pwm  > MAX_PWM) left_pwm  = MAX_PWM;
  if (right_pwm > MAX_PWM) right_pwm = MAX_PWM;
  
  // Set motor directions
  if (left_speed >= 0.0f) {
    digitalWrite(LEFT_DIR_FWD_PIN, HIGH);
    digitalWrite(LEFT_DIR_BCK_PIN, LOW);
  } else {
    digitalWrite(LEFT_DIR_FWD_PIN, LOW);
    digitalWrite(LEFT_DIR_BCK_PIN, HIGH);
  }
  if (right_speed >= 0.0f) {
    digitalWrite(RIGHT_DIR_FWD_PIN, HIGH);
    digitalWrite(RIGHT_DIR_BCK_PIN, LOW);
  } else {
    digitalWrite(RIGHT_DIR_FWD_PIN, LOW);
    digitalWrite(RIGHT_DIR_BCK_PIN, HIGH);
  }
  
  // Output PWM to motors
  analogWrite(LEFT_PWM_PIN, left_pwm);
  analogWrite(RIGHT_PWM_PIN, right_pwm);
}

void setup() {
  // Initialize serial transport for micro-ROS
  Serial.begin(115200);
  set_microros_serial_transports(Serial);
  
  // Initialize on-board LED (for error blink or debugging)
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  
  // Initialize motor control pins
  pinMode(LEFT_DIR_FWD_PIN, OUTPUT);
  pinMode(LEFT_DIR_BCK_PIN, OUTPUT);
  pinMode(LEFT_PWM_PIN, OUTPUT);
  pinMode(RIGHT_DIR_FWD_PIN, OUTPUT);
  pinMode(RIGHT_DIR_BCK_PIN, OUTPUT);
  pinMode(RIGHT_PWM_PIN, OUTPUT);
  // Ensure motors are stopped initially
  digitalWrite(LEFT_DIR_FWD_PIN, LOW);
  digitalWrite(LEFT_DIR_BCK_PIN, LOW);
  digitalWrite(RIGHT_DIR_FWD_PIN, LOW);
  digitalWrite(RIGHT_DIR_BCK_PIN, LOW);
  analogWrite(LEFT_PWM_PIN, 0);
  analogWrite(RIGHT_PWM_PIN, 0);
  
  delay(200);  // small delay before micro-ROS init (optional)
  
  // Initialize micro-ROS support and node
  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "teensy41_cmdvel_node", "", &support));
  
  // Create a subscriber for the Twist message on /cmd_vel
  RCCHECK(rclc_subscription_init_default(
    &cmd_vel_sub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "cmd_vel"
  ));
  
  // Create executor to handle the subscription callback
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  // Note: ON_NEW_DATA executes callback only when new data arrives
  RCCHECK(rclc_executor_add_subscription(&executor, &cmd_vel_sub, &cmd_vel_msg, &cmdVelCallback, ON_NEW_DATA));

  // Setup is done, keep LED on to show normal functioning
  digitalWrite(LED_PIN, LOW);
}

void loop() {
  // Spin executor to handle incoming messages
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
  // Add a small delay to avoid tight looping
  delay(10);
}
