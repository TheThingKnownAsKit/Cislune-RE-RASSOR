#include <Arduino.h>
#include <micro_ros_platformio.h>  // micro-ROS PlatformIO library
// micro-ROS and ROS 2 message headers
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>
#include <std_srvs/srv/trigger.h>

extern "C" void _reboot_Teensyduino_(void);

// Motor control pin definitions (TODO ADJUST THESE)
#define LEFT_DIR_FWD_PIN   2    // left motor forward direction pin
#define LEFT_DIR_BCK_PIN   3    // left motor backward/reverse pin
#define LEFT_PWM_PIN       4    // left motor PWM speed pin
#define RIGHT_DIR_FWD_PIN  5    // right motor forward pin
#define RIGHT_DIR_BCK_PIN  6    // right motor backward pin
#define RIGHT_PWM_PIN      7    // right motor PWM pin

// Declare functions
void destroy_entities(void);
bool create_entities(void);
void cmd_vel_callback(const void *msgin);
void reboot_callback(const void *req, void *res);

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
rcl_service_t reboot_srv;
std_srvs__srv__Trigger_Request req;
std_srvs__srv__Trigger_Response res;

const int LED_PIN = 13;  // Teensy 4.1 on-board LED (pin 13)
bool connected = false; // Is micro ros agent connected?
static volatile bool reboot_requested = false;

// Soft error checking: returns false if an error is detected. called in create_entities
#define ROS_SOFTCHECK(fn)                    \
  do {                                       \
    rcl_ret_t _rc = (fn);                    \
    if (_rc != RCL_RET_OK) {                 \
      return false;                          \
    }                                        \
  } while (0)

void destroy_entities(void) {
  // Stop all motors
  analogWrite(LEFT_PWM_PIN,  0);
  analogWrite(RIGHT_PWM_PIN, 0);

  digitalWrite(LEFT_DIR_FWD_PIN,  LOW);
  digitalWrite(LEFT_DIR_BCK_PIN,  LOW);
  digitalWrite(RIGHT_DIR_FWD_PIN, LOW);
  digitalWrite(RIGHT_DIR_BCK_PIN, LOW);

  // Clean up micro ros resources
  rclc_executor_fini(&executor);
  rcl_service_fini(&reboot_srv, &node);
  rcl_subscription_fini(&cmd_vel_sub, &node);
  rcl_node_fini(&node);
  rclc_support_fini(&support);
}

bool create_entities() {
  // Initialize micro-ROS support and node
  allocator = rcl_get_default_allocator();
  ROS_SOFTCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  ROS_SOFTCHECK(rclc_node_init_default(&node, "teensy41_cmdvel_node", "", &support));
  
  // Create a subscriber for the Twist message on /cmd_vel
  ROS_SOFTCHECK(rclc_subscription_init_default(
    &cmd_vel_sub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "cmd_vel"
  ));
  
  // Create executor to handle the subscription callback
  const size_t NUM_HANDLES = 2;
  ROS_SOFTCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  // Note: ON_NEW_DATA executes callback only when new data arrives
  ROS_SOFTCHECK(rclc_executor_add_subscription(&executor, &cmd_vel_sub, &cmd_vel_msg, &cmd_vel_callback, ON_NEW_DATA));

  // Create ROS2 service via micro ros trigger for request reboot
  std_srvs__srv__Trigger_Request__init(&req);
  std_srvs__srv__Trigger_Response__init(&res);

  ROS_SOFTCHECK(
  rclc_service_init_default(
     &reboot_srv,
     &node,
     ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, Trigger),
     "reboot_teensy"               // service name
  ));

  ROS_SOFTCHECK(rclc_executor_add_service(&executor, &reboot_srv, &req, &res, reboot_callback));

  return true;
}

// Callback function to handle service requests to reboot Teensy
void reboot_callback(const void *req, void *res) {
  std_srvs__srv__Trigger_Response *rsp = (std_srvs__srv__Trigger_Response *) res;

  rsp->success = true;
  const char msg[] = "Reboot request accepted";
  memcpy(rsp->message.data, msg, sizeof(msg));
  rsp->message.size = sizeof(msg);

  reboot_requested = true;                 // ask main loop to reboot
}

// Callback function to handle incoming Twist messages
void cmd_vel_callback(const void *msgin) {
  // Cast the incoming ROS message to the Twist type
  const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;
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
  create_entities();
}

void loop() {

  if (reboot_requested) {
    delay(20);
    _reboot_Teensyduino_();
  }

    // Ping the micro-ROS agent with a short timeout
    if (rmw_uros_ping_agent(20, 2) == RMW_RET_OK) {
        if (!connected) {
            // Agent is up, but not connected yet – initialize ROS entities
            if (create_entities()) {
              // Teensy is connected to micro ros agent, indicate with LED on
              digitalWrite(LED_PIN, HIGH);
              connected = true;
            } else {
              // Creation failed
              connected = false;
            }
        } else {
          // Already connected and agent is up – process ROS callbacks
          rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
        }
    } else {
        // Agent not reachable
        if (connected) {
            // Clean up ROS nodes if previously connected
            destroy_entities();  // (fini all ROS entities and context)
            connected = false;
        }
        // Stop motors for safety since no control commands are coming
        analogWrite(LEFT_PWM_PIN, 0);
        analogWrite(RIGHT_PWM_PIN, 0);
        digitalWrite(LEFT_DIR_FWD_PIN, LOW);
        digitalWrite(LEFT_DIR_BCK_PIN, LOW);
        digitalWrite(RIGHT_DIR_FWD_PIN, LOW);
        digitalWrite(RIGHT_DIR_BCK_PIN, LOW);
        // Blink an LED to indicate waiting for agent)
        digitalWrite(LED_PIN, !digitalRead(LED_PIN));
        delay(500);
    }
}