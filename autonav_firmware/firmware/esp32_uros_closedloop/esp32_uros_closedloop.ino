#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <esp_system.h>
#include <rmw_microros/rmw_microros.h>

#include <std_msgs/msg/float64_multi_array.h>
#include <std_msgs/msg/bool.h>
#include <TMC2209.h>

// ====================== Encoder Pins ======================
#define LEncoder_output_A 32
#define LEncoder_output_B 33
#define REncoder_output_A 26
#define REncoder_output_B 27

volatile long LencoderTicks = 0;
volatile long RencoderTicks = 0;

// ====================== Motor Driver Pins ======================
#define DRIVER_RX_PIN 16
#define DRIVER_TX_PIN 17
#define ENABLE_PIN    2  // shared enable pin for both motors

#define RUN_CURRENT_PERCENTAGE 95
#define STOP_CURRENT_PERCENTAGE 35

HardwareSerial &serial_stream = Serial2;
TMC2209 stepper_left;
TMC2209 stepper_right;

// ====================== ROS Variables ======================
rcl_subscription_t motorcommand_sub;
rcl_publisher_t motorfeedback_pub;
rcl_subscription_t motor_emergency_sub;
std_msgs__msg__Bool emergencyMsg;
std_msgs__msg__Float64MultiArray feedbackMsg;
std_msgs__msg__Float64MultiArray commandMsg;

double data_array[2] = {0};
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

bool motors_enabled = true;
bool micro_ros_init_successful;

enum states {
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
} state;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}
#define EXECUTE_EVERY_N_MS(MS, X)  do { \
  static volatile int64_t init = -1; \
  if (init == -1) { init = uxr_millis();} \
  if (uxr_millis() - init > MS) { X; init = uxr_millis();} \
} while (0)

// ====================== Encoder Interrupts ======================
void IRAM_ATTR ISR_encoder_L() {
  if (digitalRead(LEncoder_output_A) == HIGH)
    LencoderTicks++;
  else
    LencoderTicks--;
}

void IRAM_ATTR ISR_encoder_R() {
  if (digitalRead(REncoder_output_A) == HIGH)
    RencoderTicks++;
  else
    RencoderTicks--;
}

// ====================== Error Loop ======================
void error_loop() {
  int loop_counter = 0;
  while (1) {
    loop_counter++;
    delay(100);
    if (loop_counter >= 50) esp_restart();
  }
}

// ====================== ROS Motor Command Callback ======================
void motorcomand_callback(const void * msgin) {
  if (!motors_enabled) return;
  const std_msgs__msg__Float64MultiArray * msg = (const std_msgs__msg__Float64MultiArray *)msgin;

  // Convert angular velocity (rad/s) to stepper velocity (TMC internal unit)
  float velLeft_rad = msg->data.data[0];
  float velRight_rad = msg->data.data[1];

  int32_t velLeft  = (int32_t)(fabs(velLeft_rad) * 72000 / (M_PI * 2));
  int32_t velRight = (int32_t)(fabs(velRight_rad) * 72000 / (M_PI * 2));

  // Set direction explicitly based on sign
  if (velLeft_rad >= 0) {
    stepper_left.enableInverseMotorDirection();   // reverse
  } else {
    stepper_left.disableInverseMotorDirection();  // forward
  }

  if (velRight_rad >= 0) {
    stepper_right.disableInverseMotorDirection(); // forward
  } else {
    stepper_right.enableInverseMotorDirection();  // reverse
  }

  // Apply velocity (always positive)
  stepper_left.moveAtVelocity(velLeft);
  stepper_right.moveAtVelocity(velRight);
}


// ---------------------- ROS Motor Emergency Callback ---------------------- //
void motor_emergency_callback(const void * msgin)
{
  const std_msgs__msg__Bool * msg = (const std_msgs__msg__Bool *)msgin;

  if (msg->data)
  {
    // Emergency stop: disable motors
    stepper_left.disable();
    stepper_right.disable();
    motors_enabled = false;
    Serial.println("[EMERGENCY] Motors disabled!");
  }
  else
  {
    // Resume operation
    stepper_left.enable();
    stepper_right.enable();
    motors_enabled = true;
    Serial.println("[EMERGENCY] Motors enabled!");
  }
}

// ====================== ROS Entity Management ======================
bool create_entities() {
  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "esp32_node", "", &support));

  RCCHECK(rclc_publisher_init_default(
    &motorfeedback_pub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64MultiArray),
    "motor/feedback"));

  RCCHECK(rclc_subscription_init_default(
    &motorcommand_sub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64MultiArray),
    "motor/command"));

RCCHECK(rclc_subscription_init_default(
    &motor_emergency_sub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
    "motor/emergency"));
    
  commandMsg.data.capacity = 2;
  commandMsg.data.size = 2;
  commandMsg.data.data = data_array;

  feedbackMsg.data.data = data_array;
  feedbackMsg.data.size = 2;
  feedbackMsg.data.capacity = 2;

  executor = rclc_executor_get_zero_initialized_executor();
  RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &motorcommand_sub, &commandMsg, &motorcomand_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &motor_emergency_sub, &emergencyMsg, &motor_emergency_callback, ON_NEW_DATA));

  return true;
}

void destroy_entities() {
  rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
  (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  rcl_publisher_fini(&motorfeedback_pub, &node);
  rcl_subscription_fini(&motorcommand_sub, &node);
  rclc_executor_fini(&executor);
  rcl_node_fini(&node);
  rclc_support_fini(&support);
}

// ====================== Motor Initialization ======================
void init_motors() {
  serial_stream.begin(115200, SERIAL_8N1, DRIVER_RX_PIN, DRIVER_TX_PIN);

  stepper_right.setup(serial_stream, 115200, TMC2209::SERIAL_ADDRESS_0, DRIVER_RX_PIN, DRIVER_TX_PIN);
  stepper_left.setup(serial_stream, 115200, TMC2209::SERIAL_ADDRESS_2, DRIVER_RX_PIN, DRIVER_TX_PIN);

  stepper_right.setHardwareEnablePin(ENABLE_PIN);
  stepper_left.setHardwareEnablePin(ENABLE_PIN);

  stepper_right.setRunCurrent(RUN_CURRENT_PERCENTAGE);
  stepper_left.setRunCurrent(RUN_CURRENT_PERCENTAGE);
  stepper_right.setHoldCurrent(STOP_CURRENT_PERCENTAGE);
  stepper_left.setHoldCurrent(STOP_CURRENT_PERCENTAGE);

  stepper_right.enableStealthChop();
  stepper_right.enableAutomaticCurrentScaling();
  stepper_right.enableAutomaticGradientAdaptation();

  stepper_left.enableStealthChop();
  stepper_left.enableAutomaticCurrentScaling();
  stepper_left.enableAutomaticGradientAdaptation();

  stepper_right.enable();
  stepper_left.enable();
}

// ====================== Setup ======================
void setup() {
  Serial.begin(115200);
  set_microros_transports();

  pinMode(LEncoder_output_A, INPUT_PULLUP);
  pinMode(LEncoder_output_B, INPUT_PULLUP);
  pinMode(REncoder_output_A, INPUT_PULLUP);
  pinMode(REncoder_output_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(LEncoder_output_B), ISR_encoder_L, RISING);
  attachInterrupt(digitalPinToInterrupt(REncoder_output_B), ISR_encoder_R, RISING);

  init_motors();
  delay(2000);

  state = WAITING_AGENT;
}

// ====================== Main Loop ======================
void loop() {
  switch (state) {
    case WAITING_AGENT:
      EXECUTE_EVERY_N_MS(500, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;);
      break;

    case AGENT_AVAILABLE:
      state = (true == create_entities()) ? AGENT_CONNECTED : WAITING_AGENT;
      if (state == WAITING_AGENT) destroy_entities();
      break;

    case AGENT_CONNECTED:
      EXECUTE_EVERY_N_MS(200, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
      if (state == AGENT_CONNECTED) rclc_executor_spin_some(&executor, RCL_MS_TO_NS(0));
      break;

    case AGENT_DISCONNECTED:
      destroy_entities();
      state = WAITING_AGENT;
      break;
  }

  // Feedback publishing
  data_array[0] = LencoderTicks * (2 * M_PI / 1000.0);
  data_array[1] = RencoderTicks * (2 * M_PI / -1000.0);
  feedbackMsg.data.data = data_array;
  RCSOFTCHECK(rcl_publish(&motorfeedback_pub, &feedbackMsg, NULL));
}
