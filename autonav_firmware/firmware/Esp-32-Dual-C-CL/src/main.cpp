#define dirPinLeft 21
#define stepPinLeft 19
#define enableLeft 2

#define dirPinRight 18
#define stepPinRight 5
#define enableRight 2

#define REVSTEPS 1600

#define LED_NUM 24
#define LED_PIN 17
#define BRIGHTNESS 128

#define LEncoder_output_A 32
#define LEncoder_output_B 33
#define REncoder_output_A 26
#define REncoder_output_B 27

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
#include <std_msgs/msg/int64.h>

#include <AccelStepper.h>
#include <AsyncTimer.h>
#include <Adafruit_NeoPixel.h>

#include "led.h"

rcl_subscription_t motorcommand_sub;
rcl_subscription_t motor_emergency_sub;
rcl_subscription_t led_sub;
rcl_publisher_t motorfeedback_pub;

std_msgs__msg__Bool emergencyMsg;
std_msgs__msg__Float64MultiArray feedbackMsg;
std_msgs__msg__Float64MultiArray commandMsg;
std_msgs__msg__Int64 led_msg;

double data_array[2] = {0};

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

TaskHandle_t Publisher;
TaskHandle_t Loop;

bool micro_ros_init_successful;


volatile long LencoderTicks = 0;
volatile long RencoderTicks = 0;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}
#define EXECUTE_EVERY_N_MS(MS, X)  do { \
  static volatile int64_t init = -1; \
  if (init == -1) { init = uxr_millis();} \
  if (uxr_millis() - init > MS) { X; init = uxr_millis();} \
} while (0)\

enum states {
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
} state;

AccelStepper stepperLeft(AccelStepper::DRIVER, stepPinLeft, dirPinLeft);
AccelStepper stepperRight(AccelStepper::DRIVER, stepPinRight, dirPinRight);

void error_loop(){
  int loop_counter = 0;
  while(1){
    loop_counter += 1;
    delay(100);
    if(loop_counter >= 50){
      esp_restart();
    }
  }
}

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

void motorcomand_callback(const void * msgin)
{  
  const std_msgs__msg__Float64MultiArray * msg = (const std_msgs__msg__Float64MultiArray *)msgin;
  int speedLeft = (int)(msg->data.data[0] * -REVSTEPS / (M_PI * 2)); 
  int speedRight = (int)(msg->data.data[1] * REVSTEPS / (M_PI * 2));

  stepperLeft.setSpeed(speedLeft);
  stepperRight.setSpeed(speedRight);
}

void stepperrun_callback(void * pvParameters) {
  for(;;){
    stepperLeft.runSpeed();
    stepperRight.runSpeed();
  }
}

void motor_emergency_callback(const void * msgin)
{
  const std_msgs__msg__Bool * msg = (const std_msgs__msg__Bool *)msgin;

  if (msg->data)
  {
    // Emergency stop: disable motors
    digitalWrite(enableLeft, HIGH);
    digitalWrite(enableRight, HIGH);
    Serial.println("[EMERGENCY] Motors disabled!");
  }
  else
  {
    // Resume operation
    digitalWrite(enableLeft, LOW);
    digitalWrite(enableRight, LOW);
    Serial.println("[EMERGENCY] Motors enabled!");
  }
}

void led_status_callback(const void * msgin)
{
  const std_msgs__msg__Int64 * msg = (const std_msgs__msg__Int64 *)msgin;

  led_status = (int)msg->data;
}

bool create_entities()
{
  allocator = rcl_get_default_allocator();

  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  RCCHECK(rclc_node_init_default(&node, "esp32_node", "", &support));

  RCCHECK(rclc_publisher_init_default(
    &motorfeedback_pub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64MultiArray),
    "motor/feedback"));

  RCCHECK(rclc_subscription_init_default(
    &motorcommand_sub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64MultiArray),
    "motor/command"));

  RCCHECK(rclc_subscription_init_default(
    &motor_emergency_sub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
    "motor/emergency"));

  RCCHECK(rclc_subscription_init_default(
      &led_sub,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int64),
      "/led_status"));
    
  commandMsg.data.capacity = 2;
  commandMsg.data.size = 2;
//  commandMsg.data.data = (double*)malloc(commandMsg.data.capacity * sizeof(double));
  commandMsg.data.data = data_array;

  feedbackMsg.data.data = data_array;
  feedbackMsg.data.size = 2;
  feedbackMsg.data.capacity = 2;

  executor = rclc_executor_get_zero_initialized_executor();
  RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &motorcommand_sub, &commandMsg, &motorcomand_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &motor_emergency_sub, &emergencyMsg, &motor_emergency_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &led_sub, &led_msg, &led_status_callback, ON_NEW_DATA));

  return true;
}

void destroy_entities()
{
  rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
  (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  rcl_publisher_fini(&motorfeedback_pub, &node);
  rcl_subscription_fini(&motorcommand_sub, &node);
  rclc_executor_fini(&executor);
  rcl_node_fini(&node);
  rclc_support_fini(&support);
}

void setup() {
  set_microros_transports();
  
  pinMode(enableLeft, OUTPUT);
  pinMode(enableRight, OUTPUT);

  digitalWrite(enableLeft, LOW);
  digitalWrite(enableRight, LOW);

  stepperLeft.setMaxSpeed(7000);
  stepperRight.setMaxSpeed(7000);

  led_setup();

//  stepperLeft.setSpeed(100);
//  stepperRight.setSpeed(-100);

  pinMode(LEncoder_output_A, INPUT_PULLUP);
  pinMode(LEncoder_output_B, INPUT_PULLUP);
  pinMode(REncoder_output_A, INPUT_PULLUP);
  pinMode(REncoder_output_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(LEncoder_output_B), ISR_encoder_L, RISING);
  attachInterrupt(digitalPinToInterrupt(REncoder_output_B), ISR_encoder_R, RISING);
  
  delay(2000);

  state = WAITING_AGENT;

  Serial.begin(115200);

  xTaskCreatePinnedToCore(
                    stepperrun_callback,   /* Task function. */
                    "Loop",     /* name of task. */
                    50000,       /* Stack size of task */
                    NULL,        /* parameter of the task */
                    0,           /* priority of the task */
                    &Loop,      /* Task handle to keep track of created task */
                    0);          /* pin task to core 1 */
}

void loop() {
  switch (state) {
    case WAITING_AGENT:
      EXECUTE_EVERY_N_MS(500, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;);
      break;
    case AGENT_AVAILABLE:
      state = (true == create_entities()) ? AGENT_CONNECTED : WAITING_AGENT;
      if (state == WAITING_AGENT) {
        destroy_entities();
        RencoderTicks = 0;
        LencoderTicks = 0;
      };
      break;
    case AGENT_CONNECTED:
      EXECUTE_EVERY_N_MS(200, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
      if (state == AGENT_CONNECTED) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(0));
      }
      break;
    case AGENT_DISCONNECTED:
      destroy_entities();
      RencoderTicks = 0;
      LencoderTicks = 0;
      state = WAITING_AGENT;
      break;
    default:
      break;
  }

  // RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(0)));
  data_array[0] = RencoderTicks * (2 * M_PI / 1000.0);
  data_array[1] = LencoderTicks * (2 * M_PI / -1000.0);
  feedbackMsg.data.data = data_array;
  RCSOFTCHECK(rcl_publish(&motorfeedback_pub, &feedbackMsg, NULL));
  
  led_handle();
}


// ros2 topic pub /led_status std_msgs/msg/Int64 "data: 1" --once 