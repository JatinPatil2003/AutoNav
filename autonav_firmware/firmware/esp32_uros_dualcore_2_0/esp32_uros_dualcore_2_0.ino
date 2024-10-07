#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <esp_system.h>
#include <rmw_microros/rmw_microros.h>

#include <std_msgs/msg/float64_multi_array.h>

#include <AccelStepper.h>
#include <AsyncTimer.h>

rcl_subscription_t motorcommand_sub;
rcl_publisher_t motorfeedback_pub;

std_msgs__msg__Float64MultiArray feedbackMsg;
std_msgs__msg__Float64MultiArray commandMsg;

double data_array[2] = {0};

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

TaskHandle_t Publisher;
TaskHandle_t Loop;

bool micro_ros_init_successful;

#define dirPinLeft 4
#define stepPinLeft 2
#define enableLeft 5

#define dirPinRight 18
#define stepPinRight 19
#define enableRight 23

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

void motorcomand_callback(const void * msgin)
{  
  const std_msgs__msg__Float64MultiArray * msg = (const std_msgs__msg__Float64MultiArray *)msgin;
  int speedLeft = (int)(msg->data.data[0] * 1600 / (M_PI * 2)); 
  int speedRight = (int)(msg->data.data[1] * -1600 / (M_PI * 2));

  stepperLeft.setSpeed(speedLeft);
  stepperRight.setSpeed(speedRight);
}

void stepperrun_callback(void * pvParameters) {
  for(;;){
    stepperLeft.runSpeed();
    stepperRight.runSpeed();
  }
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
    
  commandMsg.data.capacity = 2;
  commandMsg.data.size = 2;
//  commandMsg.data.data = (double*)malloc(commandMsg.data.capacity * sizeof(double));
  commandMsg.data.data = data_array;

  feedbackMsg.data.data = data_array;
  feedbackMsg.data.size = 2;
  feedbackMsg.data.capacity = 2;

  executor = rclc_executor_get_zero_initialized_executor();
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &motorcommand_sub, &commandMsg, &motorcomand_callback, ON_NEW_DATA));

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

//  stepperLeft.setSpeed(100);
//  stepperRight.setSpeed(-100);
  
  delay(2000);

  state = WAITING_AGENT;

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
        stepperLeft.setCurrentPosition(0);
        stepperRight.setCurrentPosition(0);
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
      stepperLeft.setCurrentPosition(0);
      stepperRight.setCurrentPosition(0);
      state = WAITING_AGENT;
      break;
    default:
      break;
  }

  // RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(0)));
  data_array[0] = (stepperLeft.currentPosition() / 1600.0) * M_PI * 2;
  data_array[1] = (stepperRight.currentPosition() / -1600.0) * M_PI * 2;
  feedbackMsg.data.data = data_array;
  RCSOFTCHECK(rcl_publish(&motorfeedback_pub, &feedbackMsg, NULL));

}
