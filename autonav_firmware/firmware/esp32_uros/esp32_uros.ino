#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <esp_system.h>

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

#define dirPinLeft 4
#define stepPinLeft 2
#define enableLeft 5

#define dirPinRight 18
#define stepPinRight 19
#define enableRight 23

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

AccelStepper stepperLeft(AccelStepper::DRIVER, stepPinLeft, dirPinLeft);
AccelStepper stepperRight(AccelStepper::DRIVER, stepPinRight, dirPinRight);

AsyncTimer t;


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
  
//  stepperLeft.setSpeed(50); 
//  stepperRight.setSpeed(-50); 
}

void feedback_timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    feedbackMsg.data.data = data_array;
    RCSOFTCHECK(rcl_publish(&motorfeedback_pub, &feedbackMsg, NULL));
  }
}

void setup() {
  set_microros_transports();
  
  pinMode(enableLeft, OUTPUT);
  pinMode(enableRight, OUTPUT);

  digitalWrite(enableLeft, LOW);
  digitalWrite(enableRight, LOW);

  stepperLeft.setMaxSpeed(7000);
  stepperRight.setMaxSpeed(7000);

  stepperLeft.setSpeed(0); // Start moving in reverse
  stepperRight.setSpeed(-0);
  
  delay(2000);

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

  RCSOFTCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(0),  // Adjust the interval as needed
    feedback_timer_callback));

  commandMsg.data.capacity = 2;
  commandMsg.data.size = 2;
  commandMsg.data.data = (double*)malloc(commandMsg.data.capacity * sizeof(double));

  feedbackMsg.data.data = data_array;
  feedbackMsg.data.size = 2;
  feedbackMsg.data.capacity = 2;
    
  RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator));
//  RCSOFTCHECK(rclc_executor_add_timer(&executor, &timer));
  RCCHECK(rclc_executor_add_subscription(&executor, &motorcommand_sub, &commandMsg, &motorcomand_callback, ON_NEW_DATA));

  t.setInterval([]() { RCSOFTCHECK(rcl_publish(&motorfeedback_pub, &feedbackMsg, NULL)); }, 120);
}

void loop() {
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(0)));
  stepperLeft.runSpeed();
  stepperRight.runSpeed();
  data_array[0] = (stepperLeft.currentPosition() / 1600.0) * M_PI * 2;
  data_array[1] = (stepperRight.currentPosition() / -1600.0) * M_PI * 2;
  feedbackMsg.data.data = data_array;
  t.handle();
}
