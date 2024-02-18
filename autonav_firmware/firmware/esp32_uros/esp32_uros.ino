#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <esp_system.h>

#include <std_msgs/msg/int64_multi_array.h>
#include <std_msgs/msg/int32.h>
//  digitalWrite(LED_PIN, (msg->data == 0) ? LOW : HIGH);  

rcl_subscription_t leftmotor_sub;
rcl_subscription_t rightmotor_sub;
rcl_publisher_t motorfeedback_pub;
std_msgs__msg__Int64MultiArray encoderMsg;
std_msgs__msg__Int32 leftMsg;
std_msgs__msg__Int32 rightMsg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

#define LED_PIN 2

#define Encoder_output_A_L 25
#define Encoder_output_B_L 26
#define Encoder_output_A_R 32
#define Encoder_output_B_R 33

#define Motor_A_L 5
#define Motor_B_L 18
#define Motor_A_R 19
#define Motor_B_R 21

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

volatile int encodePulse_L = 0;
volatile int encodePulse_R = 0;

void error_loop(){
  int loop_counter = 0;
  while(1){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    loop_counter += 1;
    delay(100);
    if(loop_counter >= 50){
      esp_restart();
    }
  }
}

void rightsub_callback(const void * msgin)
{  
  const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;
  int pwm = msg->data;
  if(pwm > 0){
    analogWrite(Motor_A_R, pwm);
    analogWrite(Motor_B_R, 0);
  }
  else{
    analogWrite(Motor_A_R, 0);
    analogWrite(Motor_B_R, abs(pwm));
  }
}

void leftsub_callback(const void * msgin)
{  
  const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;
  int pwm = msg->data;
  if(pwm > 0){
    analogWrite(Motor_A_L, pwm);
    analogWrite(Motor_B_L, 0);
  }
  else{
    analogWrite(Motor_A_L, 0);
    analogWrite(Motor_B_L, abs(pwm));
  }
}

void setup() {
  set_microros_transports();
  
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  pinMode(Encoder_output_A_L, INPUT);
  pinMode(Encoder_output_B_L, INPUT);
  pinMode(Encoder_output_A_R, INPUT);
  pinMode(Encoder_output_B_R, INPUT);
  pinMode(Motor_A_L, OUTPUT);
  pinMode(Motor_B_L, OUTPUT);
  pinMode(Motor_A_R, OUTPUT);
  pinMode(Motor_B_R, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(Encoder_output_B_L), ISR_L, RISING);
  attachInterrupt(digitalPinToInterrupt(Encoder_output_B_R), ISR_R, RISING);
  
  delay(2000);

  allocator = rcl_get_default_allocator();

  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  RCCHECK(rclc_node_init_default(&node, "esp32_node", "", &support));

  RCCHECK(rclc_publisher_init_default(
    &motorfeedback_pub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int64MultiArray),
    "motor/feedback"));

  RCCHECK(rclc_subscription_init_default(
    &leftmotor_sub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "motor/left_cmd"));

  RCCHECK(rclc_subscription_init_default(
    &rightmotor_sub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "motor/right_cmd"));
  
  const unsigned int timer_timeout = 100;
  
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));
    
  RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
  RCCHECK(rclc_executor_add_subscription(&executor, &rightmotor_sub, &rightMsg, &rightsub_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &leftmotor_sub, &leftMsg, &leftsub_callback, ON_NEW_DATA));
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{  
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    static int64_t data_array[2] = {0}; 
    data_array[0] = encodePulse_L;
    data_array[1] = encodePulse_R;

    encoderMsg.data.data = data_array;
    encoderMsg.data.size = 2;
    encoderMsg.data.capacity = 2;
    RCSOFTCHECK(rcl_publish(&motorfeedback_pub, &encoderMsg, NULL));
  }
}

void ISR_L() {
  int b = digitalRead(Encoder_output_A_L);
  if (b > 0) {
    encodePulse_L--;
  } else {
    encodePulse_L++;
  }
}

void ISR_R() {
  int b = digitalRead(Encoder_output_A_R);
  if (b > 0) {
    encodePulse_R++;
  } else {
    encodePulse_R--;
  }
}

void loop() {
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(50)));
}
