//new
//usb1
#include <ros.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/Twist.h>
#include <PID_v1.h>

#define Encoder_output_A 2
#define Encoder_output_B 3
#define Motor_pinA 5
#define Motor_pinB 6

// linear = 0.14 angular = 1.5
int Count_pulses = 0;
int Count_pulses_prev = 0;
long currentMillis = 0;
long previousMillis = 0;
double rpm = 0;
double velocity = 0.0;
double setvelocity_r = 0.0;
double output = 0;
double Kp = 500;//1.5
double Ki = 0;
double Kd = 1;

void cmdVelCallback(const geometry_msgs::Twist& twist_msg);

ros::NodeHandle nh;
geometry_msgs::Twist cmd_vel_msg;
std_msgs::Int32 encoder_msg;
ros::Publisher encoder_pub_r("right_wheel_encoder_ticks", &encoder_msg);
ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub_r("cmd_vel", &cmdVelCallback);
PID pid(&velocity, &output, &setvelocity_r, Kp, Ki, Kd, DIRECT);

void setup() {
  nh.initNode();
  nh.advertise(encoder_pub_r);
  nh.subscribe(cmd_vel_sub_r);
  Serial.begin(57600);
  pinMode(Encoder_output_A, INPUT);
  pinMode(Encoder_output_B, INPUT);
  pinMode(Motor_pinA, OUTPUT);
  pinMode(Motor_pinB, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(Encoder_output_B), DC_Motor_Encoder, RISING);
  pid.SetSampleTime(10);
  pid.SetOutputLimits(-255, 255);
  pid.SetMode(AUTOMATIC);
}

void loop() {
  encoder_msg.data = Count_pulses;
  encoder_pub_r.publish(&encoder_msg);
  pid.Compute();
  runmotor();
  nh.spinOnce();
  delay(2);
  currentMillis = millis();
  if (currentMillis - previousMillis > 25) {
    previousMillis = currentMillis;
    int count = Count_pulses - Count_pulses_prev;
    rpm = (double)(count * 60.0 * 40.0) / 150.0;
    velocity = (PI * 0.07 * rpm) / 60.0;
    Count_pulses_prev = Count_pulses;
  }
  Serial.print(velocity);
  Serial.print(" ");
  Serial.print(output);
  Serial.print(" ");
  Serial.println(setvelocity_r);
}

void DC_Motor_Encoder() {
  int b = digitalRead(Encoder_output_A);
  if (b > 0) {
    Count_pulses++;
  } else {
    Count_pulses--;
  }
//  Serial.print("Result right: ");
//  Serial.println(Count_pulses);
}

void cmdVelCallback(const geometry_msgs::Twist& twist_msg)
{
  float linear_x = twist_msg.linear.x;
  float angular_z = twist_msg.angular.z;
//  linear_x = constrain(linear_x, -0.14, 0.14);
//  angular_z = constrain(angular_z, -1.5, 1.5);
  setvelocity_r = linear_x + (0.5 * angular_z);
//  setvelocity_r = constrain(setvelocity_r, -0.14, 0.14);

}

void runmotor(){
  if(output > 0){
    analogWrite(Motor_pinA, output);
    analogWrite(Motor_pinB, 0);
  }
  else{
    analogWrite(Motor_pinB, (output * -1));
    analogWrite(Motor_pinA, 0);
  }
}
