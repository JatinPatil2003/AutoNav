//old
//usb0
#include <PID_v1.h>
#include <Wire.h>
#define SLAVE_ADDRESS 0x08
#define wheel_radius 0.083
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
double setrpm = 0;
double output = 0;
double Kp = 0.6;//0.6
double Ki = 25;//25
double Kd = 0.01;//0.01

PID pid(&rpm, &output, &setrpm, Kp, Ki, Kd, DIRECT);

void setup() {
  pinMode(Encoder_output_A, INPUT);
  pinMode(Encoder_output_B, INPUT);
  pinMode(Motor_pinA, OUTPUT);
  pinMode(Motor_pinB, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(Encoder_output_B), DC_Motor_Encoder, RISING);
  pid.SetSampleTime(10);
  pid.SetOutputLimits(-255, 255);
  pid.SetMode(AUTOMATIC);
  Wire.begin(SLAVE_ADDRESS);
  Wire.onReceive(receiveData);
  Wire.onRequest(sendData);
  Serial.begin(115200);
}

void loop() {
  pid.Compute();
  runmotor();

  currentMillis = millis();
  if (currentMillis - previousMillis > 25) {
    previousMillis = currentMillis;
    int count = Count_pulses - Count_pulses_prev;
    rpm = (double)(count * 60.0 * 40.0) / 150.0;
    // velocity = (PI * 0.07 * rpm) / 60.0;
    Count_pulses_prev = Count_pulses;
    Serial.println(rpm);
  }
}

void DC_Motor_Encoder() {
  int b = digitalRead(Encoder_output_A);
  if (b > 0) {
    Count_pulses--;
  } else {
    Count_pulses++;
  }
//  Serial.println(Count_pulses);
}

void intToBytes(int value) {
  String strValue = String(value);
  int i;
  int len = strValue.length();
  byte bytearr[len+1];
  bytearr[0] = len;
  for(i=0;i<strValue.length(); i++){
    bytearr[i+1] = int(strValue[i]);
    Wire.write(bytearr[i]);
  }
  Wire.write(bytearr[i]);
}

void runmotor(){
  if(output > 0){
    analogWrite(Motor_pinA, output);
    analogWrite(Motor_pinB, 0);
  }
  else if(output < 0){
    analogWrite(Motor_pinB, (output * -1));
    analogWrite(Motor_pinA, 0);
  }
  else{
    analogWrite(Motor_pinB, 0);
    analogWrite(Motor_pinA, 0);
  }
}

void sendData(){
  int value = Count_pulses;
  intToBytes(value);
//  Serial.println("data sent");
}

void receiveData(int byteCount){
  String datar="";
  while (Wire.available()) {
    char c = Wire.read();
    if(c){
      datar += c;
    }
  }
  if(datar != ""){
    float rpm = (datar.toFloat() / (3.14159 * wheel_radius)) * 60;
//    Serial.print("Received: ");
//    Serial.print(datar.toFloat());
//    Serial.print(" RPM: ");
//    Serial.println(rpm);
    setrpm = rpm;
  }
  
}
