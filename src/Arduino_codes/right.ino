//old
//usb0
#include <PID_v1.h>
#include <wire.h>

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
double Kp = 1;
double Ki = 0;
double Kd = 0;

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
  Serial.begin(115200);
  Wire.begin(0x09);
  Wire.onRequest(sendData);
  Wire.onReceive(receiveData);
}

void loop() {
  pid.Compute();
//   runmotor();

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
    int data = Count_pulses;
    Wire.write((byte*)&data, sizeof(data));
    Serial.print("Sent data : ");
    Serial.println(data);
}

void receiveData(){
    while (Wire.available()){
        double data = Wire.read();
        setrpm = data;
        Serial.print("Received Data : ");
        Serial.println(setrpm);
    }
}