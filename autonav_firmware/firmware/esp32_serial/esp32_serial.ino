
#include <AccelStepper.h>
#include <AsyncTimer.h>

#define dirPinLeft 4
#define stepPinLeft 2
#define enableLeft 5

#define dirPinRight 18
#define stepPinRight 19
#define enableRight 23

AccelStepper stepperLeft(AccelStepper::DRIVER, stepPinLeft, dirPinLeft);
AccelStepper stepperRight(AccelStepper::DRIVER, stepPinRight, dirPinRight);

AsyncTimer t;

void setup() {
  
  pinMode(enableLeft, OUTPUT);
  pinMode(enableRight, OUTPUT);

  digitalWrite(enableLeft, LOW);
  digitalWrite(enableRight, LOW);

  stepperLeft.setMaxSpeed(7000);
  stepperRight.setMaxSpeed(7000);

  stepperLeft.setSpeed(1600); // Start moving in reverse
  stepperRight.setSpeed(-1600);

  Serial.begin(115200);
  
  delay(2000);

  
//  t.setInterval([]() { ; }, 150);
}

void loop() {
  if (Serial.available() > 0) {
    String receivedData = Serial.readStringUntil('\n');

    int commaIndex = receivedData.indexOf(',');

    String left = receivedData.substring(0, commaIndex);
    String right = receivedData.substring(commaIndex + 1);

    float var1 = left.toFloat();
    float var2 = right.toFloat();

    stepperLeft.setSpeed(var1); // Start moving in reverse
    stepperRight.setSpeed(var2);
  }
  
  stepperLeft.runSpeed();
  stepperRight.runSpeed();
  
  String dataToSend = String(stepperLeft.currentPosition(), 2) + "," + String(stepperRight.currentPosition(), 2);
  Serial.println(dataToSend);
}
