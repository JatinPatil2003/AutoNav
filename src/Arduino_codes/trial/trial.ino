#include <Wire.h>
#define SLAVE_ADDRESS 0x08
int a = 12345;
byte bytearray[16];
char myCharArray[16];

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


void setup() {
  Wire.begin(SLAVE_ADDRESS);
  Wire.onReceive(receiveData);
  Wire.onRequest(sendData);
  Serial.begin(115200);
}

void loop() {
  delay(100);
//  double largeDecimal = 1234.56789;
//  String strValue = String(largeDecimal);
//  Serial.print(largeDecimal);
//  Serial.print(" ");
//  Serial.print(strValue);
//  Serial.print(" ");
//  Serial.println(strValue.c_str());
}

void receiveData(int byteCount) {
  String data="";
  while (Wire.available()) {
    char c = Wire.read();
    if(c){
//      Serial.print("Received: ");
//      Serial.println(c);
      data += c;
    }
  }
  Serial.print("Received: ");
  Serial.println(float(data));
}

void sendData() {
  int value = 25689;
  intToBytes(value);
}
