#include <Servo.h>
Servo servo_6;
Servo servo_7;
Servo servo_8;
Servo servo_9;

#include <SoftwareSerial.h>
SoftwareSerial remoteSerial(10, 11);  // RX, TX

void setup() {
  servo_6.attach(6);
  servo_7.attach(7);
  servo_8.attach(8);
  servo_9.attach(9);
  servo_6.write(180);//夹爪
  servo_7.write(80);//右臂
  servo_8.write(100);//底座
  servo_9.write(90);//左臂
  
  Serial.begin(9600);
  remoteSerial.begin(9600);
}

void loop() {
  if (remoteSerial.available()) {
    char cmd = remoteSerial.read();

    if (cmd == 'J') {
      Serial.println("1-循环");
      servo_8.write(60);
    }
    else if (cmd == 'L') {
      Serial.println("2-录制");
    }
    else if (cmd == 'B') {
      Serial.println("3-播放");
    }
    else if (cmd == 'H') {
      Serial.println("4-回中");
      servo_6.write(180);
      servo_7.write(80);
      servo_8.write(100);
      servo_9.write(90);
    }
  }
}