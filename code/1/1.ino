//基础控制
#include <Servo.h>

Servo servo_6;
Servo servo_7;
Servo servo_8;
Servo servo_9;
void setup() {
  // put your setup code here, to run once:
  servo_6.attach(6);
  servo_7.attach(7);
  servo_8.attach(8);
  servo_9.attach(9);
 
  servo_6.write(180);//夹爪 
  servo_7.write(80);//右臂 
  servo_8.write(100);//底座
  servo_9.write(90);//左臂
  
}

void loop() {
  // put your main code here, to run repeatedly:
  if (analogRead(A0) < 400) {
    if (servo_6.read() > 0) {
      servo_6.write((servo_6.read() - 1));
    }
  } 
  else if (analogRead(A0) > 600) {
    if (servo_6.read() < 180) {
      servo_6.write((servo_6.read() + 1));
    }
  }

  if (analogRead(A1) < 400) {
    if (servo_7.read() > 30) {
      servo_7.write((servo_7.read() - 1));
    }
  } 
  else if (analogRead(A1) > 600) {
    if (servo_7.read() < 180) {
      servo_7.write((servo_7.read() + 1));
    }
  }

  if (analogRead(A2) < 400) {
    if (servo_8.read() > 0) {
      servo_8.write((servo_8.read() - 1));
    }
  } 
  else if (analogRead(A2) > 600) {
    if (servo_8.read() < 180) {
      servo_8.write((servo_8.read() + 1));
    }
  }

  if (analogRead(A3) < 400) {
    if (servo_9.read() > 0) {
      servo_9.write((servo_9.read() - 1));
    }
  } 
  else if (analogRead(A3) > 600) {
    if (servo_9.read() < 130) {
      servo_9.write((servo_9.read() + 1));
    }
  }

  delay(10);
}
