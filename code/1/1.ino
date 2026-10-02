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

}
