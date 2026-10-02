#include<Servo.h>

Servo myservo;

void setup() {
  // put your setup code here, to run once:
    myservo.attach(PA0);
    myservo.write(90);
}

void loop() {
  // put your main code here, to run repeatedly:

}
