//基本功能
#include <Servo.h>

Servo servo_6;
Servo servo_7;
Servo servo_8;
Servo servo_9;

int speedDelay = 15;

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

  Serial.begin(9600);
}

//xyz控制
void parseCommand(String cmd) {
  int angles[3] = {-1, -1, -1};
  int idx = 0;
  //x->底座，y->右臂，z->左臂
  for (int i = 0; i < cmd.length(); i++) {
    char c = cmd.charAt(i);
    if (c == 'x') {
      idx = 0;
    } else if (c == 'y') {
      idx = 1;
    } else if (c == 'z') {
      idx = 2;
    } else if (c >= '0' && c <= '9') {
      if (angles[idx] < 0) angles[idx] = 0;
      angles[idx] = angles[idx] * 10 + (c - '0');
    }
  }

  if (angles[0] >= 0) servo_8.write(angles[0]); 
  if (angles[1] >= 0) servo_7.write(angles[1]); 
  if (angles[2] >= 0) servo_9.write(angles[2]); 
}
  
void loop() {
  // put your main code here, to run repeatedly:
  //串口指令
  if (Serial.available() > 0) {
    String line = Serial.readStringUntil('\n');
    line.trim();

    if (line == "O") {
      servo_6.write(60);
    } 
    else if (line == "S") {
      servo_6.write(180);
    } 
    else if (line == "H") {
      speedDelay = 5;
    } 
    else if (line == "L") {
      speedDelay = 15;
    } 
    else if (line.indexOf('x') >= 0 || line.indexOf('y') >= 0 || line.indexOf('z') >= 0) {
      parseCommand(line);
    }
  }

  //摇杆控制
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

  delay(speedDelay);  
}