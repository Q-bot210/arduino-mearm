#include <Servo.h>

Servo servo_6, servo_7, servo_8, servo_9; // 6夹爪 7右臂 8底座 9左臂

const int GRIP_CLOSE = 180;
const int GRIP_OPEN  = 90;

const int BASE_FRONT = 100;
const int BASE_LEFT  = 160;
const int BASE_RIGHT = 30;

const int R_UP = 90;
const int L_UP = 90;

const int R_PICK = 140;
const int L_PICK = 0;

const int R_C = 170;
const int L_C = 65;

int cur6, cur7, cur8, cur9;

void setup() {
  servo_6.attach(6);
  servo_7.attach(7);
  servo_8.attach(8);
  servo_9.attach(9);
  Serial.begin(9600);

  cur6 = GRIP_CLOSE;
  cur7 = R_UP;
  cur8 = BASE_FRONT;
  cur9 = L_UP;

  servo_6.write(cur6);
  servo_7.write(cur7);
  servo_8.write(cur8);
  servo_9.write(cur9);
  delay(1000);
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'A' || c == 'B' || c == 'C') runCmd(c);
  }
}

void moveTo(int t6, int t7, int t8, int t9) {
  int d = 0;
  d = max(d, abs(t6 - cur6));
  d = max(d, abs(t7 - cur7));
  d = max(d, abs(t8 - cur8));
  d = max(d, abs(t9 - cur9));

  for (int i = 0; i < d; i++) {
    if (cur6 < t6) cur6++; else if (cur6 > t6) cur6--;
    if (cur7 < t7) cur7++; else if (cur7 > t7) cur7--;
    if (cur8 < t8) cur8++; else if (cur8 > t8) cur8--;
    if (cur9 < t9) cur9++; else if (cur9 > t9) cur9 -= 3;

    servo_6.write(cur6);
    servo_7.write(cur7);
    servo_8.write(cur8);
    servo_9.write(cur9);
    delay(15);
  }

  cur6 = t6; cur7 = t7; cur8 = t8; cur9 = t9;
  servo_6.write(cur6);
  servo_7.write(cur7);
  servo_8.write(cur8);
  servo_9.write(cur9);
}

void runCmd(char cmd) {
  int targetBase = BASE_FRONT;
  int targetR = R_PICK;
  int targetL = L_PICK;

  if (cmd == 'A') {
    targetBase = BASE_LEFT;
  } else if (cmd == 'B') {
    targetBase = BASE_RIGHT;
  } else if (cmd == 'C') {
    targetR = R_C;
    targetL = L_C;
  }

  moveTo(GRIP_OPEN, R_UP, BASE_FRONT, L_UP);   // 到夹取位置上方，张开夹爪
  delay(200);
  moveTo(GRIP_OPEN, R_PICK, BASE_FRONT, L_PICK); // 下降到夹取位置
  delay(200);
  moveTo(GRIP_CLOSE, R_PICK, BASE_FRONT, L_PICK); // 夹住
  delay(300);
  moveTo(GRIP_CLOSE, R_UP, BASE_FRONT, L_UP);   // 抬起
  delay(200);
  moveTo(GRIP_CLOSE, R_UP, targetBase, L_UP);   // 转到目标底座上方
  delay(200);
  moveTo(GRIP_CLOSE, targetR, targetBase, targetL); // 移到目标放置位置
  delay(200);
  moveTo(GRIP_OPEN, targetR, targetBase, targetL);  // 张开夹爪，放下
  delay(300);
  moveTo(GRIP_OPEN, R_UP, targetBase, L_UP);    // 抬起
  delay(200);
  moveTo(GRIP_CLOSE, R_UP, BASE_FRONT, L_UP);   // 复原
  delay(300);
}