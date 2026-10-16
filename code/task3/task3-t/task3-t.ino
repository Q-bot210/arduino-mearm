#include <SoftwareSerial.h>
SoftwareSerial armSerial(10, 11);  // RX, TX

void setup() {
  Serial.begin(9600);
  armSerial.begin(9600);

  pinMode(2, INPUT_PULLUP);  // 按键1 循环
  pinMode(3, INPUT_PULLUP);  // 按键2 录制
  pinMode(4, INPUT_PULLUP);  // 按键3 播放
  pinMode(5, INPUT_PULLUP);  // 按键4 回中
  pinMode(6, INPUT_PULLUP);  // 按键5 五点示教
}

void loop() {
  if (digitalRead(2) == LOW) { armSerial.write('1'); delay(300); }
  if (digitalRead(3) == LOW) { armSerial.write('2'); delay(300); }
  if (digitalRead(4) == LOW) { armSerial.write('3'); delay(300); }
  if (digitalRead(5) == LOW) { armSerial.write('4'); delay(300); }
  if (digitalRead(6) == LOW) { armSerial.write('5'); delay(300); }
}