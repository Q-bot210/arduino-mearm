#include <Servo.h>
#include <SoftwareSerial.h>

Servo servo_6, servo_7, servo_8, servo_9;
SoftwareSerial remoteSerial(10, 11);  // RX, TX

// ================== 录制参数 ==================
const int SAMPLE_MS = 60;      // 采样周期(ms)，回放时也用这个间隔
const int MAX_SAMPLES = 300;   // 最大样本数，60ms × 300 ≈ 18s

byte recData[MAX_SAMPLES][4];  // 每帧记录 4 个舵机角度
int  recCount = 0;             // 已记录帧数
bool recording = false;        // 是否正在录制
unsigned long lastSample = 0;

void setup() {
  servo_6.attach(6);
  servo_7.attach(7);
  servo_8.attach(8);
  servo_9.attach(9);
  servo_6.write(180);  // 夹爪
  servo_7.write(90);   // 右臂
  servo_8.write(90);  // 底座
  servo_9.write(90);   // 左臂

  Serial.begin(9600);
  remoteSerial.begin(9600);
}

// ================== 摇杆控制（A0~A3） ==================
void joyControl() {
  const int   pins[4] = {A0, A1, A2, A3};
  Servo*      svs[4]  = {&servo_6, &servo_7, &servo_8, &servo_9};
  const int   lo[4]   = {90,  60,   5,   0};
  const int   hi[4]   = {180, 180, 175, 130};
  const bool  inv[4]  = {true, false, false, true};

  for (int i = 0; i < 4; i++) {
    int v = analogRead(pins[i]);
    int a = svs[i]->read();
    if (v < 400) {
      if (inv[i]) { if (a > lo[i]) svs[i]->write(a - 1); }
      else        { if (a < hi[i]) svs[i]->write(a + 1); }
    } else if (v > 600) {
      if (inv[i]) { if (a < hi[i]) svs[i]->write(a + 1); }
      else        { if (a > lo[i]) svs[i]->write(a - 1); }
    }
  }
}

// ================== 播放录制的动作 ==================
void playRecord() {
  if (recCount == 0) { Serial.println("无录制数据"); return; }
  Serial.print("3-播放，共 "); Serial.print(recCount); Serial.println(" 帧");
  for (int i = 0; i < recCount; i++) {
    servo_6.write(recData[i][0]);
    servo_7.write(recData[i][1]);
    servo_8.write(recData[i][2]);
    servo_9.write(recData[i][3]);
    delay(SAMPLE_MS);   // 与录制采样周期一致，回放速度相同
  }
  Serial.println("播放完成");
}

void loop() {
  if (remoteSerial.available()) {
    char cmd = remoteSerial.read();

    if (cmd == '1') {
      Serial.println("1-循环");
      // 待补充
    }
    else if (cmd == '2') {          // 录制：按一下开始，再按一下结束
      if (!recording) {
        recCount = 0;
        recording = true;
        lastSample = millis();
        Serial.println("2-开始录制（摇杆操控机械臂，再次按下结束）");
      } else {
        recording = false;
        Serial.print("2-结束录制，样本数: ");
        Serial.println(recCount);
      }
    }
    else if (cmd == '3') {
      playRecord();
    }
    else if (cmd == '4') {
      Serial.println("4-回中");
      servo_6.write(180);
      servo_7.write(90);
      servo_8.write(90);
      servo_9.write(90);
    }
    else if (cmd == '5') {
      Serial.println("5-五点示教");
      //待补充
    }
  }

  // 摇杆实时操控（录制时人工操控，非录制时也可手动操作）
  joyControl();

  // 录制中定时采样
  if (recording && millis() - lastSample >= SAMPLE_MS) {
    lastSample = millis();
    if (recCount < MAX_SAMPLES) {
      recData[recCount][0] = servo_6.read();
      recData[recCount][1] = servo_7.read();
      recData[recCount][2] = servo_8.read();
      recData[recCount][3] = servo_9.read();
      recCount++;
    }
  }

  delay(10);  // 让摇杆走位平滑
}