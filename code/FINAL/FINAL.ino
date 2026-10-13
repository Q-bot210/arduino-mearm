#include <Servo.h>
#include <SoftwareSerial.h>

SoftwareSerial remoteSerial(10, 11);  // RX, TX（和 task3-t 一致）

Servo servo_6, servo_7, servo_8, servo_9;

// ================== 机械尺寸参数 ==================
const float L1 = 80.0, L2 = 80.0, H0 = 30.0;
const float BASE_FORWARD = 90.0;
const float ELBOW_HORIZONTAL = 90.0;

struct Point { float x, y, z; };

// ================== 按键引脚 ==================
const int PAUSE_BTN_PIN = 5;   // D5：暂停/继续
const int CANCEL_BTN_PIN = 4;  // D4：取消

// ================== 全局状态 ==================
Point currentPos;
bool isPaused = false, isCanceled = false, inReturnHome = false;

// ================== 夹爪角度 ==================
const int GRIP_OPEN = 90, GRIP_CLOSE = 180;

// ================== 摇杆速度 ==================
int speedDelay = 10;

// ================== 抓取/放置坐标 ==================
Point pickPos = {60.0,   0.0,  5.0};
Point placeA  = {30.0,  52.0,  5.0};
Point placeB  = {20.0, -56.0,  5.0};
Point placeC  = {151.0,  0.0, 10.0};
const float LIFT_Z = 80.0;

Point home = {80.0, 0.0, 110.0};

// ================== 录制参数 ==================
const int SAMPLE_MS  = 80;    // 80ms × 150 = 12s
const int MAX_SAMPLES = 150;  // 150×4 = 600B
byte recData[MAX_SAMPLES][4];
int  recCount = 0;
bool recording = false;
unsigned long lastSample = 0;

// ================== 循环任务索引 ==================
int cycleIdx = 0;

// ================== 逆运动学 ==================
bool inverseKinematics(const Point &p, float &baseAng, float &shoulderAng, float &elbowAng) {
  float r = sqrt(p.x * p.x + p.y * p.y);
  float zRel = p.z - H0;
  float d = sqrt(r * r + zRel * zRel);
  if (d > L1 + L2 || d < fabs(L1 - L2) || d < 1e-6) return false;

  float gamma = atan2(zRel, r);
  float cosDelta = (L1 * L1 + d * d - L2 * L2) / (2.0 * L1 * d);
  if (cosDelta > 1.0) cosDelta = 1.0;
  if (cosDelta < -1.0) cosDelta = -1.0;

  float delta = acos(cosDelta);
  float alpha = gamma + delta;
  float beta  = atan2(zRel - L1 * sin(alpha), r - L1 * cos(alpha));

  baseAng     = BASE_FORWARD + degrees(atan2(p.y, p.x));
  shoulderAng = 180.0 - degrees(alpha);
  elbowAng    = ELBOW_HORIZONTAL + degrees(beta);

  return baseAng >= 0 && baseAng <= 180 &&
         shoulderAng >= 0 && shoulderAng <= 180 &&
         elbowAng >= 0 && elbowAng <= 180;
}

void writeArm(float b, float s, float e) {
  servo_8.write((int)(b + 0.5));
  servo_7.write((int)(s + 0.5));
  servo_9.write((int)(e + 0.5));
}

void grip(int a) { servo_6.write(a); delay(300); }

// ================== 按键检测 ==================
void checkButtons() {
  if (inReturnHome) return;
  if (digitalRead(PAUSE_BTN_PIN) == LOW) {
    delay(20);
    if (digitalRead(PAUSE_BTN_PIN) == LOW) {
      isPaused = !isPaused;
      Serial.println(isPaused ? F("暂停") : F("继续"));
      while (digitalRead(PAUSE_BTN_PIN) == LOW);
    }
  }
  if (digitalRead(CANCEL_BTN_PIN) == LOW) {
    delay(20);
    if (digitalRead(CANCEL_BTN_PIN) == LOW) {
      isCanceled = true;
      Serial.println(F("取消"));
      while (digitalRead(CANCEL_BTN_PIN) == LOW);
    }
  }
}

// ================== 直线插补 ==================
void moveLine(Point a, Point b, int steps, int stepDelayMs) {
  for (int i = 0; i <= steps; i++) {
    checkButtons();
    if (isCanceled) return;
    while (isPaused && !isCanceled) { checkButtons(); delay(10); }
    if (isCanceled) return;

    float t = (float)i / (float)steps;
    Point p = { a.x + (b.x - a.x) * t,
                a.y + (b.y - a.y) * t,
                a.z + (b.z - a.z) * t };

    float bA, sA, eA;
    if (!inverseKinematics(p, bA, sA, eA)) {
      Serial.print(F("不可达:"));
      Serial.print(p.x); Serial.print(',');
      Serial.print(p.y); Serial.print(',');
      Serial.println(p.z);
      return;
    }
    writeArm(bA, sA, eA);
    currentPos = p;
    delay(stepDelayMs * speedDelay / 10);
  }
  currentPos = b;
}

// ================== 回 home ==================
void returnToHome() {
  Serial.println(F("取消-回待机"));
  inReturnHome = true;
  isCanceled = false;
  isPaused = false;
  moveLine(currentPos, home, 60, 20);
  currentPos = home;
  inReturnHome = false;
}

void resetToHome() {
  inReturnHome = true;
  moveLine(currentPos, home, 60, 20);
  currentPos = home;
  grip(GRIP_CLOSE);
  inReturnHome = false;
}

bool canceled() {
  if (!isCanceled) return false;
  returnToHome();
  return true;
}

// ================== 抓取-放置 ==================
void pickAndPlace(Point pick, Point place) {
  Point pickUp = pick;   pickUp.z  = LIFT_Z;
  Point placeUp = place; placeUp.z = LIFT_Z;

  grip(GRIP_OPEN);
  moveLine(currentPos, pickUp, 50, 20);   if (canceled()) return;
  moveLine(pickUp, pick, 30, 20);         if (canceled()) return;
  grip(GRIP_CLOSE);
  moveLine(pick, pickUp, 30, 20);         if (canceled()) return;
  moveLine(pickUp, placeUp, 60, 20);      if (canceled()) return;
  moveLine(placeUp, place, 30, 20);       if (canceled()) return;
  grip(GRIP_OPEN);
  moveLine(place, placeUp, 30, 20);       if (canceled()) return;
  resetToHome();
}

void pickTo(Point place, const char *name) {
  isCanceled = false; isPaused = false;
  Serial.print(F("抓取")); Serial.println(name);
  pickAndPlace(pickPos, place);
  Serial.print(name); Serial.println(F("完成"));
}

// ================== 1-循环：A -> B -> C -> A ... ==================
void cycleNext() {
  Point       places[3] = {placeA, placeB, placeC};
  const char* names[3]  = {"A", "B", "C"};
  Serial.print(F("循环-"));
  Serial.println(names[cycleIdx]);
  pickTo(places[cycleIdx], names[cycleIdx]);
  cycleIdx = (cycleIdx + 1) % 3;
}

// ================== 绘图 ==================
const float WORK_Z = 20.0;
Point A = {80.0,  80.0, WORK_Z};
Point B = {80.0, -80.0, WORK_Z};

void drawLine() {
  isCanceled = false; isPaused = false;
  Serial.println(F("画直线"));
  moveLine(currentPos, home, 60, 20);   if (canceled()) return;
  moveLine(home, A, 40, 20);            if (canceled()) return;
  delay(300);
  moveLine(A, B, 80, 20);               if (canceled()) return;
  resetToHome();
  Serial.println(F("直线OK"));
}

void drawV() {
  isCanceled = false; isPaused = false;
  Serial.println(F("画V"));
  Point V_A   = {70.0,  30.0, WORK_Z};
  Point V_MID = {150.0,  0.0, WORK_Z};
  Point V_B   = {70.0, -30.0, WORK_Z};

  moveLine(currentPos, home, 60, 20);   if (canceled()) return;
  moveLine(home, V_A, 40, 20);          if (canceled()) return;
  delay(300);
  moveLine(V_A, V_MID, 50, 20);         if (canceled()) return;
  moveLine(V_MID, V_B, 50, 20);         if (canceled()) return;
  resetToHome();
  Serial.println(F("V OK"));
}

// ================== x/y/z 角度指令（char*） ==================
void parseCommand(const char *cmd) {
  int angles[3] = {-1, -1, -1};
  int idx = 0;
  for (int i = 0; cmd[i]; i++) {
    char c = cmd[i];
    if      (c == 'x') idx = 0;
    else if (c == 'y') idx = 1;
    else if (c == 'z') idx = 2;
    else if (c >= '0' && c <= '9') {
      if (angles[idx] < 0) angles[idx] = 0;
      angles[idx] = angles[idx] * 10 + (c - '0');
    }
  }
  if (angles[0] >= 0) servo_8.write(angles[0]);
  if (angles[1] >= 0) servo_7.write(angles[1]);
  if (angles[2] >= 0) servo_9.write(angles[2]);
}

// ================== 摇杆通用读取 ==================
void joyControl(int pin, Servo &s, int lo, int hi, bool invert) {
  int v = analogRead(pin);
  int a = s.read();
  if (v < 400) {
    if (invert) { if (a > lo) s.write(a - 1); }
    else        { if (a < hi) s.write(a + 1); }
  } else if (v > 600) {
    if (invert) { if (a < hi) s.write(a + 1); }
    else        { if (a > lo) s.write(a - 1); }
  }
}

// ================== 录制 / 播放 / 回中 ==================
void toggleRecord() {
  if (!recording) {
    recCount = 0;
    recording = true;
    lastSample = millis();
    Serial.println(F("录制开始"));
  } else {
    recording = false;
    Serial.print(F("录制结束 帧:"));
    Serial.println(recCount);
  }
}

void playRecord() {
  if (recCount == 0) { Serial.println(F("无数据")); return; }
  Serial.print(F("播放 ")); Serial.print(recCount); Serial.println(F(" 帧"));
  for (int i = 0; i < recCount; i++) {
    servo_6.write(recData[i][0]);
    servo_7.write(recData[i][1]);
    servo_8.write(recData[i][2]);
    servo_9.write(recData[i][3]);
    delay(SAMPLE_MS);
  }
  Serial.println(F("播放完成"));
}

void goHome() {
  Serial.println(F("回中"));
  servo_6.write(GRIP_CLOSE);
  servo_7.write(90);
  servo_8.write(BASE_FORWARD);
  servo_9.write(90);
  currentPos = home;
}

// ================== 命令分发（char*） ==================
void handleLine(char *line) {
  // 手动 trim
  while (*line == ' ' || *line == '\r' || *line == '\t') line++;
  int len = strlen(line);
  while (len > 0 && (line[len-1] == ' ' || line[len-1] == '\r' || line[len-1] == '\t'))
    line[--len] = '\0';
  if (len == 0) return;

  // 单字符数字命令
  if (len == 1) {
    char c = line[0];
    if      (c == '1') { cycleNext();    return; }
    else if (c == '2') { toggleRecord(); return; }
    else if (c == '3') { playRecord();   return; }
    else if (c == '4') { goHome();       return; }
    else if (c == '5') { Serial.println(F("5-示教")); return; }
  }

  // 整行命令
  if      (strcmp(line, "X") == 0) drawLine();
  else if (strcmp(line, "V") == 0) drawV();
  else if (strcmp(line, "A") == 0) pickTo(placeA, "A");
  else if (strcmp(line, "B") == 0) pickTo(placeB, "B");
  else if (strcmp(line, "C") == 0) pickTo(placeC, "C");
  else if (strcmp(line, "O") == 0) servo_6.write(90);
  else if (strcmp(line, "S") == 0) servo_6.write(180);
  else if (strcmp(line, "H") == 0) speedDelay = 5;
  else if (strcmp(line, "L") == 0) speedDelay = 15;
  else if (strchr(line, 'x') || strchr(line, 'y') || strchr(line, 'z'))
    parseCommand(line);
}

// ================== 初始化 ==================
void setup() {
  Serial.begin(9600);
  remoteSerial.begin(9600);

  pinMode(PAUSE_BTN_PIN, INPUT_PULLUP);
  pinMode(CANCEL_BTN_PIN, INPUT_PULLUP);

  servo_6.attach(6);
  servo_7.attach(7);
  servo_8.attach(8);
  servo_9.attach(9);

  servo_6.write(GRIP_CLOSE);
  servo_7.write(90);
  servo_8.write(BASE_FORWARD);
  servo_9.write(90);

  delay(1000);
  currentPos = home;
  isPaused = false;
  isCanceled = false;
  inReturnHome = false;
  cycleIdx = 0;

  Serial.println(F("就绪: X直线 V字 A/B/C抓取"));
  Serial.println(F("O/S爪 H快 L慢 x/y/z角度"));
  Serial.println(F("板键: 1循环 2录 3放 4回中 5示教"));
  Serial.println(F("D5暂停 D4取消"));
}

// ================== 主循环 ==================
void loop() {
  static char cmdBuf[24];

  // USB 串口：整行命令
  if (Serial.available() > 0) {
    int idx = 0;
    while (Serial.available() && idx < 23) {
      char c = Serial.read();
      if (c == '\n') break;
      cmdBuf[idx++] = c;
    }
    cmdBuf[idx] = '\0';
    handleLine(cmdBuf);
  }

  // 软件串口：按键板单字符命令
  if (remoteSerial.available() > 0) {
    cmdBuf[0] = (char)remoteSerial.read();
    cmdBuf[1] = '\0';
    handleLine(cmdBuf);
  }

  // 摇杆控制
  joyControl(A0, servo_6,  90, 180, true);   // 夹爪
  joyControl(A1, servo_7,  60, 180, false);  // 肩
  joyControl(A2, servo_8,   5, 175, false);  // 底座
  joyControl(A3, servo_9,   0, 130, true);   // 肘

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

  delay(speedDelay);
}