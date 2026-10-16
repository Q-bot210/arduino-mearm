#include <Servo.h>
#include <SoftwareSerial.h>

SoftwareSerial remoteSerial(10, 11);
Servo servo_6, servo_7, servo_8, servo_9;

// ================== 机械尺寸与舵机基准参数 ==================
const float L1 = 80, L2 = 80, H0 = 35, ELBOW_DROOP = 8;
const float BASE_FORWARD = 100, SHOULDER_VERTICAL = 100, ELBOW_HORIZONTAL = 100;

struct Point { float x, y, z; };

// ================== 按键与全局状态 ==================
const int PAUSE_BTN_PIN = 5, CANCEL_BTN_PIN = 4;
Point currentPos;
bool isPaused = false, isCanceled = false, inReturnHome = false;

// ================== 夹爪与速度 ==================
const int GRIP_OPEN = 90, GRIP_CLOSE = 180;
int speedDelay = 10, joyInterval = 10;
unsigned long lastJoyMs = 0;

// ================== 抓取/放置坐标 ==================
Point pickPos = {60, 0, 5};
Point placeA = {30, 52, 5};
Point placeB = {20, -56, 5};
Point placeC = {151, 0, 10};
const float LIFT_Z = 80;
Point home = {80, 0, 115};

// ================== 录制参数 ==================
const int SAMPLE_MS = 80, MAX_SAMPLES = 150;
byte recData[MAX_SAMPLES][4];
int recCount = 0;
bool recording = false;
unsigned long lastSample = 0;

// ================== 循环与示教 ==================
int cycleIdx = 0;
const int TEACH_N = 5;
Point teachPts[TEACH_N];
int teachCount = 0;
bool teaching = false;
const float STEP_MM = 1.0;

// ================== 绘图工作平面 ==================
const float WORK_Z = 30.0;
Point A = {40, 0, WORK_Z}, B = {120, 0, WORK_Z};

// ================== 逆运动学：坐标 -> 三个臂舵机角度 ==================
bool inverseKinematics(const Point &p, float &baseAng, float &shoulderAng, float &elbowAng) {
  float r = sqrt(p.x * p.x + p.y * p.y);
  float zRel = p.z - H0;
  float d = sqrt(r * r + zRel * zRel);
  if (d > L1 + L2 || d < fabs(L1 - L2) || d < 1e-6) return false;

  float gamma = atan2(zRel, r);
  float cosDelta = (L1 * L1 + d * d - L2 * L2) / (2.0 * L1 * d);
  cosDelta = constrain(cosDelta, -1.0, 1.0);

  float delta = acos(cosDelta);
  float alpha = gamma + delta;
  float beta = atan2(zRel - L1 * sin(alpha), r - L1 * cos(alpha));

  baseAng = BASE_FORWARD + degrees(atan2(p.y, p.x));
  shoulderAng = SHOULDER_VERTICAL + (90.0 - degrees(alpha));
  elbowAng = ELBOW_HORIZONTAL + degrees(beta) + ELBOW_DROOP;

  return baseAng >= 0 && baseAng <= 180 &&
         shoulderAng >= 0 && shoulderAng <= 180 &&
         elbowAng >= 0 && elbowAng <= 180;
}

// ================== 舵机输出 ==================
void writeArm(float b, float s, float e) {
  servo_8.write((int)(b + 0.5));
  servo_7.write((int)(s + 0.5));
  servo_9.write((int)(e + 0.5));
}

void grip(int a) { servo_6.write(a); delay(300); }

// ================== 按键：暂停 / 取消 ==================
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

// ================== 直线插补：两点之间按 STEP_MM 细分移动 ==================
void moveLine(Point a, Point b, int stepDelayMs) {
  float dx = b.x - a.x, dy = b.y - a.y, dz = b.z - a.z;
  int steps = (int)(sqrt(dx * dx + dy * dy + dz * dz) / STEP_MM + 0.5);
  if (steps < 1) steps = 1;

  for (int i = 0; i <= steps; i++) {
    checkButtons();
    while (isPaused && !isCanceled) { checkButtons(); delay(10); }
    if (isCanceled) return;

    float t = (float)i / (float)steps;
    Point p = {a.x + dx * t, a.y + dy * t, a.z + dz * t};

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

// ================== 回待机 / 取消处理 ==================
void returnToHome() {
  Serial.println(F("取消-回待机"));
  inReturnHome = true;
  isCanceled = false;
  isPaused = false;
  moveLine(currentPos, home, 20);
  currentPos = home;
  inReturnHome = false;
}

void resetToHome() {
  inReturnHome = true;
  moveLine(currentPos, home, 20);
  currentPos = home;
  grip(GRIP_CLOSE);
  inReturnHome = false;
}

bool canceled() {
  if (!isCanceled) return false;
  returnToHome();
  return true;
}

// ================== 抓取-放置流程 ==================
void pickAndPlace(Point pick, Point place) {
  Point pickUp = pick, placeUp = place;
  pickUp.z = LIFT_Z;
  placeUp.z = LIFT_Z;

  grip(GRIP_OPEN);
  moveLine(currentPos, pickUp, 20); if (canceled()) return;
  moveLine(pickUp, pick, 20);       if (canceled()) return;
  grip(GRIP_CLOSE);
  moveLine(pick, pickUp, 20);       if (canceled()) return;
  moveLine(pickUp, placeUp, 20);    if (canceled()) return;
  moveLine(placeUp, place, 20);     if (canceled()) return;
  grip(GRIP_OPEN);
  moveLine(place, placeUp, 20);     if (canceled()) return;
  resetToHome();
}

void pickTo(Point place, const char *name) {
  isCanceled = false;
  isPaused = false;
  Serial.print(F("抓取"));
  Serial.println(name);
  pickAndPlace(pickPos, place);
  Serial.print(name);
  Serial.println(F("完成"));
}

// ================== 单次循环抓取 A/B/C ==================
void cycleNext() {
  Point places[3] = {placeA, placeB, placeC};
  const char* names[3] = {"A", "B", "C"};
  Serial.print(F("循环-"));
  Serial.println(names[cycleIdx]);
  pickTo(places[cycleIdx], names[cycleIdx]);
  cycleIdx = (cycleIdx + 1) % 3;
}

// ================== 通用绘图路径：回 home -> 首点 -> 各线段 -> 回 home ==================
void drawPath(Point pts[], int n, const __FlashStringHelper* title, const __FlashStringHelper* done) {
  isCanceled = false;
  isPaused = false;
  grip(GRIP_CLOSE);
  Serial.println(title);

  moveLine(currentPos, home, 20); if (canceled()) return;
  moveLine(home, pts[0], 20);     if (canceled()) return;
  delay(300);

  for (int i = 0; i < n - 1; i++) {
    moveLine(pts[i], pts[i + 1], 20);
    if (canceled()) return;
  }

  resetToHome();
  Serial.println(done);
}

// ================== 画直线 ==================
void drawLine() {
  Point pts[2] = {A, B};
  drawPath(pts, 2, F("画直线"), F("直线OK"));
}

// ================== 画 N 字 ==================
void drawN() {
  Point pts[4] = {
    {80, 20, WORK_Z},   // 左下
    {140, 20, WORK_Z},  // 左上
    {80, 0, WORK_Z},    // 右下
    {140, 0, WORK_Z}    // 右上
  };
  drawPath(pts, 4, F("画N"), F("N OK"));
}

// ================== x/y/z 角度指令：例如 x90y80z70 ==================
void parseCommand(const char *cmd) {
  int angles[3] = {-1, -1, -1};
  int idx = 0;

  for (int i = 0; cmd[i]; i++) {
    char c = cmd[i];
    if (c == 'x') idx = 0;
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

// ================== 摇杆控制单个舵机 ==================
void joyControl(int pin, Servo &s, int lo, int hi, bool invert) {
  int v = analogRead(pin);
  int d = 0;

  if (v < 400) d = invert ? -1 : 1;
  else if (v > 600) d = invert ? 1 : -1;

  if (d) s.write(constrain(s.read() + d, lo, hi));
}

// ================== 录制 / 播放 ==================
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
  if (recCount == 0) {
    Serial.println(F("无数据"));
    return;
  }

  Serial.print(F("播放 "));
  Serial.print(recCount);
  Serial.println(F(" 帧"));

  for (int i = 0; i < recCount; i++) {
    servo_6.write(recData[i][0]);
    servo_7.write(recData[i][1]);
    servo_8.write(recData[i][2]);
    servo_9.write(recData[i][3]);
    delay(SAMPLE_MS);
  }

  Serial.println(F("播放完成"));
}

// ================== 回中 ==================
void goHome() {
  Serial.println(F("回中"));
  servo_6.write(GRIP_CLOSE);
  servo_7.write(SHOULDER_VERTICAL);
  servo_8.write(BASE_FORWARD);
  servo_9.write(ELBOW_HORIZONTAL);
  currentPos = home;
}

// ================== 正运动学：当前舵机角度 -> 坐标 ==================
Point forwardKinematics() {
  float theta = radians((float)servo_8.read() - BASE_FORWARD);
  float alpha = radians(90.0 + SHOULDER_VERTICAL - (float)servo_7.read());
  float beta  = radians((float)servo_9.read() - ELBOW_HORIZONTAL - ELBOW_DROOP);

  float r = L1 * cos(alpha) + L2 * cos(beta);
  float z = L1 * sin(alpha) + L2 * sin(beta) + H0;

  Point p;
  p.x = r * cos(theta);
  p.y = r * sin(theta);
  p.z = z;
  return p;
}

// ================== 五点示教：记录并绘制 ==================
void drawTeachPath() {
  isCanceled = false;
  isPaused = false;
  grip(GRIP_CLOSE);

  moveLine(currentPos, home, 20);
  if (canceled()) return;

  moveLine(home, teachPts[0], 20);
  if (canceled()) return;
  delay(300);

  for (int i = 0; i < TEACH_N - 1; i++) {
    moveLine(teachPts[i], teachPts[i + 1], 20);
    if (canceled()) return;
    delay(300);
  }

  resetToHome();
  Serial.println(F("示教轨迹完成"));
}

void handleTeach() {
  if (!teaching) {
    teaching = true;
    teachCount = 0;
    grip(GRIP_CLOSE);
    Serial.println(F("示教: 摇杆移到目标点, 按5记录"));
    return;
  }

  Point p = forwardKinematics();
  p.z = WORK_Z;

  float bA, sA, eA;
  if (!inverseKinematics(p, bA, sA, eA)) {
    Serial.println(F("点不可达, 请重新示教"));
    return;
  }

  teachPts[teachCount++] = p;
  Serial.print(F("记录 "));
  Serial.print(teachCount);
  Serial.print(F("/"));
  Serial.println(TEACH_N);

  if (teachCount >= TEACH_N) {
    teaching = false;
    Serial.println(F("开始绘制"));
    drawTeachPath();
  }
}

// ================== 命令分发 ==================
void handleLine(char *line) {
  while (*line == ' ' || *line == '\r' || *line == '\t') line++;

  int len = strlen(line);
  while (len > 0 && (line[len - 1] == ' ' || line[len - 1] == '\r' || line[len - 1] == '\t'))
    line[--len] = '\0';

  if (len == 0) return;

  // 单字符命令：1 循环，2 录制，3 播放，4 回中，5 示教
  if (len == 1) {
    switch (line[0]) {
      case '1': cycleNext();    return;
      case '2': toggleRecord(); return;
      case '3': playRecord();   return;
      case '4': goHome();       return;
      case '5': handleTeach();  return;
    }
  }

  // 字符串命令
  if (!strcmp(line, "X")) drawLine();
  else if (!strcmp(line, "N")) drawN();
  else if (!strcmp(line, "A")) pickTo(placeA, "A");
  else if (!strcmp(line, "B")) pickTo(placeB, "B");
  else if (!strcmp(line, "C")) pickTo(placeC, "C");
  else if (!strcmp(line, "O") && !teaching) servo_6.write(90);
  else if (!strcmp(line, "S")) servo_6.write(180);
  else if (!strcmp(line, "H")) { speedDelay = 5; joyInterval = 5; }
  else if (!strcmp(line, "L")) { speedDelay = 15; joyInterval = 30; }
  else if (strchr(line, 'x') || strchr(line, 'y') || strchr(line, 'z')) parseCommand(line);
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
  servo_7.write(SHOULDER_VERTICAL);
  servo_8.write(BASE_FORWARD);
  servo_9.write(ELBOW_HORIZONTAL);

  delay(1000);

  currentPos = home;
  isPaused = false;
  isCanceled = false;
  inReturnHome = false;
  cycleIdx = 0;
  teaching = false;
  teachCount = 0;
  lastJoyMs = millis();

  Serial.println(F("就绪: X直线 N字 A/B/C抓取"));
  Serial.println(F("O/S爪 H快 L慢 x/y/z角度"));
  Serial.println(F("板键: 1循环 2录 3放 4回中 5示教"));
  Serial.println(F("D5暂停 D4取消"));
}

// ================== 主循环 ==================
void loop() {
  static char cmdBuf[24];

  // 主串口命令
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

  // 软件串口单字符命令
  if (remoteSerial.available() > 0) {
    cmdBuf[0] = (char)remoteSerial.read();
    cmdBuf[1] = '\0';
    handleLine(cmdBuf);
  }

  // 摇杆控制
  if (millis() - lastJoyMs >= joyInterval) {
    lastJoyMs = millis();
    if (!teaching) joyControl(A0, servo_6, 90, 180, true);
    joyControl(A1, servo_7, 60, 180, false);
    joyControl(A2, servo_8, 5, 175, false);
    joyControl(A3, servo_9, 0, 130, true);
  }

  // 录制采样
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