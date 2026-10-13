#include <Servo.h>

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
int speedDelay = 10;   // H -> 5 加速，L -> 15 减速

// ================== 抓取/放置坐标 ==================
Point pickPos = {60.0,   0.0,  5.0};
Point placeA  = {30.0,  52.0,  5.0};
Point placeB  = {20.0, -56.0,  5.0};
Point placeC  = {151.0,  0.0, 10.0};
const float LIFT_Z = 80.0;

Point home = {80.0, 0.0, 110.0};

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

// ================== 按键检测（暂停 + 取消合并） ==================
void checkButtons() {
  if (inReturnHome) return;
  if (digitalRead(PAUSE_BTN_PIN) == LOW) {
    delay(20);
    if (digitalRead(PAUSE_BTN_PIN) == LOW) {
      isPaused = !isPaused;
      Serial.println(isPaused ? "暂停" : "继续");
      while (digitalRead(PAUSE_BTN_PIN) == LOW);
    }
  }
  if (digitalRead(CANCEL_BTN_PIN) == LOW) {
    delay(20);
    if (digitalRead(CANCEL_BTN_PIN) == LOW) {
      isCanceled = true;
      Serial.println("收到取消指令");
      while (digitalRead(CANCEL_BTN_PIN) == LOW);
    }
  }
}

// ================== 直线插补（步延时会随 speedDelay 缩放） ==================
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
      Serial.print("路径点不可达: ");
      Serial.print(p.x); Serial.print(", ");
      Serial.print(p.y); Serial.print(", ");
      Serial.println(p.z);
      return;
    }
    writeArm(bA, sA, eA);
    currentPos = p;
    delay(stepDelayMs * speedDelay / 10);
  }
  currentPos = b;
}

// ================== 回 home 的两种场景 ==================
void returnToHome() {  // 取消路径：清标志、忽略按键
  Serial.println("取消任务，返回待机位置");
  inReturnHome = true;
  isCanceled = false;
  isPaused = false;
  moveLine(currentPos, home, 60, 20);
  currentPos = home;
  inReturnHome = false;
}

void resetToHome() {   // 正常结束：回 home 并合上夹爪
  inReturnHome = true;
  moveLine(currentPos, home, 60, 20);
  currentPos = home;
  grip(GRIP_CLOSE);
  inReturnHome = false;
}

// 检查取消：已取消则回 home，并返回 true
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
  Serial.print("收到 "); Serial.println(name);
  pickAndPlace(pickPos, place);
  Serial.print(name); Serial.println(" 完成");
}

// ================== 绘图 ==================
const float WORK_Z = 20.0;
Point A = {80.0,  80.0, WORK_Z};
Point B = {80.0, -80.0, WORK_Z};

void drawLine() {
  isCanceled = false; isPaused = false;
  Serial.println("收到 X：开始画直线");
  moveLine(currentPos, home, 60, 20);   if (canceled()) return;
  moveLine(home, A, 40, 20);            if (canceled()) return;
  delay(300);
  moveLine(A, B, 80, 20);               if (canceled()) return;
  resetToHome();
  Serial.println("直线完成，已复位");
}

void drawV() {
  isCanceled = false; isPaused = false;
  Serial.println("收到 V：开始画 V 字");
  Point V_A   = {70.0,  30.0, WORK_Z};
  Point V_MID = {150.0,  0.0, WORK_Z};
  Point V_B   = {70.0, -30.0, WORK_Z};

  moveLine(currentPos, home, 60, 20);   if (canceled()) return;
  moveLine(home, V_A, 40, 20);          if (canceled()) return;
  delay(300);
  moveLine(V_A, V_MID, 50, 20);         if (canceled()) return;
  moveLine(V_MID, V_B, 50, 20);         if (canceled()) return;
  resetToHome();
  Serial.println("V 字完成，已复位");
}

// ================== x/y/z 角度指令 ==================
void parseCommand(String cmd) {
  int angles[3] = {-1, -1, -1};
  int idx = 0;
  for (int i = 0; i < cmd.length(); i++) {
    char c = cmd.charAt(i);
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
// invert=true：摇杆低值方向减小角度；invert=false：摇杆低值方向增大角度
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

// ================== 初始化 ==================
void setup() {
  Serial.begin(9600);
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

  Serial.println("准备就绪：");
  Serial.println("  X -> 画直线    V -> 画 V 字");
  Serial.println("  A/B/C -> 抓取并放置");
  Serial.println("  O/S -> 夹爪开合    H -> 加速    L -> 减速");
  Serial.println("  x<角度> y<角度> z<角度> -> 直接控制底座/肩/肘");
  Serial.println("  摇杆 A0~A3 -> 手动控制夹爪/肩/底座/肘");
  Serial.println("绘图/抓取过程中：D5 暂停或继续，D4 取消并返回待机");
}

// ================== 主循环 ==================
void loop() {
  if (Serial.available() > 0) {
    String line = Serial.readStringUntil('\n');
    line.trim();

    if      (line == "X")  drawLine();
    else if (line == "V")  drawV();
    else if (line == "A")  pickTo(placeA, "A");
    else if (line == "B")  pickTo(placeB, "B");
    else if (line == "C")  pickTo(placeC, "C");
    else if (line == "O")  servo_6.write(90);
    else if (line == "S")  servo_6.write(180);
    else if (line == "H")  speedDelay = 5;
    else if (line == "L")  speedDelay = 15;
    else if (line.indexOf('x') >= 0 || line.indexOf('y') >= 0 || line.indexOf('z') >= 0)
      parseCommand(line);
  }

  // 摇杆控制：低值方向减角 = 反向；低值方向加角 = 正向
  joyControl(A0, servo_6,  90, 180, true);   // 夹爪
  joyControl(A1, servo_7,  60, 180, false);  // 肩
  joyControl(A2, servo_8,   5, 175, false);  // 底座
  joyControl(A3, servo_9,   0, 130, true);   // 肘

  delay(speedDelay);
}