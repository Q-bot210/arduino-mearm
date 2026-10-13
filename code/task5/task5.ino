#include <Servo.h>

Servo servo_6;
Servo servo_7;
Servo servo_8;
Servo servo_9;

// ================== 机械尺寸参数 ==================
const float L1 = 80.0;
const float L2 = 80.0;
const float H0 = 30.0;

// ================== 初始化姿态角度 ==================
const float BASE_FORWARD = 100.0;
const float SHOULDER_VERTICAL = 90.0;
const float ELBOW_HORIZONTAL = 90.0;

struct Point {
  float x;
  float y;
  float z;
};

// ================== 按键引脚（根据资料手册：D4、D5） ==================
const int PAUSE_BTN_PIN = 5;   // D5：暂停/继续按键
const int CANCEL_BTN_PIN = 4;  // D4：取消按键

// ================== 全局状态 ==================
Point currentPos;              // 当前末端位置
bool isPaused = false;         // 暂停标志
bool isCanceled = false;       // 取消标志

// 逆运动学函数
bool inverseKinematics(const Point &p, float &baseAng, float &shoulderAng, float &elbowAng) {
  float r = sqrt(p.x * p.x + p.y * p.y);
  float zRel = p.z - H0;
  float d = sqrt(r * r + zRel * zRel);

  if (d > L1 + L2 || d < fabs(L1 - L2) || d < 1e-6) {
    return false;
  }

  float gamma = atan2(zRel, r);
  float cosDelta = (L1 * L1 + d * d - L2 * L2) / (2.0 * L1 * d);

  if (cosDelta > 1.0) cosDelta = 1.0;
  if (cosDelta < -1.0) cosDelta = -1.0;

  float delta = acos(cosDelta);
  float alpha = gamma + delta;
  float beta = atan2(zRel - L1 * sin(alpha), r - L1 * cos(alpha));

  baseAng = BASE_FORWARD + degrees(atan2(p.y, p.x));
  shoulderAng = 180.0 - degrees(alpha);
  elbowAng = ELBOW_HORIZONTAL + degrees(beta);

  if (baseAng < 0 || baseAng > 180) return false;
  if (shoulderAng < 0 || shoulderAng > 180) return false;
  if (elbowAng < 0 || elbowAng > 180) return false;

  return true;
}

// 写舵机
void writeArm(float baseAng, float shoulderAng, float elbowAng) {
  servo_8.write((int)(baseAng + 0.5));
  servo_7.write((int)(shoulderAng + 0.5));
  servo_9.write((int)(elbowAng + 0.5));
}

// ================== 按键检测 ==================
// 暂停/继续按键：按下一次切换 isPaused
void checkPauseButton() {
  if (digitalRead(PAUSE_BTN_PIN) == LOW) {
    delay(20); // 消抖
    if (digitalRead(PAUSE_BTN_PIN) == LOW) {
      isPaused = !isPaused;
      Serial.print("暂停状态: ");
      Serial.println(isPaused ? "暂停" : "继续");
      while (digitalRead(PAUSE_BTN_PIN) == LOW); // 等待释放
    }
  }
}

// 取消按键：按下置位 isCanceled
void checkCancelButton() {
  if (digitalRead(CANCEL_BTN_PIN) == LOW) {
    delay(20);
    if (digitalRead(CANCEL_BTN_PIN) == LOW) {
      isCanceled = true;
      Serial.println("收到取消指令");
      while (digitalRead(CANCEL_BTN_PIN) == LOW);
    }
  }
}

// ================== 直线插补（带暂停/取消检测） ==================
void moveLine(Point a, Point b, int steps, int stepDelayMs) {
  for (int i = 0; i <= steps; i++) {
    // 检查取消
    checkCancelButton();
    if (isCanceled) return;

    // 检查暂停
    checkPauseButton();
    while (isPaused && !isCanceled) {
      // 暂停中：继续检测按键
      checkPauseButton();
      checkCancelButton();
      delay(10);
    }
    if (isCanceled) return;

    // 插补计算
    float t = (float)i / (float)steps;
    Point p;
    p.x = a.x + (b.x - a.x) * t;
    p.y = a.y + (b.y - a.y) * t;
    p.z = a.z + (b.z - a.z) * t;

    float baseAng, shoulderAng, elbowAng;
    if (!inverseKinematics(p, baseAng, shoulderAng, elbowAng)) {
      Serial.print("路径点不可达: ");
      Serial.print(p.x); Serial.print(", ");
      Serial.print(p.y); Serial.print(", ");
      Serial.println(p.z);
      return;
    }

    writeArm(baseAng, shoulderAng, elbowAng);
    currentPos = p; // 记录当前实际位置，便于取消后返回
    delay(stepDelayMs);
  }
  currentPos = b; // 正常完成，更新终点
}

// ================== 返回待机位置 ==================
void returnToHome() {
  Serial.println("取消任务，返回待机位置");
  isCanceled = false; // 清除取消标志，允许移动
  isPaused = false;
  moveLine(currentPos, (Point){80.0, 0.0, 110.0}, 60, 20);
  currentPos = (Point){80.0, 0.0, 110.0};
}

// ================== 绘图坐标 ==================
const float WORK_Z = 20.0;
Point home = {80.0, 0.0, 110.0};
Point A = {80.0, 80.0, WORK_Z};
Point B = {80.0, -80.0, WORK_Z};

// 画直线
void drawLine() {
  Serial.println("收到 L：开始画直线");
  isCanceled = false;
  isPaused = false;

  moveLine(currentPos, home, 60, 20);
  if (isCanceled) { returnToHome(); return; }

  moveLine(home, A, 40, 20);
  if (isCanceled) { returnToHome(); return; }

  delay(300);

  moveLine(A, B, 80, 20);
  if (isCanceled) { returnToHome(); return; }

  Serial.println("直线完成");
}

// 画 V 字（尖角更明显）
void drawV() {
  Serial.println("收到 V：开始画 V 字");
  isCanceled = false;
  isPaused = false;

  Point V_A   = {70.0,  30.0, WORK_Z};
  Point V_MID = {150.0,  0.0, WORK_Z};
  Point V_B   = {70.0, -30.0, WORK_Z};

  moveLine(currentPos, home, 60, 20);
  if (isCanceled) { returnToHome(); return; }

  moveLine(home, V_A, 40, 20);
  if (isCanceled) { returnToHome(); return; }

  delay(300);

  moveLine(V_A, V_MID, 50, 20);
  if (isCanceled) { returnToHome(); return; }

  moveLine(V_MID, V_B, 50, 20);
  if (isCanceled) { returnToHome(); return; }

  Serial.println("V 字完成");
}

// ================== 初始化 ==================
void setup() {
  Serial.begin(9600);

  // 按键引脚，使用内部上拉
  pinMode(PAUSE_BTN_PIN, INPUT_PULLUP);
  pinMode(CANCEL_BTN_PIN, INPUT_PULLUP);

  servo_6.attach(6);
  servo_7.attach(7);
  servo_8.attach(8);
  servo_9.attach(9);

  servo_6.write(180);
  servo_7.write(90);
  servo_8.write(100);
  servo_9.write(90);

  delay(1000);

  currentPos = home;
  isPaused = false;
  isCanceled = false;

  Serial.println("准备就绪：发送 L 画直线，发送 V 画 V 字");
  Serial.println("绘图过程中：暂停/继续按键 -> 暂停或继续，取消按键 -> 终止并回待机");
}

// ================== 主循环 ==================
void loop() {
  if (Serial.available() > 0) {
    char cmd = Serial.read();

    if (cmd == 'L' || cmd == 'l') {
      drawLine();
    } else if (cmd == 'V' || cmd == 'v') {
      drawV();
    }
  }
}