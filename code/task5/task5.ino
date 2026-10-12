#include <Servo.h>  // 引入 Arduino 舵机库，用来控制 Servo 对象

Servo servo_6; // 创建夹爪舵机对象，对应引脚 6
Servo servo_7; // 创建肩舵机对象，对应引脚 7
Servo servo_8; // 创建底座舵机对象，对应引脚 8
Servo servo_9; // 创建肘舵机对象，对应引脚 9

// ================== 机械尺寸参数，单位：mm ==================
const float L1 = 80.0;   // 肩关节到肘关节的长度：上臂 80mm
const float L2 = 80.0;   // 肘关节到夹爪中心的长度：前臂 80mm
const float H0 = 30.0;   // 底部转轴中心到肩关节中心的高度：30mm

// ================== 初始化姿态对应的角度 ==================
const float BASE_FORWARD = 100.0;      // 底座在正前方时的舵机角度
const float SHOULDER_VERTICAL = 90.0;  // 肩舵机让上臂竖直时的角度
const float ELBOW_HORIZONTAL = 90.0;   // 肘舵机让前臂水平时的角度

struct Point {  // 定义一个三维坐标点结构体
  float x;      // x：正前方距离，单位 mm
  float y;      // y：左方距离，单位 mm
  float z;      // z：高度，从底部转轴中心向上，单位 mm
};

// 记录当前末端点，用于串口多次绘图时衔接
Point currentPos;

// 逆运动学函数：输入目标点 p，计算三个舵机应该转到的角度
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

  // 选择“肘在上”的解，和初始化姿态一致
  float alpha = gamma + delta;

  float beta = atan2(zRel - L1 * sin(alpha), r - L1 * cos(alpha));

  // 把数学角度转换成舵机角度
  baseAng = BASE_FORWARD + degrees(atan2(p.y, p.x));
  shoulderAng = 180.0 - degrees(alpha);
  elbowAng = ELBOW_HORIZONTAL + degrees(beta);

  // 舵机角度范围保护
  if (baseAng < 0 || baseAng > 180) return false;
  if (shoulderAng < 0 || shoulderAng > 180) return false;
  if (elbowAng < 0 || elbowAng > 180) return false;

  return true;
}

// 把计算出来的三个角度写入舵机。夹爪 servo_6 不在这里动
void writeArm(float baseAng, float shoulderAng, float elbowAng) {
  servo_8.write((int)(baseAng + 0.5));     // 底座舵机
  servo_7.write((int)(shoulderAng + 0.5)); // 肩舵机
  servo_9.write((int)(elbowAng + 0.5));    // 肘舵机
}

// 直线插补函数：让末端从 a 点沿直线走到 b 点
// steps：把直线分成多少小段；stepDelayMs：每小段之间延时多少毫秒
void moveLine(Point a, Point b, int steps, int stepDelayMs) {
  for (int i = 0; i <= steps; i++) {
    float t = (float)i / (float)steps;

    Point p;
    p.x = a.x + (b.x - a.x) * t;
    p.y = a.y + (b.y - a.y) * t;
    p.z = a.z + (b.z - a.z) * t;

    float baseAng, shoulderAng, elbowAng;

    if (!inverseKinematics(p, baseAng, shoulderAng, elbowAng)) {
      Serial.print("路径点不可达: ");
      Serial.print(p.x);
      Serial.print(", ");
      Serial.print(p.y);
      Serial.print(", ");
      Serial.println(p.z);
      return;
    }

    writeArm(baseAng, shoulderAng, elbowAng);
    delay(stepDelayMs);
  }

  // 全部走完后，记录当前末端位置
  currentPos = b;
}

// ================== 绘图相关坐标点 ==================
const float WORK_Z = 20.0; // 绘图所在高度，保持原代码逻辑

Point home = {80.0, 0.0, 110.0}; // 初始化姿态对应的末端位置

// 原直线 A -> B
Point A = {80.0, 80.0, WORK_Z};
Point B = {80.0, -80.0, WORK_Z};

// 画直线：保留原 setup 中的运动逻辑
void drawLine() {
  Serial.println("收到 L：开始画直线");

  // 先回到 home，保证每次绘图从统一位置开始
  moveLine(currentPos, home, 60, 20);

  // 原逻辑：home -> A -> B
  moveLine(home, A, 40, 20);
  delay(300);
  moveLine(A, B, 80, 20);

  Serial.println("直线完成");
}

// 画 V 字：使用更尖锐的尖角
void drawV() {
  Serial.println("收到 V：开始画 V 字");

  // 先回到 home，保证每次绘图从统一位置开始
  moveLine(currentPos, home, 60, 20);

  // 定义 V 字的三个点，使尖角更加明显（夹角约 41°）
  Point V_A   = {70.0,  30.0, WORK_Z};
  Point V_MID = {150.0,  0.0, WORK_Z};
  Point V_B   = {70.0, -30.0, WORK_Z};

  // 先到 V 字左上点
  moveLine(home, V_A, 40, 20);
  delay(300);

  // 画 V 字左半边
  moveLine(V_A, V_MID, 50, 20);

  // 画 V 字右半边
  moveLine(V_MID, V_B, 50, 20);

  Serial.println("V 字完成");
}

void setup() {
  // 打开串口，主要用于接收绘图指令和打印调试信息
  Serial.begin(9600);

  // 把四个 Servo 对象绑定到对应引脚
  servo_6.attach(6); // 夹爪舵机接引脚 6
  servo_7.attach(7); // 肩舵机接引脚 7
  servo_8.attach(8); // 底座舵机接引脚 8
  servo_9.attach(9); // 肘舵机接引脚 9

  // 写入初始化姿态
  servo_6.write(180); // 夹爪
  servo_7.write(90);  // 肩：上臂竖直
  servo_8.write(100); // 底座：正前方
  servo_9.write(90);  // 肘：前臂水平

  // 等待 1 秒，让机械臂稳定到初始化姿态
  delay(1000);

  // 记录当前末端位置为 home
  currentPos = home;

  Serial.println("准备就绪：发送 L 画直线，发送 V 画 V 字");
}

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