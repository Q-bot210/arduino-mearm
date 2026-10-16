# arduino-mearm
机器人学院二面（机电 邱悦翔 3126000290）

（前期各任务代码分开完成，后期代码整合与优化由AI辅助完成）

代码说明：

稍作整理，更清晰一点：

**任务一：**

- **摇杆控制**：主循环中读取摇杆模拟量变化，调用 `joyControl()`，增减对应舵机角度。
- **串口指令**：通过改变舵机每步 `delay()` 调整整体移动速度；通过改变夹爪舵机角度控制开合。
- **xyz 协同控制**：`handleLine()` 识别含 `x/y/z` 的指令后调用 `parseCommand()`，解析各轴角度并分别写入 `servo_8`（底座）、`servo_7`（肩）、`servo_9`（肘），实现三轴协同设定。

**任务二：**

- 设定好 A/B/C 的初始与目标位置，调用 `inverseKinematics()` 进行逆运动学解算，使夹爪运动到目标位置。

**任务三：**

- **循环夹取**：`cycleNext()` 依次选择 A/B/C 放置点，调用 `pickTo()` 执行抓取放置流程，完成后索引循环。
- **录制播放**：`toggleRecord()` 开始/停止录制，`loop()` 中定时采样舵机角度存入 `recData`；`playRecord()` 逐帧回放。
- **回中**：调用 `goHome()` 将各舵机角度直接设为基准角。

**任务五：**

- **画直线**：`drawLine()` 定义起点 A 和终点 B，调用 `drawPath()` 回待机后依次直线插补到两点。
- **画“N”**：`drawN()` 定义四个点，调用 `drawPath()` 依次连接形成 N 字。
- **暂停/取消**：`checkButtons()` 检测 D5 暂停、D4 取消按键，设置 `isPaused` / `isCanceled`；`moveLine()` 中暂停时循环等待，取消时立即返回；`canceled()` 检测到取消后调用 `returnToHome()` 回待机并复位状态。
- **五点示教**：`handleTeach()` 记录五次摇杆调整后的坐标到 `teachPts`，满五点后调用 `drawTeachPath()` 依次插补绘制。
