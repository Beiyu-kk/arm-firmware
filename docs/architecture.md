# 固件架构说明

本项目是运行在 ESP32 上的图书馆机械臂控制固件，使用 PlatformIO + Arduino 框架。整理后的结构参考了成熟嵌入式项目常见做法：入口文件保持短小，协议、配置、驱动、业务模块按目录分层，避免所有头文件堆在根目录。

## 目录结构

```text
arm-firmware/
  Book_Arm_Fusion.ino      主入口，只负责 setup、loop 和模块装配
  platformio.ini           PlatformIO 编译、烧录配置

  config/                  全局配置和命令编号
    arm_config.h           舵机 ID、机械臂尺寸、限位、安全策略、全局状态
    command_ids.h          JSON 命令 T 编号定义

  core/
    command_dispatcher.h   JSON 命令分发，T -> 具体功能函数

  arm/
    roarm_m2_module.h      机械臂舵机控制、几何 FK/IK、关节/坐标控制
    roarm_m2_advance.h     mission、高级配置和动作文件播放

  end_effectors/
    external_gripper.h     外置 Gripper-B 控制
    external_rod.h         外置 ST3215 杆控制

  drivers/
    oled_ctrl.h            OLED 显示
    servo_bus_arbiter.h    共享 TTL 舵机总线互斥
    switch_module.h        12V 开关、PWM 输出

  comm/
    wifi_ctrl.h            Wi-Fi AP/STA/AP+STA
    http_server.h          HTTP/Web 控制入口
    esp_now_ctrl.h         ESP-NOW 通信

  storage/
    files_ctrl.h           LittleFS 文件读写

  web/
    m2_web_page.h          Web 控制页面 HTML

  docs/
    architecture.md        当前文件
    command_protocol.md    JSON 命令协议说明
```

## 主程序启动流程

入口在 `Book_Arm_Fusion.ino`。

```text
setup()
  -> 初始化串口和 I2C
  -> 初始化 OLED
  -> 初始化 LittleFS
  -> 初始化 12V 开关/PWM
  -> 初始化共享舵机总线锁
  -> 初始化机械臂舵机
  -> 初始化外置夹爪
  -> 初始化外置杆电机
  -> 检查舵机反馈状态
  -> 设置安全启动策略
  -> 初始化 Wi-Fi、HTTP、ESP-NOW
  -> 设置默认扭矩
```

主循环：

```text
loop()
  -> serialCtrl()                 USB 串口 JSON 命令
  -> server.handleClient()         HTTP/Web 请求
  -> espNowHandlePendingCommand()  ESP-NOW 待处理命令
  -> constantHandle()              连续运动控制
  -> RoArmM2_getPosByServoFeedback() 周期反馈
  -> jsonCmdReceiveHandler()       处理异步收到的新命令
```

## 命令数据流

所有外部控制最终都会进入同一个 JSON 命令分发器：

```text
USB 串口 / Web / ESP-NOW / mission 文件
  -> jsonCmdReceive
  -> core/command_dispatcher.h
  -> jsonCmdReceiveHandler()
  -> switch(T)
  -> 调用机械臂、夹爪、杆、Wi-Fi、文件系统等模块函数
```

命令编号集中在 `config/command_ids.h`，例如：

```cpp
#define CMD_JOINTS_RAD_CTRL 102
#define CMD_XYZT_GOAL_CTRL 104
#define CMD_SERVO_RAD_FEEDBACK 105
```

## 机械臂控制数据流

关节角控制：

```text
T=102 或 T=122
  -> RoArmM2_allJointAbsCtrl() / RoArmM2_allJointsAngleCtrl()
  -> RoArmM2_baseJointCtrlRad()
  -> RoArmM2_shoulderJointCtrlRad()
  -> RoArmM2_elbowJointCtrlRad()
  -> RoArmM2_handJointCtrlRad()
  -> goalPos[]
  -> RoArmM2_syncWritePosEx()
  -> SCServo SyncWritePosEx()
  -> TTL 总线
  -> 舵机 11/12/13/14/15
```

末端坐标控制：

```text
T=104 或 T=1041
  -> RoArmM2_baseCoordinateCtrl()
  -> cartesian_to_polar()
  -> simpleLinkageIkRad()
  -> 计算 BASE/SHOULDER/ELBOW/EOAT 目标角
  -> RoArmM2_goalPosMove()
  -> SyncWritePosEx()
```

## 反馈数据流

```text
舵机反馈 steps
  -> getFeedback()
  -> servoFeedback[]
  -> calculateRadByFeedback()
  -> radB/radS/radE/radG
  -> RoArmM2_computePosbyJointRad()
  -> lastX/lastY/lastZ/lastT
  -> RoArmM2_infoFeedback()
```

## 继续重构原则

当前这次整理只调整文件结构，不改变控制逻辑。下一步如果继续优化，建议按这个顺序做：

1. 从 `arm/roarm_m2_module.h` 中拆出 `arm_servo_io.h`，只放舵机读写和反馈。
2. 拆出 `arm_kinematics.h`，只放几何正逆运动学。
3. 拆出 `arm_motion_control.h`，只放关节控制、坐标控制、插值运动。
4. 把 `config/arm_config.h` 中的全局状态逐步收进结构体，例如 `ArmState`。

每拆一步都运行：

```bash
pio run
```

这样能避免一次性大改导致固件不可烧录。

## 参考的组织思路

- PlatformIO 推荐把项目源码、配置、库依赖清晰分层。
- Arduino 库规范强调用 `src` 或模块目录组织可复用代码。
- ESP-IDF 使用 component 思路隔离功能模块。

本项目仍是 Arduino 风格固件，所以没有强行改成 ESP-IDF component，而是采用更适合当前工程的轻量目录分层。
