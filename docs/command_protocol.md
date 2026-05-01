# JSON 命令协议说明

本固件所有控制命令都以 JSON 形式进入程序，并通过数字字段 `T` 表示命令类型。命令编号定义在 `config/command_ids.h`，命令分发逻辑在 `core/command_dispatcher.h`。
关节角度控制命令中的 `spd` 统一为 `deg/s`，`acc` 统一为 `deg/s^2`；五个关节共用同一组 `spd/acc`，raw position 校准命令仍使用舵机 raw 单位。

## 快速开始

```powershell
cd E:\master_degree\project\10.图书馆机器人\arm_code\BookArmCode1\arm-firmware
pio run
pio run -t upload
pio device monitor -p COM8 -b 921600
```

## 机械臂命令

| T | 功能 | 主要函数 | 示例 |
|---|---|---|---|
| `100` | 回到初始化姿态 | `RoArmM2_moveInit()` | `{"T":100}` |
| `101` | 单关节弧度控制 | `RoArmM2_singleJointAbsCtrl()` | `{"T":101,"joint":1,"rad":0,"spd":0,"acc":10}` |
| `102` | 多关节弧度控制，可选 `r`/`rod` 控制第五关节 | `RoArmM2_allJointAbsCtrl()` | `{"T":102,"base":0,"shoulder":0,"elbow":1.57,"hand":3.14,"r":0,"spd":30,"acc":10}` |
| `103` | 单轴坐标控制 | `RoArmM2_singlePosAbsBesselCtrl()` | `{"T":103,"axis":1,"pos":235,"spd":0.25}` |
| `104` | XYZT 插值控制 | `RoArmM2_allPosAbsBesselCtrl()` | `{"T":104,"x":235,"y":0,"z":234,"t":3.14,"spd":0.25}` |
| `1041` | XYZT 直接控制 | `RoArmM2_baseCoordinateCtrl()` + `RoArmM2_goalPosMove()` | `{"T":1041,"x":235,"y":0,"z":234,"t":3.14}` |
| `105` | 读取机械臂反馈 | `RoArmM2_getPosByServoFeedback()` + `RoArmM2_infoFeedback()` | `{"T":105}` |
| `106` | 末端手腕/夹爪舵机角度 | `RoArmM2_handJointCtrlRad()` | `{"T":106,"cmd":3.14,"spd":0,"acc":0}` |
| `107` | 末端舵机扭矩 | `RoArmM2_handTorqueCtrl()` | `{"T":107,"tor":200}` |
| `108` | 设置关节 PID | `RoArmM2_setJointPID()` | `{"T":108,"joint":3,"p":16,"i":0}` |
| `109` | 恢复默认 PID | `RoArmM2_resetPID()` | `{"T":109}` |
| `112` | 动态扭矩限制 | `RoArmM2_dynamicAdaptation()` | `{"T":112,"mode":1,"b":60,"s":110,"e":50,"h":50}` |
| `121` | 单关节角度控制 | `RoArmM2_singleJointAngleCtrl()` | `{"T":121,"joint":1,"angle":0,"spd":10,"acc":10}` |
| `122` | 多关节角度控制 | `RoArmM2_allJointsAngleCtrl()` | `{"T":122,"b":0,"s":0,"e":90,"h":180,"spd":10,"acc":10}` |
| `123` | 连续运动控制 | `constantCtrl()` | `{"T":123,"m":0,"axis":1,"cmd":1,"spd":3}` |
| `124` | 读取全部关节数组反馈 | `RoArmM2_getPosByServoFeedback()` + `BookArm_jointsArrayFeedback()` | `{"T":124}` 返回 `{"T":1241,"joints_rad":[b,s,e,t,r],"joints_torque":[b,s,e,t,r]}` |

## 外置夹爪命令

| T | 功能 | 主要函数 | 示例 |
|---|---|---|---|
| `130` | 打开夹爪 | `ExternalGripper_open()` | `{"T":130,"spd":100,"acc":10,"torque":1000}` |
| `131` | 关闭夹爪 | `ExternalGripper_close()` | `{"T":131,"spd":100,"acc":10,"torque":1000}` |
| `132` | 夹爪角度控制 | `ExternalGripper_moveToAngle()` | `{"T":132,"angle":90,"spd":100,"acc":10,"torque":1000}` |
| `133` | 读取夹爪反馈 | `ExternalGripper_getFeedback()` + `ExternalGripper_infoFeedback()` | `{"T":133}` |
| `134` | 夹爪扭矩锁 | `ExternalGripper_torqueCtrl()` | `{"T":134,"cmd":1}` |
| `135` | 夹爪动态扭矩 | `ExternalGripper_dynamicAdaptation()` | `{"T":135,"mode":1,"g":600}` |
| `140` | 机械臂和夹爪联动 | `Fusion_jointsAngleGripperCtrl()` | `{"T":140,"b":0,"s":0,"e":90,"h":180,"g":90,"spd":20,"acc":10}` |

## 外置杆命令

| T | 功能 | 主要函数 | 示例 |
|---|---|---|---|
| `150` | 杆收回 | `ExternalRod_retract()` | `{"T":150,"spd":100,"acc":10,"torque":1000}` |
| `151` | 杆伸出 | `ExternalRod_extend()` | `{"T":151,"spd":100,"acc":10,"torque":1000}` |
| `152` | 杆角度控制 | `ExternalRod_moveToAngle()` | `{"T":152,"angle":180,"spd":100,"acc":10,"torque":1000}` |
| `153` | 读取杆反馈 | `ExternalRod_getFeedback()` + `ExternalRod_infoFeedback()` | `{"T":153}` |
| `154` | 杆扭矩锁 | `ExternalRod_torqueCtrl()` | `{"T":154,"cmd":1}` |
| `155` | 杆动态扭矩 | `ExternalRod_dynamicAdaptation()` | `{"T":155,"mode":1,"r":600}` |
| `157` | 切回位置舵机模式 | `ExternalRod_setServoMode()` | `{"T":157,"id":16}` |
| `158` | 切到连续电机模式 | `ExternalRod_setMotorMode()` | `{"T":158,"id":16}` |
| `159` | 连续电机速度控制 | `ExternalRod_motorSpeed()` | `{"T":159,"speed":600,"acc":10}` |
| `160` | 机械臂、夹爪、杆联动 | `Fusion_jointsGripperRodAngleCtrl()` | `{"T":160,"b":0,"s":0,"e":90,"h":180,"g":90,"r":20,"spd":5,"acc":5}` |

## 系统、通信和文件命令

| T | 功能 | 所在模块 |
|---|---|---|
| `0` | 急停 | `arm/roarm_m2_module.h` |
| `999` | 清除急停标志 | `core/command_dispatcher.h` |
| `200-208` | 文件创建、读取、修改、删除 | `storage/files_ctrl.h` |
| `210` | 机械臂舵机扭矩开关 | `arm/roarm_m2_module.h` |
| `220-232` | mission 创建、编辑、播放 | `arm/roarm_m2_advance.h` |
| `300-305` | ESP-NOW 设置 | `comm/esp_now_ctrl.h` |
| `401-408` | Wi-Fi 设置 | `comm/wifi_ctrl.h` |
| `501-503` | 舵机 ID、中位、PID 设置 | `arm/roarm_m2_module.h` |
| `600-605` | ESP32 重启、Flash、NVS、日志设置 | 多模块 |

## 注意事项

- `T=101` 和 `T=102` 使用弧度，速度/加速度参数按舵机 steps 解释。
- `T=121` 和 `T=122` 使用角度，内部会转换为弧度和舵机 steps。
- `T=104` 会做插值运动，适合普通末端移动。
- `T=1041` 不做插值，计算 IK 后直接下发目标。
- `T=105`、`T=133`、`T=153` 会通过串口输出反馈 JSON。
