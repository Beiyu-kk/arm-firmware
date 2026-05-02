# Arm Firmware

这是运行在 ESP32 上的图书馆机械臂控制固件，参考 BookArmControl 和 Waveshare RoArm-M2-S JSON 控制协议实现。新机械臂硬件映射为：ID11 到 ID15 构成前四个关节，ID16 外杆电机作为第五关节，ID1 为外接夹爪。

如果串口不是 `COM8`：

```powershell
pio run -t upload --upload-port COMx
```

## 安全策略

当前固件默认关闭危险的自动动作：

```cpp
#define ARM_AUTO_MOVE_INIT_ON_BOOT 0
#define ARM_DISABLE_AUTO_TORQUE_RELEASE 1
#define RUN_BOOT_MISSION_ON_STARTUP 0
```

也就是说，上电后不会自动回初始化位，不会自动播放 `boot` mission，也不会在启动流程中自动释放肩部从动舵机扭矩。

## 目录结构

```text
arm-firmware/
  Book_Arm_Fusion.ino      固件入口、setup/loop
  platformio.ini           PlatformIO 配置
  config/                  全局配置和命令编号
  core/                    命令分发、raw 校准命令
  arm/                     主机械臂运动学和舵机控制
  end_effectors/           外部夹爪和外部杆
  drivers/                 OLED、开关模块、总线仲裁
  comm/                    Wi-Fi、HTTP、ESP-NOW
  storage/                 LittleFS 文件操作
  web/                     Web 页面资源
  docs/                    架构和协议文档
  data/                    LittleFS 默认数据文件
```

## 快速开始

```powershell
cd E:\master_degree\project\10.图书馆机器人\arm_code\BookArmCode1\arm-firmware
pio run
pio run -t upload
pio device monitor -p COM8 -b 921600
```

## 命令协议

所有串口/HTTP/ESP-NOW 控制命令都是 JSON，命令编号字段统一为 `T`。串口命令需要以换行符结尾。

关节角度控制命令中的 `spd` 统一表示角速度，单位为 `deg/s`；`acc` 统一表示角加速度，单位为 `deg/s^2`。五个关节共用同一组 `spd/acc`，固件会在下发舵机前自动换算为舵机 raw steps。`raw position` 校准命令除外，仍使用舵机 raw 单位。

### 急停

| T | 功能 | 示例 |
|---:|---|---|
| 0 | 急停 | `{"T":0}` |
| 999 | 恢复急停标志 | `{"T":999}` |

### 末端类型和装配参数

| T | 功能 | 示例 |
|---:|---|---|
| 1 | 设置 EoAT 模式，`mode=0` 夹爪，`mode=1` 腕部 | `{"T":1,"mode":0}` |
| 2 | 配置 EoAT 装配参数 | `{"T":2,"pos":3,"ea":0,"eb":20}` |

### 主机械臂控制

| T | 功能 | 示例 |
|---:|---|---|
| 100 | 移动到初始化位置 | `{"T":100}` |
| 101 | 单关节弧度控制，`joint=1..5`，其中 `joint=5` 控制 ID16 外杆；`spd/acc` 为 `deg/s`、`deg/s^2` | `{"T":101,"joint":5,"rad":1.57,"spd":100,"acc":10}` |
| 102 | 五关节弧度控制；必须提供 `r` 或 `rod`；ID11-16 同一帧同步写；五个关节共用 `spd/acc` | `{"T":102,"base":0,"shoulder":0,"elbow":0,"hand":0,"r":0,"spd":25,"acc":5}` |
| 103 | 单轴笛卡尔控制，`axis=1:x,2:y,3:z,4:t` | `{"T":103,"axis":2,"pos":0,"spd":0.25}` |
| 104 | XYZT 目标控制，带平滑插补 | `{"T":104,"x":235,"y":0,"z":234,"t":3.14,"spd":0.25}` |
| 1041 | XYZT 直接控制 | `{"T":1041,"x":235,"y":0,"z":234,"t":3.14}` |
| 105 | 读取主臂位姿、前四关节角、第五关节 ID16 和负载反馈 | `{"T":105}` |
| 106 | 末端手腕/夹爪弧度控制 | `{"T":106,"cmd":1.57,"spd":0,"acc":0}` |
| 107 | 设置 EoAT 抓取扭矩 | `{"T":107,"tor":200}` |
| 108 | 设置关节 PID | `{"T":108,"joint":3,"p":16,"i":0}` |
| 109 | 重置 PID | `{"T":109}` |
| 110 | 设置新的 X 轴 | `{"T":110,"xAxisAngle":0}` |
| 111 | 延时 | `{"T":111,"cmd":3000}` |
| 112 | 主臂动态力矩适应 | `{"T":112,"mode":1,"b":60,"s":110,"e":50,"h":50}` |
| 113 | 12V 开关 PWM 控制 | `{"T":113,"pwm_a":-255,"pwm_b":-255}` |
| 114 | 灯光控制 | `{"T":114,"led":255}` |
| 115 | 关闭开关输出 | `{"T":115}` |
| 121 | 读取全部关节角度和力矩数组反馈，顺序为 `b,s,e,t,杆电机` | `{"T":121}` |
| 122 | 五个逻辑关节扭矩锁定/释放，控制 `base,shoulder,elbow,EOAT,杆电机`；shoulder 会同时控制 ID12/ID13；`cmd=0` 会释放保持力矩 | `{"T":122,"cmd":1}` / `{"T":122,"cmd":0}` |
| 123 | 五关节弧度控制；功能同 `T=102`；必须提供 `r` 或 `rod`；ID11-16 同一帧同步写；五个关节共用 `spd/acc` | `{"T":123,"base":0,"shoulder":0,"elbow":0,"hand":0,"r":-0.5,"spd":25,"acc":5}` |
| 508 | 单个舵机 raw position 控制，`joint=5` 对应 ID16 外杆，`joint=6` 对应 ID1 夹爪 | `{"T":508,"joint":5,"pos":2047,"spd":120,"acc":10}` |
| 508 | 也可以直接用物理 ID 控制 ID16 外杆 | `{"T":508,"id":16,"pos":2100,"spd":120,"acc":10}` |
| 508 | 也可以直接用物理 ID 控制 ID1 外接夹爪 | `{"T":508,"id":1,"pos":2100,"spd":120,"acc":10}` |

弧度/角度输入不再做软件角度限位，包括 `base/shoulder/elbow/hand/r/rod`；最终下发给舵机的 raw position 仍会按各舵机 raw 边界限制，避免越界写入。外部夹爪角度控制会额外保留开闭范围保护，角度换算后的 raw 会限制在 `2047..3150`。

`T=121` 返回格式：

```json
{
  "T": 1211,
  "joints_rad": [0, 0, 0, 0, 0],
  "joints_torque": [0, 0, 0, 0, 0]
}
```

其中 `joints_rad` 单位为弧度，`joints_torque` 为舵机 load/力矩反馈值。两个数组的数据顺序均为 `b, s, e, t, 杆电机`。

主臂关节编号：

```text
1 BASE_JOINT
2 SHOULDER_JOINT
3 ELBOW_JOINT
4 EOAT_JOINT / fourth joint / ID15
5 EXT_ROD_JOINT / fifth joint / ID16
6 EXT_GRIPPER_JOINT / external gripper / ID1
```

### 外部 Gripper-B

| T | 功能 | 示例 |
|---:|---|---|
| 130 | 打开外部夹爪 | `{"T":130,"spd":100,"acc":10,"torque":1000}` |
| 131 | 关闭外部夹爪 | `{"T":131,"spd":100,"acc":10,"torque":1000}` |
| 132 | 外部夹爪角度控制，单位 deg | `{"T":132,"angle":90,"spd":100,"acc":10,"torque":1000}` |
| 133 | 读取外部夹爪反馈 | `{"T":133}` |
| 134 | 外部夹爪扭矩锁定 | `{"T":134,"cmd":1}` |
| 135 | 外部夹爪动态力矩适应 | `{"T":135,"mode":1,"g":600}` |
| 136 | 将外部夹爪当前位置设为中位 | `{"T":136,"id":1}` |
| 137 | 将外部夹爪切到电机模式 | `{"T":137,"id":1,"tor":120}` |
| 138 | 先按 `T=131` 逻辑发送一次闭合位置指令，延迟 `delay` ms 后直接切入负方向恒力模式持续收紧；默认延迟 `500ms`；收到 `T=130` 打开命令会切回位置模式并打开 | `{"T":138,"spd":100,"acc":10,"torque":1000,"hold":-200,"delay":500}` |

`T=138` 闭合后恒力保持参数：

```json
{"T":138,"spd":100,"acc":10,"torque":1000,"hold":-200,"delay":500}
```

`spd/acc/torque` 用于第一步的位置闭合，和 `T=131` 相同。`hold` 或 `holdtor` 用于第二步恒力模式，固件会强制转为负方向，例如 `hold=80` 和 `hold=-80` 最终都会按 `-80` 下发。`delay` 是闭合位置指令发出后到切入恒力模式的等待时间，单位 ms，默认 `500`。固件不会等待位置反馈或判断是否到位，时间到就直接发送恒力模式指令。

外部夹爪按 Gripper_CF 示例中的 CF35 / CFSCL 方式控制，不再使用普通 SCSCL 位置包。夹爪 `WritePosEx` 的底层写入为 `addr=41,len=7`，数据顺序为 `acc, posL, posH, torqueL, torqueH, speedL, speedH`。`torque` 会限制在 `50..1000`，并用于夹爪位置控制和扭矩上限写入。

### 五关节和夹爪整组控制

`T=141/142/143` 面向五个逻辑关节和外部夹爪的一体化控制。ID11、ID12、ID13、ID14、ID15、ID16 继续按机械臂/杆电机方式控制；ID1 外部夹爪单独走与 `T=130..137` 相同的 CF35/CFSCL 夹爪接口。

| T | 功能 | 示例 |
|---:|---|---|
| 141 | 读取五个逻辑关节和夹爪状态；ID11-16 使用机械臂/杆反馈读取，ID1 使用夹爪反馈读取；夹爪状态返回 `open` 或 `closed` | `{"T":141}` |
| 142 | 所有电机扭矩锁定/释放；ID11-16 使用机械臂/杆扭矩控制，ID1 使用夹爪 `ExternalGripper_torqueCtrl()` | `{"T":142,"cmd":1}` / `{"T":142,"cmd":0}` |
| 143 | 前五个逻辑关节弧度同步控制，并同步控制夹爪开闭；`gripper=0` 打开，`gripper=1` 闭合；闭合后持续保持扭矩；必须提供 `r` 或 `rod`；夹爪部分与 `T=130/131` 使用同一入口 | `{"T":143,"base":0,"shoulder":0,"elbow":0,"hand":0,"r":0,"gripper":1,"spd":100,"acc":10,"torque":1000}` |

`T=141` 会先读取 ID11-16 的机械臂/杆反馈，再用 `ExternalGripper_getFeedback()` 读取 ID1 夹爪反馈。`T=142` 会先控制 ID11-16 的扭矩，再用 `ExternalGripper_torqueCtrl()` 控制 ID1。`T=143` 会先调用 `BookArm_syncAllJointsRad()` 控制 ID11-16；夹爪部分则直接调用 `ExternalGripper_open()` 或 `ExternalGripper_close()`，因此 `gripper=1` 的夹爪关闭逻辑与 `T=131` 完全一致，使用同样的 `spd/acc/torque` 参数和 `3150` 闭合目标。`gtorque` 仍可作为旧字段兼容，优先使用 `torque`。

```text
ID11 base
ID12 shoulder driving
ID13 shoulder driven
ID14 elbow
ID15 fourth joint / hand
ID16 fifth joint / external rod
ID1  external gripper
```

`T=141` 返回格式：

```json
{
  "T": 1411,
  "ok": true,
  "joints_rad": [0, 0, 0, 0, 0],
  "joints_torque": [0, 0, 0, 0, 0],
  "raw_pos": [2049, 2054, 2039, 1947, 2590, 2047, 2047],
  "online": [true, true, true, true, true, true, true],
  "gripper_state": "open",
  "gripper_pos": 2047
}
```

数组顺序：

```text
joints_rad:    base, shoulder, elbow, hand, rod
joints_torque: base, shoulder, elbow, hand, rod
raw_pos:       ID11, ID12, ID13, ID14, ID15, ID16, ID1
online:        ID11, ID12, ID13, ID14, ID15, ID16, ID1
```

`gripper_state` 为 `open` 或 `closed`。

`T=142` 返回格式：

```json
{
  "T": 1421,
  "ok": true,
  "requested": 1,
  "cmd": 1,
  "gripper_hold": true,
  "ids": "11,12,13,14,15,16,1"
}
```

`requested` 是收到的原始请求，`cmd` 是实际下发的扭矩状态。当前配置启用 `ARM_FORCE_TORQUE_LOCK_ALWAYS_ON` 时，`cmd=0` 会被强制保持为 `1`。夹爪已经通过 `T=143` 闭合并进入 `gripper_hold=true` 时，即使请求 `{"T":142,"cmd":0}`，实际 `cmd` 仍会保持为 `1`，避免夹爪松开。需要释放抓取时，先用 `T=143` 发送 `gripper=0` 打开夹爪。

如果总线忙：

```json
{
  "T": 1421,
  "ok": false,
  "error": "servo bus busy"
}
```

`T=143` 返回格式：

```json
{
  "T": 1431,
  "ok": true,
  "arm": true,
  "gripper_ok": true,
  "gripper": "closed",
  "gripper_hold": true,
  "pos": [2049, 2054, 2039, 1947, 2590, 2047, 3150]
}
```

`pos` 顺序为 `ID11, ID12, ID13, ID14, ID15, ID16, ID1`。`gripper=0` 时夹爪目标为 `open` / `2047`，`gripper=1` 时夹爪目标为 `closed` / `3150`。
`arm=true` 表示 ID11-16 的五个逻辑关节控制已下发；`gripper_ok=true` 表示 ID1 夹爪控制已通过 CF35/CFSCL 夹爪接口下发。`gripper_hold=true` 表示夹爪处于持续抓取保持状态。

如果缺少 `r` 或 `rod`：

```json
{
  "T": 1431,
  "ok": false,
  "error": "T=143 requires r or rod"
}
```

`T=143` 的 `ok=true` 表示机械臂/杆和夹爪控制命令均已发出，不表示每个电机已经运动到位。确认实际状态可再发送 `{"T":141}`。

### 文件和扭矩控制

| T | 功能 | 示例 |
|---:|---|---|
| 200 | 扫描 LittleFS 文件 | `{"T":200}` |
| 201 | 创建文件 | `{"T":201,"name":"file.txt","content":"hello"}` |
| 202 | 读取文件 | `{"T":202,"name":"file.txt"}` |
| 203 | 删除文件 | `{"T":203,"name":"file.txt"}` |
| 204 | 文件末尾追加一行 | `{"T":204,"name":"file.txt","content":"hello"}` |
| 205 | 插入一行 | `{"T":205,"name":"file.txt","lineNum":3,"content":"hello"}` |
| 206 | 替换一行 | `{"T":206,"name":"file.txt","lineNum":3,"content":"hello"}` |
| 207 | 读取一行 | `{"T":207,"name":"file.txt","lineNum":3}` |
| 208 | 删除一行 | `{"T":208,"name":"file.txt","lineNum":3}` |
| 210 | 主臂舵机扭矩锁定；当前固件强制永不断力矩锁，并强制最大扭矩保持当前位置，`cmd=0` 会被忽略 | `{"T":210,"cmd":1}` |

### Mission

| T | 功能 | 示例 |
|---:|---|---|
| 220 | 创建 mission | `{"T":220,"name":"mission_a","intro":"test mission"}` |
| 221 | 读取 mission 内容 | `{"T":221,"name":"mission_a"}` |
| 222 | 追加 JSON step | `{"T":222,"name":"mission_a","step":"{\"T\":114,\"led\":255}"}` |
| 223 | 追加当前反馈为 step | `{"T":223,"name":"mission_a","spd":0.25}` |
| 224 | 追加延时 step | `{"T":224,"name":"mission_a","delay":3000}` |
| 225 | 插入 JSON step | `{"T":225,"name":"mission_a","stepNum":3,"step":"{\"T\":114,\"led\":255}"}` |
| 226 | 插入当前反馈为 step | `{"T":226,"name":"mission_a","stepNum":3,"spd":0.25}` |
| 227 | 插入延时 step | `{"T":227,"stepNum":3,"delay":3000}` |
| 228 | 替换为 JSON step | `{"T":228,"name":"mission_a","stepNum":3,"step":"{\"T\":114,\"led\":255}"}` |
| 229 | 替换为当前反馈 step | `{"T":229,"name":"mission_a","stepNum":3,"spd":0.25}` |
| 230 | 替换为延时 step | `{"T":230,"name":"mission_a","stepNum":3,"delay":3000}` |
| 231 | 删除 step | `{"T":231,"name":"mission_a","stepNum":3}` |
| 241 | 移动到 mission 某一步 | `{"T":241,"name":"mission_a","stepNum":3}` |
| 242 | 播放 mission，`times=-1` 无限循环 | `{"T":242,"name":"mission_a","times":3}` |

### ESP-NOW

| T | 功能 | 示例 |
|---:|---|---|
| 300 | 配置是否跟随广播 | `{"T":300,"mode":1}` |
| 301 | 设置 ESP-NOW 模式 | `{"T":301,"mode":3}` |
| 302 | 获取本机 MAC 地址 | `{"T":302}` |
| 303 | 添加 follower | `{"T":303,"mac":"FF:FF:FF:FF:FF:FF"}` |
| 304 | 移除 follower | `{"T":304,"mac":"FF:FF:FF:FF:FF:FF"}` |
| 305 | 组播控制 | `{"T":305,"dev":0,"b":0,"s":0,"e":0,"h":0,"cmd":0,"megs":"hello"}` |
| 306 | 单播或广播控制 | `{"T":306,"mac":"FF:FF:FF:FF:FF:FF","dev":0,"b":0,"s":0,"e":0,"h":0,"cmd":0,"megs":"hello"}` |

### Wi-Fi

| T | 功能 | 示例 |
|---:|---|---|
| 401 | 设置开机 Wi-Fi 模式，`0` 关闭，`1` AP，`2` STA，`3` AP+STA | `{"T":401,"cmd":3}` |
| 402 | 配置 AP | `{"T":402,"ssid":"RoArm-M2","password":"12345678"}` |
| 403 | 配置 STA | `{"T":403,"ssid":"your_wifi","password":"your_password"}` |
| 404 | 配置 AP+STA | `{"T":404,"ap_ssid":"RoArm-M2","ap_password":"12345678","sta_ssid":"your_wifi","sta_password":"your_password"}` |
| 405 | 获取 Wi-Fi 信息 | `{"T":405}` |
| 406 | 根据当前状态创建 Wi-Fi 配置文件 | `{"T":406}` |
| 407 | 根据输入创建 Wi-Fi 配置文件 | `{"T":407,"mode":3,"ap_ssid":"RoArm-M2","ap_password":"12345678","sta_ssid":"your_wifi","sta_password":"your_password"}` |
| 408 | 停止 Wi-Fi | `{"T":408}` |

### 舵机设置和零位校准

| T | 功能 | 示例 |
|---:|---|---|
| 501 | 修改舵机 ID | `{"T":501,"raw":1,"new":11}` |
| 502 | 将当前位置写为舵机中位 | `{"T":502,"id":11}` |
| 503 | 设置单个舵机 PID | `{"T":503,"id":14,"p":16}` |
| 504 | 将 7 个校准电机移动到同一个 raw position | `{"T":504,"pos":2047,"spd":120,"acc":10}` |
| 505 | 读取 7 个校准电机 raw position | `{"T":505,"target":2047,"tol":1}` |
| 506 | 移动 7 个校准电机并循环验证 raw feedback | `{"T":506,"pos":2047,"spd":120,"acc":10,"tol":1,"timeout":10000}` |
| 507 | 读取当前编译进固件的关节零位映射 | `{"T":507}` |

修改舵机 ID 时也可以写成 `{"T":501,"id":1,"new":16}`。命令会返回 `T=5011`，其中 `ok=true` 表示写入流程完成，`newOnline=true` 表示新 ID 已经能读到。改 ID 时总线上建议只接这一颗要修改的电机。

`T=504/505/506` 覆盖的 7 个电机：

```text
ID 11 base
ID 12 shoulder driving
ID 13 shoulder driven
ID 14 elbow
ID 15 fourth joint / hand
ID 1  external gripper
ID 16 fifth joint / external rod
```

### ESP32 设置

| T | 功能 | 示例 |
|---:|---|---|
| 600 | 重启 ESP32 | `{"T":600}` |
| 601 | 获取剩余 flash 空间 | `{"T":601}` |
| 602 | 获取 boot mission 信息 | `{"T":602}` |
| 603 | 重置 boot mission | `{"T":603}` |
| 604 | 清除 NVS，Wi-Fi 异常时使用 | `{"T":604}` |
| 605 | 设置串口调试输出，`0` 关闭，`1` 调试，`2` flow feedback | `{"T":605,"cmd":1}` |

## 零位映射说明

当前主臂 C 固件使用统一映射：

```text
pos = ZERO_POS + DIRECTION * angle_rad / (2*pi) * 4096
```

零位参数在 `config/arm_config.h` 中：

```cpp
#define BASE_ZERO_POS              2049
#define SHOULDER_DRIVING_ZERO_POS  2054
#define SHOULDER_DRIVEN_ZERO_POS   2039
#define ELBOW_ZERO_POS             1947
#define GRIPPER_ZERO_POS           2590
#define EXT_ROD_ZERO_POS           2047
```

调试时可用：

```json
{"T":507}
```

## 维护建议

- 新增命令时，先在 `config/command_ids.h` 定义编号，再在 `core/command_dispatcher.h` 增加分发。
- 访问舵机总线时必须通过 `drivers/servo_bus_arbiter.h` 的共享总线仲裁。
- 修改机械臂运动学和零位映射前，先阅读 `arm/roarm_m2_module.h`。
- 每次结构调整后运行 `pio run`，确认固件仍可编译。
