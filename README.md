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
| 102 | 前四关节弧度控制；可选 `r` 或 `rod` 同步控制第五关节 ID16；五个关节共用 `spd/acc` | `{"T":102,"base":0,"shoulder":0,"elbow":0,"hand":0,"r":0,"spd":10,"acc":5}` |
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
| 121 | 单关节角度控制，单位 deg，`joint=1..5`，其中 `joint=5` 控制 ID16 外杆；`spd/acc` 为 `deg/s`、`deg/s^2` | `{"T":121,"joint":5,"angle":180,"spd":100,"acc":10}` |
| 122 | 五关节角度控制，单位 deg；可选 `r` 或 `rod` 同步控制第五关节 ID16；五个关节共用 `spd/acc` | `{"T":122,"b":0,"s":0,"e":0,"h":0,"r":0,"spd":30,"acc":8}` |
| 123 | 连续控制，`m=0` 角度，`axis=5` 连续控制 ID16 外杆 | `{"T":123,"m":0,"axis":5,"cmd":1,"spd":3}` |
| 124 | 读取全部关节角度和力矩数组反馈，顺序为 `b,s,e,t,杆电机` | `{"T":124}` |
| 508 | 单个舵机 raw position 控制，`joint=5` 对应 ID16 外杆，`joint=6` 对应 ID1 夹爪 | `{"T":508,"joint":5,"pos":2047,"spd":120,"acc":10}` |
| 508 | 也可以直接用物理 ID 控制 ID16 外杆 | `{"T":508,"id":16,"pos":2100,"spd":120,"acc":10}` |
| 508 | 也可以直接用物理 ID 控制 ID1 外接夹爪 | `{"T":508,"id":1,"pos":2100,"spd":120,"acc":10}` |

`T=124` 返回格式：

```json
{
  "T": 1241,
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
| 140 | 主臂和外部夹爪联动 | `{"T":140,"b":0,"s":0,"e":0,"h":0,"g":90,"spd":20,"acc":10,"gspd":100,"gacc":10,"torque":1000}` |

### 外部 ST3215 杆电机

| T | 功能 | 示例 |
|---:|---|---|
| 150 | 外部杆收回，`spd/acc` 为 `deg/s`、`deg/s^2` | `{"T":150,"spd":100,"acc":10,"torque":1000}` |
| 151 | 外部杆伸出，`spd/acc` 为 `deg/s`、`deg/s^2` | `{"T":151,"spd":100,"acc":10,"torque":1000}` |
| 152 | 外部杆角度控制，单位 deg，`spd/acc` 为 `deg/s`、`deg/s^2` | `{"T":152,"angle":180,"spd":100,"acc":10,"torque":1000}` |
| 153 | 读取外部杆反馈 | `{"T":153}` |
| 154 | 外部杆扭矩锁定 | `{"T":154,"cmd":1}` |
| 155 | 外部杆动态力矩适应 | `{"T":155,"mode":1,"r":600}` |
| 156 | 将外部杆当前位置设为中位 | `{"T":156,"id":16}` |
| 157 | 外部杆切回舵机位置模式 | `{"T":157,"id":16}` |
| 158 | 外部杆切到连续电机模式 | `{"T":158,"id":16}` |
| 159 | 外部杆连续电机速度控制，速度可为负，0 停止 | `{"T":159,"speed":600,"acc":10}` |
| 160 | 主臂、外部夹爪、外部杆联动；主臂和杆共用 `spd/acc`，夹爪 `gspd/gacc` 仍为外部夹爪舵机 raw 单位 | `{"T":160,"b":0,"s":0,"e":0,"h":0,"g":90,"r":180,"spd":20,"acc":10,"gspd":100,"gacc":10,"gtorque":1000,"rtorque":1000}` |

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
