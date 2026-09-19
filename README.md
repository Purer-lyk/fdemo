# fireDemo — 自动火焰探测与瞄准触发系统

基于**可见光 + 红外热成像**双光谱感知的嵌入式火焰探测演示系统：Paddle Lite 部署 PP-YOLO 火焰检测模型，配合 UV 紫外 / 烟雾传感器多源确认，驱动两轴步进电机自动扫描、追踪、瞄准火源，确认后触发执行机构。

运行平台为嵌入式 Linux（wiringPi GPIO，树莓派类板卡）。

## 功能特性

- **双光谱采集**：可见光图像流 + 红外温度流同步接入，支持 G1280s / CS640 / TC2C 三款机芯，通过 `stream.conf` 配置切换（V4L2 / UVC 通道、分辨率、帧率）
- **火焰检测**：Paddle Lite（light_api）加载 PP-YOLO / PP-YOLO tiny `.nb` 模型，主检测 + 二次识别模型（`recognize.nb`）复核，火焰按大小分为四级（无火 / 小火 / 中火 / 大火）
- **多源确认**：视觉检测结果 + UV 紫外传感器 + 烟雾传感器共同判定火情，降低误报
- **扫描与追踪**：状态机驱动，无目标时自动扫描视场，发现火源后闭环反馈控制水平 / 垂直两轴步进电机持续瞄准（带 yaw / pitch 限位保护）
- **触发与报警**：瞄准确认后触发执行机构（TRIGGER1/2），支持报警输出与温控回路
- **远程通信**：Modbus TCP 客户端 / 服务端 + TCP 客户端，断线自动重连，可对外上报状态与位置

## 系统架构

采用「采集 → 推理 → 运动」三线程流水线，通过阻塞队列传递帧数据：

```
producer（双光谱采集） ──> blocking_queue ──> inference（Paddle Lite 检测） ──> motion（GPIO 运动控制）
                                                                       │
                                                   状态机：RESET → STILL → SCAN → CONTROL → TRIGGER
```

- **RESET**：回零位 / 状态复位
- **STILL**：静默监视
- **SCAN**：视场扫描搜索火源
- **CONTROL**：锁定目标，反馈控制瞄准
- **TRIGGER**：确认后触发执行机构

## 目录结构

```
fdemo/
├── Pack/                        # 目标机部署包（整体复制到 /home/l/Pack）
│   ├── fireDemo                 # 可执行文件
│   ├── model/                   # PP-YOLO .nb 模型 + recognize.nb 二次识别模型
│   ├── param/
│   │   ├── params.txt           # 算法与运行参数
│   │   └── gpio.txt             # wiringPi 引脚映射
│   ├── deploy/                  # 运行时动态库（OpenCV 4.x / libmodbus / wiringPi / Paddle Lite）
│   ├── autostart/fire.desktop   # 开机自启动
│   ├── start_fireDemo.sh        # 启动脚本（先配置相机曝光再拉起程序）
│   ├── stream.conf              # 热成像机芯配置（G1280s / CS640 / TC2C）
│   └── readme.txt               # 部署步骤原始记录
└── ioControl_thread/
    ├── fire.desktop             # 桌面启动项
    └── fdemo/                   # 源码（CMake 工程）
        ├── master/              # 程序入口（main.cpp、信号处理）
        ├── process/             # 核心业务逻辑
        │   ├── main_system.*    # 主状态机（扫描 / 追踪 / 触发 / 重连管理）
        │   ├── thread_system.*  # 三线程流水线调度
        │   ├── paddle_detection.*  # Paddle Lite 检测封装
        │   ├── control.*        # wiringPi GPIO 控制（电机 / 限位 / 触发 / 传感器）
        │   ├── modbus_m_*       # Modbus TCP 客户端 / 服务端
        │   └── tcp_client.*     # TCP 通信
        ├── thermal/             # 热成像机芯驱动与组件（UVC 采集、测温、cJSON 解析）
        ├── paddle/              # Paddle Lite 头文件与动态库
        └── model/               # 模型文件
```

## 核心模块

| 模块 | 说明 |
|------|------|
| `main_system` | 主控制器：维护状态机，目标查找与丢失处理、扫描策略、追踪偏差反馈、触发逻辑，以及 Modbus / TCP 断线重连 |
| `thread_system` | 三线程（produce / inference / motion）调度与同步 |
| `paddle_detection` | PP-YOLO 模型加载、前处理（HSV 光照补偿、ROI 裁剪）与推理，输出 `Object` 列表 |
| `control` | GPIO 层：两轴步进电机（脉冲 + 方向 + 使能）、限位开关、触发 / 报警 / 温控输出，UV、烟雾传感器读取 |
| `thermal` | 热成像机芯接入：V4L2 图像流 + 温度流解析、UVC 设备、测温组件 |
| `modbus / tcp` | 对外通信：Modbus TCP 双端（默认 `192.168.3.60:502`）与 TCP 客户端，带状态监测与自动重连 |

## 配置说明

**`stream.conf`** — 热成像机芯选择与通道配置：每款机芯对应一段 `product` 配置，声明控制方式（UART/I2C/USB）、图像格式、分辨率、V4L2 / UVC 设备节点与帧率。更换机芯时保留对应段即可。

**`param/params.txt`** — 主要运行参数：

| 参数 | 含义 |
|------|------|
| `modelFile` / `reModelFile` | 主检测模型与二次识别模型路径 |
| `threshold` / `rethreshold` | 检测阈值 / 复核阈值 |
| `imgLight` | 图像亮度补偿 |
| `ycOffset` / `xcOffset` | 光心偏移（像素） |
| `rangePosx` / `rangePosy` | 目标偏移判定范围 |
| `yawLimit` / `pitchLimit` | 水平 / 垂直限位角度 |
| `leftDirect` / `upDirect` | 电机方向极性 |
| `distinct` | 距离档位表 |
| `modbusIP` / `modbusPORT` | Modbus TCP 服务地址 |
| `flipFlag` / `device` | 图像翻转与设备编号 |

**`param/gpio.txt`** — wiringPi 引脚映射：水平轴（`LAR_PUL/DIR/EN` = 5/6/13）、垂直轴（`UAD_PUL/DIR/EN` = 23/24/25）、限位（`LAR_LIMIT/UAD_LIMIT` = 3/4）、触发（`TRIGGER1/2` = 17/27）、UV（8）、烟雾（`SMOKE/SMOKE_` = 20/16）、温控（`TEMPERATE/TEMPERATE_` = 11/21）。

## 编译

依赖：CMake ≥ 3.10、C++11、OpenCV 4.5+、wiringPi、libmodbus、libusb-1.0、libuvc、Paddle Lite（light_api 共享库，已随源码附带）。

```bash
cd ioControl_thread/fdemo
cd build
cmake ..
make
```

编译产物为 `fireDemo` 可执行文件。

## 部署（目标机）

以下对应 `Pack/readme.txt`，假设目标机用户目录为 `/home/l`：

```bash
# 1. 部署动态库
sudo cp Pack/deploy/* /usr/local/lib/
sudo ldconfig

# 2. 配置开机自启动
cp Pack/autostart/fire.desktop ~/.config/autostart/
sudo apt install gnome-terminal
chmod +x fire.desktop

# 3. 放置部署包并赋权
#    将整个 Pack 目录放到 /home/l/Pack
chmod +x /home/l/Pack/fireDemo
```

## 启动

```bash
cd /home/l/Pack
./start_fireDemo.sh
```

启动脚本会先通过 `v4l2-ctl` 设置可见光相机（SYD USB Camera）的曝光参数，然后以 `stream.conf` 启动 `fireDemo`。程序接收一个参数：

```bash
./fireDemo stream.conf
```

电机速度、电流峰值等执行器参数（speed / current_peek）在调试时通过控制接口下发，参考 `Pack/readme.txt` 第 7 条记录。
