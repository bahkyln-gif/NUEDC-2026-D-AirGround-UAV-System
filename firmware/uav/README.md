# 无人机端程序（树莓派/香橙派伴机，Python 3.10+）

伴机电脑运行任务状态机，通过 MAVLink 控制 ArduPilot 飞控，通过 WiFi UDP 与小车/地面站通信，下视相机实现动态降落视觉引导。

## 结构

```
├── config.py          全局参数（网络/飞控/任务）
├── comms.py           UDP 通信（UAV_START 接收、CAR_TELEMETRY 转发、遥测上报）
├── mavlink_ctl.py     MAVLink 控制封装（起飞/悬停/位置/降落/状态）
├── vision.py          下视同心圆识别（OpenCV，自标定分辨率）
├── delivery.py        舵机抛投机构（GPIO，含仿真模式）
├── state_machine.py   任务状态机（核心逻辑）
└── uav_main.py        主程序（20Hz 控制环 + 10Hz 遥测）
```

## 部署

```bash
# 树莓派（Ubuntu/Debian）
sudo apt install python3-opencv python3-pip
pip3 install -r requirements.txt
# 若用 RPi.GPIO 控制舵机：sudo apt install python3-rpi.gpio（或用 pigpio）

# 飞控（ArduPilot）参数
#   SYSID_MYGCS = 伴机 255
#   MAV_OPTIONS 允许地面站控制；FRAME 四旋翼 X
#   室内无 GPS：启用 EKF3 光流或使用 OPTFLOW；简单方案：飞控以 "LOCAL_POSITION_NED" 提供位置环
#   （无外部定位时 ArduPilot 无法做位置模式，建议加装光流传感器或用 VIO）

python3 uav_main.py            # 实际运行
python3 uav_main.py --sim      # 仿真模式（无飞控/无相机/无舵机硬件）
```

## 坐标系约定

- 场地坐标：cm，原点左下角，x 右 y 上（与地面站一致）
- 本地 NED：原点 = 起降点 H（场地坐标 126,392），x 东（场地方向 x）、y 北（场地 y 方向）、z 下
- 高度：NED z 为负值，`alt = -z`

## 关键流程

1. 收到 `UAV_START`（mode=DELIVERY/DYNAMIC_LAND）→ 解锁起飞至 150cm
2. 悬停 3s → 进入伴飞：跟踪地面站转发的小车位置（10Hz）
3. 抛投任务：到达小车上空且相对速度 <5cm/s → 舵机抛投 → 返航降落
4. 动态起降：巡航至小车上空 → 下视相机对准同心圆 → 0.3m/s 下降 → 落地停桨保持 5s → 复飞返航
5. 全程 10Hz 上报 `UAV_TELEMETRY`（位置/高度/状态）

## 注意

- 室内定位是本题最大工程点：无 GPS 时推荐**光流 + 激光定高**（PX4Flow/PMW3901 + VL53L1X），或使用动捕/UWB 外源定位；本代码假定飞控已具备 `LOCAL_POSITION_NED` 源
- 桨叶必须全防护；调试佩戴防护眼镜/手套
- 抛投物 25±5g 软质物，舵机安装于机腹中心
