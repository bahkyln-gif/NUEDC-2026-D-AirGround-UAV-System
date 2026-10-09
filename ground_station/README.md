# 地面站（Python + Tkinter）

图形化实时显示小车/无人机在 400×500 场地中的位置，文字显示无人机状态，支持数据落盘与**仿真模式**（无需硬件即可演示全流程）。

## 运行

```bash
pip install -r requirements.txt
python gs_main.py --sim      # 仿真模式：点击"启动仿真"即可看到完整流程动画
python gs_main.py            # 真实模式：接收小车/无人机 UDP 遥测
```

## 功能

- 场地画布：环线轨迹、A/B/C/D/H 关键点、小车（带朝向）、无人机（带高度）、平台同心圆图案实时绘制
- 状态栏：无人机状态文字（IDLE/TAKEOFF/HOVER/FORMATION/DELIVERY/RETURN/LAND/DYN_LAND...）大字显示并着色
- 数据面板：小车里程/速度/位置，无人机高度/速度/电池，链路状态
- 控制：急停/恢复（发送 GS_CMD）；仿真模式下的一键启动
- 数据记录：`logs/gs_YYYYmmdd_HHMMSS.csv`

## 文件

```
├── config.py       端口/颜色/界面参数
├── track.py        轨迹几何（与小车固件一致，供仿真/显示）
├── network.py      UDP 收发（遥测接收、GS_CMD 发送）
├── simulator.py    仿真小车/无人机（按协议发消息）
└── gs_main.py      主界面
```

## 与硬件联调

- 小车（ESP32）→ UDP `CAR_TELEMETRY` 到本机 14551
- 无人机 → UDP `UAV_TELEMETRY` 到本机 14551
- 本机 IP 需与小车/无人机同一 WiFi 局域网，`config.py` 中 `BIND_IP` 默认 0.0.0.0
