# Elephant AGV DeviceShifu ROS Driver

这是一个用于控制大象机器人AGV的deviceShifu ROS驱动。

## 功能特点

- 支持AGV的基本运动控制（前进、后退、转向）
- 支持AGV状态监控
- 支持ROS标准接口

## 依赖要求

- ROS Noetic或更高版本
- Python 3.8+
- deviceShifu

## 安装说明

1. 创建工作空间：
```bash
# 在Windows PowerShell中执行
mkdir D:\code\python\ERAGV\src
cd D:\code\python\ERAGV\src
```

2. 克隆仓库：
```bash
git clone https://github.com/qruueiwwwe/ERAGV.git
```

3. 安装依赖：
```bash
pip install -r requirements.txt
```

4. 编译ROS包：
```bash
cd D:\code\python\ERAGV
catkin_make
```

5. 设置环境变量：
```bash
# 在Windows PowerShell中执行
.\devel\setup.ps1
```

## 使用方法

1. 启动驱动节点：
```bash
roslaunch elephant_agv_driver elephant_agv.launch
```

2. 发送控制命令：
```bash
rostopic pub /cmd_vel geometry_msgs/Twist "linear:
  x: 1.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0"
```

## 配置说明

配置文件位于 `config/elephant_agv_config.yaml`，包含以下主要参数：

- serial_port: AGV串口设备名（Windows下通常为 COM3 等）
- baud_rate: 串口波特率
- timeout: 通信超时时间

## 目录结构

```
D:\code\python\ERAGV\
├── src\
│   └── elephant_agv_driver\
│       ├── CMakeLists.txt
│       ├── package.xml
│       ├── requirements.txt
│       ├── config\
│       │   └── elephant_agv_config.yaml
│       ├── launch\
│       │   └── elephant_agv.launch
│       └── scripts\
│           └── elephant_agv_node.py
├── build\
├── devel\
└── README.md
```

## 许可证

MIT License 