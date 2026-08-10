# ORP 系统架构设计

## 1. 项目名称

Open Robot Platform（ORP）

## 2. 项目目标

构建一套开放、模块化、可扩展的移动机器人平台，支持：

- 麦克纳姆四轮底盘
- Arduino Mega2560 / STM32G474 底层控制
- Raspberry Pi 4 边缘计算
- 双目视觉
- IMU / GPS / 编码器融合
- RTAB-Map SLAM
- Navigation2 自主导航
- 场景识别
- 云边协同
- 多机器人地图融合
- 后续机械臂抓取

## 3. 系统总体架构

```text
                 Cloud / Server
                      |
          地图融合 / AI / 数据分析
                      |
                   Wi-Fi
                      |
              Raspberry Pi 4
                      |
        +-------------+-------------+
        |             |             |
      ROS2          Vision        Sensors
        |             |             |
        |         OV9281 Stereo     |
        |                           |
   Serial Bridge              WT901 / GPS
        |
 Arduino Mega2560
        |
   Motor Control
        |
  TB6612 + Encoder
        |
  Mecanum Chassis
