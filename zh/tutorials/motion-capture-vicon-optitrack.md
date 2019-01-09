# 使用 Motion Capture 飞行（VICON，Optitrack）

> **Warning** **WORK IN PROGRESS**. 该主题与 [External Position Estimation (ROS)](../ros/external_position_estimation.md) 是重叠的。

像 VICON和Optitrack 这样的室内运动捕捉系统可用于为车辆状态估计提供位置和姿态数据，或作为分析的基础事实。 动作捕捉数据可用于更新 PX4 相对于本地原点的本地位置估计。 来自运动捕捉系统的航向（偏航）也可以由姿态估计器可选地集成。

来自运动捕捉系统的姿势（位置和方向）数据通过 MAVLink 使用 [ATT_POS_MOCAP](https://mavlink.io/en/messages/common.html#ATT_POS_MOCAP) 消息发送到自动驾驶仪。 请参阅下面有关数据表示约定的坐标框架的部分。 [mavros](../ros/mavros_installation.md) ROS-Mavlink 接口有一个默认插件来发送此消息。 这些消息也可以直接使用MAVLINK库并编写C/C++代码来发送和接收。

## 计算架构

**highly recommended** 通过 **onboard** 计算机（例如 Raspberry Pi，ODroid 等）发送动作捕捉数据以进行可靠通信。 机载计算机可以通过WiFi连接到动作捕捉计算机，提供可靠的高带宽连接。

大多数标准遥测链路（如 3DR/SiK 无线电）**不** 适合高带宽运动捕捉应用。

## 坐标系

本节演示如何使用适当的参考坐标系。 关于坐标系有各种各样的表示, 但我们将使用其中两个: ENU 和 NED。

* ENU 是地面固定的框架，其中 **X** 轴指向东，**Y** 指向北并且 **Z** 向上。 机器人/车身框架朝向前方 **X**，向左朝向 **Z** 和 **Y**。
* NED 具有朝向北方的 **X**，向东具有 **Y**，并且 **Z** 向下。 机器人/车身框架具有朝向前方的 **X**，相应地具有 **Z** 和 **Y**。

框架如下所示。 左边的 NED，右边的 ENU： ![参考机架](../../assets/lpe/ref_frames.png)

然而，利用外部航向估计，磁北被忽略并且用对应于世界 *x* 轴的矢量（其可以在 mocap 校准处自由放置）伪造; 偏航角将给予局部 *x*。

> **Warning** 在动作捕捉软件中创建刚体时，请记住首先将机器人与世界 **X** 轴对齐，否则偏航估计将具有初始偏移。

## Estimator 选择

### LPE 和态度估计 Q

### EKF2

Mocap系统的运动上限 `mocap_pose_estimate` 的 ROS 主题和视觉的 `vision_pose_estimate`。 有关详细信息, 请检查 [ mavros_extras ](http://wiki.ros.org/mavros_extras)。

## 测试

## 故障处理