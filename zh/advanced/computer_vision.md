# 计算机视觉（VIO, 避障）

[计算机视觉](https://en.wikipedia.org/wiki/Computer_vision) 技术使计算机能够使用视觉数据来理解他们的环境。

PX4 使用计算机视觉系统（主要在机载计算机上运行）以支持以下功能：

- [光流](#optical_flow)提供 2D 速度估计（使用向下的相机和向下的距离传感器）。
- [运动捕捉](#mocap)使用载具*外部*的视觉系统进行3D姿态估计。 它主要用于室内导航。
- [视觉惯性测距法](#vio)使用机载视觉系统和IMU提供3D姿势和速度估计 当 GPS 不存在或不可靠时，它用于导航。
- [障碍避免](https://docs.px4.io/en/computer_vision/obstacle_avoidance.html)在飞行计划路径时提供绕障碍物的导航（支持当前的任务）。 这依赖机载电脑上运行的 [PX4/avoidance](https://github.com/PX4/avoidance)
- [碰撞预防](https://docs.px4.io/en/computer_vision/collision_prevention.html)使载具在撞到障碍物之前停止（主要是在手动模式下飞行时）。

## 运动捕捉 {#mocap}

运动捕捉（MoCap）是一种使用载具*外部*的定位机构来估计飞机的3D *姿势*（位置和姿势）的技术。 MoCap系统最常使用红外摄像机检测运动，但也可以使用其他类型的摄像机，激光雷达或Ultra Wideband (UWB)。

> **Note** MoCap 通常用于在 GPS 不存在（例如室内）的情况下导航飞机，并提供相对于*本地*坐标系统的位置。

有关MoCap的信息，请参阅：

- [外部位置的估计](../ros/external_position_estimation.md)
- [使用 Motion Capture 飞行（VICON，Optitrack）](../tutorials/motion-capture-vicon-optitrack.md)
- [EKF > 外部视觉系统](https://docs.px4.io/master/en/advanced_config/tuning_the_ecl_ekf.html#external-vision-system)

## 视觉惯性测距法 {#vio}

视觉惯性测距（VIO）用于估计移动车辆相对于*起始点*起始位置的3D *姿势*（位置和方向）。 它通常用于在GPS不存在（例如室内）或不可靠的情况下（例如在桥下飞行时）导航载具。

VIO使用[视觉测距](https://en.wikipedia.org/wiki/Visual_odometry)来从视觉信息估计车辆*姿势*，结合来自IMU的惯性测量（以校正与载具快速移动导致不良的图像捕获）。

> **Note** VIO 和 [MoCap](#mocap) 之间的区别在于 VIO 摄像机、IMU 是基于飞机本身的，并且还提供速度信息。

有关VIO的信息，请参阅：

- [EKF > 外部视觉系统](https://docs.px4.io/master/en/advanced_config/tuning_the_ecl_ekf.html#external-vision-system)
- [Snapdragon > 安装 > 安装 Snap VIO](https://docs.px4.io/master/en/flight_controller/snapdragon_flight_software_installation.html#install-snap-vio)

## 光流 {#optical_flow}

[光流](https://docs.px4.io/en/sensor/optical_flow.html)提供2D速度估计（使用向下的相机和向下的距离传感器）。

有关光流的信息，请参阅：

- [光流](https://docs.px4.io/en/sensor/optical_flow.html) 
  - [PX4Flow 智能摄像机](https://docs.px4.io/en/sensor/px4flow.html)
- [EKF > 光流](https://docs.px4.io/master/en/advanced_config/tuning_the_ecl_ekf.html#optical-flow)