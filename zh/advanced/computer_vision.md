# Computer Vision (VIO, Avoidance)

[计算机视觉](https://en.wikipedia.org/wiki/Computer_vision)技术使计算机能够使用视觉数据来理解他们的环境。

PX4使用计算机视觉系统（主要在Companion Computers上运行）以支持以下功能：

- [光流](#optical_flow)提供2D速度估计（使用向下的相机和向下的距离传感器）。
- [运动捕捉](#mocap)使用载具*外部*的视觉系统进行3D姿态估计。 它主要用于室内导航。
- [视觉惯性测距法](#vio)使用机载视觉系统和IMU提供3D姿势和速度估计 当 GPS 不存在或不可靠时，它用于导航。
- [障碍避免](https://docs.px4.io/en/computer_vision/obstacle_avoidance.html)在飞行计划路径时提供绕障碍物的导航（支持当前的任务）。 这依赖机载电脑上运行的 [PX4/avoidance](https://github.com/PX4/avoidance)
- [碰撞预防](https://docs.px4.io/en/computer_vision/collision_prevention.html)使载具在撞到障碍物之前停止（主要是在手动模式下飞行时）。

## 运动捕捉 {#mocap}

运动捕捉（MoCap）是一种使用载具*外部*的定位机构来估计车辆的3D *姿势*（位置和姿势）的技术。 MoCap系统最常使用红外摄像机检测运动，但也可以使用其他类型的摄像机，激光雷达或Ultra Wideband (UWB)。

> **Note** MoCap通常用于在GPS不存在 (例如室内) 的情况下导航车辆，并提供相对于*本地*坐标系统的位置。

有关MoCap的信息，请参阅：

- [外部位置的估计](../ros/external_position_estimation.md)
- [使用 Motion Capture 飞行（VICON，Optitrack）](../tutorials/motion-capture-vicon-optitrack.md)
- [EKF > 外部视觉系统](../tutorials/tuning_the_ecl_ekf.md#external-vision-system)

## 视觉惯性测距法 {#vio}

视觉惯性测距（VIO）用于估计移动车辆相对于*起始点*起始位置的3D *姿势*（位置和方向）。 It is commonly used to navigate a vehicle in situations where GPS is absent (e.g. indoors) or unreliable (e.g. when flying under a bridge).

VIO uses [Visual Odometry](https://en.wikipedia.org/wiki/Visual_odometry) to estimate vehicle *pose* from visual information, combined with inertial measurements from an IMU (to correct for errors associated with rapid vehicle movement resulting in poor image capture).

> **Note** On difference between VIO and [MoCap](#mocap) is that VIO cameras/IMU are vehicle-based, and additionally provide velocity information.

For information about VIO see:

- [EKF > External Vision System](../tutorials/tuning_the_ecl_ekf.md#external-vision-system)
- [Snapdragon > Installation > Install Snap VIO](https://docs.px4.io/en/flight_controller/snapdragon_flight_software_installation.html#install-snap-vio)

## Optical Flow {#optical_flow}

[Optical Flow](https://docs.px4.io/en/sensor/optical_flow.html) provides 2D velocity estimation (using a downward facing camera and a downward facing distance sensor).

For information about optical flow see:

- [Optical Flow](https://docs.px4.io/en/sensor/optical_flow.html) 
  - [PX4Flow Smart Camera](https://docs.px4.io/en/sensor/px4flow.html)
- [EKF > Optical Flow](../tutorials/tuning_the_ecl_ekf.md#optical-flow)