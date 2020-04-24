# Computer Vision (Optical Flow, MoCap, VIO, Avoidance)

[计算机视觉](https://en.wikipedia.org/wiki/Computer_vision) 技术使计算机能够使用视觉数据来理解他们的环境。

PX4 使用计算机视觉系统（主要在机载计算机上运行）以支持以下功能：

- [光流](#optical_flow)提供 2D 速度估计（使用向下的相机和向下的距离传感器）。
- [运动捕捉](#mocap)使用载具*外部*的视觉系统进行3D姿态估计。 它主要用于室内导航。
- [视觉惯性测距法](#vio)使用机载视觉系统和IMU提供3D姿势和速度估计 当 GPS 不存在或不可靠时，它用于导航。
- [Obstacle Avoidance](https://docs.px4.io/master/en/computer_vision/obstacle_avoidance.html) provides full navigation around obstacles when flying a planned path (currently missions are supported). 这依赖机载电脑上运行的 [PX4/avoidance](https://github.com/PX4/avoidance)
- [Collision Prevention](https://docs.px4.io/master/en/computer_vision/collision_prevention.html) is used to stop vehicles before they can crash into an obstacle (primarily when flying in manual modes).

> **Tip** The [PX4 Vision Autonomy Development Kit](https://docs.px4.io/master/en/complete_vehicles/px4_vision_kit.html) (Holybro) is a robust and inexpensive kit for developers working with computer vision on PX4. It comes with [PX4 avoidance](https://github.com/PX4/avoidance#obstacle-detection-and-avoidance) software pre-installed, and can be used as the base for your own algorithms.

## 运动捕捉 {#mocap}

运动捕捉（MoCap）是一种使用载具*外部*的定位机构来估计飞机的3D *姿势*（位置和姿势）的技术。 MoCap系统最常使用红外摄像机检测运动，但也可以使用其他类型的摄像机，激光雷达或Ultra Wideband (UWB)。

> **Note** MoCap 通常用于在 GPS 不存在（例如室内）的情况下导航飞机，并提供相对于*本地*坐标系统的位置。

有关MoCap的信息，请参阅：

- [外部位置的估计](../ros/external_position_estimation.md)
- [使用 Motion Capture 飞行（VICON，Optitrack）](../tutorials/motion-capture-vicon-optitrack.md)
- [EKF > 外部视觉系统](https://docs.px4.io/master/en/advanced_config/tuning_the_ecl_ekf.html#external-vision-system)

## 视觉惯性测距法 {#vio}

Visual Inertial Odometry (VIO) is used for estimating the 3D *pose* (position and orientation) and *velocity* of a moving vehicle relative to a *local* starting position. 它通常用于在GPS不存在（例如室内）或不可靠的情况下（例如在桥下飞行时）导航载具。

VIO使用[视觉测距](https://en.wikipedia.org/wiki/Visual_odometry)来从视觉信息估计车辆*姿势*，结合来自IMU的惯性测量（以校正与载具快速移动导致不良的图像捕获）。

> **Note** VIO 和 [MoCap](#mocap) 之间的区别在于 VIO 摄像机、IMU 是基于飞机本身的，并提供速度信息。

For information about configuring VIO on PX4 see:

- [EKF > 外部视觉系统](https://docs.px4.io/master/en/advanced_config/tuning_the_ecl_ekf.html#external-vision-system)
- [T265 Setup guide](https://docs.px4.io/master/en/peripheral/t265_vio.md)
- [Snapdragon > Installation > Install Snap VIO](https://docs.px4.io/master/en/flight_controller/snapdragon_flight_software_installation.html#install-snap-vio)

## 光流 {#optical_flow}

[Optical Flow](https://docs.px4.io/master/en/sensor/optical_flow.html) provides 2D velocity estimation (using a downward facing camera and a downward facing distance sensor).

有关光流的信息，请参阅：

- [光流](https://docs.px4.io/master/en/sensor/optical_flow.html) 
  - [PX4Flow 智能摄像机](https://docs.px4.io/master/en/sensor/px4flow.html)
- [EKF > 光流](https://docs.px4.io/master/en/advanced_config/tuning_the_ecl_ekf.html#optical-flow)

## External Resources

- [XTDrone](https://github.com/robin-shaun/XTDrone/blob/master/README.en.md) - ROS + PX4 v1.9 simulation environment for computer vision. The [XTDrone Manual](https://www.yuque.com/xtdrone/manual_en) has everything you need to get started!