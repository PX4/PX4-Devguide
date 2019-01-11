# Computer Vision (VIO, Avoidance)

[计算机视觉](https://en.wikipedia.org/wiki/Computer_vision)技术使计算机能够使用视觉数据来理解他们的环境。

PX4使用计算机视觉系统（主要在Companion Computers上运行）以支持以下功能：

- [光流](#optical_flow)提供2D速度估计（使用向下的相机和向下的距离传感器）。
- [运动捕捉](#mocap)使用载具*外部*的视觉系统进行3D姿态估计。 它主要用于室内导航。
- [视觉惯性测距法](#vio)使用机载视觉系统和IMU提供3D姿势和速度估计 It is used for navigation when global position information is absent or unreliable.
- [Obstacle Avoidance](https://docs.px4.io/en/computer_vision/obstacle_avoidance.html) provides navigation around obstacles when flying a planned path (currently missions are supported). This uses [PX4/avoidance](https://github.com/PX4/avoidance) running on a companion computer.
- [Collision Prevention](https://docs.px4.io/en/computer_vision/collision_prevention.html) is used to stop vehicles before they can crash into an obstacle (primarily when flying in manual modes).

## 运动捕捉 {#mocap}

Motion Capture (MoCap) is a technique for estimating the 3D *pose* (position and orientation) of a vehicle using a positioning mechanism that is *external* to the vehicle. MoCap systems most commonly detect motion using infrared cameras, but other types of cameras, Lidar, or Ultra Wideband (UWB) may also be used.

> **Note** MoCap is commonly used to navigate a vehicle in situations where GPS is absent (e.g. indoors), and provides position relative to a a *local* co-ordinate system.

For information about MoCap see:

- [External Position Estimation](../ros/external_position_estimation.md)
- [Flying with Motion Capture (VICON, Optitrack)](../tutorials/motion-capture-vicon-optitrack.md)
- [EKF > External Vision System](../tutorials/tuning_the_ecl_ekf.md#external-vision-system)

## Visual Inertial Odometry {#vio}

Visual Inertial Odometry (VIO) is used for estimating the 3D *pose* (position and orientation) of a moving vehicle relative to a *local* starting position. It is commonly used to navigate a vehicle in situations where GPS is absent (e.g. indoors) or unreliable (e.g. when flying under a bridge).

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