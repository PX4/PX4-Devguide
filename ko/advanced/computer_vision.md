# 컴퓨터 비전(Optical Flow, MoCap, VIO, Avoidance)

[컴퓨터 비전](https://en.wikipedia.org/wiki/Computer_vision)은 컴퓨터가 실재하는 환경을 시각 데이터를 활용하여 이해할 수 있게 하는 기술입니다.

PX4는 다음 기능을 지원하기 위해 컴퓨터 비전 시스템([보조 컴퓨터](../companion_computer/pixhawk_companion.md)에서 주로 실행)을 활용합니다.

- [광학 추적](#optical_flow) 기술로 2차원 평면상의 속도를 추정합니다(아래 방향으로 향한 카메라와 아래 방향으로 향한 거리 센서 활용).
- [움직임 촬영](#mocap)을 통해 이동 장비 *외부*의 비전 시스템으로 3차원 자세를 추정합니다. 실내 공간 탐색에 주로 활용합니다.
- [비주얼 관성 주행거리 측정](#vio)기술로 내장 비전 시스템과 관성 측정부를 활용하여 3차원 자세와 속도를 추정합니다. 광역 위치 정보가 빠져있거나 신뢰할 수 없을 때, 공간 탐색에 활용합니다.
- [장애물 회피](https://docs.px4.io/master/en/computer_vision/obstacle_avoidance.html) 기술은 계획 경로를 비행할 때 장애물 주변의 완전한 이동 가능 공간 정보를 제공합니다(현재 missions에서 지원함). 이 기술은 보조 컴퓨터에서 실행하는 [PX4/avoidance](https://github.com/PX4/avoidance)를 활용합니다.
- [충돌 방지](https://docs.px4.io/master/en/computer_vision/collision_prevention.html) 기술은 (주로 매뉴얼 모드로 비행할 때) 비행체가 장애물로 돌진하기 전에 이동을 멈출 때 활용합니다.

> **팁** [PX4 비전 자율 개발 키트](https://docs.px4.io/master/en/complete_vehicles/px4_vision_kit.html) (Holybro)는 개발자들이 PX4 컴퓨터 비전 기술을 다루는데 활용할 수 있는 견고하고 저렴한 키트입니다. It comes with [PX4 avoidance](https://github.com/PX4/avoidance#obstacle-detection-and-avoidance) software pre-installed, and can be used as the base for your own algorithms.

## Motion Capture {#mocap}

Motion Capture (MoCap) is a technique for estimating the 3D *pose* (position and orientation) of a vehicle using a positioning mechanism that is *external* to the vehicle. MoCap systems most commonly detect motion using infrared cameras, but other types of cameras, Lidar, or Ultra Wideband (UWB) may also be used.

> **Note** MoCap is commonly used to navigate a vehicle in situations where GPS is absent (e.g. indoors), and provides position relative to a a *local* co-ordinate system.

For information about MoCap see:

- [External Position Estimation](../ros/external_position_estimation.md)
- [Flying with Motion Capture (VICON, Optitrack)](../tutorials/motion-capture-vicon-optitrack.md)
- [EKF > External Vision System](https://docs.px4.io/master/en/advanced_config/tuning_the_ecl_ekf.html#external-vision-system)

## Visual Inertial Odometry {#vio}

Visual Inertial Odometry (VIO) is used for estimating the 3D *pose* (position and orientation) and *velocity* of a moving vehicle relative to a *local* starting position. It is commonly used to navigate a vehicle in situations where GPS is absent (e.g. indoors) or unreliable (e.g. when flying under a bridge).

VIO uses [Visual Odometry](https://en.wikipedia.org/wiki/Visual_odometry) to estimate vehicle *pose* from visual information, combined with inertial measurements from an IMU (to correct for errors associated with rapid vehicle movement resulting in poor image capture).

> **Note** On difference between VIO and [MoCap](#mocap) is that VIO cameras/IMU are vehicle-based, and additionally provide velocity information.

For information about configuring VIO on PX4 see:

- [EKF > External Vision System](https://docs.px4.io/master/en/advanced_config/tuning_the_ecl_ekf.html#external-vision-system)
- [T265 Setup guide](https://docs.px4.io/master/en/peripheral/t265_vio.md)
- [Snapdragon > Installation > Install Snap VIO](https://docs.px4.io/master/en/flight_controller/snapdragon_flight_software_installation.html#install-snap-vio)

## Optical Flow {#optical_flow}

[Optical Flow](https://docs.px4.io/master/en/sensor/optical_flow.html) provides 2D velocity estimation (using a downward facing camera and a downward facing distance sensor).

For information about optical flow see:

- [Optical Flow](https://docs.px4.io/master/en/sensor/optical_flow.html) 
  - [PX4Flow Smart Camera](https://docs.px4.io/master/en/sensor/px4flow.html)
- [EKF > Optical Flow](https://docs.px4.io/master/en/advanced_config/tuning_the_ecl_ekf.html#optical-flow)

## External Resources

- [XTDrone](https://github.com/robin-shaun/XTDrone/blob/master/README.en.md) - ROS + PX4 simulation environment for computer vision. The [XTDrone Manual](https://www.yuque.com/xtdrone/manual_en) has everything you need to get started!