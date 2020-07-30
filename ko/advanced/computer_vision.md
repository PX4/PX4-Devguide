# 컴퓨터 비전(Optical Flow, MoCap, VIO, Avoidance)

[컴퓨터 비전](https://en.wikipedia.org/wiki/Computer_vision)은 컴퓨터가 실재하는 환경을 시각 데이터를 활용하여 이해할 수 있게 하는 기술입니다.

PX4는 다음 기능을 지원하기 위해 컴퓨터 비전 시스템([보조 컴퓨터](../companion_computer/pixhawk_companion.md)에서 주로 실행)을 활용합니다.

- [광학 추적(Optical Flow)](#optical_flow) 기술은 2차원 평면상의 속도 추정 정보를 제공합니다(아래 방향으로 향한 카메라와 아래 방향으로 향한 거리 센서 활용).
- [움직임 촬영(Motion Capture)](#mocap) 기술은 비행체 *외부*의 비전 시스템을 통해 3차원 자세 추정 정보를 제공합니다. 실내 공간 탐색에 주로 활용합니다.
- [시각적 관성 주행 측정](#vio) 기술은 내장 비전 시스템과 관성 측정부(IMU)를 활용하여 3차원 자세와 속도 추정 정보를 제공합니다. 광역 위치 정보가 빠져있거나 신뢰할 수 없을 때, 공간 탐색에 활용합니다.
- [장애물 회피](https://docs.px4.io/master/en/computer_vision/obstacle_avoidance.html) 기술은 계획 경로를 비행할 때 장애물 주변의 완전한 이동 가능 공간 정보를 제공합니다(현재 missions에서 지원함). 이 기술은 보조 컴퓨터에서 실행하는 [PX4/avoidance](https://github.com/PX4/avoidance)를 활용합니다.
- [충돌 방지](https://docs.px4.io/master/en/computer_vision/collision_prevention.html) 기술은 (주로 매뉴얼 모드로 비행할 때) 비행체가 장애물로 돌진하기 전에 이동을 멈출 때 활용합니다.

> **팁** [PX4 비전 자율 개발 키트](https://docs.px4.io/master/en/complete_vehicles/px4_vision_kit.html) (Holybro)는 개발자들이 PX4 컴퓨터 비전 기술을 다루는데 활용할 수 있는 견고하고 저렴한 키트입니다. [PX4 avoidance](https://github.com/PX4/avoidance#obstacle-detection-and-avoidance) 프로그램을 미리 설치한 상태로 나오며, 개발자 여러분이 자체적으로 보유한 알고리즘을 시험해볼 수 있는 기반으로 활용할 수 있습니다.

## 움직임 촬영(Motion Capture) {#mocap}

움직임 촬영(Motion Capture, a.k.a MoCap)은 비행체 *외부*의 위치 결정 방법으로 3차원 *자세*(위치와 방향) 를 추정하는 기술입니다. MoCap 시스템은 보통 적외선 카메라로 움직임을 감지하나, 광선 레이더, 광대역 주파(UWB) 형태 기술을 활용할 수도 있습니다.

> **참고** MoCap은 GPS가 빠져있는 상황에서 비행체 탐색 운용을 할 때 활용하며, 상대적인 *로컬* 좌표 체계 위치 정보를 제공합니다.

MoCap 기술에 대해 더 알아보려면 다음을 참고하십시오:

- [외부 위치 추정](../ros/external_position_estimation.md)
- [움직임 촬영(Motion Capture)기술을 활용한 비행 (VICON, Optitrack)](../tutorials/motion-capture-vicon-optitrack.md)
- [EKF > 외부 비전 시스템](https://docs.px4.io/master/en/advanced_config/tuning_the_ecl_ekf.html#external-vision-system)

## 시각적 관성 주행 측정 {#vio}

시각적 관성 주행 측정(VIO) 기술은 *로컬* 시작점으로부터 상대적인 위치로 비행체가 이동할 때 3차원 *자세* (위치와 방향)와 *속도*를 추정할 때 활용합니다. It is commonly used to navigate a vehicle in situations where GPS is absent (e.g. indoors) or unreliable (e.g. when flying under a bridge).

VIO uses [Visual Odometry](https://en.wikipedia.org/wiki/Visual_odometry) to estimate vehicle *pose* from visual information, combined with inertial measurements from an IMU (to correct for errors associated with rapid vehicle movement resulting in poor image capture).

> **Note** On difference between VIO and [MoCap](#mocap) is that VIO cameras/IMU are vehicle-based, and additionally provide velocity information.

PX4의 VIO 설정 방법을 더 알아보려면 다음을 참고하십시오:

- [EKF > 외부 비전 시스템](https://docs.px4.io/master/en/advanced_config/tuning_the_ecl_ekf.html#external-vision-system)
- [T265 설정 안내서](https://docs.px4.io/master/en/peripheral/t265_vio.md)
- [스냅드래곤 > 설치 > 스냅 VIO 설치](https://docs.px4.io/master/en/flight_controller/snapdragon_flight_software_installation.html#install-snap-vio)

## 광학 추적(Optical Flow) {#optical_flow}

[광학 추적(Optical Flow)](https://docs.px4.io/master/en/sensor/optical_flow.html) 기술로 2차원 평면상의 속도를 추정합니다(아래 방향으로 향한 카메라와 아래 방향으로 향한 거리 센서 활용).

광학 추적(Optical Flow) 기술에 대해 더 알아보려면 다음을 참고하십시오:

- [광학 추적(Optical Flow)](https://docs.px4.io/master/en/sensor/optical_flow.html) 
  - [PX4Flow 스마트 카메라](https://docs.px4.io/master/en/sensor/px4flow.html)
- [EKF > 광학 추적(Optical Flow)](https://docs.px4.io/master/en/advanced_config/tuning_the_ecl_ekf.html#optical-flow)

## 외부 참고 자료

- [XTDrone](https://github.com/robin-shaun/XTDrone/blob/master/README.en.md) - 컴퓨터 비전용 ROS + PX4 시뮬레이션 환경입니다. [XTDrone 설명서](https://www.yuque.com/xtdrone/manual_en)에 시작에 필요한 모든 내용을 넣었습니다!