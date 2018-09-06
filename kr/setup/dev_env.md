# 파일과 코드 설치하기

PX4 코드는 [Linux](../setup/dev_env_linux.md)나 [Mac OS](../setup/dev_env_mac.md)에서 개발될 수 있습니다. [모든 PX4 대상 장비](#supported-targets) 제작과 대부분의 [시뮬레이터](../simulation/README.md) 및 [ROS](../ros/README.md) 사용을 가능하게 하기 때문에 [우분투 Linux LTS 에디션](https://wiki.ubuntu.com/LTS) 사용을 추천합니다.

## 지원 대상 장비

아래의 표는 각 OS에서 어떤 PX 대상 장비가 개발될 수 있는지 보여줍니다.

대상 장비 | Linux (Ubuntu) | Mac | Windows
--|:--:|:--:|:--:
**NuttX 기반 하드웨어:** [Pixhawk 시리즈](https://docs.px4.io/en/flight_controller/pixhawk_series.html), [Crazyflie](https://docs.px4.io/en/flight_controller/crazyflie2.html), [Intel® Aero Ready to Fly Drone](https://docs.px4.io/en/flight_controller/intel_aero.html) | X | X | X
[Qualcomm Snapdragon Flight 하드웨어](https://docs.px4.io/en/flight_controller/snapdragon_flight.html) | X | | 
**Linux 기반 하드웨어:** [Raspberry Pi 2/3](https://docs.px4.io/en/flight_controller/raspberry_pi_navio2.html), [Parrot Bebop](https://docs.px4.io/en/flight_controller/bebop.html)  | X | | 
**시뮬레이션:** [jMAVSim SITL](../simulation/jmavsim.md) | X | X | X
**시뮬레이션:** [Gazebo SITL](../simulation/gazebo.md) | X | X | 
**시뮬레이션:** [ROS with Gazebo](../simulation/ros_interface.md) | X | | 


## 개발 환경

개발 환경 설치는 아래를 참조하세요.

  * [Mac OS](../setup/dev_env_mac.md)
  * [Linux](../setup/dev_env_linux.md)
  * [Windows](../setup/dev_env_windows.md)

Docker에 익숙하다면, 미리 준비된 컨테이너들 중 하나를 사용할 수 있습니다: [Docker Containers](../test_and_ci/docker.md)
