# 파일과 코드 설치하기

PX4 코드는 [Mac OS](../setup/dev_env_mac.md), [Linux](../setup/dev_env_linux.md)나 [Windows](../setup/dev_env_windows.md) 에서 개발할 수 있습니다. 이미지 프로세싱과 고급 네비게이션은 윈도우에서 쉽게 개발이 어려우므로 Mac OS와 Linux를 추천합니다. 새로 시작하는 개발자는 기본적으로 Linux로 하고 현재 버전은 [Ubuntu LTS edition](https://wiki.ubuntu.com/LTS)입니다.


## Supported Targets {#supported-targets}

The table below show what PX targets you can build on each OS.

Target | Linux (Ubuntu) | Mac | Windows
--|:--:|:--:|:--:
**NuttX based hardware:** [Pixhawk Series](https://docs.px4.io/en/flight_controller/pixhawk_series.html), [Crazyflie](https://docs.px4.io/en/flight_controller/crazyflie2.html), [Intel® Aero Ready to Fly Drone](https://docs.px4.io/en/flight_controller/intel_aero.html) | X | X | X
[Qualcomm Snapdragon Flight hardware](https://docs.px4.io/en/flight_controller/snapdragon_flight.html) | X | | 
**Linux-based hardware:** [Raspberry Pi 2/3](https://docs.px4.io/en/flight_controller/raspberry_pi_navio2.html), [Parrot Bebop](https://docs.px4.io/en/flight_controller/bebop.html)  | X | | 
**Simulation:** [jMAVSim SITL](../simulation/jmavsim.md) | X | X | X
**Simulation:** [Gazebo SITL](../simulation/gazebo.md) | X | X | 


## 개발환경

개발환경 설치는 아래에서 다루고 있습니다 :

  * [Mac OS](../setup/dev_env_mac.md)
  * [Linux](../setup/dev_env_linux.md)
  * [Windows](../setup/dev_env_windows.md)

Docker에 익숙하다면 준비한 컨테이너 중에 하나를 사용할 수도 있습니다 : [Docker Containers](../test_and_ci/docker.md)

완료되면 [build 방법](../setup/building_px4.md)을 이어서 진행합니다.
