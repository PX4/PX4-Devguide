# 리눅스 개발환경

리눅스에서는 [모든 PX4 타겟들](../setup/dev_env.md#supported-targets) (NuttX based hardware, Qualcomm Snapdragon Flight hardware, Linux-based hardware, Simulation, ROS)에 대해서 빌드가 가능합니다.

> **Tip** 지원하는 리눅스 배포판으로 Debian / [Ubuntu Linux LTS](https://wiki.ubuntu.com/LTS) (16.04)을 표준으로 합니다. [boutique 배포판](../setup/dev_env_linux_boutique.md) (Cent OS and Arch Linux)에 관련된 설치에 대해서도 설명합니다.

다음은 Ubuntu LTS에서 간편하게 bash 스크립트를 이용한 개발환경 셋업방법을 설명합니다. *수동 설치* 와 추가 타겟에 관련된 내용은 [Ubuntu/Debian Linux](../setup/dev_env_linux_ubuntu.md)을 참고하세요.


## 개발 툴체인

아래에서는 Ubuntu LTS에서 bash 스크립트를 사용해서 개발 툴체인을 설치하는 방법을 설명합니다. 모든 스크립트는 *Qt Creator IDE*, [Ninja Build System](https://ninja-build.org/), [Common Dependencies](../setup/dev_env_linux_ubuntu.md#common-dependencies), [FastRTPS](../setup/dev_env_linux_ubuntu.md#fastrtps-installation) 를 포함하며 여러분 컴퓨터의 (**~/src/Firmware**)에 PX4 소스코드를 다운받습니다.

> **Tip** 스크립트는 Ubuntu LTS 16.04 설치에서 테스트하였습니다. 기본 시스템에 사용하거나 다른 Ubuntu 릴리즈 버전을 설치했다면 동작하지 않을 수도 있습니다. 만약 문제가 발생하면 [수동 설치](../setup/dev_env_linux_ubuntu.md)를 참고하세요.

먼저 user를 "dialout" 그룹의 멤버로 설정합니다.
1. 커맨드 프롬프트로 진입:
   ```sh
   sudo usermod -a -G dialout $USER
   ```
1. 로그아웃 후 다시 로그인 (새로 로그인을 해야 변경내용이 적용됨).

아래 섹션에서는 여러분의 개발 타겟에 따라 설명합니다.

### Pixhawk/NuttX와 jMAVSim/Gazebo 시뮬레이션

개발 툴체인 설치하기:

1. <a href="https://raw.githubusercontent.com/PX4/Devguide/master/build_scripts/ubuntu_sim_nuttx.sh" target="_blank" download>ubuntu_sim_nuttx.sh</a> 다운로드.
1. bash 쉘에서 해당 스크립트 실행:
   ```bash
   source ubuntu_sim_nuttx.sh
   ```
   스크립트 진행되면서 프롬프트가 나타날 수 있습니다.
1. 완료되면 컴퓨터를 재시작합니다.


### Snapdragon Flight 혹은 Raspberry Pi

개발 툴체인 설치하기:
1. <a href="https://raw.githubusercontent.com/PX4/Devguide/master/build_scripts/ubuntu_sim.sh" target="_blank" download>ubuntu_sim.sh</a> 다운로드 (시뮬레이터와 공통 툴체인 관련 내용이 포함).
1. bash 쉘에서 해당 스크립트 실행:
   ```bash
   source ubuntu_sim.sh
   ```
   스크립트 진행되면서 프롬프트가 나타날 수 있습니다.
1. [Ubuntu/Debian Linux](../setup/dev_env_linux_ubuntu.md)에서 플랫폼에 따라 해당 지시를 따릅니다:
   * [Snapdragon Flight](../setup/dev_env_linux_ubuntu.md#snapdragon-flight)
   * [Raspberry Pi](../setup/dev_env_linux_ubuntu.md#raspberry-pi-hardware)

### Parrot Bepop

(수동) 절차는 다음을 참고하세요: [Ubuntu/Debian Linux > Parrot Bebop](../setup/dev_env_linux_ubuntu.md#raspberry-pi-hardware)


### ROS의 Gazebo

개발 툴체인 설치하기:

1. <a href="https://raw.githubusercontent.com/PX4/Devguide/master/build_scripts/ubuntu_sim_ros_gazebo.sh" target="_blank" download>ubuntu_sim_ros_gazebo.sh</a> 다운로드.
1. bash 쉘에서 해당 스크립트 실행:
   ```bash
   source ubuntu_sim_ros_gazebo.sh
   ```
   스크립트 진행되면서 프롬프트가 나타날 수 있습니다.

Note:
* ROS는 기본으로 Gazebo7와 함께 설치됩니다.(ROS 개발을 단순화시키려면 Gazebo8보다는 기본 버전을 사용)
* 여러분의 catkin (ROS 빌드 시스템) workspace는 **~/catkin_ws/**에 생성됩니다.


<!-- Additional DevTools Common to all Platforms -->

## Ground Control 소프트웨어

[QGroundControl Daily Build](https://docs.qgroundcontrol.com/en/releases/daily_builds.html)을 다운로드 받고 설치하기

![QGroundControl](../../assets/qgc_goto.jpg)


## 편집기 / IDE

개발팀이 사용하는 환경:

* [Sublime Text](https://www.sublimetext.com): 빠르고 가벼운 편집기
* [Qt Creator](http://www.qt.io/download-open-source/#section-6): 유명한 오픈소스 IDE
  > **Note** 설치 스크립트는 자동으로 *Qt Creator* 를 설치합니다. bash 터미널에서 `qtcreator`를 입력하면 실행할 수 있습니다.

## 다음 단계

일단 환경설정을 마치면, [build 방법](../setup/building_px4.md)으로 넘어가세요.
