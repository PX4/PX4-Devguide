
# MAVROS

[mavros](http://wiki.ros.org/mavros#mavros.2BAC8-Plugins.sys_status) ros 패키지는 ROS가 실행중인 컴퓨터와 MAVLink가 활성화된 autopilot사이에 MAVLink 확장 통신을 가능하게 합니다. MAVRos가 MAVLink가 활성화된 어떤 autopilot과도 통신이 가능하지만 이 문서에서는 PX4 flight stack과 ROS 컴패니온 컴퓨터 사이에 통신에 대해서 내용을 다룹니다.

## 설치

MAVRos는 소스나 바이너리로 설치할 수 있습니다. ROS를 작업하는 개발자는 소스 설치를 사용하는 것을 추천합니다.

### 바이너리 설치 (Debian / Ubuntu)

x86과 amd64 (x86\_64)용으로 미리 컴퍼알된 debian 패키지에서 사용가능한 프로그램은 v0.5 이후부터입니다.
Ubuntu armhf용은 ARMv7 repo에 있는 v0.9+가 있습니다.
설치를 할려면 `apt-get`를 사용:
```sh
$ sudo apt-get install ros-indigo-mavros ros-indigo-mavros-extras
```

### 소스 설치
**의존성**

이 설치 과정에서 catkin workspace는 `~/catkin_ws`에 위치한다고 가정합니다. 다음과 같이 생성:
```sh
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws
$ catkin init
```

설치에 다음과 같은 `wstool, rosinstall,and catkin_tools` ROS python 툴을 사용합니다. ROS 설치하는 동안에 이미 설치되었을 수도 있습니다. 다음과 같이 설치:
```sh
$ sudo apt-get install python-wstool python-rosinstall-generator python-catkin-tools
```

패키지는 catkin_make를 사용해서 빌드할 수 있지만 catkin_tools을 사용하는 것을 선호합니다. 기능도 많고 친근한 빌드 도구입니다.

여러분이 처음으로 wstool을 사용하는 것이라면 소스 공간을 초기화:
```sh
$ wstool init ~/catkin_ws/src
```

이제 빌드할 준비가 되었습니다.
```sh
    # 1. get source (upstream - released)
$ rosinstall_generator --upstream mavros | tee /tmp/mavros.rosinstall
    # alternative: latest source
$ rosinstall_generator --upstream-development mavros | tee /tmp/mavros.rosinstall

    # 2. get latest released mavlink package
    # you may run from this line to update ros-*-mavlink package
$ rosinstall_generator mavlink | tee -a /tmp/mavros.rosinstall

    # 3. Setup workspace & install deps
$ wstool merge -t src /tmp/mavros.rosinstall
$ wstool update -t src
$ rosdep install --from-paths src --ignore-src --rosdistro indigo -y

    # finally - build
$ catkin build
```

> **Note** mavros을 라즈베리파이에 설치한다면 "rosdep install ..."을 실행할 때, os에 관련된 에러가 발생할 수 있습니다. "--os=OS_NAME:OS_VERSION "을 rosdep 명령에 추가하고 OS_NAME을 여러분의 OS 이름으로 변경하고 OS_VERSION도 알맞는 버전을 사용하도록 합니다.(예제 --os=debian:jessie)
