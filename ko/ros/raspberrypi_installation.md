# 라즈베리 파이 - ROS 설치

픽스호크용 보조 컴퓨터로 활약할 라즈베리 파이 2에 ROS-indigo를 설치하는 방법을 안내합니다.

## 준비 요건

* 모니터, 키보드, SSH 설정 연결로 동작하는 라즈베리 파이
* 이 안내서는 라즈베리 파이에 라즈비안 "Jessie"를 설치했음을 가정합니다. 만일 이를 설치하지 않았다면 [이걸 받아서 설치](https://www.raspberrypi.org/downloads/raspbian/) 하거나 라즈비안 Wheezy에서 Jessie로 [업그레이드](http://raspberrypi.stackexchange.com/questions/27858/upgrade-to-raspbian-jessie)하십시오.

## 설치

[이 안내서](http://wiki.ros.org/ROSberryPi/Installing%20ROS%20Indigo%20on%20Raspberry%20Pi)를 따라 ROS Indigo의 실제 설치 과정을 진행하십시오. Note: "ROS-Comm" 변형 버전을 설치하십시오. 데스크톱 버전은 너무 무겁습니다.

### 패키지 설치 중 오류

패키지를 다운로드(예: `sudo apt-get install ros-indigo-ros-tutorials`)할 경우, "unable to locate package ros-indigo-ros-tutorials" 오류를 만날 수 있습니다.

이렇게 뜬다면 다음 절차를 따르십시오: catkin 작업 영역으로 이동(예: ~/ros_catkin_ws)한 후, 패키지의 이름을 바꾸십시오.

```sh
$ cd ~/ros_catkin_ws

$ rosinstall_generator ros_tutorials --rosdistro indigo --deps --wet-only --exclude roslisp --tar > indigo-custom_ros.rosinstall
```

그 다음 wstool로 작업 영역을 업데이트하십시오.

```sh
$ wstool merge -t src indigo-custom_ros.rosinstall

$ wstool update -t src
```

다음(여전히 작업 영역 폴더에 있는 상황에서), 모든 환경 설정 변수 값을 적용하고 소스 코드 파일을 빌드하십시오.

```sh
$ source /opt/ros/indigo/setup.bash

$ source devel/setup.bash

$ catkin_make
```