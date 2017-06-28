# 시뮬레이션과 ROS 인터페이스

시뮬레이션 autopilot은 포트 14557로 2번째 MAVLink 인터페이스를 구동시킵니다. MAVROS를 이 포트로 연결은 실제 비행에서는 비행체가 외부로 전달하는 모든 데이터를 받는 것과 같습니다.

## MAVROS 런칭

만약 ROS에 인터페이스를 원하면, 이미 실행중인 2번째 MAVLink 인스턴스는 [mavros](../ros/mavros_offboard.md)를 통해 ROS에 연결할 수 있습니다. 특정 IP에(`fcu_rul`는 SITL의 IP / port) 연결하기 위해서, 다음 형태로 URL을 사용합니다:

<div class="host-code"></div>

```sh
roslaunch mavros px4.launch fcu_url:="udp://:14540@192.168.1.36:14557"
```

localhost에 연결하기위해 다음 URL을 사용:

<div class="host-code"></div>

```sh
roslaunch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14557"
```


## ROS를 위한 Gazebo 설치

Gazebo ROS SITL 시뮬레이션은 Gazebo 6 과 Gazebo 7와 동작한다고 알려져 있습니다. 다음처럼 설치할 수 있습니다 :

```sh
sudo apt-get install ros-$(ROS_DISTRO)-gazebo7-ros-pkgs    //Recommended
```
or
```sh
sudo apt-get install ros-$(ROS_DISTRO)-gazebo6-ros-pkgs
```

## ROS wrapper로 Gazebo 런칭

ROS topic에(Gazebo ROS laser 플러그인) 직접 publish하기 위해 센서를 통합하는 Gazebo 시뮬레이션을 수정하고자 한다면, Gazebo는 반드시 적당한 ROS wrapper로 런치해야만 합니다.

ROS launch 스크립트는 ROS에 포함해서 시뮬레이션을 실행하는 것이 가능 :

  * [posix_sitl.launch](https://github.com/PX4/Firmware/blob/master/launch/posix_sitl.launch): plain SITL launch
  * [mavros_posix_sitl.launch](https://github.com/PX4/Firmware/blob/master/launch/mavros_posix_sitl.launch): SITL 와 MAVROS

ROS로 SITL을 포함시킬려면 ROS 환경의 업데이트가 필요합니다. 그런 다음 런치합니다:

(선택): 컴파일된 MAVROS나 소스에서 다른 ROS 패키지를 컴파일 한다면 catkin workspace를 기본으로 합니다.

```sh
cd <Firmware_clone>
make posix_sitl_default gazebo
source ~/catkin_ws/devel/setup.bash    // (optional)
source Tools/setup_gazebo.bash $(pwd) $(pwd)/build_posix_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/sitl_gazebo
roslaunch px4 posix_sitl.launch
```

위에 언급한 launch 파일 중에 하나를 launch 파일에 포함해서 시뮬레이션에서 ROS 어플리케이션을 실행합니다.

### 실제 동작

(혹은 수동으로 실행하는 방법)

```sh
no_sim=1 make posix_sitl_default gazebo
```

시뮬레이션을 구동시키고 콘솔은 다음과 같이 표시됩니다.


```sh
[init] shell id: 46979166467136
[init] task name: px4

______  __   __    ___
| ___ \ \ \ / /   /   |
| |_/ /  \ V /   / /| |
|  __/   /   \  / /_| |
| |     / /^\ \ \___  |
\_|     \/   \/     |_/

Ready to fly.


INFO  LED::init
729 DevObj::init led
736 Added driver 0x2aba34001080 /dev/led0
INFO  LED::init
742 DevObj::init led
INFO  Not using /dev/ttyACM0 for radio control input. Assuming joystick input via MAVLink.
INFO  Waiting for initial data on UDP. Please start the flight simulator to proceed..
```

이제 새로운 터미널에서 Gazebo 메뉴를 통해 Iris 모델을 삽입할 수 있고 적절한 `sitl_gazebo` 폴더에 환경 변수를 설정할 수 있습니다.

```sh
cd <Firmware_clone>
source Tools/setup_gazebo.bash $(pwd) $(pwd)/build_posix_sitl_default
```

ROS와 동작할 때와 Iris 쿼드콥터 모델을 삽입할때처럼 이제 Gazebo를 구동시킵니다. 일단 Iris가 로드되면 자동으로 px4 app에 연결될 것입니다.

```sh
roslaunch gazebo_ros empty_world.launch world_name:=$(pwd)/Tools/sitl_gazebo/worlds/iris.world
```
