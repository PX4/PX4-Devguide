# Gazebo 시뮬레이션

[Gazebo](http://gazebosim.org)는 자동 로못을 위한 3D 시뮬레이션 환경입니다. ROS없이 독립적으로 사용하거나 SITL + ROS를 지원합니다.

{% youtube %}https://www.youtube.com/watch?v=qfFF9-0k4KA&vq=hd720{% endyoutube %}


{% mermaid %}
graph LR;
  Gazebo-->Plugin;
  Plugin-->MAVLink;
  MAVLink-->SITL;
{% endmermaid %}

## 설치

Gazebo와 시뮬레이션 플러그인 설치가 필요합니다.

> ** Note ** Gazebo 버전 7을 추천합니다.(최소 Gazebo 6 버전) Linux을 실행하고 Jade보다 이전 ROS 버전이 설치되어 있다면 예전 버전인 번들 Gazebo를 언인스톨해야 합니다.(sudo apt-get remove ros-indigo-gazebo)

### Mac OS

Mac OS가 Gazebo 7을 필요로하고 다음으로 xquartz 필요하며 OpenCV 없이 실행할 수 없습니다.

```sh
brew cask install xquartz
brew install homebrew/science/opencv
brew install gazebo7
```

### Linux

PX4 SITL은 Gazebo 시뮬레이터를 사용하며 ROS에 의존하지 않습니다. 시뮬레이션은 [ROS로 인터페이스](../simulation/ros_interface.md)될 수 있고 동일한 방식으로 일반 flight 코드에도 적용될 수 있습니다.

#### ROS 사용자

PX4를 ROS와 사용할려고 한다면, 다음 ROS를 위한 [Gazebo 버전 7 가이드](http://gazebosim.org/tutorials?tut=ros_wrapper_versions#Gazebo7.xseries)를 참고하세요.

#### 일반 설치

Gazebo 7용 [Linux 설치 방법](http://gazebosim.org/tutorials?tut=install_ubuntu&ver=7.0&cat=install)을 참고하세요.

모두 설치되었는지 확인 : `gazebo7` and `libgazebo7-dev`.

## 시뮬레이션 실행

PX4 펌웨어의 소스 디렉토리내에서 airframe 중에 하나로(Quads, planes, VTOL을 지원하며 optical flow 포함) PX4 SITL을 실행합니다 :

> **Note** Gazebo 실행을 유지하면서 PX4를 재실행하려면 아래와 같이 합니다.

### Quadrotor

```sh
cd ~/src/Firmware
make posix_sitl_default gazebo
```

### Quadrotor with Optical Flow

```sh
make posix gazebo_iris_opt_flow
```

### 3DR Solo

```sh
make posix gazebo_solo
```

![](../../assets/gazebo/solo.png)

### 표준 Plane

```sh
make posix gazebo_plane
```

![](../../assets/gazebo/plane.png)

### 표준 VTOL

```sh
make posix_sitl_default gazebo_standard_vtol
```

![](../../assets/gazebo/standard_vtol.png)

### Tailsitter VTOL

```sh
make posix_sitl_default gazebo_tailsitter
```

![](../../assets/gazebo/tailsitter.png)

## World 바꾸기

현재 기본 world는 iris.wold로 [worlds](https://github.com/PX4/sitl_gazebo/tree/367ab1bf55772c9e51f029f34c74d318833eac5b/worlds) 디렉토리에 있습니다. iris.world에 기본 환경에서 ground로 heightmap을 사용합니다. 이 ground로는 거리 센서 사용이 어렵습니다. heightmap으로 예상치 못한 결과가 나오면 iris.model에 있는 uneven_ground를 asphalt_plane으로 model을 변경하는 것을 추천합니다.

## 하늘로 날리기

> ** Note ** 에러가 발생하면 [파일과 코드 설치](../setup/dev_env_mac.md) 가이드를 참고하세요.

PX4 쉘이 나타납니다:

```sh
[init] shell id: 140735313310464
[init] task name: px4

______  __   __    ___
| ___ \ \ \ / /   /   |
| |_/ /  \ V /   / /| |
|  __/   /   \  / /_| |
| |     / /^\ \ \___  |
\_|     \/   \/     |_/

px4 starting.


pxh>
```

> ** Note ** Right-clicking the quadrotor model allows to enable follow mode from the context menu, which is handy to keep it in view.

![](../../assets/sim/gazebo.png)

The system will print the home position once it finished intializing (`telem> home: 55.7533950, 37.6254270, -0.00`). You can bring it into the air by typing:

```sh
pxh> commander takeoff
```

> ** Note ** Joystick or thumb-joystick support is available through QGroundControl (QGC). To use manual input, put the system in a manual flight mode (e.g. POSCTL, position control). Enable the thumb joystick from the QGC preferences menu.

## Set custom takeoff location

The default takeoff location in SITL Gazebo can be overridden using environment variables.

The variables to set are: `PX4_HOME_LAT`, `PX4_HOME_LON`, and `PX4_HOME_ALT`.

As an example:
```
export PX4_HOME_LAT=28.452386
export PX4_HOME_LON=-13.867138
export PX4_HOME_ALT=28.5
make posix gazebo
```

## Starting Gazebo and PX4 separately

For extended development sessions it might be more convenient to start Gazebo and PX4 separately or even from within an IDE.

In addition to the existing cmake targets that run `sitl_run.sh` with parameters for px4 to load the correct model it creates a launcher targets named `px4_<mode>` that is a thin wrapper around original sitl px4 app. This thin wrapper simply embeds app arguments like current working directories and the path to the model file.

### How to use it

  * Run gazebo (or any other sim) server and client viewers via the terminal:
```
make posix_sitl_default gazebo_none_ide
```
  * In your IDE select `px4_<mode>` target you want to debug (e.g. `px4_iris`)
  * Start the debug session directly from IDE
This approach significantly reduces the debug cycle time because simulator (e.g. gazebo) is always running in background and you only re-run the px4 process which is very light.

## Extending and Customizing

To extend or customize the simulation interface, edit the files in the `Tools/sitl_gazebo` folder. The code is available on the [sitl_gazebo repository](https://github.com/px4/sitl_gazebo) on Github.

> ** Note ** The build system enforces the correct GIT submodules, including the simulator. It will not overwrite changes in files in the directory.

## Interfacing to ROS

The simulation can be [interfaced to ROS](../simulation/ros_interface.md) the same way as onboard a real vehicle.
