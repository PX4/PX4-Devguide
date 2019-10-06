# RTPS/ROS2 인터페이스: PX4-FastRTPS 브릿지

*PX4-FastRTPS 브릿지*는 PX4 컴포넌트와 *Fast RTPS* 프로그램(ROS2/ROS 프레임워크를 사용하여 빌드한 것 포함)간의 [uORB messages](../middleware/uorb.md)를 교환하는 RTPS(Real Time Publish Subscribe) 인터페이스를 PX4에 추가했다.

> **Note** RTPS는 OMG(Object Management Group)의 DDS(Data Distribution Service) 표준의 기본 프로토콜입니다. 이 프로토콜은 Pub/Sub을 사용하여 확장성, 실시간성, 의존성, 고성능, 상호교환성을 만족합니다. *Fast RTPS*는 최신 RTPS와 최소한의 DDS API를 크로스플랫폼 환경에서 경량화해 구현한 것입니다.

RTPS는 ROS2(Robot Operating System)의 미들웨어로 채택되었습니다. *Fast RTPS 브릿지*는 센서의 값, 명령어 그리고 다른 기체의 정보를 공유하기 쉽게하는 ROS2와의 통합을 용이하게 합니다.

이 토픽에서는 RTPS 브릿지 구조에 대해 설명할 것입니다. 그리고 그것이 ROS2/ROS 어플리케이션 파이프라인내에서 어떻게 사용되는지 살펴볼 것입니다. 그리고 필요한 코드를 어떻게 컴파일할지도 알아볼 것입니다.

1. PX4의 변화를 구독하기 위한 간단한 *Fast RTPS* 어플리케이션 작성하기
2. ROS2 노드들과 PX4 연결하기(RTPS 브릿지를 통해 연결되며 `px4_ros_com` 패키지를 사용함)
3. ROS (version 1) 노드들과 PX4를 연결하기, 추가적으로 ROS2와 ROS를 연결하기 위해`ros1_bridge` 패키지를 사용하기.

## RTPS는 언제 사용되어야 하나요?

비행 조종기와 오프보드 컴포넌의 간의 정보를 확실히 time-critical/real-time하게 공유하고자 한다면 RTPS가 사용되어야 합니다. 특히 오프보드 소프트웨어가 PX4에서 구동되는 소프트웨어 컴포넌트(uORB 토픽들을 송수신)들의 *peer*가 되어야 할 때 유용합니다.

사용가능한 경우는, 컴퓨터 비전을 위한 로보틱스 라이브러리들간의 통신, 기체 조종을 위한 액추에이터와 센서간의 실시간 데이터 통신입니다.

> **Note** *Fast RTPS*는 MAVLink를 대체하기 위한 목적이 아닙니다. MAVLink는 GS, 짐벌, 카메라 그 외의 오프보드 컴포넌트들과의 통신을 위해 알맞은 프로토콜입니다.

<span></span>

> **Tip** RTPS는 느린 채널(e.g. radio telemetry)에도 사용될 수 있지만 대역폭을 넘기지 않도록 조심해야 합니다.

## 아키텍쳐 개요

### RTPS 브릿지

RTPS브릿지는 [uORB](../middleware/uorb.md)와 RTPS 메시지를 매끄럽게 변환하여 PX4와 RTPS 어플리케이션들간의 메시지를 교환합니다.

![basic example flow](../../assets/middleware/micrortps/architecture.png)

구조의 주된 요소들은 위에보이는 클라이언트와 에이전트 프로세스입니다.

* *Client*는 flight controller에서 실행되는 PX4 미들웨어 데몬입니다. 다른 PX4 컴포넌트들이 퍼블리시하는 토픽들을 구독하고, *Agent*로 업데이트를 보냅니다(UART 또는 UDP 포트). *Agent*로 부터 메시지도 받으며 PX4로 uORB 메시지를 퍼블리시하기도 합니다.
* *Agent*는 offboard computer에서 데몬으로 실행됩니다. *Client*의 업데이트 메시지를 검사하고 그것들을 RTPS를 통해 퍼블리시합니다. RTPS 어플리케이션으로부터 오는 "uORB" RTPS 메시지들 또한 구독하며 *Client*에 전달합니다.
* *Agent*과 *Client*는 시리얼링크(UART)나 UDP 네트워크로 연결됩니다. uORB 정보는 전송을 위해 [CDR serialized](https://en.wikipedia.org/wiki/Common_Data_Representation)됩니다. *CDR serialization*는 다른 플랫폼들과 시리얼 데이터 교환을 위한 일반적인 포맷을 제공합니다.
* *Agent*와 *Fast RTPS* 어플리케이션은 UDP를 통해 연결되며, 다른 장치에 있을수도 있습니다. 일반적인 구성에서는 와이파이나 USB로 *Client*에 연결된 동일한 시스템에 있을 것입니다 (예. 개발 컴퓨터, 리눅스 컴퓨터 또는 컴퓨터 보드).

### ROS2/ROS 어플리케이션 파이프라인

ROS2를 위한 어플리케이션 파이프라인은 아주 직관적입니다. ROS2는 기본적인 통신 미들웨어로 DDS/RTPS를 사용하기 때문에, *PX4 Fast RTPS 브릿지*를 통해 PX4에 퍼블리시하거나 구독하는 ROS2 리스너나 애드벌타이저를 만들 수 있습니다. 아래의 그림과 같습니다.

> **Note** 클라이언트와 에이전트에서 사용되는 메시지 타입, 헤더, 소스 파일들이 동일한 IDL(Interface Description Language) 파일에서 생성됐는지 확실히 해야합니다. `px4_ros_com` 패키지는 ROS2가 필요로 하는 메시지나 헤더들을 생성하기 위한 인프라를 제공합니다.

![Architecture with ROS2](../../assets/middleware/micrortps/architecture_ros2.png)

ROS 어플리케이션과 PX4를 통합하기 위한 구조는 아래에 나왔습니다.

![Architecture with ROS](../../assets/middleware/micrortps/architecture_ros.png)

ROS2와 ROS간의 메시지를 연결하는 [ros1_bridge](https://github.com/ros2/ros1_bridge) 사용에 주의하세요. ROS의 첫번째 버전은 RTPS를 지원하기 않기 때문에 필요합니다.

## 코드 생성

> **Note** 코드를 생성하기 위해서는 [Fast RTPS 가 반드시 설치](../setup/fast-rtps-installation.md) 되어야 합니다. *Fast RTPS* 는 여러분이 [macOS](../setup/dev_env_mac.md), [Windows Cygwin](../setup/dev_env_windows_cygwin.md), [Ubuntu](../setup/dev_env_linux_ubuntu.md)를 위한 일반적인 인스톨러/스크립트를 사용했다면 *기본적*으로 설치됩니다.

### ROS에 독립적인 어플리케이션

브릿지를 만들고, 빌드하고, 사용하기 위한 모든 코드들은 PX4 펌웨어가 컴파일될 때 자동적으로 생성됩니다.

*Client* 어플리케이션의 컴파일, 빌드 또한 일반적인 빌드 과정에서 처리됩니다. *Agent*는 타켓 컴퓨터에 맞게 독립적, 수동적으로 컴파일 되어야합니다.

<span></span>

> **Tip** 브릿지 코드 또한 [수동적으로 생성](micrortps_manual_code_generation.md)될 수 있습니다. 대부분의 사용자들은 그럴 필요가 없습니다. 자세한 빌드 과정과 트러블슈팅을 위해서는 링크를 참고하세요.

### ROS2/ROS 어플리케이션 {#px4_ros_com}

빌드될 때 [px4_ros_com](https://github.com/PX4/px4_ros_com) 패키지는 ROS2 노드에서 PX4 uORB 메시지에 접근하기 위한 모든 것을 생성합니다(ROS일 경우 [ros1_bridge](https://github.com/ros2/ros1_bridge)가 필요할 것입니다). `micrortps_agent`와 `micrortps_agent`에 필요한 IDL 파일을 포함해서 *PX4 RTPS bridge*를 위한 모든 컴포넌트를 포함합니다.

ROS, ROS2의 메시지 정의 헤더와 인터페이스는 [px4_msgs](https://github.com/PX4/px4_msgs) 패키지에서 생성되며 PX4 펌웨어의 uORB 메시지와 일치합니다. 이 파일들은 `px4_ros_com`가 `micrortps_agent`가 사용하는 IDL 파일을 생성하기 위해 필요로합니다.

`px4_ros_com`와 `px4_msgs`는 2개의 독립된 브랜치를 갖고 있습니다.

* ROS2에 사용되는 `master` 브랜치 이 브랜치는 PX4와 ROS2 노드를 연결하기 위한 ROS2 메시지와 IDL 파일을 생성하는 코드를 포함합니다.
* ROS를 위한 `ros1` 브랜치 이 브랜치는 PX4와 ROS간의 데이터를 공유하기 위해 사용되는 `ros1_bridge`에 사용되는 ROS 메시지 헤더와 소스파일을 생성하는 코드를 포함합니다.

`px4_ros_com`의 두 브랜치 모두 리스너와 애드벌타이저 예제를 포함합니다.

## 지원하는 uORB 메시지

생성된 브릿지 코드는 특정 토픽들에 대해 RTPS를 통해 Pub/Sub이 가능하도록 합니다. ROS와 non-ROS 어플리케이선 모두 해당됩니다.

*자동 코드 생성*을 위해 **Firmware/msg/tools/** 디렉토리에 **uorb_rtps_message_ids.yaml** 파일이 있습니다(*yaml*). 이 파일은 RTPS에 사용될 uORB 메시지의 집합을 정의 합니다. 메시지의 송, 수신 여부와 DDS/RTPS 미들웨어에 사용될 RTPS ID를 정의합니다.

> **Note** 모든 메시지들에 대해 RTPS ID가 설정해야 합니다.

```yaml
rtps:
  - msg: actuator_armed
    id: 0
  - msg: actuator_control
    id: 1
  - ...
  - msg: airspeed
    id: 5
    send: true
  - msg: battery_status
    id: 6
    send: true
  - msg: camera_capture
    id: 7
  - msg: camera_trigger
    id: 8
    receive: true
  - ...
  - msg: sensor_baro
    id: 63
    receive: true
    send: true
```

> **Note** `px4_msgs`는 빌드 과정에서 ROS2 메시지 헤더 파일을 생성하기 위해, CMake 매크로 `rosidl_generate_interfaces()를 사용합니다. 반면에 <code>px4_ros_com`는 `rosidl_generate_dds_interfaces()` CMake 매크로를 통해 IDL 파일을 생성합니다. PX4 펌웨어는 빌드과정에서 사용되는 IDL 파일 생성 템플릿을 포함합니다.
> 
> `px4_ros_com`는 ROS/ROS2에 사용되는 *약간 다른* IDL 파일을 생성합니다. **uorb_rtps_message_ids.yaml**는 *PascalCased*방식으로 메시지 이름을 짓습니다(이름을 바꾸는 것은 client-agent 통신과는 상관없지만 ROS2에는 크리티컬합니다, 따라서 메시지 네이밍은 PascalCase 컨벤션을 따라야합니다). 새로운 IDL 파일들은 송수신한 메시지들을 다시보냅니다(왜냐하면 클라이언트에서 메시지를 보내고 에이전트 에서 수신하거나 반대의 경우를 위해).

## 클라이언트 (PX4 펌웨어) {#client_firmware}

*Client* 소스코드는 일반적인 빌드 과정을 거치면서 생성, 컴파일, 빌드되어 PX4 펌웨어에 포함됩니다.

NuttX/Pixhawk 비행 컨트롤러의 펌웨어를 빌드하려면 설정 타켓으로 `_rtps`을 사용하세요. 예를 들어, RTPS를 px4_fmu-v4에 빌드하려면

```sh
make px4_fmu-v4_rtps
```

SITL 타켓을 위한 펌웨어를 빌드하려면

```sh
make px4_sitl_rtps
```

*Client* 어플리케이션은 [NuttShell/System Console](../debug/system_console.md)에서 실행할 수 있습니다. 명령어는 아래에 나와있습니다.

```sh
> micrortps_client start|stop [options]
  -t <transport>          [UART|UDP] Default UART
  -d <device>             UART device. Default /dev/ttyACM0
  -u <update_time_ms>     Time in ms for uORB subscribed topics update. Default 0
  -l <loops>              How many iterations will this program have. -1 for infinite. Default -1.
  -w <sleep_time_ms>      Time in ms for which each iteration sleep. Default 1ms
  -b <baudrate>           UART device baudrate. Default 460800
  -p <poll_ms>            Time in ms to poll over UART. Default 1ms
  -r <reception port>     UDP port for receiving. Default 2019
  -s <sending port>       UDP port for sending. Default 2020
```

> **Note** *Client*는 기본적으로 데몬으로 동작하지만, 수동으로 실행할 수도 있습니다. 나중에는 PX4 펌웨어 초기화 코드에서 *Client*를 자동적으로 영구적인 데몬으로 실행할 것입니다.

예를 들어 SITL에서 UDP를 통해 Agent에 연결하는 *Client*를 실행하기 위해서는 아래와 같이 실행해야 합니다.

```sh
micrortps_client start -t UDP
```

## Fast RTPS interface를 사용하는 ROS에 독립적인 오프보드 에이전트

*Agent* 코드는 PX4 펌웨어와 관련된 것을 빌드할 때 자동적으로 *생성*됩니다. 소스코드는 여기서 찾을 수 있습니다. **build/<target-platform>/src/modules/micrortps_bridge/micrortps_client/micrortps_agent/**.

*Agent* 어플리케이션을 필드하기 위해서는 코드를 컴파일 하세요.

```sh
cd build/<target-platform>/src/modules/micrortps_bridge/micrortps_client/micrortps_agent
mkdir build && cd build
cmake ..
make
```

> **Note** *Qualcomm Snapdragon Flight* 플랫폼을 위한 크로스 컴파일을 [여기](https://github.com/eProsima/PX4-FastRTPS-PoC-Snapdragon-UDP#how-to-use)를 참고하세요.

*Agent*를 위한 명령어들은 아래와 같습니다.

```sh
$ ./micrortps_agent [options]
  -t <transport>          [UART|UDP] Default UART.
  -d <device>             UART device. Default /dev/ttyACM0.
  -w <sleep_time_us>      Time in us for which each iteration sleep. Default 1ms.
  -b <baudrate>           UART device baudrate. Default 460800.
  -p <poll_ms>            Time in ms to poll over UART. Default 1ms.
  -r <reception port>     UDP port for receiving. Default 2019.
  -s <sending port>       UDP port for sending. Default 2020.
```

*Agent*를 실행하려면 `micrortps_agent`를 *Client*에 연결하기 위한 적절한 옵션을 주어 실행하세요(리눅스 디바이스는 기본적으로 UART 포트를 통해 *Client*에 연결합니다).

하나의 예로, UDP로 연결하는 *micrortps_agent*을 시작하기 위해서는

```sh
./micrortps_agent -t UDP
```

## Agent와 ROS2 미들웨어

`px4_ros_com`를 빌드할때 의존성 때문에 에이전트 어플리케이션을 생성하고 빌드합니다, `px4_msgs` 패키지또한 같은 ROS2 워크스페이스나 오버레이한 ROS2 스페이스에 빌드됩니다. [`colcon`](http://design.ros2.org/articles/build_tool.html) 빌드 툴을 사용할때도 설치되기 때문에, 위에 설명한 것과 완벽히 동일하게 동작합니다. 자세한 빌드 구조에 대한 것은 ** `px4_ros_com` 패키지 빌드하기 **를 참고하세요.

## `px4_ros_com`와 `px4_msgs` 패키지 빌드하기

개발용 컴퓨터에 ROS2와 ROS 환경을 설치하고 세팅하세요, 그리고 `px4_ros_com`와 `px4_msgs` 저장소를 `master`와 `ros1`브랜치에 독립적으로 클론하세요([더 자세한 정보는 여기를 보세요](#px4_ros_com)).

> **Note** ROS2는 마스터 브랜치만 필요합니다(ROS는 두 브랜치 다 필요합니다).

### ROS와 ROS2 설치와 의존성

ROS Melodic, ROS2 Crystal 또는 우분투 18.04 에서 Bouncy를 설치하려면 아래의 링크를 참고하세요.

1. [ROS Melodic 설치하기](http://wiki.ros.org/melodic/Installation/Ubuntu)
2. [ROS2 Crystal 설치하기](https://index.ros.org/doc/ros2/Linux-Install-Debians/)
3. 이 설치과정은 *colcon* 빌드 툴 설치가 필요할 것 입니다. 설치되지 않으면 수동으로 설치해주세요.
    
    ```sh
    sudo apt install python3-colcon-common-extensions
    ```

4. *setuptools* 또한 반드시 설치되어야 합니다. (*pip*나 *apt*를 사용하세요)
    
    ```sh
    sudo pip3 install -U setuptools
    ```
    
    > **Note** 이 설치 및 빌드 가이드는 ROS2 Ardent에서 더이상 유효하지 않습니다. 2018 12월에 지원이 중단되었습니다.
    
    <span></span>
    
    > **Caution** debian 저장소를 이용해 `ros1_bridge` 패키지를 설치하지 마세요. 이 패키지는 소스로 빌드해야 합니다.

### 워크스페이스 세팅하기

ROS와 ROS2가 다른 환경을 필요로 하기 때문에 각 ROS를 위한 워크스페이스를 분리할 필요가 있습니다. 예:

1. ROS2 워킹 스페이스는 다음과 같이 만드세요
    
    ```sh
    mkdir -p ~/px4_ros_com_ros2/src
    ```
    
    그리고 ROS2 (`master`) 브랜치를 `/src` 디렉토리에 클론하세요.
    
    ```sh
    $ git clone https://github.com/PX4/px4_ros_com.git ~/px4_ros_com_ros2/src/px4_ros_com # clones the master branch
    $ git clone https://github.com/PX4/px4_msgs.git ~/px4_ros_com_ros2/src/px4_msgs
    ```

2. ROS도 똑같지만, 다른 디렉토리를 생성해 다른 브랜치를 클론하세요.
    
    ```sh
    mkdir -p ~/px4_ros_com_ros1/src
    ```
    
    ROS2 (`ros1`) 브랜치를 `/src` 디렉토리에 클론하세요.
    
    ```sh
    $ git clone https://github.com/PX4/px4_ros_com.git ~/px4_ros_com_ros1/src/px4_ros_com -b ros1 # clones the 'ros1' branch
    $ git clone https://github.com/PX4/px4_msgs.git ~/px4_ros_com_ros1/src/px4_msgs -b ros1
    ```

### 워크스페이스 빌드하기

`px4_ros_com/scripts` 디렉토리는 두 워킹스페이스를 빌드하기위해 사용되는 여러개의 스크립트를 포함합니다.

두 워크스페이스를 하나의 스크립트로 빌드하기 위해서는 `build_all.bash`를 사용하세요. `source build_all.bash --help`로 사용법을 확인하세요. 사용하는 가장 일반적인 방법은 ROS 워크스페이스 디렉토리 경로와 PX4 펌웨어 디렉토리 경로를 전달하는 것입니다.

```sh
$ source build_all.bash --ros1_ws_dir <path/to/px4_ros_com_ros1/ws>
```

> **Note** The build process will open new tabs on the console, corresponding to different stages of the build process that need to have different environment configurations sourced.

One can also use the following individual scripts in order to build the individual parts:

* `build_ros1_bridge.bash`, to build the `ros1_bridge`;
* `build_ros1_workspace.bash` (only the `ros1` branch of `px4_ros_com`), to build the ROS1 workspace to where the `px4_ros_com` and `px4_msgs` `ros1` branches were cloned;
* `build_ros2_workspace.bash`, to build the ROS2 workspace to where the `px4_ros_com` and `px4_msgs` `master` branches were cloned;

The steps below show how to *manually* build the packages (provided for your information/better understanding only):

1. `cd` into `px4_ros_com_ros2` dir and source the ROS2 environment. Don't mind if it tells you that a previous workspace was set before:
    
    ```sh
    source /opt/ros/crystal/setup.bash
    ```

2. Clone the `ros1_bridge` package so it can be built on the ROS2 workspace:
    
    ```sh
    git clone https://github.com/ros2/ros1_bridge.git -b crystal ~/px4_ros_com_ros2/src/ros1_bridge
    ```

3. Build the `px4_ros_com` and `px4_msgs` packages, excluding the `ros1_bridge` package:
    
    ```sh
    colcon build --symlink-install --packages-skip ros1_bridge --event-handlers console_direct+
    ```
    
    > **Note** `--event-handlers console_direct+` only serves the purpose of adding verbosity to the `colcon` build process and can be removed if one wants a more "quiet" build.

4. Then, follows the process of building the ROS(1) packages side. For that, one requires to open a new terminal window and source the ROS(1) environment that has installed on the system:
    
    ```sh
    source /opt/ros/melodic/setup.bash
    ```

5. On the terminal of the previous step, build the `px4_ros_com` and `px4_msgs` packages on the ROS end:
    
    ```sh
    cd ~/px4_ros_com_ros1 && colcon build --symlink-install --event-handlers console_direct+
    ```

6. Before building the `ros1_bridge`, one needs to open a new terminal and then source the environments and workspaces following the order below:
    
    ```sh
    source ~/px4_ros_com_ros1/install/local_setup.bash
    source ~/px4_ros_com_ros2/install/local_setup.bash
    ```

7. Finally, build the `ros1_bridge`. Note that the build process may consume a lot of memory resources. On a resource limited machine, reduce the number of jobs being processed in parallel (e.g. set environment variable `MAKEFLAGS=-j1`). For more details on the build process, see the build instructions on the [ros1_bridge](https://github.com/ros2/ros1_bridge) package page.
    
    ```sh
    cd ~/px4_ros_com_ros2 && colcon build --symlink-install --packages-select ros1_bridge --cmake-force-configure --event-handlers console_direct+
    ```

### 워크스페이스 정리하기

After building the workspaces there are many files that must be deleted before you can do a clean/fresh build (for example, after you have changed some code and want to rebuild). Unfortunately *colcon* does not currently have a way of cleaning the generated **build**, **install** and **log** directories, so these directories must be deleted manually.

The **clean_all.bash** script (in **px4_ros_com/scripts**) is provided to ease this cleaning process. The most common way of using it is by passing it the ROS(1) workspace directory path (since it's usually not on the default path):

```sh
$ source clean_all.bash --ros1_ws_dir <path/to/px4_ros_com_ros1/ws>
```

## Creating a Fast RTPS Listener application

Once the *Client* (on the flight controller) and the *Agent* (on an offboard computer) are running and connected, *Fast RTPS* applications can publish and subscribe to uORB topics on PX4 using RTPS.

This example shows how to create a *Fast RTPS* "listener" application that subscribes to the `sensor_combined` topic and prints out updates (from PX4). A connected RTPS application can run on any computer on the same network as the *Agent*. For this example the *Agent* and *Listener application* will be on the same computer.

The *fastrtpsgen* script can be used to generate a simple RTPS application from an IDL message file.

> **Note** RTPS messages are defined in IDL files and compiled to C++ using *fastrtpsgen*. As part of building the bridge code, IDL files are generated for the uORB message files that may be sent/received (see **build/BUILDPLATFORM/src/modules/micrortps_bridge/micrortps_agent/idl/*.idl**). These IDL files are needed when you create a *Fast RTPS* application to communicate with PX4.

Enter the following commands to create the application:

```sh
cd /path/to/PX4/Firmware/src/modules/micrortps_bridge
mkdir micrortps_listener
cd micrortps_listener
fastrtpsgen -example x64Linux2.6gcc ../micrortps_agent/idl/sensor_combined_.idl
```

This creates a basic subscriber and publisher, and a main-application to run them. To print out the data from the `sensor_combined` topic, modify the `onNewDataMessage()` method in **sensor_combined_Subscriber.cxx**:

```c++
void sensor_combined_Subscriber::SubListener::onNewDataMessage(Subscriber* sub)
{
    // Take data
    sensor_combined_ st;

    if(sub->takeNextData(&st, &m_info))
    {
        if(m_info.sampleKind == ALIVE)
        {
            // Print your structure data here.
            ++n_msg;
            std::cout << "\n\n\n\n\n\n\n\n\n\n";
            std::cout << "Sample received, count=" << n_msg << std::endl;
            std::cout << "=============================" << std::endl;
            std::cout << "gyro_rad: " << st.gyro_rad().at(0);
            std::cout << ", " << st.gyro_rad().at(1);
            std::cout << ", " << st.gyro_rad().at(2) << std::endl;
            std::cout << "gyro_integral_dt: " << st.gyro_integral_dt() << std::endl;
            std::cout << "accelerometer_timestamp_relative: " << st.accelerometer_timestamp_relative() << std::endl;
            std::cout << "accelerometer_m_s2: " << st.accelerometer_m_s2().at(0);
            std::cout << ", " << st.accelerometer_m_s2().at(1);
            std::cout << ", " << st.accelerometer_m_s2().at(2) << std::endl;
            std::cout << "accelerometer_integral_dt: " << st.accelerometer_integral_dt() << std::endl;
            std::cout << "magnetometer_timestamp_relative: " << st.magnetometer_timestamp_relative() << std::endl;
            std::cout << "magnetometer_ga: " << st.magnetometer_ga().at(0);
            std::cout << ", " << st.magnetometer_ga().at(1);
            std::cout << ", " << st.magnetometer_ga().at(2) << std::endl;
            std::cout << "baro_timestamp_relative: " << st.baro_timestamp_relative() << std::endl;
            std::cout << "baro_alt_meter: " << st.baro_alt_meter() << std::endl;
            std::cout << "baro_temp_celcius: " << st.baro_temp_celcius() << std::endl;

        }
    }
}
```

To build and run the application on Linux:

```sh
make -f makefile_x64Linux2.6gcc
bin/*/sensor_combined_PublisherSubscriber subscriber
```

Now you should see the sensor information being printed out:

```sh
Sample received, count=10119
Received sensor_combined data
=============================
gyro_rad: -0.0103228, 0.0140477, 0.000319406
gyro_integral_dt: 0.004
accelerometer_timestamp_relative: 0
accelerometer_m_s2: -2.82708, -6.34799, -7.41101
accelerometer_integral_dt: 0.004
magnetometer_timestamp_relative: -10210
magnetometer_ga: 0.60171, 0.0405879, -0.040995
baro_timestamp_relative: -17469
baro_alt_meter: 368.647
baro_temp_celcius: 43.93
```

> **Note** If the *Listener application* does not print anything, make sure the *Client* is running.

## Creating a ROS2 listener

With the `px4_ros_com` built successfully, one can now take advantage of the generated *micro-RTPS* agent app and also from the generated sources and headers of the ROS2 msgs from `px4_msgs`, which represent a one-to-one matching with the uORB counterparts.

To create a listener node on ROS2, lets take as an example the `sensor_combined_listener.cpp` node under `px4_ros_com/src/listeners`:

```c++
#include <rclcpp/rclcpp.hpp>
#include <px4_ros_com/msg/sensor_combined.hpp>
```

The above brings to use the required C++ libraries to interface with the ROS2 middleware. It also includes the required message header file.

```c++
/**
 * @brief Sensor Combined uORB topic data callback
 */
class SensorCombinedListener : public rclcpp::Node
{
```

The above creates a `SensorCombinedListener` class that subclasses the generic `rclcpp::Node` base class.

```c++
public:
    explicit SensorCombinedListener() : Node("sensor_combined_listener") {
        auto callback =
        [this](const px4_ros_com::msg::SensorCombined::SharedPtr msg)->void
        {
            std::cout << "\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n";
            std::cout << "RECEIVED DATA ON SENSOR COMBINED" << std::endl;
            std::cout << "================================" << std::endl;
            std::cout << "gyro_rad[0]: " << msg->gyro_rad[0] << std::endl;
            std::cout << "gyro_rad[1]: " << msg->gyro_rad[1] << std::endl;
            std::cout << "gyro_rad[2]: " << msg->gyro_rad[2] << std::endl;
            std::cout << "gyro_integral_dt: " << msg->gyro_integral_dt << std::endl;
            std::cout << "accelerometer_timestamp_relative: " << msg->accelerometer_timestamp_relative << std::endl;
            std::cout << "accelerometer_m_s2[0]: " << msg->accelerometer_m_s2[0] << std::endl;
            std::cout << "accelerometer_m_s2[1]: " << msg->accelerometer_m_s2[1] << std::endl;
            std::cout << "accelerometer_m_s2[2]: " << msg->accelerometer_m_s2[2] << std::endl;
            std::cout << "accelerometer_integral_dt: " << msg->accelerometer_integral_dt << std::endl;
        };
```

This creates a callback function for when the `sensor_combined` messages are received. It outputs the content of the message fields each time the message is received.

```c++
        subscription_ = this->create_subscription<px4_ros_com::msg::SensorCombined>("SensorCombined_topic", callback);
    }

private:
    rclcpp::Subscription<px4_ros_com::msg::SensorCombined>::SharedPtr subscription_;
};
```

The above create a subscription to the `sensor_combined_topic` which can be matched with one or more compatible ROS publishers.

```c++
int main(int argc, char *argv[])
{
    std::cout << "Starting sensor_combined listener node..." << std::endl;
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SensorCombinedListener>());

    rclcpp::shutdown();
    return 0;
}
```

The instantiation of the `SensorCombinedListener` class as a ROS node is done on the `main` function.

## Creating a ROS2 advertiser

A ROS2 advertiser node publishes data into the DDS/RTPS network (and hence to PX4).

Taking as an example the `debug_vect_advertiser.cpp` under `px4_ros_com/src/listeners`:

```c++
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <px4_ros_com/msg/debug_vect.hpp>

using namespace std::chrono_literals;
```

Bring in the required headers, including the `debug_vect` msg header.

```c++
class DebugVectAdvertiser : public rclcpp::Node
{
```

The above creates a `DebugVectAdvertiser` class that subclasses the generic `rclcpp::Node` base class.

```c++
public:
    DebugVectAdvertiser() : Node("debug_vect_advertiser") {
        publisher_ = this->create_publisher<px4_ros_com::msg::DebugVect>("DebugVect_topic");
        auto timer_callback =
        [this]()->void {
            auto debug_vect = px4_ros_com::msg::DebugVect();
            debug_vect.timestamp = this->now().nanoseconds() * 1E-3;
            debug_vect.x = 1.0;
            debug_vect.y = 2.0;
            debug_vect.z = 3.0;
            RCLCPP_INFO(this->get_logger(), "Publishing debug_vect: time: %f x:%f y:%f z:%f",
                                debug_vect.timestamp, debug_vect.x, debug_vect.y, debug_vect.z)
            this->publisher_->publish(debug_vect);
        };
        timer_ = this->create_wall_timer(500ms, timer_callback);
    }

private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<px4_ros_com::msg::DebugVect>::SharedPtr publisher_;
};
```

This creates a function for when messages are to be sent. The messages are sent based on a timed callback, which sends two messages per second based on a timer.

```c++
int main(int argc, char *argv[])
{
    std::cout << "Starting debug_vect advertiser node..." << std::endl;
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DebugVectAdvertiser>());

    rclcpp::shutdown();
    return 0;
}
```

The instantiation of the `DebugVectAdvertiser` class as a ROS node is done on the `main` function.

## Creating a ROS listener

The creation of ROS nodes is a well known and documented process. An example of a ROS listener for `sensor_combined` messages can be found in the `ros1` branch repo, under `px4_ros_com/src/listeners`.

## Examples/tests of ROS-independent apps

The following examples provide additional real-world demonstrations of how to use the features described in this topic.

* [Throughput test](../middleware/micrortps_throughput_test.md): A simple test to measure the throughput of the bridge.

## Testing the PX4-FastRPTS bridge with ROS2 and ROS

To quickly test the package (using PX4 SITL with Gazebo):

1. Start PX4 SITL with Gazebo using:
    
    ```sh
    make px4_sitl_rtps gazebo
    ```

2. On one terminal, source the ROS2 environment and workspace and launch the `ros1_bridge` (this allows ROS2 and ROS nodes to communicate with each other). Also set the `ROS_MASTER_URI` where the `roscore` is/will be running:
    
    ```sh
    $ source /opt/ros/crystal/setup.bash
    $ source ~/px4_ros_com_ros2/install/local_setup.bash
    $ export ROS_MASTER_URI=http://localhost:11311
    $ ros2 run ros1_bridge dynamic_bridge
    ```

3. On another terminal, source the ROS workspace and launch the `sensor_combined` listener node. Since you are launching through `roslaunch`, this will also automatically start the `roscore`:
    
    ```sh
    $ source ~/px4_ros_com_ros1/install/local_setup.bash
    $ roslaunch px4_ros_com sensor_combined_listener.launch
    ```

4. On a terminal, source the ROS2 workspace and then start the `micrortps_agent` daemon with UDP as the transport protocol:
    
    ```sh
    $ source ~/px4_ros_com_ros2/install/local_setup.bash
    $ micrortps_agent -t UDP
    ```

5. On the [NuttShell/System Console](../debug/system_console.md), start the `micrortps_client` daemon also in UDP:
    
    ```sh
    > micrortps_client start -t UDP
    ```
    
    Now you will be able to see the data being printed on the terminal/console where you launched the ROS listener:
    
    ```sh
    RECEIVED DATA FROM SENSOR COMBINED
    ================================
    gyro_rad[0]: 0.00341645
    gyro_rad[1]: 0.00626475
    gyro_rad[2]: -0.000515705
    gyro_integral_dt: 4739
    accelerometer_timestamp_relative: 0
    accelerometer_m_s2[0]: -0.273381
    accelerometer_m_s2[1]: 0.0949186
    accelerometer_m_s2[2]: -9.76044
    accelerometer_integral_dt: 4739
    
    Publishing back...
    ```
    
    You can also verify the rate of the message using `rostopic hz`. For the case of `sensor_combined`:
    
    ```sh
    average rate: 248.187
    min: 0.000s max: 0.012s std dev: 0.00147s window: 2724
    average rate: 248.006
    min: 0.000s max: 0.012s std dev: 0.00147s window: 2972
    average rate: 247.330
    min: 0.000s max: 0.012s std dev: 0.00148s window: 3212
    average rate: 247.497
    min: 0.000s max: 0.012s std dev: 0.00149s window: 3464
    average rate: 247.458
    min: 0.000s max: 0.012s std dev: 0.00149s window: 3712
    average rate: 247.485
    min: 0.000s max: 0.012s std dev: 0.00148s window: 3960
    ```

6. You can also test the `sensor_combined` ROS2 listener by typing in a terminal:
    
    ```sh
    $ source ~/px4_ros_com_ros2/install/local_setup.bash
    $ sensor_combined_listener # or ros2 run px4_ros_com sensor_combined_listener
    ```

And it should also get data being printed to the console output.

> **Note** If ones uses the `build_all.bash` script, it automatically open and source all the required terminals so one just has to run the respective apps in each terminal.

## Troubleshooting

### Client reports that selected UART port is busy

If the selected UART port is busy, it's possible that the MAVLink application is already being used. If both MAVLink and RTPS connections are required you will have to either move the connection to use another port or configure the port so that it can be shared. <!-- https://github.com/PX4/Devguide/issues/233 -->

> **Tip** A quick/temporary fix to allow bridge testing during development is to stop MAVLink from *NuttShell*: 
> 
>     sh
>       mavlink stop-all

### Agent not built/fastrtpsgen is not found

The *Agent* code is generated using a *Fast RTPS* tool called *fastrtpsgen*.

If you haven't installed Fast RTPS in the default path then you must specify its installation directory by setting the `FASTRTPSGEN_DIR` environment variable before executing *make*.

On Linux/Mac this is done as shown below:

```sh
export FASTRTPSGEN_DIR=/path/to/fastrtps/install/folder/bin
```

> **Note** This should not be a problem if [Fast RTPS is installed in the default location](../setup/fast-rtps-installation.md).

### Enable UART on an OBC (onboard computer)

For UART transport on a Raspberry Pi or any other OBC you will have to enable the serial port:

1. Make sure the `userid` (default is pi on a Raspberry Pi) is a member of the `dialout` group:
    
    ```sh
    groups pi
    sudo usermod -a -G dialout pi
    ```

2. For the Raspberry Pi in particular, you need to stop the GPIO serial console that is using the port:
    
    ```sh
    sudo raspi-config
    ```
    
    In the menu showed go to **Interfacing options > Serial**. Select **NO** for *Would you like a login shell to be accessible over serial?*. Valid and reboot.

3. Check UART in kernel:
    
    ```sh
    sudo vi /boot/config.txt
    ```
    
    And make sure that the `enable_uart` value is set to 1:
    
    ```txt
    enable_uart=1
    ```

## Additional information

* [Fast RTPS Installation](../setup/fast-rtps-installation.md)
* [Manually Generate Client and Agent Code](micrortps_manual_code_generation.md)
* [DDS and ROS middleware implementations](https://github.com/ros2/ros2/wiki/DDS-and-ROS-middleware-implementations)