# RTPS/ROS2 인터페이스: PX4-FastRTPS 브릿지

*PX4-FastRTPS 브릿지*는 PX4 구성요소와 (보드 외부의) *Fast RTPS* 프로그램(ROS2/ROS 프레임워크로 빌드한 결과 포함)간 [uORB 메세지](../middleware/uorb.md)를 송수신하는 RTPS(Real Time Publish Subscribe) 인터페이스를 PX4에 추가했습니다.

> **Note** RTPS는 OMG(Object Management Group)의 DDS(Data Distribution Service) 표준 기본 프로토콜입니다. 이 프로토콜은 Pub/Sub 패턴을 활용, 확장성, 실시간성, 의존성, 고성능, 상호운용 규격을 만족합니다. *Fast RTPS*는 최신 RTPS 프로토콜과 최소한의 DDS API를 크로스플랫폼 환경에 맞춰 경량화한 구현체입니다.

ROS2(Robot Operating System)의 미들웨어로 RTPS를 채택했습니다. *Fast RTPS 브릿지*는 센서의 값, 명령어 그리고 다른 기체의 정보를 공유하기 쉽게하는 ROS2와의 통합을 용이하게 합니다.

이 주제에서는 RTPS 브릿지 구조(와 ROS2/ROS 어플리케이션 파이프라인에서 어떻게 활용하는지)를 설명하겠습니다. 그리고 필요한 코드를 어떻게 컴파일할지도 알아보도록 하겠습니다:

1. PX4의 변화를 지속적으로 살펴볼 간단한 *Fast RTPS* 어플리케이션 작성
2. 여러 ROS2 노드와 PX4 연결(RTPS 브릿지와 `px4_ros_com` 활용)
3. 추가로, ROS2와 ROS 브릿지 `ros1_bridge`를 활용, 여러 ROS("버전 1") 노드와 PX4 연결

## RTPS는 언제 사용해야 할까?

비행체 제어 장치와 보드 외부 요소간 지정 제한 시간내 실시간 정보 공유를 분명히 하려면 RTPS를 사용해야합니다. 특히 보드 외부 프로그램이 PX4에서 (uORB 토픽 송수신으로) 동작하는 프로그램 구성요소의 *피어*로 동작할 경우 유용합니다.

사용가능한 경우는, 컴퓨터 비전을 위한 로보틱스 라이브러리들간의 통신, 기체 조종을 위한 액추에이터와 센서간의 실시간 데이터 통신입니다.

> **Note** *Fast RTPS*는 MAVLink의 대체 수단이 아닙니다. (*Fast RTPS*는 일부 주변기기와 동작할 목적의 다른 기회가 열려있지만) MAVLink는 지상 통제 장치, 짐벌, 카메라 그 외의 보드 외부 요소와의 통신에 적합한 프로토콜로 남아있습니다

<span></span>

> **Tip** RTPS는 느린 채널(e.g. 무선 텔레메트리)에서도 사용할 수 있지만 채널 대역폭을 넘기지 않도록 조심해야 합니다.

## 아키텍쳐 개요

### RTPS 브릿지

RTPS브릿지는 [uORB](../middleware/uorb.md)와 RTPS 메시지를 매끄럽게 변환하여 PX4와 RTPS 어플리케이션간 주고 받는 메시지를 송수신합니다.

![basic example flow](../../assets/middleware/micrortps/architecture.png)

구조의 주된 요소는 상단 그림에 있는 클라이언트와 에이전트 프로세스입니다.

* *Client*는 비행체 제어 장치에서 실행하는 PX4 미들웨어 데몬입니다. 다른 PX4 컴포넌트가 보내는 토픽을 지속적으로 수신하고, (UART 또는 UDP 포트로) *Agent*를 대상으로 업데이트 내용을 보냅니다. *Agent*로 부터 메시지도 받으며 PX4로 uORB 메시지를 내보내기도 합니다.
* *Agent*는 외부 컴퓨터에서 데몬으로 실행합니다. *Client*에서 보낸 uORB 업데이트 메시지를 검사한 후 RTPS에 실어 보냅니다. RTPS 어플리케이션에서 오는 "uORB" RTPS 메시지도 지속적으로 수신하며 Client에 전달합니다.
* *Agent*과 *Client*는 직렬 연결(UART) 또는 UDP 네트워크로 연결합니다. uORB 정보는 전송 전 [CDR 직렬화](https://en.wikipedia.org/wiki/Common_Data_Representation) 처리합니다(*CDR 직렬화* 수단은 다른 플랫폼들간 직렬 데이터 송수신에 활용하는 일반 형식을 제공합니다).
* *Agent*와 *Fast RTPS* 어플리케이션은 UDP를 통해 연결되며, 다른 장치에 있을수도 있습니다. 일반 설정상으론 무선랜 또는 USB로 연결한 *클라이언트*에 연결한 동일한 시스템(예: 개발 컴퓨터, 리눅스 보조 컴퓨터, 처리 보드)입니다.

### ROS2/ROS 어플리케이션 파이프라인

ROS2를 위한 어플리케이션 파이프라인은 아주 직관적입니다. ROS2는 자체 통신 미들웨어로 DDS/RTPS를 사용하기 때문에, *PX4 Fast RTPS 브릿지*를 통해 PX4에서 내보내거나 지속 수신하는 ROS2 감청 유닛 또는 광역 전달 노드를 만들 수 있습니다. 이 내용을 아래의 그림으로 정리했습니다.

> **Note** 클라이언트와 에이전트(그리고 ROS 노드에서 계속)에서 사용하는 메시지 형식, 헤더, 소스 파일이 동일한 인터페이스 기술 언어(IDL) 파일에서 만들었는지 확인해야합니다. `px4_ros_com` 패키지는 ROS2에서 필요한 메시지, 헤더 생성에 필요한 기반입니다.

![Architecture with ROS2](../../assets/middleware/micrortps/architecture_ros2.png)

ROS 어플리케이션과 PX4를 통합하기 위한 구조는 아래에 나왔습니다.

![Architecture with ROS](../../assets/middleware/micrortps/architecture_ros.png)

ROS2와 ROS간의 메시지를 주고 받는 [ros1_bridge](https://github.com/ros2/ros1_bridge) 활용을 참고하십시오. ROS의 처음 버전이 RTPS를 지원하기 않기 때문에 필요합니다.

## 코드 생성

> **Note** [Fast RTPS 1.8.2 와 FastRTPSGen 1.0.4 또는 이후의 버전을 설치해야합니다](../setup/fast-rtps-installation.md). 그래야 필요한 코드를 만들 수 있습니다!

### ROS-독립 어플리케이션

All the code needed to create, build and use the bridge is automatically generated when PX4-Autopilot is compiled.

*Client* 어플리케이션 또한 일반 빌드 과정의 일부로 컴파일하고 빌드하여 펌웨어에 들어갑니다. *Agent*는 대상 컴퓨터에 맞게 따로 직접 컴파일해야합니다.

<span></span>

> **Tip** 브릿지 코드 또한 [직접 만들](micrortps_manual_code_generation.md)수 있습니다. 대부분의 사용자는 그럴 필요가 없지만, 연결한 주제에서는 빌드 과정을 자세하게 안내하며, 이 내용을 통해 문제 해결의 도움을 받을 수 있습니다.

<a id="px4_ros_com"></a>

### ROS2/ROS applications

The [px4_ros_com](https://github.com/PX4/px4_ros_com) package, when built, generates everything needed to access PX4 uORB messages from a ROS2 node (for ROS you also need [ros1_bridge](https://github.com/ros2/ros1_bridge)). This includes all the required components of the *PX4 RTPS bridge*, including the `micrortps_agent` and the IDL files (required by the `micrortps_agent`).

The ROS and ROS2 message definition headers and interfaces are generated from the [px4_msgs](https://github.com/PX4/px4_msgs) package, which match the uORB messages counterparts under PX4-Autopilot. These are required by `px4_ros_com` when generating the IDL files to be used by the `micrortps_agent`.

Both `px4_ros_com` and `px4_msgs` packages have two separate branches:

* ROS2에서 사용하는 `master` 브랜치. 이 브랜치는 PX4와 ROS2 노드를 연결할 ROS2 메시지와 IDL 파일을 만드는 코드가 들어있습니다.
* ROS에서 사용하는 `ros1` 브랜치. 이 브랜치는 `ros1_bridge`로 PX4와 ROS의 데이터를 공유하는데, *이를* 활용할 ROS 메시지 헤더와 소스 파일을 생성하는 코드가 들어있습니다.

Both branches in `px4_ros_com` additionally include some example listener and advertiser example nodes.

## 지원하는 uORB 메시지

The generated bridge code will enable a specified subset of uORB topics to be published/subscribed via RTPS. This is true for both ROS or non-ROS applications.

For *automatic code generation* there's a *yaml* definition file in the PX4 **PX4-Autopilot/msg/tools/** directory called **uorb_rtps_message_ids.yaml**. This file defines the set of uORB messages to be used with RTPS, whether the messages are to be sent, received or both, and the RTPS ID for the message to be used in DDS/RTPS middleware.

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

> **Note** ROS2 Dashing에서의 API 변경으로 `rosidl_generate_interfaces()` (`px4_msgs`에서의) CMake 모듈을 활용하여 microRTPS 에이전트 생성에 필요한 IDL 파일을 만들 수 있습니다. PX4-Autopilot includes a template for the IDL file generation, which is only used during the PX4 build process.
> 
> `px4_msgs` 빌드 과정에서는 ROS2/ROS에 활용할 *약간 다른* IDL 파일(PX4 펌웨어 용으로 빌드)을 만듭니다. **uorb_rtps_message_ids.yaml**는 *PascalCased*방식으로 메시지 이름을 짓습니다(이름을 바꾸는 것은 client-agent 통신과는 상관없지만 ROS2에는 크리티컬합니다, 따라서 메시지 네이밍은 PascalCase 컨벤션을 따라야합니다). 새 IDL 파일은 송수신한 메세지를 되돌립니다(메시지를 클라이언트에서 보냈을 때, 에이전트에서 보내거나 그 반대의 경우로도 가능하기 때문에 필요).

<a id="client_firmware"></a>

## Client (PX4/PX4-Autopilot)

The *Client* source code is generated, compiled and built into the PX4 firmware as part of the normal build process.

To build the firmware for NuttX/Pixhawk flight controllers use the `_rtps` feature in the configuration target. For example, to build RTPS for px4_fmu-v4:

```sh
make px4_fmu-v4_rtps
```

To build the firmware for a SITL target:

```sh
make px4_sitl_rtps
```

The *Client* application can be launched from [NuttShell/System Console](../debug/system_console.md). The command syntax is shown below (you can specify a variable number of arguments):

```sh
> micrortps_client start|stop|status [options]
  -t <transport>          [UART|UDP] Default UART
  -d <device>             UART device. Default /dev/ttyACM0
  -l <loops>              How many iterations will this program have. -1 for infinite. Default -1.
  -w <sleep_time_ms>      Time in ms for which each iteration sleep. Default 1ms
  -b <baudrate>           UART device baudrate. Default 460800
  -p <poll_ms>            Time in ms to poll over UART. Default 1ms
  -r <reception port>     UDP port for receiving. Default 2019
  -s <sending port>       UDP port for sending. Default 2020
  -i <ip_address>         Select IP address (remote) values: <x.x.x.x>. Default: 127.0.0.1
```

> **Note** 기본적으로 *Client*는 데몬으로 동작하지만, 직접 실행해야 할 수도 있습니다. PX4 펌웨어 초기화 코드는 나중에 *Client*를 영구 실행 데몬 프로세스로 자동 시작합니다.

For example, in order to run the *Client* daemon with SITL connecting to the Agent via UDP, start the daemon as shown:

```sh
micrortps_client start -t UDP
```

## Fast RTPS interface를 사용하는 ROS에 독립적인 오프보드 에이전트

The *Agent* code is automatically *generated* when you build the associated PX4 firmware. You can find the source here: **build/<target-platform>/src/modules/micrortps_bridge/micrortps_client/micrortps_agent/**.

To build the *Agent* application, compile the code:

```sh
cd build/<target-platform>/src/modules/micrortps_bridge/micrortps_client/micrortps_agent
mkdir build && cd build
cmake ..
make
```

> **Note** *Qualcomm Snapdragon Flight* 플랫폼을 위한 크로스 컴파일을 [여기](https://github.com/eProsima/PX4-FastRTPS-PoC-Snapdragon-UDP#how-to-use)를 참고하세요.

The command syntax for the *Agent* is listed below:

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

To launch the *Agent*, run `micrortps_agent` with appropriate options for specifying the connection to the *Client* (the default options connect from a Linux device to the *Client* over a UART port).

As an example, to start the *micrortps_agent* with connection through UDP, issue:

```sh
./micrortps_agent -t UDP
```

## Agent와 ROS2 미들웨어

Building `px4_ros_com` automatically generates and builds the agent application, though it requires (as a dependency), that the `px4_msgs` package also gets build on the same ROS2 workspace (or overlaid from another ROS2 workspace). Since it is also installed using the [`colcon`](http://design.ros2.org/articles/build_tool.html) build tools, running it works exactly the same way as the above. Check the **Building the `px4_ros_com` package** for details about the build structure.

## `px4_ros_com`와 `px4_msgs` 패키지 빌드

Install and setup both ROS2 and ROS environments on your development machine and separately clone the `px4_ros_com` and `px4_msgs` repo for both the `master` and `ros1` branches (see [above for more information](#px4_ros_com)).

> **Note** ROS2는 마스터 브랜치만 필요합니다(ROS는 두 브랜치 다 필요합니다).

### ROS와 ROS2 설치와 의존성

> **Note**이 설치 빌드 안내서는 ROS Melodic과 ROS2 Dashing을 다룹니다(ROS2 Ardent, Bouncy, Crystal은 지원이 끝나 다루지 않습니다).

In order to install ROS Melodic and ROS2 Dashing (officially supported) on a Ubuntu 18.04 machine, follow the links below, respectively:

1. [ROS Melodic을 설치하십시오](http://wiki.ros.org/melodic/Installation/Ubuntu)
2. [ROS2 Dashing을 설치하십시오](https://index.ros.org/doc/ros2/Installation/Dashing/Linux-Install-Debians/)
3. 이 설치과정은 *colcon* 빌드 툴 설치가 필요할 것 입니다. 설치되지 않으면 수동으로 설치해주세요.
    
    ```sh
    sudo apt install python3-colcon-common-extensions
    ```

4. Eigen3를 변환 라이브러리에서 활용하므로 *eigen3_cmake_module* 모듈도 필요합니다:
    
    ```sh
    sudo apt install ros-dashing-eigen3-cmake-module
    ```

5. *setuptools*를 설치해야합니다(*apt* 또는 *apt* 활용):
    
    ```sh
    sudo pip3 install -U setuptools
    ```
    
    > **Caution** 데비안 저장소에서 `ros1_bridge` 패키지를 설치하지 마십시오. 이 패키지는 소스 코드를 빌드해야합니다.

### 작업 영역 설정

Since the ROS2 and ROS require different environments you will need a separate workspace for each ROS version. As an example:

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

### 작업 영역 빌드하기

The directory `px4_ros_com/scripts` contains multiple scripts that can be used to build both workspaces.

To build both workspaces with a single script, use the `build_all.bash`. Check the usage with `source build_all.bash --help`. The most common way of using it is by passing the ROS(1) workspace directory path and also the PX4-Autopilot directory path:

```sh
$ source build_all.bash --ros1_ws_dir <path/to/px4_ros_com_ros1/ws>
```

> **Note** `--verbose` 인자는 *colcon* 빌드 출력 전체 내용을 보여줍니다.
> 
> **Note** 빌드 과정 도중 다른 환경 설정을 적용해야 하는 각 빌드 과정 단계에 따라 콘솔의 새 탭을 엽니다.

One can also use the following individual scripts in order to build the individual parts:

* `ros1_bridge`를 빌드할 `build_ros1_bridge.bash`.
* `px4_ros_com`과 `px4_msgs`의 `ros1` 브랜치를 가져온 위치에 ROS1 작업 영역을 빌드하는 `build_ros1_workspace.bash`(`px4_ros_com`의 `ros1` 브랜치에만 있음).
* `px4_ros_com`과 `px4_msgs`의 `master` 브랜치를 가져온 위치에 ROS2 작업 영역을 빌드하는 `build_ros2_workspace.bash`.

The steps below show how to *manually* build the packages (provided for your information/better understanding only):

1. `px4_ros_com_ros2` 디렉터리를 대상으로 `cd` 명령을 실행하고 ROS2 환경에 필요한 모든 설정을 적용(source)하십시오. 앞서 작업 영역을 설정했다고 하더라도 신경쓰지 마십시오:
    
    ```sh
    source /opt/ros/dashing/setup.bash
    ```

2. ROS2 작업 영역에 빌드할 수 있도록 `ros1_bridge` 패키지를 원격에서 가져오십시오:
    
    ```sh
    git clone https://github.com/ros2/ros1_bridge.git -b dashing ~/px4_ros_com_ros2/src/ros1_bridge
    ```

3. `ros1_bridge` 패키지를 제외하고, `px4_ros_com`와 `px4_msgs` 패키지를 빌드하세요.
    
    ```sh
    colcon build --symlink-install --packages-skip ros1_bridge --event-handlers console_direct+
    ```
    
    > **Note** ` --event-handlers console_direct+`는 단지 `colcon` 빌드 과정을 자세하게 보여주는 용도로만 사용되고 원하지 않으면 삭제하면 됩니다.

4. 이제 ROS 패키지를 빌드하십시오. 빌드 과정 진행을 위해 새 터미널 창을 열고 시스템에 설치한 ROS(1) 환경 전용 설정 값을 적용하십시오:
    
    ```sh
    source /opt/ros/melodic/setup.bash
    ```

5. 새로운 터미널에서, `px4_ros_com`와 `px4_msgs` 패키지를 빌드하세요.
    
    ```sh
    cd ~/px4_ros_com_ros1 && colcon build --symlink-install --event-handlers console_direct+
    ```

6. `ros1_bridge`를 빌드하기 전, 새 터미널을 열고 아래 순서대로 환경과 작업 영역을 설정하십시오:
    
    ```sh
    source ~/px4_ros_com_ros1/install/setup.bash
    source ~/px4_ros_com_ros2/install/setup.bash
    ```

7. 이제 `ros1_bridge`를 빌드하십시오. 참고로, 빌드 과정은 메모리를 많이 소비합니다. 한정 자원을 지닌 환경에서는, 병렬 처리 건수를 줄이십시오(예: 환경 변수 설정 `MAKEFLAGS=-j1`). 자세한 빌드 과정은 [ros1_bridge](https://github.com/ros2/ros1_bridge) 패키지의 빌드 명령을 참고하십시오.
    
    ```sh
    cd ~/px4_ros_com_ros2 && colcon build --symlink-install --packages-select ros1_bridge --cmake-force-configure --event-handlers console_direct+
    ```

### 작업 영역 정리하기

After building the workspaces there are many files that must be deleted before you can do a clean/fresh build (for example, after you have changed some code and want to rebuild). Unfortunately *colcon* does not currently have a way of cleaning the generated **build**, **install** and **log** directories, so these directories must be deleted manually.

The **clean_all.bash** script (in **px4_ros_com/scripts**) is provided to ease this cleaning process. The most common way of using it is by passing it the ROS(1) workspace directory path (since it's usually not on the default path):

```sh
$ source clean_all.bash --ros1_ws_dir <path/to/px4_ros_com_ros1/ws>
```

## Fast RTPS 감청 어플리케이션 만들기

Once the *Client* (on the flight controller) and the *Agent* (on an offboard computer) are running and connected, *Fast RTPS* applications can publish and subscribe to uORB topics on PX4 using RTPS.

This example shows how to create a *Fast RTPS* "listener" application that subscribes to the `sensor_combined` topic and prints out updates (from PX4). A connected RTPS application can run on any computer on the same network as the *Agent*. For this example the *Agent* and *Listener application* will be on the same computer.

The *fastrtpsgen* script can be used to generate a simple RTPS application from an IDL message file.

> **Note** RTPS 메시지는 IDL 파일에 정의해두고 *fastrtpsgen* 명령으로 C++ 언어로 작성한 코드를 컴파일합니다. 브릿지 코드 빌드 과정에서 송수신할 uORB 메세지 파일에 대한 IDL 파일을 만듭니다(**build/BUILDPLATFORM/src/modules/micrortps_bridge/micrortps_agent/idl/*.idl** 참고). PX4와 통신할 *Fast RTPS* 어플리케이션을 만들 때 IDL 파일이 필요합니다.

Enter the following commands to create the application:

```sh
cd /path/to/PX4/PX4-Autopilot/build/px4_sitl_rtps/src/modules/micrortps_bridge
mkdir micrortps_listener
cd micrortps_listener
fastrtpsgen -example x64Linux2.6gcc ../micrortps_client/micrortps_agent/idl/sensor_combined.idl
```

This creates a basic subscriber and publisher, and a main-application to run them. To print out the data from the `sensor_combined` topic, modify the `onNewDataMessage()` method in **sensor_combined_Subscriber.cxx**:

```cpp
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

> **Note** *감청 어플리케이션*에서 아무 내용도 출력하지 않는다면 *클라이언트*를 실행하고 있는지 확인하십시오.

## ROS2 감청 유닛 만들기

With the `px4_ros_com` built successfully, one can now take advantage of the generated *micro-RTPS* agent app and also from the generated sources and headers of the ROS2 msgs from `px4_msgs`, which represent a one-to-one matching with the uORB counterparts.

To create a listener node on ROS2, lets take as an example the `sensor_combined_listener.cpp` node under `px4_ros_com/src/listeners`:

```cpp
#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/sensor_combined.hpp>
```

The above brings to use the required C++ libraries to interface with the ROS2 middleware. It also includes the required message header file.

```cpp
/**
 * @brief Sensor Combined uORB topic data callback
 */
class SensorCombinedListener : public rclcpp::Node
{
```

The above creates a `SensorCombinedListener` class that subclasses the generic `rclcpp::Node` base class.

```cpp
public:
    explicit SensorCombinedListener() : Node("sensor_combined_listener") {
        subscription_ = this->create_subscription<px4_msgs::msg::SensorCombined>(
            "SensorCombined_PubSubTopic", 10,
            [this](const px4_msgs::msg::SensorCombined::UniquePtr msg) {
            std::cout << "\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n";
            std::cout << "RECEIVED SENSOR COMBINED DATA"   << std::endl;
            std::cout << "============================="   << std::endl;
            std::cout << "ts: "          << msg->timestamp    << std::endl;
            std::cout << "gyro_rad[0]: " << msg->gyro_rad[0]  << std::endl;
            std::cout << "gyro_rad[1]: " << msg->gyro_rad[1]  << std::endl;
            std::cout << "gyro_rad[2]: " << msg->gyro_rad[2]  << std::endl;
            std::cout << "gyro_integral_dt: " << msg->gyro_integral_dt << std::endl;
            std::cout << "accelerometer_timestamp_relative: " << msg->accelerometer_timestamp_relative << std::endl;
            std::cout << "accelerometer_m_s2[0]: " << msg->accelerometer_m_s2[0] << std::endl;
            std::cout << "accelerometer_m_s2[1]: " << msg->accelerometer_m_s2[1] << std::endl;
            std::cout << "accelerometer_m_s2[2]: " << msg->accelerometer_m_s2[2] << std::endl;
            std::cout << "accelerometer_integral_dt: " << msg->accelerometer_integral_dt << std::endl;
        });
    }
```

This creates a callback function for when the `sensor_combined` uORB messages are received (now as DDS messages). It outputs the content of the message fields each time the message is received.

```cpp
private:
    rclcpp::Subscription<px4_msgs::msg::SensorCombined>::SharedPtr subscription_;
};
```

The above create a subscription to the `sensor_combined_topic` which can be matched with one or more compatible ROS publishers.

```cpp
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

## ROS2 광역 전달 노드 만들기

A ROS2 advertiser node publishes data into the DDS/RTPS network (and hence to PX4).

Taking as an example the `debug_vect_advertiser.cpp` under `px4_ros_com/src/advertisers`:

```cpp
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/debug_vect.hpp>

using namespace std::chrono_literals;
```

Bring in the required headers, including the `debug_vect` msg header.

```cpp
class DebugVectAdvertiser : public rclcpp::Node
{
```

The above creates a `DebugVectAdvertiser` class that subclasses the generic `rclcpp::Node` base class.

```cpp
public:
    DebugVectAdvertiser() : Node("debug_vect_advertiser") {
        publisher_ = this->create_publisher<px4_msgs::msg::DebugVect>("DebugVect_PubSubTopic", 10);
        auto timer_callback =
        [this]()->void {
            auto debug_vect = px4_msgs::msg::DebugVect();
            debug_vect.timestamp = this->now().nanoseconds() * 1E-3;
            std::string name = "test";
            std::copy(name.begin(), name.end(), debug_vect.name.begin());
            debug_vect.x = 1.0;
            debug_vect.y = 2.0;
            debug_vect.z = 3.0;
            RCLCPP_INFO(this->get_logger(), "\033[97m Publishing debug_vect: time: %f x:%f y:%f z:%f \033[0m",
                                debug_vect.timestamp, debug_vect.x, debug_vect.y, debug_vect.z);
            this->publisher_->publish(debug_vect);
        };
        timer_ = this->create_wall_timer(500ms, timer_callback);
    }

private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<px4_msgs::msg::DebugVect>::SharedPtr publisher_;
};
```

This creates a function for when messages are to be sent. The messages are sent based on a timed callback, which sends two messages per second based on a timer.

```cpp
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

## ROS(1) 감청 유닛 만들기

The creation of ROS nodes is a well known and documented process. An example of a ROS listener for `sensor_combined` messages can be found in the `ros1` branch repo, under `px4_ros_com/src/listeners`.

## ROS-독립 어플리케이션 예제와 테스트

The following examples provide additional real-world demonstrations of how to use the features described in this topic.

* [Throughput test](../middleware/micrortps_throughput_test.md): 브릿지의 처리량을 측적하는 간단한 테스트입니다.

## ROS2/ROS와 브릿징한 PX4-FastRPTS 테스트하기

To quickly test the package (using PX4 SITL with Gazebo):

1. PX4 SITL와 가제보를 빌드하십시오
    
    ```sh
    make px4_sitl_rtps gazebo
    ```

2. 하나의 터미널에서 ROS2 환경과 작업 영역의 설정을 가져오고 `ros1_bridge` 를 실행하십시오(ROS2와 ROS가 서로 통신할 수 있게 합니다). 또한 `roscore`를 실행할 위치인 `ROS_MASTER_URI`를 설정하십시오:
    
    ```sh
    $ source /opt/ros/dashing/setup.bash
    $ source ~/px4_ros_com_ros2/install/local_setup.bash
    $ export ROS_MASTER_URI=http://localhost:11311
    $ ros2 run ros1_bridge dynamic_bridge
    ```

3. 다른 터미널에서 ROS 작업 영역 설정을 적용하고 `sensor_combined` 감청 노드를 실행하십시오. `roslaunch`를 실행했다면 `roscore`도 자동으로 시작합니다:
    
    ```sh
    $ source ~/px4_ros_com_ros1/install/setup.bash
    $ roslaunch px4_ros_com sensor_combined_listener.launch
    ```

4. 터미널에서 ROS2 작업 영역의 설정을 적용하고 `micrortps_agent` 데몬을 시작하면서 UDP를 전송 프로토콜로 지정하십시오:
    
    ```sh
    $ source ~/px4_ros_com_ros2/install/setup.bash
    $ micrortps_agent -t UDP
    ```

5. [NuttShell/System Console](../debug/system_console.md)에서는 UDP를 사용하는 `micrortps_client`를 실행하세요.
    
    ```sh
    > micrortps_client start -t UDP
    ```
    
    이제 ROS 리스너를 실행한 콘솔에서 데이터가 출력되는 것을 볼 수 있을 것입니다.
    
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
    
    `rostopic hz`로 메시지의 속도를 확인할 수 있습니다. `sensor_combined`의 경우
    
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

6. 다음과 같이 `sensor_combined` ROS2 리스너를 테스를 할 수 있습니다.
    
    ```sh
    $ source ~/px4_ros_com_ros2/install/local_setup.bash
    $ ros2 launch px4_ros_com sensor_combined_listener.launch.py
    ```

And it should also get data being printed to the console output.

> **Note** 누군가가 `build_all.bash` 스크립트를 사용하면, 필요한 모든 터미널을 자동으로 열고 모든 환경 변수 설정을 적용하여 각 터미널에서 제각각의 앱이 올바른 설정으로 동작하게 합니다.

## 문제 해결

### 클라이언트에서 선택한 UART 포트가 사용 중이라고 할 경우

If the selected UART port is busy, it's possible that the MAVLink application is already being used. If both MAVLink and RTPS connections are required you will have to either move the connection to use another port or configure the port so that it can be shared. <!-- https://github.com/PX4/Devguide/issues/233 -->

> **Tip** 개발 과정에서 브릿지 시험을 허용하도록 재빠르게 임시로 조치하는 방법은 *NuttShell*에서 MAVLink를 중단하는 방법입니다: 
> 
>     sh
>       mavlink stop-all

### 에이전트 빌드 안됨, fastrtpsgen 찾을 수 없음

The *Agent* code is generated using a *Fast RTPS* tool called *fastrtpsgen*.

If you haven't installed Fast RTPS in the default path then you must specify its installation directory by setting the `FASTRTPSGEN_DIR` environment variable before executing *make*.

On Linux/Mac this is done as shown below:

```sh
export FASTRTPSGEN_DIR=/path/to/fastrtps/install/folder/bin
```

> **Note** [Fast RTPS 를 기본 경로에 설치](../setup/fast-rtps-installation.md)했다면 문제가 나타나지 않아야합니다.

### OBC(온보드 컴퓨터)에서 UART 활성화하기

For UART transport on a Raspberry Pi or any other OBC you will have to enable the serial port:

1. `userid`(라즈베리 파이에서는 pi가 기본)가 `dialout` 그룹의 구성원인지 확인하세요.
    
    ```sh
    groups pi
    sudo usermod -a -G dialout pi
    ```

2. 일부 라즈베리 파이에서는 포트를 사용하는 GPIO 직렬 콘솔 사용을 멈춰야 합니다.
    
    ```sh
    sudo raspi-config
    ```
    
    보여지는 메뉴에서 **Interfacing options > Serial**로 이동합니다. *Would you like a login shell to be accessible over serial?* 질문에 대해서는 **NO**를 선택하세요. 확인하고 재부팅하세요.

3. 커널에서 UART 확인하기
    
    ```sh
    sudo vi /boot/config.txt
    ```
    
    `enable_uart` 값이 1로 설정되어 있는지 확인하세요.
    
        enable_uart=1
        

## 추가 정보

* [Fast RTPS 설치](../setup/fast-rtps-installation.md)
* [Client와 Agent 코드 직접 생성](micrortps_manual_code_generation.md)
* [DDS와 ROS 미들웨어 구현](https://github.com/ros2/ros2/wiki/DDS-and-ROS-middleware-implementations)