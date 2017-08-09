# RTPS/ROS2 인터페이스: PX4-FastRTPS 브리지

*PX4-FastRTPS 브리지* 는 PX4에 RTPS(Real Time Publish Subscribe) 인터페이스를 추가하여, PX4 컴포넌트와 (오프보드) *FastRTPS* 어플리케이션 사이에서 [uORB messages](../middleware/uorb.md) 교환을 가능하게 합니다.

> **Note** RTPS는 OMG(The Object Management Group) 데이터 분산 서비스 표준의 기반이 되는 프로토콜입니다. publish/subscribe 패턴을 이용해서 확장성, 실시간성, 독립성, 고성능, 상호 정보 교환이 가능하는 것을 목표로 합니다. *FastRTPS* 는 RTPS 프로토콜과 최소 DDS API의 최신 버전의 매우 가벼운 크로스플랫폼에 돌아가도록 구현한 것입니다.

RTPS는 ROS2의 미들웨어로 채택되었습니다. 브리지를 이용하면 ROS2와 더 쉽게 통합할 수 있으며 센서 값, 명령, 다른 비행체 정보를 공유하기 쉽게 합니다.

이 토픽에서는 브리지 아키텍쳐를 설명합니다. 어떻게 컴파일하는지 PX4 변경 사항을 subscribe하기 위한 간단한 FastRTSP 어플리케이션을 작성하는 방법에 대해서 알아봅니다.


## 언제 RTPS를 사용할까?

RTPS는 flight controller와 오프보드 컴포넌트 사이에 time-critical/실시간으로 안전하게 정보 공유가 필요한 환경에서 사용합니다. 특히 오프보드 소프트웨어가 PX가 실행되는 주변 소프트웨어 컴포넌트가 되는 경우에 유용합니다. (uORB topic을 주고 받기) RTPS는 느린 속도의 링크에서 사용하기에는 적합하지 않습니다.(예로 radio telemetry)

컴퓨터 비전을 위해 로보틱스 라이브러리와 통신을 포함하는 유스 케이스와 실시간으로 액츄레이터와 센서 정보를 주고받는 것이 제어의 핵심인 유스 케이스가 있습니다.

> **Note** FastRTPS는 MAVLink의 대체할려는 목적은 아닙니다. MAVLink는 여전히 ground station, 짐벌, 카메라 등과 통신하는 가장 적합한 프로토콜입니다. (FastRTPS는 다른 주변장치와 동작하는 것도 가능합니다.)


## 아키텍쳐 개요

![basic example flow](../../assets/middleware/micrortps/basic_example_flow.png)

아키텍쳐의 주요 요소는 클라이언트와 에이젠트 프로세스로 위 다이어그램에서 볼 수 있습니다.

- *Client* 는 PX4 미들웨어 데몬 프로세스로 flight controller에서 실행됩니다. 다른 PX4 컴포넌트로 publish되는 uORB topic을 subscribe 하며 업데이트가 발생시 *Agent* 로 전송합니다.(UART나 UDP 포트 사용) *Agent* 로부터 메시지를 수신하며 PX4에서 uORB 메시지로 publish합니다.
- *Agent* 는 오프보드 컴퓨터에서 데몬 프로세스로 실행됩니다. *Client* 가 보내는 uORB 업데이트 메시지를 감시하며 RTPS로 (re)publish합니다. 다른 RTPS 어플리케이션으로부터 "uORB" RTPS 메시지를 subscribe하고 *Client* 로 포워딩합니다.
- *Agent* 와 *Client* 는 시리얼 링크(UART)나 UDP 네트워크를 통해 연결합니다. uORB 정보는 전송을 위해서 [CDR serialized](https://en.wikipedia.org/wiki/Common_Data_Representation)됩니다. (*CDR serialization* 는 다른 플랫폼 간에 시리얼 데이터를 교환하기 위한 공통 포맷을 제공합니다.)
- *Agent* 와 *FastRTPS* 어플리케이션이 UDP로 연결되는 경우 동일 기기나 다른 기기일 수도 있습니다. 일반적인 설정에서 모두 동일한 시스템일 수 있습니다. (예로 개발 컴퓨터, 리눅스 컴패니온 컴퓨터나 컴퓨터 보드) Wifi 링크나 USB를 통해 *Client* 에 연결됩니다.

## 코드 생성

PX4 펌웨어가 컴파일되면, 브리지를 생성, 빌드, 사용하는데 필요한 모든 코드는 자동으로 생성됩니다.

*Client* 어플리케이션은 컴파일되고 일반 빌드 프로세스의 일부로 펌웨어로 생성됩니다. *Agent* 는 타겟 컴퓨터에 대해서 반드시 분리/수동으로 컴파일해야만 합니다.

> **Note** 필요한 코드를 생성하기 위해서는 [Fast RTPS가 반드시 설치되어야 한다](../setup/fast-rtps-installation.md)!

<span></span>
> **Tip** 브리지 코드는 [수동 생성](micrortps_manual_code_generation.md)이 가능합니다. 대부분 사용자는 그렇게 할 필요가 없지만 linked topic은 빌드 과정에 대한 보다 상세한 정보를 제공하며 문제해결에 도움이 됩니다.


## uORB 메시지 지원

생성된 브리지 코드는 uORB topic의 특정 서브 집합이 RTPS를 통해 publish/subscribe되는 것을 가능하게 합니다.

*자동 코드 생성*(일반 PX4 펌웨어 빌드 프로세스를 통해)을 위해서 타겟 플랫폼에 대해서 아래 부분이 **.cmake** 파일(**cmake/configs**)에 들어있어야만 합니다.

```cmake
set(config_rtps_send_topics
  sensor_combined
   # Add new topic...
   )

set(config_rtps_receive_topics
   sensor_baro
   # Add new topic...
   )
```

> **Caution** At time of writing (August 2017), only the small set of uORB topics listed above are included in our cmake files: **posix_sitl_default.cmake**, **nuttx_px4fmu-v4_default.cmake**, **posix_sdflight_default.cmake**. It is likely you will need to edit your *cmake* file and add additional uORB topics. In future we hope to define a larger standard set.

For *manual code generation* the uORB topics that will be supported by the bridge are specified when you call **generate_microRTPS_bridge.py** (using the `-s`/`--send` and `-r`/`--receive` flags). See [Manual Generation of the Code](../middleware/micrortps_manual_code_generation.md) for more information.


## Client (PX4 Firmware)

The *Client* source code is generated, compiled and built into the PX4 firmware as part of the normal build process.

To build and upload the firmware for NuttX/Pixhawk flight controllers:
```sh
make px4fmu-v4_default upload
```

To build and upload the firmware for Qualcomm Snapdragon Flight:
```sh
$ make eagle_default upload
```


The *Client* application can be launched from [NuttShell/System Console](../debug/system_console.md). The command syntax is shown below (you can specify a variable number of arguments):

```sh
> micrortps_client start|stop [options]
  -t <transport>          [UART|UDP] Default UART
  -d <device>             UART device. Default /dev/ttyACM0
  -u <update_time_ms>     Time in ms for uORB subscribed topics update. Default 0
  -l <loops>              How many iterations will this program have. -1 for infinite. Default 10000.
  -w <sleep_time_ms>      Time in ms for which each iteration sleep. Default 1ms
  -b <baudrate>           UART device baudrate. Default 460800
  -p <poll_ms>            Time in ms to poll over UART. Default 1ms
  -r <reception port>     UDP port for receiving. Default 2019
  -s <sending port>       UDP port for sending. Default 2020
```

By default the *Client* runs for 10000 loops and then stops. To run the *Client* continuously, enter the following command:

```sh
micrortps_client start -l -1
```

> **Note** The PX4 Firmware initialisation code may in future automatically start the *Client* as a permanent daemon process. In the meantime you will need to start the client manually.<!-- at that point, most of this section would move into the "manual generation" doc: https://github.com/PX4/Firmware/pull/7663#issuecomment-317928506 -->


## Agent (Off Board FastRTPS Interface)

The *Agent* code is automatically *generated* when you build the associated PX4 firmware. You can find the source here: **build_BUILDPLATFORM/src/modules/micrortps_bridge/micrortps_agent/**.

To build the *Agent* application, compile the code:

```sh
cd src/modules/micrortps_bridge/microRTPS_agent
mkdir build && cd build
cmake ..
make
```

> **Note** To cross-compile for the *Qualcomm Snapdragon Flight* platform see [this link](https://github.com/eProsima/PX4-FastRTPS-PoC-Snapdragon-UDP#how-to-use).


The command syntax for the *Agent* is listed below:

```text
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


## Creating a FastRTPS Listener application

Once the *Client* (on the flight controller) and the *Agent* (on an offboard computer) are running and connected, *FastRTPS* applications can publish and subscribe to uORB topics on PX4 using RTPS.

This example shows how to create a *FastRTPS* "listener" application that subscribes to the `sensor_combined` topic and prints out updates (from PX4). A connected RTPS application can run on any computer on the same network as the *Agent*. For this example the *Agent* and *Listener application* will be on the same computer.

The *fastrtpsgen* script can be used to generate a simple RTPS application from an IDL message file.

> **Note** RTPS messages are defined in IDL files and compiled to C++ using *fastrtpsgen*. As part of building the bridge code, IDL files are generated for the uORB message files that may be sent/received (see **build_BUILDPLATFORM/src/modules/micrortps_bridge/micrortps_agent/idl/*.idl**). These IDL files are needed when you create a FastRTPS application to communicate with PX4.

Enter the following commands to create the application:

```sh
cd /path/to/PX4/Firmware/src/modules/micrortps_bridge
mkdir micrortps_listener
cd micrortps_listener
fastrtpsgen -example x64Linux2.6gcc ../micrortps_agent/idl/sensor_combined_.idl
```

This creates a basic subscriber and publisher, and a main-application to run them. To print out the data from the `sensor_combined` topic, modify the `onNewDataMessage()` method in **sensor_combined_Subscriber.cxx**:

```sh
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


## Examples/tests

The following examples provide additional real-world demonstrations of how to use the features described in this topic.

* [Throughput test](../middleware/micrortps_throughput_test.md): A simple simple test to measure the throughput of the bridge.


## Troubleshooting

### Client reports that selected UART port is busy

If the selected UART port is busy, it's possible that the MAVLink application is already being used. If both MAVLink and RTPS connections are required you will have to either move the connection to use another port or configure the port so that it can be shared. <!-- https://github.com/PX4/Devguide/issues/233 -->

> **Tip** A quick/temporary fix to allow bridge testing during development is to stop MAVLink from *NuttShell*:
  ```sh
  mavlink stop-all
  ```

### Agent not built/fastrtpsgen is not found

The *Agent* code is generated using a *FastRTPS* tool called *fastrtpsgen*.  

If you haven't installed Fast RTPS in the default path then you must to specify its installation directory by setting the `FASTRTPSGEN_DIR` environment variable before executing *make*.

On Linux/Mac this is done as shown below:

```sh
export FASTRTPSGEN_DIR=/path/to/fastrtps/install/folder/bin
```

> **Note** This should not be a problem if [Fast RTPS is installed in the default location](../setup/fast-rtps-installation.md).

### Enable UART on Raspberry Pi

For UART transport on Raspberry Pi you will have to enable the serial port:

1. Make sure the `userid` (default is pi) is a member of the `dialout` group:

  ```sh
  groups pi
  sudo usermod -a -G dialout pi
  ```

2. You need to stop the already running on the GPIO serial console:

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

* [FastRTPS Installation](../setup/fast-rtps-installation.md)
* [Manually Generate Client and Agent Code](micrortps_manual_code_generation.md)
* [DDS and ROS middleware implementations](https://github.com/ros2/ros2/wiki/DDS-and-ROS-middleware-implementations)
