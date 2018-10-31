# RTPS/ROS2 Interface: PX4-FastRTPS Bridge

The *PX4-FastRTPS Bridge* adds a Real Time Publish Subscribe (RTPS) interface to PX4, enabling the exchange of [uORB messages](../middleware/uorb.md) between PX4 components and (offboard) *Fast-RTPS* applications, including ones built over the ROS2/ROS frameworks.

> **Note** RTPS is the underlying protocol of the Object Management Group's (OMG) Data Distribution Service (DDS) standard. It aims to enable scalable, real-time, dependable, high-performance and interoperable data communication using the publish/subscribe pattern. *FastRTPS* is a very lightweight cross-platform implementation of the latest version of the RTPS protocol and a minimum DDS API.

RTPS has been adopted as the middleware for the ROS2 (Robot Operating System). The bridge allows us to better integrate with ROS2, making it easy to share sensor values, commands, and other vehicle information.

This topic describes the bridge architecture, how it is compiled, and how to:
1. Write a simple FastRTSP application to subscribe to PX4 changes;
2. Or, if your application is on the ROS2/ROS1 side, understand how the `px4_ros_com` package works and how to use it to bridge your ROS nodes with PX4.


## When should RTPS be used?

RTPS should be used in circumstances where there is a need to reliably share time-critical/real-time information between the flight controller and off board components. In particular it is useful in cases where off-board software needs to become a peer software component running in PX4 (by sending and receiving uORB topics).

Possible use cases include communicating with robotics libraries for computer vision, and other use cases where real time data to/from actuators and sensors is essential for vehicle control.

> **Note** FastRTPS is not intended as a replacement for MAVLink. MAVLink remains the most appropriate protocol for communicating with ground stations, gimbals, cameras, etc. (although FastRTPS may open other opportunities for working with some peripherals).

<span></span>
> **Tip** RTPS can be used over slower links (e.g. like radio telemetry, but care should be taken not to overload the channel.


## Architectural overview

![basic example flow](../../assets/middleware/micrortps/architecture.png)

The main elements of the architecture are the client and agent processes shown in the diagram above.

- The *Client* is PX4 middleware daemon process that runs on the flight controller. It subscribes to uORB topics published by other PX4 components and sends any updates to the *Agent* (via a UART or UDP port). It also receives messages from the *Agent* and publishes them as uORB message on PX4.
- The *Agent* runs as a daemon process on an offboard computer. It watches for uORB update messages from the *Client* and (re)publishes them over RTPS. It also subscribes to "uORB" RTPS messages from other RTPS applications and forwards them to the *Client*.
- The *Agent* and *Client* are connected via a serial link (UART) or UDP network. The uORB information is [CDR serialized](https://en.wikipedia.org/wiki/Common_Data_Representation) for sending (*CDR serialization* provides a common format for exchanging serial data between different platforms).
- The *Agent* and any *FastRTPS* applications are connected via UDP, and may be on the same or another device. In a typical configuration they will both be on the same system (e.g. a development computer, Linux companion computer or compute board), connected to the *Client* over a Wifi link or via USB.


## Architectural overview for a ROS2/ROS application pipeline

<!--![basic example flow](../../assets/middleware/micrortps/architecture_ros.png)-->

ROS2 has been developed on top of the DDS/RTPS, which is what composes its middleware. This same middleware serves as the end-to-end architecture for plugging different applications that rely on distributed discovery, serialization and QoS control over the transportation layer.

So one is able to publish and subscribe to uORB data from PX4 using ROS2 (and ROS1 if using the [`ros1_bridge`](https://github.com/ros2/ros1_bridge), it just needs to create it's own listener our advertiser nodes in the respective frameworks. The only consideration to take into account is that the msg types, headers and source files being used on both client and agent side (and consequently, on the ROS side) need to be the same and generated from the same IDL (Interface Description Language).


## Code generation

### Code generation for ROS-independent applications

All the code needed to create, build and use the bridge is automatically generated when the PX4 Firmware is compiled.

The *Client* application is also compiled and built into the firmware as part of the normal build process. The *Agent* must be separately/manually compiled for the target computer.

> **Note** [Fast RTPS must be installed](../setup/fast-rtps-installation.md) in order to generate the required code!

<span></span>
> **Tip** The bridge code can also be [manually generated](micrortps_manual_code_generation.md). Most users will not need to do
so, but the linked topic provides a more detailed overview of the build process and can be useful for troubleshooting.

### Code generation for ROS2/ROS applications - `px4_ros_com`

The [`px4_ros_com`](https://github.com/PX4/px4_ros_com) package allows the developer to generate all the required ROS2/ROS message headers and sources, but also the agent application which will bridge the client with the RTPS middleware on the ROS2 side. The package is basically composed by two separate branches, where the `master` branch generates all the required ROS2 messages and IDL files, besides some listener and advertiser example nodes, in order to bridge PX4 with ROS2 nodes, and a `ros1` branch, which similarly to the `master` branch, generates the ROS(1) message headers and source files, besides the ROS nodes examples, that allow, by using the `ros1_bridge`, share data between PX4 and ROS.

The `px4_ros_com` package build structure is structured in a way that, when one builds the package, it automatically generates all the required components of the bridge that allow the developer to use the generated messages on its own nodes, being it on the ROS2 or in the ROS side.

## Supported uORB messages

The generated bridge code will enable a specified subset of uORB topics to be published/subscribed via RTPS. This applicable on both ROS or non-ROS applications.

For *automatic code generation* (via the normal PX4 Firmware build process or, in the case of the `px4_ros_com`, through the build process of the package), there's a yaml definition file on the PX4 Firmware directory called `uorb_rtps_message_ids.yaml`, found in **/path/to/PX4/Firmware/msg/tools/** dir, where all the uORB messages (RTPS) ID's are set and where is also set which messages should be set to be sent, received, or both, in the RTPS stream.

> **Caution** All new uORB messages which one adds to the Firmware side should have a respective RTPS ID set when one wants the data to be published and/or subscribed in the RTPS stream.

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
```

In the case of `px4_ros_com`, the `uorb_rtps_message_ids.yaml` is transformed in a way that the message names become PascalCased and there's a swap between the received and the sent messages. The logic behind this is that if a message is sent from the client side, then it's received on the agent side, and vice-versa. The naming of the message is indifferent on the client-agent communication, but it is critical on the ROS2 side, since the message naming has to follow the the PascalCase convention, which means that the generated IDL files also need to follow this same convention.

This simply means that, between agent and client, there are IDL files which are generated on the build process of PX4 Firmware and which are used only for communicate between each other, and from the agent side to the ROS2 RTPS middleware, there's another set of IDL files generated that are compatible with the ROS2 conventions. In the end, what really matters is that the generated IDL keep the same ID's and fields for each message.


## Client (PX4 Firmware)

The *Client* source code is generated, compiled and built into the PX4 firmware as part of the normal build process.

To build the firmware for NuttX/Pixhawk flight controllers:
```sh
make px4fmu-v4_rtps
```

To build the firmware for a SITL POSIX target:
```sh
make posix_sitl_rtps
```

The *Client* application can be launched from [NuttShell/System Console](../debug/system_console.md). The command syntax is shown below (you can specify a variable number of arguments):

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

> **Note** By default the *Client* runs as a daemon, but you will need to start it manually. The PX4 Firmware initialization code may in future automatically start the *Client* as a permanent daemon process.

For example, in order to run the *Client* daemon with SITL, and since the data is being shared through UDP, it's required that the daemon is started using the UDP as the transport protocol:

```sh
micrortps_client start -t UDP
```

## Agent in a ROS-independent Offboard Fast-RTPS interface

The *Agent* code is automatically *generated* when you build the associated PX4 firmware. You can find the source here: **build/<target-platform>/src/modules/micrortps_bridge/micrortps_client/micrortps_agent/**.

To build the *Agent* application, compile the code:

```sh
cd build/<target-platform>/src/modules/micrortps_bridge/micrortps_client/micrortps_agent
mkdir build && cd build
cmake ..
make
```

> **Note** To cross-compile for the *Qualcomm Snapdragon Flight* platform see [this link](https://github.com/eProsima/PX4-FastRTPS-PoC-Snapdragon-UDP#how-to-use).


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

## Agent interfacing with a ROS2 middleware

`px4_ros_com` build structure allows to automatically generate and build the agent application. Since it is also installed using the [`colcon`](http://design.ros2.org/articles/build_tool.html) build tools, running it works exactly the same way as the above. Check the **Building the `px4_ros_com` package** for details about the build structure.


## Building the `px4_ros_com` package

As aforementioned, the `px4_ros_com` comes bundled with two branches, where one links the ROS2 with the PX4-agent/client bridge and the other allows, through `ros1_bridge`, to link the ROS framework with the ROS2 framework through the same set of messages. Therefore, both branches need to be cloned separately so the build process can happen correctly. Before that can happen, one requires to install and setup both ROS2 and ROS environments on its machine.

### Installing ROS and ROS2 and respective dependencies

In order to install ROS Melodic and ROS2 Bouncy on an Ubuntu 18.04 machine, follow the links bellow, respectively:

1. [Install ROS Melodic](http://wiki.ros.org/melodic/Installation/Ubuntu)
2. [Install ROS2 Bouncy](https://index.ros.org/doc/ros2/Linux-Install-Debians/)

For the ROS2 case, so the package properly generates the IDL files, the following should be installed:

```sh
sudo apt install ros-bouncy-rmw-opensplice-cpp
```

The install process should also install the `colcon` build tools, but in case that doesn't happen, you can install it manually:

```sh
sudo apt install python3-colcon-common-extensions
```

> **Note** This install and build guide is also aplicable in an environment with Ubuntu 16.04, ROS Kinetic and ROS2 Ardent installed.

> **Note** ROS2 requires Python3, which will be install on your system. Be aware of any incompatibilities you may find with other packages that were installed, for example by pip, using Python2 and you may want to install using `pip3` so to solve these incompatibilities.

> **Caution** Do not install the `ros1_bridge` package through the deb repository. The package has to be built by source.

### Setting up the workspaces

Since the ROS2 and ROS require different environments to be set, there should exist two different workspaces for each ROS version. As an example:

1. For the ROS2 case, create a workspace using:

```sh
mkdir -p ~/px4_ros_com_ros2/src
```

Then, clone the respective ROS2 (`master`) branch to the `/src` directory:

```sh
$ git clone https://github.com/PX4/px4_ros_com.git ~/px4_ros_com_ros2/src/px4_ros_com # clones the master branch
```

2. For the ROS case, we follow exactly the same process, but for a different directory and cloning a different branch:

```sh
mkdir -p ~/px4_ros_com_ros1/src
```

Then, clone the respective ROS2 (`master`) branch to the `/src` directory:

```sh
$ git clone https://github.com/PX4/px4_ros_com.git ~/px4_ros_com_ros1/src/px4_ros_com # clones the 'ros1' branch
```

### Building the workspaces

For building the workspaces, there's already a script available on the `px4_ros_com` package that can be used to automate the build process. But, for a matter of understanding the process, bellow are the steps to manually build the packages:

1. `cd` into `px4_ros_com_ros2` dir and source the ROS2 environment. Don't mind if it tells you that a previous workspace was set before:

```sh
source /opt/ros/bouncy/setup.bash
```

2. Clone the `ros1_bridge` package so it can be built on the ROS2 workspace:

```sh
git clone https://github.com/ros2/ros1_bridge.git ~/px4_ros_com_ros2/src/ros1_bridge
```

3. Build the `px4_ros_com` package, excluding the `ros1_bridge` package:

```sh
colcon build --symlink-install --packages-skip ros1_bridge --event-handlers console_direct+
```

> **Note** `--event-handlers console_direct+` only serves the purpose of adding verbosity to the `colcon` build process and can be removed if one wants a more "quite" build.

4. Then, follows the process of building the ROS(1) packages side. For that, one requires to source the environments so when the `ros1_bridge` is built with support for any messages that are on PATH and have an associated mapping between ROS1 and ROS2:

```sh
source /opt/ros/melodic/setup.bash
source /opt/ros/bouncy/setup.bash
```

5. Build the `px4_ros_com` package on the ROS side:

```sh
cd ~/px4_ros_com_ros1 && colcon build --symlink-install --event-handlers console_direct+
```

6. Complementing 4., one also needs to source the workspaces after those are built:

```sh
source ~/px4_ros_com_ros1/install/setup.bash
source ~/px4_ros_com_ros2/install/setup.bash
```

7. At last, build the `ros1_bridge`. Note that the build process may consume a lot of memory resources, so if one is using a resource limited machine, reduce the number of jobs being processed in parallel, by for example, setting the environmental variable `MAKEFLAGS=-j1`. For more details on the build process, one can consult the build instructions on the [`ros1_bridge`](https://github.com/ros2/ros1_bridge) package page.

```sh
cd ~/px4_ros_com_ros2 && colcon build --symlink-install --packages-select ros1_bridge --cmake-force-configure --event-handlers console_direct+
```

## Creating a Fast-RTPS Listener application

Once the *Client* (on the flight controller) and the *Agent* (on an offboard computer) are running and connected, *FastRTPS* applications can publish and subscribe to uORB topics on PX4 using RTPS.

This example shows how to create a *FastRTPS* "listener" application that subscribes to the `sensor_combined` topic and prints out updates (from PX4). A connected RTPS application can run on any computer on the same network as the *Agent*. For this example the *Agent* and *Listener application* will be on the same computer.

The *fastrtpsgen* script can be used to generate a simple RTPS application from an IDL message file.

> **Note** RTPS messages are defined in IDL files and compiled to C++ using *fastrtpsgen*. As part of building the bridge code, IDL files are generated for the uORB message files that may be sent/received (see **build/BUILDPLATFORM/src/modules/micrortps_bridge/micrortps_agent/idl/*.idl**). These IDL files are needed when you create a FastRTPS application to communicate with PX4.

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

* [Throughput test](../middleware/micrortps_throughput_test.md): A simple test to measure the throughput of the bridge.


## Troubleshooting

### Client reports that selected UART port is busy

If the selected UART port is busy, it's possible that the MAVLink application is already being used. If both MAVLink and RTPS connections are required you will have to either move the connection to use another port or configure the port so that it can be shared. <!-- https://github.com/PX4/Devguide/issues/233 -->

> **Tip** A quick/temporary fix to allow bridge testing during development is to stop MAVLink from *NuttShell*:
  ```sh
  mavlink stop-all
  ```

### Agent not built/fastrtpsgen is not found

The *Agent* code is generated using a *FastRTPS* tool called *fastrtpsgen*.

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

2. For the Raspberry Pi in specific, you need to stop the GPIO serial console that is using the port:

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
