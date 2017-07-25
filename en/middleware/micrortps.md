# RTPS/ROS2 Interface: PX4-FastRTPS Bridge

The *PX4-FastRTPS Bridge* adds a Real Time Publish Subscribe (RTPS) interface to PX4, enabling the exchange of [uORB messages](../middleware/uorb.md) between PX4 components and (offboard) FastRTPS applications.

> **Note** RTPS is the underlying protocol of the Object Management Group's (OMG) Data Distribution Service (DDS) standard. It aims to enable scalable, real-time, dependable, high-performance and interoperable data communication using the publish/subscribe pattern. *FastRTPS* is a very lightweight cross-platform implementation of the latest version of the RTPS protocol and a minimum DDS API.

RTPS has been adopted as the middleware for the ROS2 (Robot Operating System). The bridge allows us to better integrate with ROS2, making it easy to share sensor values, commands, and other vehicle information.

This topic describes the bridge architecture and how it is compiled (including how to specify which uORB messages are exchanged), and how to write a simple FastRTSP application to subscribe to PX4 changes.


## Architectural overview

![basic example flow](../../assets/middleware/micrortps/basic_example_flow.png)

The main elements of the architecture are the client and agent processes shown in the diagram above.

- The *Client* is PX4 middleware daemon process that runs on the flight controller. It subscribes to uORB topics published by other PX4 components and sends any updates to the *Agent* (via a UART or UDP port). It also receives messages from the *Agent* and publishes them as uORB message on PX4.
- The *Agent* runs as a daemon process on an offboard computer. It watches for uORB update messages from the *Client* and (re)publishes them over RTPS. It also subscribes to "uORB" RTPS messages from other RTPS applications and forwards them to the *Client*.
- The *Agent* and *Client* are connected via a serial link (UART) or UDP network. The uORB information is [CDR serialized](https://en.wikipedia.org/wiki/Common_Data_Representation) for sending (*CDR serialization* provides a common format for exchanging serial data between different platforms).
- The *Agent* and any *FastRTPS* applications are connected via UDP, and may be on the same or another device. In a typical configuration they will both be on the same system (e.g. a development computer, Linux companion computer or compute board), connected to the *Client* over a Wifi link or via USB.


## Code generation

The code needed for the *Client*, *Agent*, *CDR serialization/deserialization* of uORB messages, and the definition of the associated RTPS messages is automatically generated when the PX4 Firmware is compiled (and can also be [generated manually](micrortps_manual_code_generation.md)). 

> **Note** [Fast RTPS must be installed](../setup/fast-rtps-installation.md) in order to generate the required code! 

The source code for the *Client* and *Agent* will be generated in **build\__BUILDPLATFORM_/src/modules/micrortps_bridge**, under the **/micrortps_client** and **/micrortps_agent** folders (respectively).


### Supported uORB/RTSP messages

The set of uORB topics that can be sent and received by the bridge are listed in the **.cmake** file (**cmake/configs**) for each target platform:

```cmake
set(config_rtps_send_topics
  sensor_combined
   # Add new topic...
   )

set(config_rtps_receive_topics
   vehicle_command
   # Add new topic...
   )
```

A uORB serialization function will be generated for each topic in both lists. An IDL file will also be created for each topic from the associated uORB **.msg** files 

> **Note** FastRTSP uses IDL files to define the structure of RTPS messages (in this case, RTPS messages for sending uORB topics). The IDL files are compiled to C++ by the *fastrtpsgen* tool, which is then used by the Agent and other FastRTPS applications.


#### uORB serialization code

Serialization functions are generated for the uORB topics that are to be sent/received. 
For example, the following functions would be generated for the *sensor_combined.msg*:

```sh
void serialize_sensor_combined(const struct sensor_combined_s *input, char *output, uint32_t *length, struct microCDR *microCDRWriter);
void deserialize_sensor_combined(struct sensor_combined_s *output, char *input, struct microCDR *microCDRReader);
```


## Client (PX4 Firmware)

The *Client* source code is generated, and the executable is automatically built and included when you make the firmware (in the normal way). For example, to build and upload the firmware:

For NuttX/Pixhawk flight controllers:
```sh
make px4fmu-v4_default upload
```
For Qualcomm Snapdragon Flight:
```sh
$ make eagle_default upload
  ```

After uploading the firmware, the *Client* application can be launched by typing its name and passing a variable number of arguments as shown below:

```sh
> micrortps_client start|stop [options]
  -t <transport>          [UART|UDP] Default UART
  -d <device>             UART device. Default /dev/ttyACM0
  -u <update_time_ms>     Time in ms for uORB subscribed topics update. Default 0
  -l <loops>              How many iterations will this program have. -1 for infinite. Default 10000
  -w <sleep_time_ms>      Time in ms for which each iteration sleep. Default 1ms
  -b <baudrate>           UART device baudrate. Default 460800
  -p <poll_ms>            Time in ms to poll over UART. Default 1ms
  -r <reception port>     UDP port for receiving. Default 2019
  -s <sending port>       UDP port for sending. Default 2020
```

> **Note** If the selected UART port is busy, it's possible that the MAVLink application is already being used. If it is the case, you can stop MAVLink from NuttShell by typing:
  ```sh
  > mavlink stop-all
  ```


## Agent (Off Board FastRTPS Interface)

The *Agent* code is automatically generated when you compile/build the associated PX4 firmware (into **build_BUILDPLATFORM/src/modules/micrortps_bridge/micrortps_agent/**)

To build the *Agent* application, compile the code:
```sh
cd src/modules/micrortps_bridge/microRTPS_agent
mkdir build && cd build
cmake ..
make
  ```

> **Note** To cross-compile for the *Qualcomm Snapdragon Flight* platform see [this link](https://github.com/eProsima/PX4-FastRTPS-PoC-Snapdragon-UDP#how-to-use).

To launch the *Agent* run:

```text
$ ./micrortps_agent [options]
  -t <transport>          [UART|UDP] Default UART
  -d <device>             UART device. Default /dev/ttyACM0
  -w <sleep_time_us>      Time in us for which each iteration sleep. Default 1ms
  -b <baudrate>           UART device baudrate. Default 460800
  -p <poll_ms>            Time in ms to poll over UART. Default 1ms
  -r <reception port>     UDP port for receiving. Default 2019
  -s <sending port>       UDP port for sending. Default 2020
```


## Creating a Listener application

Once the *Client* is running on the flight controller and the *Agent* on an offboard computer, FastRTPS applications running on the RTPS network can publish and subscribe to uORB topics on PX4 using RTPS. 

> **Note** A connected RTPS application can run on any computer on the same network as the *Agent*. For this example they will be on the same computer.

The *fastrtpsgen* script can be used to generate a simple application from an IDL message file. Below will use it to create a Listener application that subscribes to the `sensor_combined` topic and prints out updates (from PX4). Enter the following commands to create the application:

```sh
cd /path/to/PX4/Firmware/src/modules/micrortps_bridge
mkdir micrortps_listener
cd micrortps_listener
fastrtpsgen -example x64Linux2.6gcc ../micrortps_agent/idl/sensor_combined_.idl
```

This creates a sample subscriber, a publisher and a main-application to run them. To print out the data from the `sensor_combined` topic, modify the `onNewDataMessage()`-method in **sensor_combined_Subscriber.cxx**:

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

To build and run the Listener application:

```sh
make -f makefile_x64Linux2.6gcc
bin/*/sensor_combined_PublisherSubscriber subscriber
```

Now you should see the sensor inforation being printed out:

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

If the Listener does not print anything, make sure the *Client* is running. By default the *Client* runs for 10000 loops and than stops. To run the Client continuously:

```sh
micrortps_client start -l -1
```


## Examples/tests

The following examples provide additional real-world demonstrations of how to use the features described in this topic.

* [Throughput test](../middleware/micrortps_throughput_test.md): A simple simple test to measure the throughput of the bridge.


## Troubleshooting

### Agent not built/fastrtpsgen is not found

The *Agent* code is generated using a *FastRTPS* tool called *fastrtpsgen* (environment variable ).  

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

  In the menu showed go to **Interfacing options > Serial**. Select **NO for *Would you like a login shell to be accessible over serial?*. Valid and reboot.

3. Check UART in kernel:

  ```sh
  sudo vi /boot/config.txt
  ```

  And make sure that the `enable_uart` value is set to 1:
  ```txt
  enable_uart=1
  ```


## Additional information

* [FastRTPS Installation](../setup/fast-rtps-installation.md).
* [DDS and ROS middleware implementations](https://github.com/ros2/ros2/wiki/DDS-and-ROS-middleware-implementations)

