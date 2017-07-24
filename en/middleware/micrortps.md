# RTPS/ROS2 interface: the PX4-FastRTPS bridge

This bridge adds communication capabilities between a *PX4 Autopilot* and a *Fast RTPS* application, through serial ports or
UDP sockets, using [CDR serialization](https://en.wikipedia.org/wiki/Common_Data_Representation). The goal is to provide a RTPS (Real Time Publish Subscribe Protocol) interface to PX4. This interface will also allow sharing information with the forthcoming release of ROS2 (Robot Operating System).

RTPS is the underlying protocol of DDS, an standard from the OMG (Object Management Group) providing a real-time publish/subscribe middleware that is widely used in aerospace, defense and IoT applications. It has also been adopted as the middleware for the ROS2 robotics toolkit.

Fast RTPS implements the latest version of the RTPS protocol and a minimum DDS API, resulting in a very lightweight implementation of the standard and full access to the RTPS extensive configurability.

![basic example flow](../../assets/middleware/micrortps/basic_example_flow.png)

## Code generation

Support for the functionality is mainly implemented within three new (automatically generated) code blocks:

- *CDR serialization functions* are generated for specified uORB topics (those that are to be sent/received). *CDR serialization* provides a common format for exchanging serial data between different platforms. For example, the following functions are generated for the *sensor_combined.msg*:

  ```sh
  void serialize_sensor_combined(const struct sensor_combined_s *input, char *output, uint32_t *length, struct microCDR *microCDRWriter);
  void deserialize_sensor_combined(struct sensor_combined_s *output, char *input, struct microCDR *microCDRReader);
  ```

- The *Client* acts as a bridge between uORB-messages on the flight-controller and CDR-messages from either UART or UDP.

- The *Agent* similarly acts as a bridge between Fast RTPS messages on the offboard computer and CDR-messages on either UART or UDP.

These pieces of code are generated within the normal PX4 Firmware generation process. They can also can be [generated manually](micrortps_manual_code_generation.md).

### Automatically generate Client and Agent

> **Note** Before continuing we need to have [installed Fast RTPS](../setup/fast-rtps-installation.md).

The code needed for the Client, Agent, and CDR serialization is automatically generated when the PX4 Firmware is compiled. The topics which will be handled by the bridge are listed in the **.cmake** file (**cmake/configs**) for each target platform:

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

The Client application will be generated in *build_OURPLATFORM/src/modules/micrortps_bridge/micrortps_client/* folder and the Agent will be created in *src/modules/micrortps_bridge/micrortps_agent/* folder.

> **Note** To generate the Agent we use a Fast RTPS tool called *fastrtpsgen*. If you haven't installed Fast RTPS in the default path you need to specify the directory of installation of *fastrtpsgen* setting the environment variable `FASTRTPSGEN_DIR` before executing *make*.
> On Linux/Mac this is done as shown below:
>
>  ```sh
  export FASTRTPSGEN_DIR=/path/to/fastrtps/install/folder/bin
  ```


## PX4 Firmware side: The Client

- Construct and upload the firmware executing, for example:

  ```sh
  # For NuttX/Pixhawk flight controllers:
  $ make px4fmu-v4_default upload
  ```
  ```sh
  # For Snapdragon Flight:
  $ make eagle_default upload
  ```

After uploading the firmware, the application can be launched typing its name and passing a variable number of arguments as shown below:

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


## Fast RTPS side: The Agent

To create the application, compile the code:

  ```sh
  $ cd src/modules/micrortps_bridge/microRTPS_agent
  $ mkdir build && cd build
  $ cmake ..
  $ make
  ```

> **Note** To cross-compile for the Qualcomm Snapdragon Flight platform see [this link](https://github.com/eProsima/PX4-FastRTPS-PoC-Snapdragon-UDP#how-to-use).

To launch the publisher run:

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


## Creating a Listener

Now that we have the Client running on the flight controller and the Agent on an offboard computer, we can create an application to communicate with the flight controller through FastRTPS. The *fastrtpsgen* script allows us to quickly generate a simple application from a IDL message file. We will use it to create a Listener which subscribes to the sensor_combined topic. The Listener can be run on any computer on the same network as the Agent, but here they will be on the same computer.

```sh
$ cd /path/to/PX4/Firmware/src/modules/micrortps_bridge
$ mkdir micrortps_listener
$ cd micrortps_listener
$ fastrtpsgen -example x64Linux2.6gcc ../micrortps_agent/idl/sensor_combined_.idl
```

This creates a sample subscriber, a publisher and a main-application to run them. To print out the data from the sensor_combined topic, we modify the onNewDataMessage-method in sensor_combined_Subscriber.cxx:

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

Now build and run the Listener:

```sh
$ make -f makefile_x64Linux2.6gcc
$ bin/*/sensor_combined_PublisherSubscriber subscriber
```

Now you should see the alititude being printed out by the Listener

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

If the Listener does not print anything, make sure the Client is running. By default the Client runs for 10000 loops and than stops. To run the Client continuously, run
```sh
$ micrortps_client start -l -1
```


## Throughput test

[Throughput test](../middleware/micrortps_throughput_test.md) show some real-world examples of how to use the features described in this topic.


## Troubleshooting

### Extra steps for Raspberry Pi

> **Note** Normally, for UART transport it's necessary set up the UART port in the Raspberry Pi. To enable the serial port available on Raspberry Pi connector:

1. Make sure the `userid` (default is pi) is a member of the `dialout` group:

  ```sh
  $ groups pi
  $ sudo usermod -a -G dialout pi
  ```

2. You need to stop the already running on the GPIO serial console:

  ```sh
  $ sudo raspi-config
  ```

  In the menu showed go to **Interfacing options > Serial**. Select **NO for *Would you like a login shell to be accessible over serial?*. Valid and reboot.

3. Check UART in kernel:

  ```sh
  $ sudo vi /boot/config.txt
  ```

And enable UART setting `enable_uart=1`.
