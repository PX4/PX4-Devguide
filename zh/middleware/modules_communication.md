!REDIRECT "https://docs.px4.io/master/zh/middleware/modules_communication.html"

# 模块参考：通信（Communication）

## frsky_telemetry

Source: [drivers/telemetry/frsky_telemetry](https://github.com/PX4/Firmware/tree/master/src/drivers/telemetry/frsky_telemetry)

FrSky 数传支持， 会自动检测使用 D.PORT 还是 S.PORT 协议。
<a id="frsky_telemetry_usage"></a>

### Usage

    frsky_telemetry <command> [arguments...]
     Commands:
       start
         [-d <val>]  Select Serial Device
                     values: <file:dev>, default: /dev/ttyS6
         [-t <val>]  Scanning timeout [s] (default: no timeout)
                     default: 0
         [-m <val>]  Select protocol (default: auto-detect)
                     values: sport|sport_single|sport_single_invert|dtype, default:
                     auto
    
       stop
    
       status
    

## mavlink

Source: [modules/mavlink](https://github.com/PX4/Firmware/tree/master/src/modules/mavlink)

### 描述

此模块实现了 MAVLink 协议，该协议可在串口或 UDP 网络上使用。 它通过 uORB 实现与系统的通信：部分消息会在此模块中直接进行处理 （例，mission protocol），其它消息将通过 uORB 发布出去 （例，vehicle_command）。

流（Stream）被用来以特定速率发送周期性的消息，例如飞机姿态信息。 在启动 mavlink 实例时可以设定一个模式，该模式定义了所启用的流集合的发送速率。 对于一个正在运行的实例而言，可以使用 `mavlink stream` 命令来配置流。

可以存在多个该模块的实例，每个实例连接到一个串口设备或者网络端口。

### 实现

命令的具体实现使用了两个线程，分别为数据发送线程和接收线程。 发送线程以一个固定的速率运行，并会在组合带宽（combined bandwidth）高于设定速率(`-r`)，或者物理链路出现饱和的情况下动态降低信息流的发送速率。 可使用 `mavlink status` 命令检查是否发生降速，如果 `rate mult` 小于 1 则发生了降速。

**Careful**: 两个线程会共同访问和修改某些数据，在修改代码或者扩展功能是需要考虑到这一点以避免出现资源竞争（race conditions）或者造成数据损坏（corrupt data）。

### 示例

在 ttyS1 串口启动 mavlink ，并设定波特率为 921600、最大发送速率为 80kB/s：

    mavlink start -d /dev/ttyS1 -b 921600 -m onboard -r 80000
    

在 UDP 端口 14556 启动 mavlink 并启用 50Hz 的 HIGHRES_IMU 消息：

    mavlink start -u 14556 -r 1000000
    mavlink stream -u 14556 -s HIGHRES_IMU -r 50
    

<a id="mavlink_usage"></a>

### Usage

    mavlink <command> [arguments...]
     Commands:
       start         Start a new instance
         [-d <val>]  Select Serial Device
                     values: <file:dev>, default: /dev/ttyS1
         [-b <val>]  Baudrate (can also be p:<param_name>)
                     default: 57600
         [-r <val>]  Maximum sending data rate in B/s (if 0, use baudrate / 20)
                     default: 0
         [-u <val>]  Select UDP Network Port (local)
                     default: 14556
         [-o <val>]  Select UDP Network Port (remote)
                     default: 14550
         [-t <val>]  Partner IP (broadcasting can be enabled via MAV_BROADCAST
                     param)
                     default: 127.0.0.1
         [-m <val>]  Mode: sets default streams and rates
                     values:
                     custom|camera|onboard|osd|magic|config|iridium|minimal|extvsisi
                     on, default: normal
         [-n <val>]  wifi/ethernet interface name
                     values: <interface_name>
         [-c <val>]  Multicast address (multicasting can be enabled via
                     MAV_BROADCAST param)
                     values: Multicast address in the range
                     [239.0.0.0,239.255.255.255]
         [-f]        Enable message forwarding to other Mavlink instances
         [-w]        Wait to send, until first message received
         [-x]        Enable FTP
         [-z]        Force hardware flow control always on
         [-Z]        Force hardware flow control always off
    
       stop-all      Stop all instances
    
       status        Print status for all instances
         [streams]   Print all enabled streams
    
       stream        Configure the sending rate of a stream for a running instance
         [-u <val>]  Select Mavlink instance via local Network Port
         [-d <val>]  Select Mavlink instance via Serial Device
                     values: <file:dev>
         -s <val>    Mavlink stream to configure
         -r <val>    Rate in Hz (0 = turn off, -1 = set to default)
    
       boot_complete Enable sending of messages. (必须) 作为启动脚本的最后一步被调用。
    

## micrortps_client

Source: [modules/micrortps_bridge/micrortps_client](https://github.com/PX4/Firmware/tree/master/src/modules/micrortps_bridge/micrortps_client)

<a id="micrortps_client_usage"></a>

### Usage

    micrortps_client <command> [arguments...]
     Commands:
       start
         [-t <val>]  Transport protocol
                     values: UART|UDP, default: UART
         [-d <val>]  Select Serial Device
                     values: <file:dev>, default: /dev/ttyACM0
         [-b <val>]  Baudrate (can also be p:<param_name>)
                     default: 460800
         [-p <val>]  Poll timeout for UART in ms
         [-l <val>]  Limit number of iterations until the program exits
                     (-1=infinite)
                     default: 10000
         [-w <val>]  Time in ms for which each iteration sleeps
                     default: 1
         [-r <val>]  Select UDP Network Port for receiving (local)
                     default: 2019
         [-s <val>]  Select UDP Network Port for sending (remote)
                     default: 2020
         [-i <val>]  Select IP address (remote)
                     values: <x.x.x.x>, default: 127.0.0.1
         [-f]        Activate UART link SW flow control
         [-h]        Activate UART link HW flow control
         [-v]        Add more verbosity
    
       stop
    
       status
    

## uorb

Source: [modules/uORB](https://github.com/PX4/Firmware/tree/master/src/modules/uORB)

### 描述

uORB is the internal pub-sub messaging system, used for communication between modules.

It is typically started as one of the very first modules and most other modules depend on it.

### 实现

No thread or work queue is needed, the module start only makes sure to initialize the shared global state. Communication is done via shared memory. The implementation is asynchronous and lock-free, ie. a publisher does not wait for a subscriber and vice versa. This is achieved by having a separate buffer between a publisher and a subscriber.

The code is optimized to minimize the memory footprint and the latency to exchange messages.

The interface is based on file descriptors: internally it uses `read`, `write` and `ioctl`. Except for the publications, which use `orb_advert_t` handles, so that they can be used from interrupts as well (on NuttX).

Messages are defined in the `/msg` directory. They are converted into C/C++ code at build-time.

If compiled with ORB_USE_PUBLISHER_RULES, a file with uORB publication rules can be used to configure which modules are allowed to publish which topics. This is used for system-wide replay.

### 示例

Monitor topic publication rates. Besides `top`, this is an important command for general system inspection:

    uorb top
    

<a id="uorb_usage"></a>

### Usage

    uorb <command> [arguments...]
     Commands:
       start
    
       status        Print topic statistics
    
       top           Monitor topic publication rates
         [-a]        print all instead of only currently publishing topics with
                     subscribers
         [-1]        run only once, then exit
         [<filter1> [<filter2>]] topic(s) to match (implies -a)