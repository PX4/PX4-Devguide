# 模块参考：通信（Communication）

## frsky_telemetry

源码：[drivers/telemetry/frsky_telemetry](https://github.com/PX4/Firmware/tree/master/src/drivers/telemetry/frsky_telemetry)

FrSky 数传支持， 会自动检测使用 D.PORT 还是 S.PORT 协议。

### 用法 {#frsky_telemetry_usage}

    frsky_telemetry &lt;command&gt; [arguments...]
     Commands:
       start
         [-d &lt;val&gt;]  选择串口设备
                    取值 &lt;file:dev&gt;, 默认值： /dev/ttyS6
         [-t &lt;val&gt;]  扫描超时时间 [s] (默认值： no timeout)
                     default: 0
         [-m &lt;val&gt;]  选择通信协议 (默认值：auto-detect)
                     取值： sport|sport_single|dtype, 默认值： auto
    
       stop
    
       status
    

## mavlink

源码： [modules/mavlink](https://github.com/PX4/Firmware/tree/master/src/modules/mavlink)

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
    

### 用法 {#mavlink_usage}

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
         [-z]        Force flow control always on
    
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

源码：[modules/micrortps_bridge/micrortps_client](https://github.com/PX4/Firmware/tree/master/src/modules/micrortps_bridge/micrortps_client)

### 用法 {#micrortps_client_usage}

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
    
       stop
    
       status
    

## uorb

源码：[modules/uORB](https://github.com/PX4/Firmware/tree/master/src/modules/uORB)

### 描述

uORB 是各模块之间进行通讯的基于 发布-订阅 机制的内部消息传递系统。

uORB 模块通常作为第一个模块启动，并且绝大多数其它模块均依赖于它，

### 实现

不需要任何线程或工作队列， 该模块的启动只是确保初始化共享全局状态（shared global state）。 通信是通过共享内存（shared memory）完成的。 模块的实现是异步的，且无需进行锁定，例如， 发布者不需要等待订阅者，反之也成立。 这一特性是通过在发布者和订阅者之间建立单独的缓冲区来实现的。

我们对代码以最大限度地减少内存占用和交换消息的延迟为目标进行了优化。

该接口基于文件描述符（file descriptor）实现：它在内部使用 `read`、`write` 和 `ioctl`。 唯一例外的是数据的发布，它使用了 `orb_advert_t` 句柄以使得其也可以从中断中使用（在 Nuttx 平台上）。

消息在 `/msg` 文件夹下定义。 在构建时它们会被转化为 C/C++ 代码。

如果使用 ORB_USE_PUBLISHER_RULES 进行编译，那么可以使用一个包含了 uORB 发布规则的文件来配置允许哪些模块发布哪些主题。 这可以用于全系统范围的回放。

### 示例

监控主题发布速率。 除了 `top`命令，这也是进行常规系统检查的一个重要命令：

    uorb top
    

### 用法 {#uorb_usage}

    uorb <command> [arguments...]
     Commands:
       start
    
       status        Print topic statistics
    
       top           Monitor topic publication rates
         [-a]        print all instead of only currently publishing topics
         [-1]        run only once, then exit
         [<filter1> [<filter2>]] topic(s) to match (implies -a)