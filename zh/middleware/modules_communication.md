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

    mavlink &lt;command&gt; [arguments...]
     Commands:
       start         开始新的实例
         [-d &lt;val&gt;]  选择串口设备
                     取值： &lt;file:dev&gt;, 默认值： /dev/ttyS1
         [-b &lt;val&gt;]  波特率 (也可设置为 p:&lt;param_name&gt;)
                     默认值： 57600
         [-r &lt;val&gt;]  最大数据发送速率，单位 B/s (如为 0, 则使用 波特率 / 20)
                     默认值： 0
         [-u &lt;val&gt;]  选择 UDP 网络端口 (local)
                     默认值：14556
         [-o &lt;val&gt;]  选择 UDP 网络端口 (remote)
                     默认值：14550
         [-t &lt;val&gt;]  Partner IP (可使用 MAV_BROADCAST 参数启用广播)
                     默认值： 127.0.0.1
         [-m &lt;val&gt;]  模式：设置默认的流和速率
                     取值： custom|camera|onboard|osd|magic|config|iridium|minimal,
                     默认值： normal
         [-n &lt;val&gt;]  wifi/以太网 接口名称
                     取值： &lt;interface_name&gt;
         [-c &lt;val&gt;]  组播（Multicast）地址 (可使用 MAV_BROADCAST 参数启用组播)
                     取值： 组播地址区间
                     [239.0.0.0,239.255.255.255]
         [-f]        启用将信息推送到其它 Mavlink 实例
         [-w]       直到收到第一条消息才开始发送
         [-x]        启用 FTP
         [-z]        强制一直开启流量控制
    
       stop-all      停止所有实例
    
       status        打印所有实例的状态
         [streams]   打印所有启用的流
    
       stream        配置一个正在运行的示例的流的发送速率
         [-u &lt;val&gt;]  通过本地网络端口选择 Mavlink 实例
                     默认值： 0
         [-d &lt;val&gt;]  通过串口设备选择 Mavlink 实例
                     取值： &lt;file:dev&gt;
         -s &lt;val&gt;    待配置的 Mavlink 流
         -r &lt;val&gt;    速率，单位 Hz (0 = 关闭发送, -1 = 设为默认值)
    
       boot_complete 允许发送消息。 (必须) 作为启动脚本的最后一步被调用。
    

## micrortps_client

源码：[modules/micrortps_bridge/micrortps_client](https://github.com/PX4/Firmware/tree/master/src/modules/micrortps_bridge/micrortps_client)

### 用法 {#micrortps_client_usage}

    micrortps_client &lt;command&gt; [arguments...]
     Commands:
       start
         [-t &lt;val&gt;]  传输协议
                     取值： UART|UDP, 默认值： UART
         [-d &lt;val&gt;]  选择串口设备
                      取值：&lt;file:dev&gt;, 默认值：/dev/ttyACM0
         [-b &lt;val&gt;]  波特率 (也可设置为 p:&lt;param_name&gt;)
                     默认值： 460800
         [-p &lt;val&gt;]  UART设备轮询时间 ，单位 ms
                     默认值： 1
         [-u &lt;val&gt;]  所有发送主题更新速率限制区间，单位 ms
                     (0=不限制)
                     默认值： 0
         [-l &lt;val&gt;]  程序退出前的迭代次数限制
                     (-1=无限)
                     默认值： 10000
         [-w &lt;val&gt;]  每次循环的休眠时间，单位 ms
                     默认值： 1
         [-r &lt;val&gt;]  选择 UDP 接收端口 (local)
                     默认值：2019
         [-s &lt;val&gt;]  选择 UDP 发送端口 (remote)
                     默认值： 2020
    
       stop
    
       status
    

## uorb

源码：[modules/uORB](https://github.com/PX4/Firmware/tree/master/src/modules/uORB)

### 描述

uORB 是各模块之间进行通讯的基于 发布-订阅 机制的内部消息传递系统。

uORB 模块通常作为第一个模块启动，并且绝大多数其它模块均依赖于它，

### 实现

No thread or work queue is needed, the module start only makes sure to initialize the shared global state. Communication is done via shared memory. The implementation is asynchronous and lock-free, ie. a publisher does not wait for a subscriber and vice versa. This is achieved by having a separate buffer between a publisher and a subscriber.

The code is optimized to minimize the memory footprint and the latency to exchange messages.

The interface is based on file descriptors: internally it uses `read`, `write` and `ioctl`. Except for the publications, which use `orb_advert_t` handles, so that they can be used from interrupts as well (on NuttX).

Messages are defined in the `/msg` directory. They are converted into C/C++ code at build-time.

If compiled with ORB_USE_PUBLISHER_RULES, a file with uORB publication rules can be used to configure which modules are allowed to publish which topics. This is used for system-wide replay.

### 示例

Monitor topic publication rates. Besides `top`, this is an important command for general system inspection:

    uorb top
    

### 用法 {#uorb_usage}

    uorb <command> [arguments...]
     Commands:
       start
    
       status        Print topic statistics
    
       top           Monitor topic publication rates
         [-a]        print all instead of only currently publishing topics
         [<filter1> [<filter2>]] topic(s) to match (implies -a)