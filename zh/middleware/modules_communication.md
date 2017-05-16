# 模块参考：通信
## mavlink
源代码: [modules/mavlink](https://github.com/PX4/Firmware/tree/master/src/modules/mavlink)


### 说明
该模块实现MAVLink协议，该协议可以在串行链路或UDP网络连接上使用。它通过uORB与系统通信：一些消息直接在模块中处理（例如，任务协议），其他则通过uORB发布（例如，vehicle_command）。

流用于发送指定速率的周期消息，例如姿态信息。启动mavlink实例时，可以指定一个模式，它定义了一组可用的流以及对应的速率。

对于正在运行的实例，流可以通过`mavlink stream`命令进行配置。

该模块可以有多个独立的实例，每个实例连接到一个串行设备或网络端口。

### 实现
该实现使用2个线程，一个发送和一个接收。 发送线程以固定的速率运行，如果混合带宽高于配置的速率（`-r`）或者物理链路饱和，还可以动态地降低流的速率。这可以用`mavlink status`检查，查看`rate mult`是否小于1。

### 示例
在ttyS1串口上启动mavlink，波特率为921600，最大发送速率为80kB / s：
```
mavlink start -d /dev/ttyS1 -b 921600 -m onboard -r 80000
```

在UDP 14556端口上启动mavlink，并以50Hz的速率发送HIGHRES_IMU消息：
```
mavlink start -u 14556 -r 1000000
mavlink stream -u 14556 -s HIGHRES_IMU -r 50
```

### 用法
```
mavlink <命令> [参数...]
 命令:
   start         启动一个新的实例
     [-d <val>]  选择串行设备
                 可选: <file:dev>, 缺省: /dev/ttyS1
     [-b <val>]  波特率
                 缺省: 57600
     [-r <val>]  最大发送速率，单位：B/s (如果为0，使用波特率/20)
                 缺省: 0
     [-u <val>]  选择UDP网络端口(本地)
                 缺省: 14556
     [-o <val>]  选择UDP网络端口(远端)
                 缺省: 14550
     [-t <val>]  伴随IP(可以通过MAV_BROADCAST参数启用广播)
                 缺省: 127.0.0.1
     [-m <val>]  模式：设置缺省流以及速率
                 可选: custom|camera|onboard|osd|magic|config|iridium
                 缺省: normal
     [-f]        启用到其他Mavlink实例的消息转发
     [-v]        详细输出
     [-w]        等待发送，直到收到第一条消息
     [-x]        使能FTP

   stop-all      停止所有实例

   status        打印所有实例的状态

   stream        配置正在运行的实例的流的发送速率
     [-u <val>]  通过本地网络端口号选择Mavlink实例
                 缺省: 0
     [-d <val>]  通过串行设备选择Mavlink实例
                 可选: <file:dev>
     -s <val>    需要配置的Mavlink流
     -r <val>    速率，单位Hz (值为0代表关闭流)

   boot_complete 启用消息发送。（必须）在启动脚本的最后一步调用。
```
## uorb
源代码: [modules/uORB](https://github.com/PX4/Firmware/tree/master/src/modules/uORB)


### 说明
uORB是内部发布-订阅消息系统，用于模块之间的通信。

它通常作为前期模块之一启动，大多数模块依赖于它。

### 实现
不需要线程或工作队列，该模块启动只需要确保初始化共享全局状态。通过共享内存进行通信。该实现是异步和无锁的，即，发布者不需要等待订阅者，反之亦然。这通过在发布者和订阅者之间设置单独的缓冲器来实现。

代码经过优化，使得内存占用空间最小以及消息交换延迟最低。

该接口基于文件描述符：内部使用`read`，`write`和`ioctl`。除了发布者，其使用`orb_advert_t`处理，以便它们可以从中断使用（在NuttX上）。

消息在`/msg`目录中定义。 它们在构建时被转换成C/C ++代码。

如果使用ORB_USE_PUBLISHER_RULES编译，则可以使用具有uORB发布规则的文件配置哪个模块允许发布哪些主题。这用于系统范围的重播。

### 示例
监控主题发布速率。除了`top`，它同样是一般系统检查的重要命令：
```
uorb top
```

### 用法
```
uorb <命令> [参数...]
 命令:
   start

   status        打印主题统计数据

   top           监视主题发布速率
     [-a]        打印全部主题，而不仅仅是当前发布的主题
     [<filter1> [<filter2>]] 主题过滤器 (包含 -a)
```
