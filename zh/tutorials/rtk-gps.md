---
translated_page: https://github.com/PX4/Devguide/blob/master/en/tutorials/tutorials.md
translated_sha: bacc884dcc91b626c0c5a668068442178d986e38
---

# u-blox M8P RTK GPS 配置

RTK（Real Time Kinematic）可将GPS精度提高到厘米级。 它使用信号载波相位的测量值，而不是信号的信息内容，并依靠单个参考站提供实时校正，提供高达厘米级的定位精度。

PX4目前仅支持基于u-blox M8P的单频（L1） GNSS接收器用于RTK。

> **Note** 本页面介绍如何将RTK集成到PX4中，如果你只想知道如何使用它，请阅读PX4用户指南中的[相关页面](https://docs.px4.io/en/advanced_features/rtk-gps.html)。

需要两个M8P GPS模块和数据链路才能使用PX4设置RTK。地面上的GPS单元（固定位置）称为基站(Base)，空中的GPS单元称为流动站(Rover)。基站连接到QGroundControl（通过USB），并使用数据链路向飞行器传输RTCM校正数据（使用MAVLink传过来的 `GPS_RTCM_DATA`消息）。在自驾仪上，MAVLink数据包被解包并发送到机载GNSS单元，在那里进行处理以获得RTK解决方案。

数据链路通常应能够处理每秒300字节的上行速率。 更多有关信息，请参阅下面的`上行数据速率`部分。

### 自动配置


QGroundControl和自驾仪固件共享相同的[PX4 GPS驱动程序栈](https://github.com/PX4/GpsDrivers)。 实际上，这意味着对新协议和/或消息的支持只需要添加到一个地方。


PX4 GPS栈自动设置了u-blox M8P模块，其通过UART或USB发送和接收正确的消息，具体取决于模块连接的位置（QGroundControl或自驾仪）。不需要使用U-Center的配置。


一旦自动驾驶仪接收到`GPS_RTCM_DATA` mavlink消息，它将自动将RTCM数据转发到附加的GPS模块。

> **Note**  u-blox有两种M8P芯片，即M8P-0和M8P-2。 M8P-0只能用作流动站，而不能用作基地，而M8P-2可以用作流动站或基地。

#### RTCM 消息

QGroundControl配置RTK基站以输出以下RTCM3.2消息帧，每帧为1 Hz：
- **1005** - 天线参考点的基站坐标XYZ(基本点)。
- **1077** - 全GPS伪距，载波相位，多普勒速度以及信号强度(高精度)
- **1087** - 全GLONASS伪距，载波相位，多普勒速度以及信号强度(高精度)。


### 上行数据速率

来自基站的原始RTCM信息被打包到一个MAVLink消息帧`GPS_RTCM_DATA`中并通过数据链发送出去。每个MAVLink消息长度为182个字节，并将RTCM信息封装到其主体中。根据RTCM信息的特点，MAVLink消息帧不会被填满。

基本位置消息(1005)的长度为22个字节，而根据可见卫星的数量和来自卫星的信号数不同（对于诸如M8P的L1单元仅有1个），其他消息的长度都是可变的。 由于在给定的时间，从任何单个星座可见的`最大`卫星数为12个，在实际情况下，理论上300B / s的上行速率是足够的。



如果使用**MAVLink 1**，则不会进行数据包截断。因此，为每个RTCM信息发送整个182字节的`GPS_RTCM_DATA`消息。这意味着上行速率需要增加到近700+字节每秒，这可能到时低带宽半双工数传模块(如3DR的电台)的链路饱和。



如果使用**MAVLink 2**（如果GCS和数传模块支持，PX4会自动切换到MAVLink 2），数据包中的空闲空间将被截断，从而导致每秒300字节的上行链路需求。为了获得良好的RTK表现，在低带宽链路上使用MAVLink 2显得尤为重要。因此，必须注意确保数传链路在整个过程中使用MAVLink 2。 你也可以使用系统控制台上的`mavlink status`命令验证MAVLink协议的版本：

```
nsh> mavlink status
instance #0:
        GCS heartbeat:  593486 us ago
        mavlink chan: #0
        type:           3DR RADIO
        rssi:           219
        remote rssi:    219
        txbuf:          94
        noise:          61
        remote noise:   58
        rx errors:      0
        fixed:          0
        flow control:   ON
        rates:
        tx: 1.285 kB/s
        txerr: 0.000 kB/s
        rx: 0.021 kB/s
        rate mult: 0.366
        accepting commands: YES
        MAVLink version: 2
        transport protocol: serial (/dev/ttyS1 @57600)
```

