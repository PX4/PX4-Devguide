---
translated_page: https://github.com/PX4/Devguide/blob/master/en/tutorials/tutorials.md
translated_sha: bacc884dcc91b626c0c5a668068442178d986e38
---

# u-blox M8P RTK GPS 配置

RTK（Real Time Kinematic）可将GPS精度提高到厘米级。 它使用信号载波相位的测量值，而不是信号的信息内容，并依靠单个参考站提供实时校正，提供高达厘米级的定位精度。

PX4目前仅支持基于u-blox M8P的单频（L1） GNSS接收器用于RTK。

> **Note** 本页面介绍如何将RTK集成到PX4中，如果你只想知道如何使用它，请阅读PX4用户指南中的[相关页面]((https://docs.px4.io/en/advanced_features/rtk-gps.html))。

需要两个M8P GPS模块和数据链路才能使用PX4设置RTK。地面上的GPS单元（固定位置）称为基站(Base)，空中的GPS单元称为流动站(Rover)。基站连接到QGroundControl（通过USB），并使用数据链路向飞行器传输RTCM校正数据（使用MAVLink传过来的 `GPS_RTCM_DATA`消息）。在自驾仪上，MAVLink数据包被解包并发送到机载GNSS单元，在那里进行处理以获得RTK解决方案。

数据链路通常应能够处理每秒300字节的上行速率。 更多有关信息，请参阅下面的`上行数据速率`部分。

### 自动配置


QGroundControl和自驾仪固件共享相同的[PX4 GPS驱动程序栈](https://github.com/PX4/GpsDrivers)。 实际上，这意味着对新协议和/或消息的支持只需要添加到一个地方。


PX4 GPS栈自动设置了u-blox M8P模块，其通过UART或USB发送和接收正确的消息，具体取决于模块连接的位置（QGroundControl或自驾仪）。不需要使用U-Center的配置。


一旦自动驾驶仪接收到`GPS_RTCM_DATA` mavlink消息，它将自动将RTCM数据转发到附加的GPS模块。

> **Note**  u-blox有两种M8P芯片，即M8P-0和M8P-2。 M8P-0只能用作流动站，而不能用作基地，而M8P-2可以用作流动站或基地。

#### RTCM 消息

QGroundControl配置RTK基站以输出以下RTCM3.2消息帧，每帧为1 Hz：
- **1005** - Station coordinates XYZ for antenna reference point (Base position).
- **1077** - Full GPS pseudo-ranges, carrier phases, Doppler and signal strength (high resolution).
- **1087** - Full GLONASS pseudo-ranges, carrier phases, Doppler and signal strength (high resolution).


### Uplink Datarate

The raw RTCM messages from the base are packed into a MAVLink `GPS_RTCM_DATA` message and sent over the datalink. The length of each MAVLink message is 182 bytes, and it encapsulates RTCM messages in its body. Depending on the RTCM message, the MAVLink message is almost never completely filled.

The Base Position message (1005) is of length 22 bytes, while the others are all of variable length depending on the number of visible satellites and the number of signals from the satellite (only 1 for L1 units like M8P). Since at a given time, the _maximum_ number of satellites visible from any single constellation is 12, under real-world conditions, an uplink rate of 300 B/s is sufficient in theory.

If **MAVLink 1** is used, no packet truncation is done. Therefore the whole 182-byte `GPS_RTCM_DATA` message is sent for every RTCM message. This means that the approximate uplink requirement is increased to 700+ bytes per second, which can lead to link saturation on low-bandwidth half-duplex telemetry modules like 3DR radios.

If **MAVLink 2** is used (PX4 automatically switches to MAVLink 2 if the GCS and telemetry modules support it), empty space in a packet is truncated, leading to a much leaner uplink requirement of ~300 bytes per second. It is important that MAVLink 2 is used  on low-bandwidth links for good RTK performance. So care must be taken to make sure that the telemetry chain uses MAVLink 2 throughout. You can verify the protocol version by using the `mavlink status` command on the system console : 

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

