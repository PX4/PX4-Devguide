# RTK GPS（背景知识）

实时载波相位差分定位能够提供厘米级的定位信息。 这一章节将介绍RTK是如何集成到PX4中的。

> 注意：RTK的使用说明可以在PX4的用户指南中找到。

## 综述

RTK是使用导航信号的载波相位来进行测距的，而不是使用导航信号所搭载的信息。 多个移动的用户可以共用同一个差分基准站发播的差分修正信息，移动用户离差分基准站的距离越近，差分定位越精确。

在PX4系统中，为达到RTK的差分效果，需要2个RTK GPS模块和一个数据链路。 固定在地面的RTK GPS模块称作基站，另一个在空中的模块称作移动站。 基站通过USB接口与QGC地面站连接，同时利用数据链将RTCM协议修正信息发送给移动站（使用MAVLink中GPS_RTCM_DATA消息）。 在自驾仪上，MAVLink消息包被解包得到RTCM的修正信息，并把这些信息发送给移动站，移动站结合修正信息最终解算得到RTK解。

数据链通常能够处理上行数据率为300字节每秒的数据（更详细的信息参考下面的上行数据率章节）。

## 支持的 RTK GPS 模块

PX4目前仅支持u-blox M8P单频（L1频点）RTK接收机。

许多制造商都用这种接收器来制造产品。 下面列举的这些设备是经过我们测试的可以[在用户手册中](https://docs.px4.io/en/advanced_features/rtk-gps.html#supported-rtk-devices)找到。

> **注意**u-blox有两种基于M8P芯片的衍生型号：M8P-0 和 M8P-2。 M8P-0只能作为移动端使用，不能作为基站。而M8P-2既可以作为移动端也可以作为基站使用。

## 自动配置

PX4 GPS堆栈自动设置u-blox M8P模块，通过UART或USB发送和接收正确的消息，具体取决于模块的连接位置（* QGroundControl *或自动驾驶仪）。

一旦自动驾驶仪接收到` GPS_RTCM_DATA ` MAVLink消息，它就会自动将RTCM数据转发到附加的GPS模块。

> **Note** 不需要/不使用 U-Center RTK 模块配置工具！

<span></span>

> **Note** * QGroundControl *和自动驾驶仪固件共享相同的[ PX4 GPS驱动程序堆栈](https://github.com/PX4/GpsDrivers)。 实际上，这意味着只需要将新协议和/或消息的支持添加到一个地方。

### RTCM 报文

QGroundControl配置RTK基站输出依据RTCM3.2框架，每帧为1 Hz：

- 1005-差分基准站天线相位中心在地心地固XYZ坐标系下的坐标
- 1077-差分基准站所有可视GPS卫星的伪距、载波相位、多普勒和信号强度
- 1077-差分基准站所有可视GLONASS卫星的伪距、载波相位、多普勒和信号强度

## 上行数据速率

来自差分基准站的RTCM信息，在MAVLink数据链中打包成GPS_RTCM_DATA数据包，并通过MAVLink数据链发播出去。 MAVLink的信息长度最大为182字节。 根据RTCM的信息类型，MAVLink信息是不会填满的。

The RTCM Base Position message (1005) is of length 22 bytes, while the others are all of variable length depending on the number of visible satellites and the number of signals from the satellite (only 1 for L1 units like M8P). Since at a given time, the *maximum* number of satellites visible from any single constellation is 12, under real-world conditions, theoretically an uplink rate of 300 B/s is sufficient.

If *MAVLink 1* is used, a 182-byte `GPS_RTCM_DATA` message is sent for every RTCM message, irrespective of its length. As a result the approximate uplink requirement is around 700+ bytes per second. This can lead to link saturation on low-bandwidth half-duplex telemetry modules (e.g. 3DR Telemetry Radios).

If *MAVLink 2* is used then any empty space in the `GPS_RTCM_DATA message` is removed. The resulting uplink requirement is about the same as the theoretical value (~300 bytes per second).

> **Tip** PX4 automatically switches to MAVLink 2 if the GCS and telemetry modules support it.

MAVLink 2 must be used on low-bandwidth links for good RTK performance. Care must be taken to make sure that the telemetry chain uses MAVLink 2 throughout. You can verify the protocol version by using the `mavlink status` command on the system console:

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