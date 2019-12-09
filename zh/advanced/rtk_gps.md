# RTK GPS（背景知识）

实时载波相位差分定位能够提供厘米级的定位信息。 这一章节将介绍RTK是如何集成到PX4中的。

> 注意：RTK的使用说明可以在PX4的用户指南中找到。

## 综述

RTK是使用导航信号的载波相位来进行测距的，而不是使用导航信号所搭载的信息。 多个移动的用户可以共用同一个差分基准站发播的差分修正信息，移动用户离差分基准站的距离越近，差分定位越精确。

在PX4系统中，为达到RTK的差分效果，需要2个RTK GPS模块和一个数据链路。 固定在地面的RTK GPS模块称作基站，另一个在空中的模块称作移动站。 基站通过USB接口与QGC地面站连接，同时利用数据链将RTCM协议修正信息发送给移动站（使用MAVLink中GPS_RTCM_DATA消息）。 在自驾仪上，MAVLink消息包被解包得到RTCM的修正信息，并把这些信息发送给移动站，移动站结合修正信息最终解算得到RTK解。

数据链通常能够处理上行数据率为300字节每秒的数据（更详细的信息参考下面的上行数据率章节）。

## 支持的 RTK GPS 模块

PX4目前仅支持u-blox M8P单频（L1频点）RTK接收机。

许多制造商都用这种接收器来制造产品。 下面列举的这些设备是经过我们测试的可以在 [用户手册](https://docs.px4.io/master/en/gps_compass/rtk_gps.html#supported-rtk-devices) 找到。

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

1005信息长度固定为22字节，而其他两个信息依赖于可用卫星的个数和每颗卫星的信号通道数（频点数），针对M8P只有L1即一个频点。 在真实环境中，对于任一时刻，任何一个导航系统的可用卫星个数不超过12个，因此300B/s的上行速率就足够了。

如果使用 *MAVLink 1* ，则不论其长度，每条 RTCM 消息都会发送182字节 `GPS_RTCM_DATA` 消息。 因此，大约每秒上行需求是700多个字节。 这可能导致低带宽半双轨遥测模块 (如3DR Telemetry Radios) 连接的饱和。

如果 *MAVLink 2* 被使用，则 `GPS_RTCM_DATA消息` 中的所有空格将被删除。 由此产生的上行链路需求与理论值 (~300 字节/秒) 大致相同。

> **Tip** 如果 GCS 和数传模块支持，PX4 会自动切换到 MAVLink 2。

MAVLink 2 必须用于低带宽链接以保证 RTK 性能。 必须注意确保数传链在整个过程中使用 MAVLink 2。 您可以使用系统控制台上的 `mavlink status` 命令验证协议版本：

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