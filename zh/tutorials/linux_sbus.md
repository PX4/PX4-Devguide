# 用于 Linux 的 S.Bus 驱动

*S.Bus Driver for Linux* 允许基于 Linux 的无人机通过串行端口从 *Futaba S.Bus 接收机* 访问多达 16 个通道。 驱动程序还应该与使用 S.Bus 协议的其他接收器一起工作，包括作为 FrSky，RadioLink，甚至是 S.Bus 编码器。

需要信号反相器电路（如下所述）以使器件串行端口能够从接收器读取数据。

> **Note** 当通过板载串行端口或通过 USB 转 TTL 串行电缆连接到接收器时，驱动程序已经在运行 Raspbian Linux的Raspberry Pi 上进行了测试。 它可以在所有 Linux 版本和所有串行端口上运行。

## 信号逆变器电路

S.Bus 是 *inverted* UART 通信信号。 由于许多串行端口/飞行控制器无法读取反向 UART 信号，因此接收器和串行端口之间需要信号反相器电路来反转信号。 本节介绍如何创建适当的电路。

> **Tip** Raspberry Pi 需要此电路才能通过串行端口或 USB-to-TTY 串行转换器读取 S.Bus 远程控制信号。 许多其他飞行控制器也需要它。

### 所需组件

* 1x NPN 晶体管（例如 NPN S9014 TO92） 
* 1x 10K 电阻
* 1x 1K 电阻

> **Note** 可以使用任何类型/型号的晶体管，因为电流消耗非常低。

<span></span>

> **Tip** Raspberry Pi 只有一个串口。 如果已经使用，您可以通过 USB 转 TTY 串行电缆（例如 PL2302 USB 转 TTL 串行转换器）将 S.Bus 接收器连接到 RaPi USB 端口。

### 电路图/连接

按如下所述连接组件（并在电路图中显示）：

* S.Bus 信号&rarr;1K 电阻&rarr;NPN 晶体管
* NPN晶体管发射&rarr;GND
* 3.3VCC＆&rarr; 10K电阻&rarr; NPN晶体管集合&rarr; USB-to-TTY rxd
* 5.0VCC&rarr;S.Bus VCC
* GND &rarr; S.Bus GND

![信号逆变器电路](../../assets/driver_sbus_signal_inverter_circuit_diagram.png)

### 电路板图像

下图显示了电路板上的连接。

![信号逆变器电路板](../../assets/driver_sbus_signal_inverter_breadboard.png)

## 源代码

* [Firmware/src/drivers/linux_sbus](https://github.com/PX4/Firmware/tree/master/src/drivers/linux_sbus)

## 用法

命令语法是：

    linux_sbus start|stop|status -d <device> -c <channel>
    

因此，例如，要在设备 `/dev/ttyUSB0` 上自动启动侦听 8 个通道的驱动程序，您可以将以下行添加到启动配置文件中。

    linux_sbus start -d /dev/ttyUSB0 -c 8
    

> **Note** 原始配置文件位于 **Firmware / posix-configs** 中。 根据官方文档，在完成 `make upload` 相关操作后，所有与 posix 相关的配置文件将被放置在 **/home/pi** 中。 您可以修改要在那里使用的文件。