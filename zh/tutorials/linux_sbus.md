# 用于 Linux 的 S.Bus 驱动

*S.Bus Driver for Linux* 允许基于 Linux 的无人机通过串行端口从 *Futaba S.Bus receiver* 访问多达 16 个通道。 驱动程序还应该与使用 S.Bus 协议的其他接收器一起工作，包括作为 FrSky，RadioLink，甚至是 S.Bus 编码器。

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

Connect the components as described below (and shown in the circuit diagram):

* S.Bus signal &rarr; 1K resistor &rarr; NPN transistor base
* NPN transistor emit &rarr; GND
* 3.3VCC &rarr; 10K resistor &rarr; NPN transistor collection &rarr; USB-to-TTY rxd
* 5.0VCC &rarr; S.Bus VCC
* GND &rarr; S.Bus GND

![Signal inverter circuit diagram](../../assets/driver_sbus_signal_inverter_circuit_diagram.png)

### Breadboard image

The image below shows the connections on a breadboard.

![Signal inverter breadboard](../../assets/driver_sbus_signal_inverter_breadboard.png)

## 源代码

* [Firmware/src/drivers/linux_sbus](https://github.com/PX4/Firmware/tree/master/src/drivers/linux_sbus)

## 用法

The command syntax is:

    linux_sbus start|stop|status -d <device> -c <channel>
    

So for example, to automatically start the driver listening to 8 channels on device `/dev/ttyUSB0`, you would add the following line to the startup configuration file.

    linux_sbus start -d /dev/ttyUSB0 -c 8
    

> **Note** The original configuration files are located in **Firmware/posix-configs**. According to the official documentation, after you finish `make upload` related operations, all posix related configuration files will be placed in **/home/pi**. You can modify the file you want to use there.