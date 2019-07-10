---
translated_page: https://github.com/PX4/Devguide/blob/master/en/sensor/uart_ulanding_radar.md
translated_sha: 95b39d747851dd01c1fe5d36b24e59ec865e323e
translated: true
---

# uLanding 雷达


uLanding雷达是[Aerotenna](http://aerotenna.com/sensors/)的产品，可用于测量与物体的距离。



## 启用硬件的驱动程序



目前，该雷达设备支持任何运行NuttX操作系统的硬件，并以串行端口作为接口。由于某些硬件上的闪存空间很小，因此须自行为目标启用驱动程序。为此，请将以下行添加到cmake配置文件中，并对应于要构建的目标：

```
drivers/ulanding
```

所有配置文件都位于[此处](https://github.com/PX4/Firmware/tree/master/cmake/configs)。

##  启动驱动程序


在系统启动期间，须告诉系统启动雷达驱动程序，将以下行添加到位于SD卡上的[extras.txt](../concept/system_startup.md)文件即可。

```
ulanding_radar start /dev/serial_port
```

在上面的命令中，您必须将最后一个参数替换为已连接硬件的串行端口。



如果没有指定任何端口，驱动程序将使用`/dev/ttyS2`，这是Pixhawk飞控板上的TELEM2端口。



**警告**



如果要将雷达设备连接到TELEM2，请确保将参数`SYS_COMPANION`设置为0。否则串行端口将被另一个应用程序使用，并且实际结果不可控。

 

