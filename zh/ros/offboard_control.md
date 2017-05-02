---
translated_page: https://github.com/PX4/Devguide/blob/master/en/ros/offboard_control.md
translated_sha: 95b39d747851dd01c1fe5d36b24e59ec865e323e
---

# 外部控制


> **警告：** 外部控制是很危险的。在进行外部控制飞行之前，开发者需要保证有充分的准备、测试以及安全预防措施。


外部控制允许使用运行在飞控板外部的软件去控制px4飞行控制栈。通过MAVLink协议完成这些操作，特别是[SET_POSITION_TARGET_LOCAL_NED](http://mavlink.org/messages/common#SET_POSITION_TARGET_LOCAL_NED)和[SET_ATTITUDE_TARGET](http://mavlink.org/messages/common#SET_ATTITUDE_TARGET)消息.

## 外部控制固件设置

在开始外部控制开发之前，固件方面需要做两项设置。

### 1. 映射一个RC切换开关为外部模式激活开关

在QGroundcontrol中载入参数，并设置RC_MAP_OFFB_SW参数为想要控制外部模式激活的RC通道。这样做是非常有用的，当在外部模式出现问题时可以切换到位置控制模式。

尽管这一步并不是强制的，因为通过MAVLink消息同样可以激活外部模式。但是我们认为这种方式更加安全。

### 2. 使能协同计算机接口

将参数[SYS_COMPANION](https://pixhawk.org/firmware/parameters#system)设置为921600（推荐）或者57600。这个参数将会以合适的波特率(921600 8N1或者57600 8N1)激活TELEM2端口上的MAVLink消息流，这与内部模式的数据流是相同的。

有关这些数据流的更多信息，参考[source code](https://github.com/PX4/Firmware/blob/master/src/modules/mavlink/mavlink_main.cpp)中的"MAVLINK_MODE_ONBOARD"。

## 硬件设置

通常，有3种方式配置板外通讯

### 1. 数传

1. 一个连接到飞控板的UART端口
2. 一个连接到地面站计算机

参考数传包括：

- [Lairdtech RM024](http://www.lairdtech.com/products/rm024)
- [Digi International XBee Pro](http://www.digi.com/products/xbee-rf-solutions/modules)

{% mermaid %}
graph TD;
  gnd[Ground Station] --MAVLink--> rad1[Ground Radio];
  rad1 --RadioProtocol--> rad2[Vehicle Radio];
  rad2 --MAVLink--> a[Autopilot];
{% endmermaid %}

### 2. 机载协同计算机

一个挂载在飞行器上的小型计算机，通过串口转USB适配器连接到飞控板。有许多可用的选择，主要取决于除了向飞控板发送指令外还想要进行的额外操作。

低性能机载计算机:

- [Odroid C1+](http://www.hardkernel.com/main/products/prdt_info.php?g_code=G143703355573) or [Odroid XU4](http://www.hardkernel.com/main/products/prdt_info.php?g_code=G143452239825)
- [Raspberry Pi](https://www.raspberrypi.org/)
- [Intel Edison](http://www.intel.com/content/www/us/en/do-it-yourself/edison.html)

高性能机载计算机：

- [Intel NUC](http://www.intel.com/content/www/us/en/nuc/overview.html)
- [Gigabyte Brix](http://www.gigabyte.com/products/list.aspx?s=47&ck=104)
- [Nvidia Jetson TK1](https://developer.nvidia.com/jetson-tk1)

{% mermaid %}
graph TD;
  comp[Companion Computer] --MAVLink--> uart[UART Adapter];
  uart --MAVLink--> Autopilot;
{% endmermaid %}

### 3. 机载计算机和到ROS的WIFI连接（***推荐***）

一个挂载在飞行器上的小型计算机，通过串口转USB适配器连接到飞控板，同时提供到运行ROS的地面站的WIFI连接。可以是上一部分的任意一个机载计算机，同时再加一个WIFI适配器。例如：Intel NUC D34010WYB有一个PCI Express Half-Mini接口，可以连接一个[Intel Wifi Link 5000](http://www.intel.com/products/wireless/adapters/5000/)适配器。
{% mermaid %}
graph TD
subgraph Ground  Station
gnd[ROS Enabled Computer] --- qgc[qGroundControl]
end
gnd --MAVLink/UDP--> w[WiFi];
qgc --MAVLink--> w;
subgraph Vehicle
comp[Companion Computer] --MAVLink--> uart[UART Adapter]
uart --- Autopilot
end
w --- comp
{% endmermaid %}

