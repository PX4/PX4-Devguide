# 离板控制

> **Warning** 使用 [Offboard 模式控制](https://docs.px4.io/en/flight_modes/offboard.html) 无人机是有危险的。 开发者有责任确保在离机前采取充分的准备、测试和安全预防措施。

离板控制背后的想法是能够使用在自动驾驶仪外运行的软件来控制 PX4 飞控。 这是通过 Mavlink 协议完成的, 特别是 [SET_POSITION_TARGET_LOCAL_NED](https://mavlink.io/en/messages/common.html#SET_POSITION_TARGET_LOCAL_NED) 和 [SET_ATTITUDE_TARGET](https://mavlink.io/en/messages/common.html#SET_ATTITUDE_TARGET) 消息。

## 离板控制固件设置

在开始离板开发之前，您需要在固件端做两个安装。

### 1. 将遥控开关映射到离板模式激活

To do this, load up the parameters in *QGroundControl* and look for the RC_MAP_OFFB_SW parameter to which you can assign the RC channel you want to use to activate offboard mode. It can be useful to map things in such a way that when you fall out of offboard mode you go into position control.

Although this step isn't mandatory since you can activate offboard mode using a MAVLink message. We consider this method much safer.

### 2. Enable the companion computer interface

Enable MAVLink on the serial port that you connect to the companion computer (see [Companion computer setup](../companion_computer/pixhawk_companion.md)).

## 硬件安装

Usually, there are three ways of setting up offboard communication.

### 1. Serial radios

1. One connected to a UART port of the autopilot
2. One connected to a ground station computer

Example radios include:

* [Lairdtech RM024](http://www.lairdtech.com/products/rm024)
* [Digi International XBee Pro](http://www.digi.com/products/xbee-rf-solutions/modules)

{% mermaid %} graph TD; gnd[Ground Station] --MAVLink--> rad1[Ground Radio]; rad1 --RadioProtocol--> rad2[Vehicle Radio]; rad2 --MAVLink--> a[Autopilot]; {% endmermaid %}

### 2. On-board processor

A small computer mounted onto the vehicle connected to the autopilot through a UART to USB adapter. There are many possibilities here and it will depend on what kind of additional on-board processing you want to do in addition to sending commands to the autopilot.

Small low power examples:

* [Odroid C1+](http://www.hardkernel.com/main/products/prdt_info.php?g_code=G143703355573) 或 [Odroid XU4](http://www.hardkernel.com/main/products/prdt_info.php?g_code=G143452239825)
* [Raspberry Pi](https://www.raspberrypi.org/)
* [Intel Edison](http://www.intel.com/content/www/us/en/do-it-yourself/edison.html)

Larger high power examples:

* [Intel NUC](http://www.intel.com/content/www/us/en/nuc/overview.html)
* [Gigabyte Brix](http://www.gigabyte.com/products/list.aspx?s=47&ck=104)
* [Nvidia Jetson TK1](https://developer.nvidia.com/jetson-tk1)

{% mermaid %} graph TD; comp[Companion Computer] --MAVLink--> uart[UART Adapter]; uart --MAVLink--> Autopilot; {% endmermaid %}

### 3. On-board processor and wifi link to ROS (***Recommended***)

A small computer mounted onto the vehicle connected to the autopilot through a UART to USB adapter while also having a WiFi link to a ground station running ROS. This can be any of the computers from the above section coupled with a WiFi adapter. For example, the Intel NUC D34010WYB has a PCI Express Half-Mini connector which can accommodate an [Intel Wifi Link 5000](http://www.intel.com/products/wireless/adapters/5000/) adapter.

{% mermaid %} graph TD subgraph Ground Station gnd[ROS Enabled Computer] \--- qgc[qGroundControl] end gnd --MAVLink/UDP--> w[WiFi]; qgc --MAVLink--> w; subgraph Vehicle comp[Companion Computer] --MAVLink--> uart[UART Adapter] uart \--- Autopilot end w \--- comp {% endmermaid %}