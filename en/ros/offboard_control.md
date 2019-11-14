# Offboard Control

> **Warning** [Offboard control](https://docs.px4.io/en/flight_modes/offboard.html) is dangerous.
  It is the responsibility of the developer to ensure adequate preparation, testing and safety precautions are taken before offboard flights.

The idea behind off-board control is to be able to control the PX4 flight stack using software running outside of the autopilot. This is done through the MAVLink protocol, specifically the [SET_POSITION_TARGET_LOCAL_NED](https://mavlink.io/en/messages/common.html#SET_POSITION_TARGET_LOCAL_NED) and the [SET_ATTITUDE_TARGET](https://mavlink.io/en/messages/common.html#SET_ATTITUDE_TARGET) messages.

## Offboard Control Firmware Setup

There are two things you want to setup on the firmware side before starting offboard development.

### Map an RC switch to offboard mode activation

To do this, load up the parameters in *QGroundControl* and look for the RC_MAP_OFFB_SW parameter to which you can assign the RC channel you want to use to activate offboard mode.
It can be useful to map things in such a way that when you fall out of offboard mode you go into position control.

Although this step isn't mandatory since you can activate offboard mode using a MAVLink message. We consider this method much safer.

### Enable the companion computer interface

Enable MAVLink on the serial port that you connect to the companion computer (see [Companion computer setup](../companion_computer/pixhawk_companion.md)).

## Hardware setup

Usually, there are three ways of setting up offboard communication.

### Serial radios

1. One connected to a UART port of the autopilot
2. One connected to a ground station computer

Example radios include:
* [Lairdtech RM024](http://www.lairdtech.com/products/rm024)
* [Digi International XBee Pro](http://www.digi.com/products/xbee-rf-solutions/modules)

{% mermaid %}
graph TD;
  gnd[Ground Station] --MAVLink--> rad1[Ground Radio];
  rad1 --RadioProtocol--> rad2[Vehicle Radio];
  rad2 --MAVLink--> a[Autopilot];
{% endmermaid %}

### On-board processor

A small computer mounted onto the vehicle connected to the autopilot through a UART to USB adapter. 
There are many possibilities here and it will depend on what kind of additional on-board processing you want to do in addition to sending commands to the autopilot.

Small low power examples:
* [Odroid C1+](https://www.hardkernel.com/shop/odroid-c1/) or [Odroid XU4](https://magazine.odroid.com/odroid-xu4)
* [Raspberry Pi](https://www.raspberrypi.org/)
* [Intel Edison](http://www.intel.com/content/www/us/en/do-it-yourself/edison.html)

Larger high power examples:
* [Intel NUC](http://www.intel.com/content/www/us/en/nuc/overview.html)
* [Gigabyte Brix](http://www.gigabyte.com/products/list.aspx?s=47&ck=104)
* [Nvidia Jetson TK1](https://developer.nvidia.com/jetson-tk1)

{% mermaid %}
graph TD;
  comp[Companion Computer] --MAVLink--> uart[UART Adapter];
  uart --MAVLink--> Autopilot;
{% endmermaid %}

### On-board processor and wifi link to ROS (***Recommended***)

A small computer mounted onto the vehicle connected to the autopilot through a UART to USB adapter while also having a WiFi link to a ground station running ROS. 
This can be any of the computers from the above section coupled with a WiFi adapter.
For example, the Intel NUC D34010WYB has a PCI Express Half-Mini connector which can accommodate an [Intel Wifi Link 5000](http://www.intel.com/products/wireless/adapters/5000/) adapter.


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
