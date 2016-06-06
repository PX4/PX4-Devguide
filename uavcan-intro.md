# UAVCAN Introduction

![](images/uavcan-logo-transparent.png)

[UAVCAN](http://uavcan.org) is an onboard network which allows the autopilot to connect to avionics. It supports hardware like:

  * Motor controllers
    * [Pixhawk ESC](https://pixhawk.org/modules/pixhawk_esc)
    * [SV2740 ESC](https://github.com/thiemar/vectorcontrol)
  * Airspeed sensors
    * [Thiemar airspeed sensor](https://github.com/thiemar/airspeed)
  * GNSS receivers for GPS and GLONASS
    * [Zubax GNSS](http://zubax.com/product/zubax-gnss)

In contrast to hobby-grade devices it uses rugged, differential signalling and supports firmware upgrades over the bus. All motor controllers provide status feedback and implement field-oriented-control (FOC).

## Upgrading Node Firmware

The PX4 middleware will automatically upgrade firmware on UAVCAN nodes if the matching firmware is supplied. The process and requirements are described on the [UAVCAN Firmware](uavcan-node-firmware.md) page.

## Enumerating and Configuring Motor Controllers

The ID and rotational direction of each motor controller can be assigned after installation in a simple setup routine: [UAVCAN Node Enumeration](uavcan-node-enumeration.md). The routine can be started by the user through QGroundControl.

## Useful links

* [Homepage](http://uavcan.org)
* [Specification](http://uavcan.org/Specification)
* [Implementations and tutorials](http://uavcan.org/Implementations)
