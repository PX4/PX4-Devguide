# 无线数传

无线数传 (选配) 可以用于建立 *QGroundControl* 地面站与PX4飞控之间的无线MAVLink连接。 本节包含两个主题：已经支持的无线数传 和 在PX4系统中集成新的数传。

## 已支持的无线数传

[PX4 用户手册 > 数传](http://docs.px4.io/en/telemetry/) 列举了PX4已支持的无线数传系统。 包括使用 *SiK Radio*固件的数传 和 *3DR WiFi 无线数传*。

## 集成新类型数传系统

PX4支持通过数传端口将一个基于 MAVLink 的数传连接到Pixhawk飞控。 Provided that a telemetry radio supports MAVLink and provides a UART interface with compatible voltage levels/connector, no further integration is required.

Telemetry systems that communicate using some other protocol will need more extensive integration, potentially covering both software (e.g. device drivers) and hardware (connectors etc.). While this has been done for specific cases (e.g. [FrSky Telemetry](https://docs.px4.io/en/peripherals/frsky_telemetry.html) enables sending vehicle status to an RC controller via an FrSky receiver) providing general advice is difficult. We recommend you start by [discussing with the development team](../README.md#support).