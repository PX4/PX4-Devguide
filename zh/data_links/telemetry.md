# 无线数传

无线数传 (选配) 可以用于建立 *QGroundControl* 地面站与PX4飞控之间的无线MAVLink连接。 本节包含两个主题：已经支持的无线数传 和 在PX4系统中集成新的数传。

## 已支持的无线数传

[PX4 User Guide > Telemetry](https://docs.px4.io/master/en/telemetry/) contains information about telemetry radio systems already supported by PX4. 包括使用 *SiK Radio*固件的数传 和 *3DR WiFi 无线数传*。

## 集成新类型数传系统

PX4支持通过数传端口将一个基于 MAVLink 的数传连接到Pixhawk飞控。 只需要一个支持MAVLink的数传和一个兼容UART电平/连接器的端口，无需更多。

使用其它协议的数传系统的集成则比较困难，软件(比如设备驱动)和硬件(连接器等)都需要考虑到。 While this has been done for specific cases (e.g. [FrSky Telemetry](https://docs.px4.io/master/en/peripherals/frsky_telemetry.html) enables sending vehicle status to an RC controller via an FrSky receiver) providing general advice is difficult. 遇到此类问题，我们推荐你[与开发团队进行沟通](../README.md#support)。