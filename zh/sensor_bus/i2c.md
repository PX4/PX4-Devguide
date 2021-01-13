!REDIRECT "https://docs.px4.io/master/zh/sensor_bus/i2c.html"

# I2C 总线概述

I2C 是一种分组交换串行通信协议，允许多个主设备连接到多个从属设备，每个连接只需使用2根电线。 它用于在短距离、板内通信中将低速外设 IC 连接到处理器和微控制器。

Pixhawk/PX4 支持：

* 连接需要比严格的串行 UART 更高数据速率的板载组件：例如测距仪。
* 与仅支持 I2C 的外围设备兼容。
* 允许多个设备连接到单个总线（有效保护端口）。 例如，LED、指南针、测距仪等。

> **Tip** IMU（加速度计/陀螺仪）不应通过 I2C 连接（通常使用 [SPI](https://en.wikipedia.org/wiki/Serial_Peripheral_Interface_Bus) 总线）。 即使连一个设备可以进行振动过滤（实例），总线的速度也不够快，并且总线上的每一个额外设备都会进一步降低性能。

## 集成 I2C 设备

Drivers should `#include <drivers/device/i2c.h>` and then provide an implementation of the abstract base class `I2C` defined in **I2C.hpp** for the target hardware (i.e. for NuttX [here](https://github.com/PX4/PX4-Autopilot/blob/master/src/lib/drivers/device/nuttx/I2C.hpp)).

Drivers will also need to include headers for their type of device (**drv_*.h**) in [/src/drivers/](https://github.com/PX4/PX4-Autopilot/tree/master/src/drivers) - e.g. [drv_baro.h](https://github.com/PX4/PX4-Autopilot/blob/master/src/drivers/drv_baro.h).

To include a driver in firmware you must add the driver to the board-specific cmake file that corresponds to the target you want to build for:

    drivers/sf1xx
    

> **Tip** For example, you can see/search for this driver in the [px4_fmu-v4_default](https://github.com/PX4/PX4-Autopilot/blob/master/boards/px4/fmu-v4/default.cmake) configuration.

## I2C 驱动程序示例

To find I2C driver examples, search for **i2c.h** in [/src/drivers/](https://github.com/PX4/PX4-Autopilot/tree/master/src/drivers).

仅举几个例子：

* [drivers/sf1xx](https://github.com/PX4/PX4-Autopilot/tree/master/src/drivers/distance_sensor/sf1xx) - I2C Driver for [Lightware SF1XX LIDAR](https://docs.px4.io/master/en/sensor/sfxx_lidar.html).
* [drivers/ms5611](https://github.com/PX4/PX4-Autopilot/tree/master/src/drivers/barometer/ms5611) - I2C Driver for the MS5611 and MS6507 barometric pressure sensor connected via I2C (or SPI).

## 更多信息

* [I2C](https://en.wikipedia.org/wiki/I%C2%B2C)（维基百科）
* [I2C 比较审查 ](https://learn.sparkfun.com/tutorials/i2c)（learn.sparkfun.com）
* [驱动程序框架](../middleware/drivers.md)