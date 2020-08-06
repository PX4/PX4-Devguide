# I2C 버스 개요

I2C는 선 두가닥만 활용하여 다중 마스터 장비를 다중 슬레이브 장비와 연결할 수 있게 하는 패킷 전환 직렬 통신 프로토콜입니다. 저속 주변기기 IC 칩을 프로세서와 마이크로컨트롤러에 짧은 길이로 연결, 보드내 통신을 수행하기 위한 목적입니다.

Pixhawk/PX4 support it for:

* Connecting off board components that require greater data rates than provided by a strict serial UART: e.g. rangefinders.
* Compatibility with peripheral devices that only support I2C.
* Allowing multiple devices to attach to a single bus (useful for conserving ports). For example, LEDs, Compass, rangefinders etc.

> **Tip** IMUs (accelerometers/gyroscopes) should not be attached via I2C (typically the [SPI](https://en.wikipedia.org/wiki/Serial_Peripheral_Interface_Bus) bus is used). The bus is not fast enough even with a single device attached to allow vibration filtering (for instance), and the performance degrades further with every additional device on the bus.

## Integrating I2C Devices

Drivers should `#include <drivers/device/i2c.h>` and then provide an implementation of the abstract base class `I2C` defined in **I2C.hpp** for the target hardware (i.e. for NuttX [here](https://github.com/PX4/Firmware/blob/master/src/lib/drivers/device/nuttx/I2C.hpp)).

Drivers will also need to include headers for their type of device (**drv_*.h**) in [/src/drivers/](https://github.com/PX4/Firmware/tree/master/src/drivers) - e.g. [drv_baro.h](https://github.com/PX4/Firmware/blob/master/src/drivers/drv_baro.h).

To include a driver in firmware you must add the driver to the board-specific cmake file that corresponds to the target you want to build for:

    drivers/sf1xx
    

> **Tip** For example, you can see/search for this driver in the [px4_fmu-v4_default](https://github.com/PX4/Firmware/blob/master/boards/px4/fmu-v4/default.cmake) configuration.

## I2C Driver Examples

To find I2C driver examples, search for **i2c.h** in [/src/drivers/](https://github.com/PX4/Firmware/tree/master/src/drivers).

Just a few examples are:

* [drivers/sf1xx](https://github.com/PX4/Firmware/tree/master/src/drivers/distance_sensor/sf1xx) - I2C Driver for [Lightware SF1XX LIDAR](https://docs.px4.io/master/en/sensor/sfxx_lidar.html).
* [drivers/ms5611](https://github.com/PX4/Firmware/tree/master/src/drivers/barometer/ms5611) - I2C Driver for the MS5611 and MS6507 barometric pressure sensor connected via I2C (or SPI).

## Further Information

* [I2C](https://en.wikipedia.org/wiki/I%C2%B2C) (Wikipedia)
* [I2C Comparative Overview](https://learn.sparkfun.com/tutorials/i2c) (learn.sparkfun.com)
* [Driver Framework](../middleware/drivers.md)