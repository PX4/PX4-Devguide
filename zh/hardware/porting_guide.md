# 飞行控制器移植指南

本主题主要针对希望将 PX4 移植到 *新* 飞控硬件平台上的开发人员。

## PX4 架构

PX4 由两个主要层次组成：基于主机操作系统（NuttX，Linux 或者其他 POSIX 平台，比如 Mac OS）的 [板卡支持和中间件层](../middleware/README.md) ，以及应用层（位于 [src/modules](https://github.com/PX4/Firmware/tree/master/src/modules)\ 的飞行控制栈）。 更多有关详细信息请参阅： [PX4 系统架构概述](../concept/architecture.md) 。

本指南仅关注主机操作系统和中间件，因为 应用层/飞行控制栈 可以在任何目标平台上运行。

## 飞行控制器配置文件分布位置

飞控板的启动和配置文件位于特定于每类飞控板供应商（vendor-specific）的目录下，这些目录都位于 [/boards](https://github.com/PX4/Firmware/tree/master/boards/) 文件夹下（例如**boards/*VENDOR*/*MODEL*/**文件夹)。

例如，对于 FMUv5 飞控硬件平台：

* （所有的）针对该飞控板的文件位于：[/boards/px4/fmu-v5](https://github.com/PX4/Firmware/tree/master/boards/px4/fmu-v5)。 
* 编译配置：[/boards/px4/fmu-v5/default.cmake](https://github.com/PX4/Firmware/blob/master/boards/px4/fmu-v5/default.cmake)。
* Board-specific initialisation file: [/boards/px4/fmu-v5/init/rc.board_defaults](https://github.com/PX4/Firmware/blob/master/boards/px4/fmu-v5/init/rc.board_defaults) 
  * 如果在飞控板平台目录下可以找到 **init/rc.board** 文件，则针对该飞控板平台的初始化文件将会自动包含在启动脚本中。
  * 该文件用于启动仅存在于特定主板上的传感器 (和其他东西)。 它也被用于完成对飞控板的默认参数、 UART 映射关系和其它特殊情况的设定。
  * For FMUv5 you can see all the Pixhawk 4 sensors being started, and it also sets a larger LOGGER_BUF. 

## 主机操作系统配置

This section describes the purpose and location of the configuration files required for each supported host operating system to port them to new flight controller hardware.

### NuttX

See [NuttX Board Porting Guide](porting_guide_nuttx.md).

### Linux

Linux boards do not include the OS and kernel configuration. These are already provided by the Linux image available for the board (which needs to support the inertial sensors out of the box).

* [boards/px4/raspberrypi/default.cmake](https://github.com/PX4/Firmware/blob/master/boards/px4/raspberrypi/default.cmake) - RPI cross-compilation. 

## 中间件组件和配置

This section describes the various middleware components, and the configuration file updates needed to port them to new flight controller hardware.

### QuRT / Hexagon

* The start script is located in [posix-configs/](https://github.com/PX4/Firmware/tree/master/posix-configs).
* The OS configuration is part of the default Linux image (TODO: Provide location of LINUX IMAGE and flash instructions).
* The PX4 middleware configuration is located in [src/boards](https://github.com/PX4/Firmware/tree/master/boards). TODO: ADD BUS CONFIG 
* Reference config: Running `make eagle_default` builds the Snapdragon Flight reference config.

## RC UART 接线建议

It is generally recommended to connect RC via separate RX and TX pins to the microcontroller. If however RX and TX are connected together, the UART has to be put into singlewire mode to prevent any contention. This is done via board config and manifest files. One example is [px4fmu-v5](https://github.com/PX4/Firmware/blob/master/boards/px4/fmu-v5/src/manifest.c).

## 官方支持的硬件

The PX4 project supports and maintains the [FMU standard reference hardware](../hardware/reference_design.md) and any boards that are compatible with the standard. This includes the [Pixhawk-series](https://docs.px4.io/en/flight_controller/pixhawk_series.html) (see the user guide for a [full list of officially supported hardware](https://docs.px4.io/en/flight_controller/)).

Every officially supported board benefits from:

* PX4 Port available in the PX4 repository
* Automatic firmware builds that are accessible from *QGroundControl*
* Compatibility with the rest of the ecosystem
* Automated checks via CI - safety remains paramount to this community
* [Flight testing](../test_and_ci/test_flights.md)

We encourage board manufacturers to aim for full compatibility with the [FMU spec](https://pixhawk.org/). With full compatibility you benefit from the ongoing day-to-day development of PX4, but have none of the maintenance costs that come from supporting deviations from the specification.

> **Tip** Manufacturers should carefully consider the cost of maintenance before deviating from the specification (the cost to the manufacturer is proportional to the level of divergence).

We welcome any individual or company to submit their port for inclusion in our supported hardware, provided they are willing to follow our [Code of Conduct](https://github.com/PX4/Firmware/blob/master/CODE_OF_CONDUCT.md) and work with the Dev Team to provide a safe and fulfilling PX4 experience to their customers.

It's also important to note that the PX4 dev team has a responsibility to release safe software, and as such we require any board manufacturer to commit any resources necessary to keep their port up-to-date, and in a working state.

If you want to have your board officially supported in PX4:

* Your hardware must be available in the market (i.e. it can be purchased by any developer without restriction).
* Hardware must be made available to the PX4 Dev Team so that they can validate the port (contact <lorenz@px4.io> for guidance on where to ship hardware for testing).
* The board must pass full [test suite](../test_and_ci/README.md) and [flight testing](../test_and_ci/test_flights.md).

**The PX4 project reserves the right to refuse acceptance of new ports (or remove current ports) for failure to meet the requirements set by the project.**

You can reach out to the core developer team and community on the official [Forums and Chat](../README.md#support).

## 相关信息

* [Device Drivers](../middleware/drivers.md) - How to support new peripheral hardware (device drivers)
* [Building the Code](../setup/building_px4.md) - How to build source and upload firmware 
* Supported Flight Controllers: 
  * [Autopilot Hardware](https://docs.px4.io/en/flight_controller/) (PX4 User Guide)
  * [Supported boards list](https://github.com/PX4/Firmware/#supported-hardware) (Github)
* [Supported Peripherals](https://docs.px4.io/en/peripherals/) (PX4 User Guide)