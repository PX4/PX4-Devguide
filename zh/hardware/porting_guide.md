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
* 针对该飞控板的的初始化文件：[/boards/px4/fmu-v5/init/rc.board](https://github.com/PX4/Firmware/blob/master/boards/px4/fmu-v5/init/rc.board) 
  * 如果在飞控板平台目录下可以找到 **init/rc.board** 文件，则针对该飞控板平台的初始化文件将会自动包含在启动脚本中。
  * 该文件用于启动仅存在于特定主板上的传感器 (和其他东西)。 它也被用于完成对飞控板的默认参数、 UART 映射关系和其它特殊情况的设定。
  * 对于 FMUv5 飞控板而言你可以在该文件内看到所有的 Pixhawk 4 传感器都被启动了，该文件还设置了一个较大的 LOGGER_BUF。同时，在 AUTOCNF （初始设置）这一部分该文件还会设定 [SYS_FMU_TASK](../advanced/parameter_reference.md#SYS_FMU_TASK) 这一参数。

此外，在整个代码库中，每个飞控板都还有一些其它的配置文件：

* 引导文件系统 (启动脚本) 位于：[ROMFS/px4fmu\_common](https://github.com/PX4/Firmware/tree/master/ROMFS/px4fmu_common)
* 驱动程序文件位于: [src/drivers](https://github.com/PX4/Firmware/tree/master/src/drivers) 。

## 主机操作系统配置

本节介绍了移植每个受支持的主机操作系统到新的飞控板硬件平台上需要用到的配置文件的用途和所处位置。

### NuttX

为了将基于 Nuttx 的 PX4 移植到新的硬件平台上，Nuttx 必须支持该硬件平台。 NuttX 项目中维护着一个出色的 [移植指南](http://www.nuttx.org/Documentation/NuttxPortingGuide.html) 可以帮助你实现将 Nuttx 移植到一个新的计算平台上。

对于所有的基于 Nuttx 的飞行控制器来说（例如 Pixhawk 系列），操作系统在加载时都是作为应用程序的一部分进行加载的。

所有飞控板的配置文件，包括链接脚本和其它必需的设置都位于 [/boards](https://github.com/PX4/Firmware/tree/master/boards/) 文件夹下特定于供应商（vendor- specific）和飞控板种类（ board-specific）的目录下 (例如 **boards/*VENDOR*/*MODEL*/**)。

The following example uses FMUv5 as it is a recent [reference configuration](../debug/reference-design.md) for NuttX based flight controllers:

* Running `make px4_fmu-v5_default` from the **Firmware** directory will build the FMUv5 config
* The base FMUv5 configuration files are located in: [/boards/px4/fmu-v5](https://github.com/PX4/Firmware/tree/master/boards/px4/fmu-v5).
* Board specific header: [/boards/px4/fmu-v5/nuttx-config/include/board.h](https://github.com/PX4/Firmware/blob/master/boards/px4/fmu-v5/nuttx-config/include/board.h). 
* NuttX OS config (created with Nuttx menuconfig): [/boards/px4/fmu-v5/nuttx-config/nsh/defconfig](https://github.com/PX4/Firmware/blob/master/boards/px4/fmu-v5/nuttx-config/nsh/defconfig).
* Build configuration: [PX4/Firmware/boards/px4/fmu-v5/default.cmake](https://github.com/PX4/Firmware/blob/master/boards/px4/fmu-v5/default.cmake).

The function of each of these files, and perhaps more, will need to be duplicated for a new flight controller board.

#### NuttX Menuconfig（基于文本选择的配置界面）

If you need to modify the NuttX OS configuration, you can do this via [menuconfig](https://bitbucket.org/nuttx/nuttx) using the PX4 shortcuts:

```sh
make px4_fmu-v5_default menuconfig
make px4_fmu-v5_default qconfig
```

For fresh installs of PX4 onto Ubuntu using [ubuntu_sim_nuttx.sh](https://raw.githubusercontent.com/PX4/Devguide/master/build_scripts/ubuntu_sim_nuttx.sh) you will also need to install *kconfig* tools from [NuttX tools](https://bitbucket.org/nuttx/tools/src/master/).

> **Note** The following steps are not required if using the [px4-dev-nuttx](https://hub.docker.com/r/px4io/px4-dev-nuttx/) docker container or have installed to macOS using our normal instructions (as these include`kconfig-mconf`).

Run the following commands from any directory:

```sh
git clone https://bitbucket.org/nuttx/tools.git
cd tools/kconfig-frontends
sudo apt install gperf
./configure --enable-mconf --disable-nconf --disable-gconf --enable-qconf --prefix=/usr
make
sudo make install
```

The `--prefix=/usr` is essential as it determines the specific installation location where PX4 is hardcoded to look for `kconfig-tools`. The `--enable-mconf` and `--enable-qconf` options will enable the `menuconfig` and `qconfig` options respectively.

To run `qconfig` you may need to install additional Qt dependencies.

### Linux

Linux boards do not include the OS and kernel configuration. These are already provided by the Linux image available for the board (which needs to support the inertial sensors out of the box).

* [cmake/configs/posix\_rpi\_cross.cmake](https://github.com/PX4/Firmware/blob/master/cmake/configs/posix_rpi_cross.cmake) - RPI cross-compilation.

## 中间件组件和配置

This section describes the various middleware components, and the configuration file updates needed to port them to new flight controller hardware.

### QuRT / Hexagon

* The start script is located in [posix-configs/](https://github.com/PX4/Firmware/tree/master/posix-configs).
* The OS configuration is part of the default Linux image (TODO: Provide location of LINUX IMAGE and flash instructions).
* The PX4 middleware configuration is located in [src/drivers/boards](https://github.com/PX4/Firmware/tree/master/src/drivers/boards). TODO: ADD BUS CONFIG
* Drivers: [DriverFramework](https://github.com/px4/DriverFramework).
* Reference config: Running `make eagle_default` builds the Snapdragon Flight reference config.

## RC UART 接线建议

It is generally recommended to connect RC via separate RX and TX pins to the microcontroller. If however RX and TX are connected together, the UART has to be put into singlewire mode to prevent any contention. This is done via board config and manifest files. One example is [px4fmu-v5](https://github.com/PX4/Firmware/blob/master/src/drivers/boards/px4fmu-v5/manifest.c).

## 官方支持的硬件

The PX4 project supports and maintains the [FMU standard reference hardware](../debug/reference-design.md) and any boards that are compatible with the standard. This includes the [Pixhawk-series](https://docs.px4.io/en/flight_controller/pixhawk_series.html) (see the user guide for a [full list of officially supported hardware](https://docs.px4.io/en/flight_controller/)).

Every officially supported board benefits from:

* PX4 Port available in the PX4 repository
* Automatic firmware builds that are accessible from *QGroundControl*
* Compatibility with the rest of the ecosystem
* Automated checks via CI - safety remains paramount to this community
* [Flight testing](../test_and_ci/test_flights.md)

We encourage board manufacturers to aim for full compatibility with the [FMU spec](https://pixhawk.org/). With full compatibility you benefit from the ongoing day-to-day development of PX4, but have none of the maintenance costs that come from supporting deviations from the specification.

> **Tip** Manufacturers should carefully consider the cost of maintenance before deviating from the specification (the cost to the manufacturer is proportional to the level of divergence).

We welcome any individual or company to submit their port for inclusion in our supported hardware, provided they are willing to follow our [Code of Conduct](../contribute/README.md#code-of-conduct) and work with the Dev Team to provide a safe and fulfilling PX4 experience to their customers.

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