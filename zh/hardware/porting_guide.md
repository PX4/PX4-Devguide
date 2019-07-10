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

下面的示例中使用了 FMUv5 飞控板平台，因为它是基于 NuttX 的飞行控制器的最新 < 0>参考配置 </a0 >：

* 在 **Firmware** 目录下运行 `make px4_fmu-v5_default` 命令将生成 FMUv5 配置。
* 基准的 FMUv5 配置文件位于：[/boards/px4/fmu-v5](https://github.com/PX4/Firmware/tree/master/boards/px4/fmu-v5)。
* 针对该飞控板的头文件位于：[/boards/px4/fmu-v5/nuttx-config/include/board.h](https://github.com/PX4/Firmware/blob/master/boards/px4/fmu-v5/nuttx-config/include/board.h)。 
* NuttX 操作系统配置（由 Nuttx 的文本配置界面（menuconfig ）生成）位于： [/boards/px4/fmu-v5/nuttx-config/nsh/defconfig](https://github.com/PX4/Firmware/blob/master/boards/px4/fmu-v5/nuttx-config/nsh/defconfig)。
* 编译配置位于： [PX4/Firmware/boards/px4/fmu-v5/default.cmake](https://github.com/PX4/Firmware/blob/master/boards/px4/fmu-v5/default.cmake)。

在移植到新的飞控板上时我们需要复现上述文件的功能，也许还会需要复现更多文件的功能。

#### NuttX Menuconfig（基于文本选择的配置界面）

如果你需要修改 NuttX 操作系统的设置，你可以通过 PX4 快捷方式调用 [menuconfig](https://bitbucket.org/nuttx/nuttx) 这个基于文本的配置界面来执行此操作：

```sh
make px4_fmu-v5_default menuconfig
make px4_fmu-v5_default qconfig
```

针对使用 [ubuntu_sim_nuttx.sh](https://raw.githubusercontent.com/PX4/Devguide/master/build_scripts/ubuntu_sim_nuttx.sh) 脚本在 Ubuntu 上全新安装 PX4 的情况，你还需要从 [NuttX tools](https://bitbucket.org/nuttx/tools/src/master/) 中额外安装 *kconfig* 工具。

> **Note** 如果使用的是 [px4-dev-nuttx](https://hub.docker.com/r/px4io/px4-dev-nuttx/) docker 容器作为开发环境或者根据我们的标准指南在 macOS 上安装的开发环境（这些情况下已经默认安装了 `kconfig-mconf` ），那么你并不需要执行下述步骤。

在任意目录运行以下命令：

```sh
git clone https://bitbucket.org/nuttx/tools.git
cd tools/kconfig-frontends
sudo apt install gperf
./configure --enable-mconf --disable-nconf --disable-gconf --enable-qconf --prefix=/usr
make
sudo make install
```

`--prefix=/usr` 是必不可少的，因为该选项在 PX4 中写死了 PX4 应该在哪个特定安装位置去寻找 `kconfig-tools`。 `--enable-mconf` 和 `--enable-qconf` 选项将会分别启用 `menuconfig` 和 `qconfig` 这两个选项。

想运行 `qconfig` 的话你可能还需要安装额外的 Qt 依赖项。

### Linux

基于 Linux 的飞控板不包含任何 操作系统和内核的配置。 这些配置已经由可用于飞控板的 Linux 镜像提供了（操作系统需要原生支持惯性传感器）。

* [cmake/configs/posix\_rpi\_cross.cmake](https://github.com/PX4/Firmware/blob/master/cmake/configs/posix_rpi_cross.cmake) - RPI 交叉编译。

## 中间件组件和配置

本节介绍各类中间件组件，以及将它们移植到新的飞行控制器硬件所需更新的配置文件。

### QuRT / Hexagon

* 启动脚本位于： [posix-configs/](https://github.com/PX4/Firmware/tree/master/posix-configs)。
* 操作系统配置是默认 Linux 镜像的一部分（TODO: 需要提供 LINUX 镜像文件位置和程序烧写指南）。
* PX4 中间件配置文件位于： [src/drivers/boards](https://github.com/PX4/Firmware/tree/master/src/drivers/boards)。 TODO: 需要添加总线配置（BUS CONFIG）。
* 驱动：[DriverFramework](https://github.com/px4/DriverFramework)。
* 参考配置：运行 `make eagle_default` 命令可构建 Snapdragon Flight 的参考配置文件。

## RC UART 接线建议

通常建议使用单独的 RX 和 TX 针脚来连接 RC 遥控器和微型控制器。 如果 RX 和 TX 连在了一起，那么 UART 需要设置为单线模式以防止出现争用。 这可以用过对飞控板的配置文件和 manifest 文件进行更改来实现。 示例可见： [px4fmu-v5](https://github.com/PX4/Firmware/blob/master/src/drivers/boards/px4fmu-v5/manifest.c)。

## 官方支持的硬件

PX4 项目支持并维护着 [FMU 标准参考硬件](../debug/reference-design.md) 及任何与标准相兼容的飞控板平台。 这就包括了 [Pixhawk 系列](https://docs.px4.io/en/flight_controller/pixhawk_series.html) （请翻阅用户手册获取 [官方支持硬件完整列表](https://docs.px4.io/en/flight_controller/))。

每个受官方支持的飞控板平台都将受益于：

* PX4 项目仓库中可用的 PX4 移植
* 可从 *QGroundControl* 中直接访问的自动固件编译
* 与生态系统其余部分的兼容性
* 可通过 CI 进行自动检查 — 安全仍是这个社区的重中之重
* [飞行测试](../test_and_ci/test_flights.md)

我们鼓励飞控板制造商以与 [FMU 规格](https://pixhawk.org/) 完全兼容为目标进行生产。 通过完全兼容, 您可以从 PX4 的日常开发中受益，而不需要付出任何维护成本来支持偏离了规范的硬件规格。

> **Tip** 制造商们在偏离标准硬件规格前应谨慎考虑需要付出的维护成本（制造商们需要付出的成本与偏离程度成正比）。

We welcome any individual or company to submit their port for inclusion in our supported hardware, provided they are willing to follow our [Code of Conduct](https://github.com/PX4/Firmware/blob/master/CODE_OF_CONDUCT.md) and work with the Dev Team to provide a safe and fulfilling PX4 experience to their customers.

还需要注意的是 PX4 开发团队有责任发布安全的软件，因此我们要求所有飞控板制造商都应投入必要的资源来保证他们的意志平台始终处于最新状态并且可用。

如果你想让你的飞控板被 PX4 项目正式支持：

* 你的硬件必须在市场上可用（例如它可以被任何开发人员不受限制地购买到） 。
* 必须向 PX4 开发团队提供硬件以便他们能够验证移植平台（联系 <lorenz@px4.io> 获取进行硬件测试的寄送地址的帮助信息）。
* 飞控板必须通过完整的 [测试套件（test suite）](../test_and_ci/README.md) 和 [飞行测试](../test_and_ci/test_flights.md)。

**PX4 项目团队保留因未能满足项目规定需求而 拒绝接收新的移植平台（或者移除现有移植平台）的权利。**

你可以在官方的 [论坛和聊天组](../README.md#support) 中与核心开发团队和开发社区取得联系。

## 相关信息

* [Device Drivers](../middleware/drivers.md) - 如何支持新的外围硬件设备（设备驱动）
* [构建代码](../setup/building_px4.md) - 如何编译和上传固件。 
* 受支持的飞行控制器： 
  * [Autopilot 硬件](https://docs.px4.io/en/flight_controller/) (PX4 用户手册)
  * [支持硬件平台列表](https://github.com/PX4/Firmware/#supported-hardware) (Github)
* [受支持的外围硬件](https://docs.px4.io/en/peripherals/) (PX4 用户指南)