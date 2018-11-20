# Flight Controller Porting Guide

This topic is for developers who want to port PX4 to work with *new* flight controller hardware.

## PX4 Architecture

PX4 consists of two main layers: The [board support and middleware layer](../middleware/README.md) on top of the host OS (NuttX, Linux or any other POSIX platform like Mac OS), and the applications (Flight Stack in [src/modules](https://github.com/PX4/Firmware/tree/master/src/modules)\).  Please reference the [PX4 Architectural Overview](../concept/architecture.md) for more information.

This guide is focused only on the host OS and middleware as the applications/flight stack will run on any board target.

## Flight Controller Configuration File Layout

In addition to the host operating system specific configuration files described below, there are several groups of configuration files for each board located throughout the code base:
* Board startup and configuration files are located in: [src/drivers/boards](https://github.com/PX4/Firmware/tree/master/src/drivers/boards).
  * This folder contains bus mappings, GPIO mappings, and the initialization code for each board.
  * FMUv5 example: [src/drivers/boards/px4fmu-v5](https://github.com/PX4/Firmware/tree/master/src/drivers/boards/px4fmu-v5).
* The boot file system (startup script) is located in: [ROMFS/px4fmu\_common](https://github.com/PX4/Firmware/tree/master/ROMFS/px4fmu_common)
* The board specific build configurations are located in: [cmake/configs/](https://github.com/PX4/Firmware/blob/master/cmake/configs/).
* Driver files are located in: [src/drivers](https://github.com/PX4/Firmware/tree/master/src/drivers).

## Host Operating System Configuration

This section describes the purpose and location of the configuration files required for each supported host operating system to port them to new flight controller hardware.

### NuttX

In order to port PX4 on NuttX to a new hardware target, that hardware target must be supported by NuttX.  The NuttX project maintains an excellent [porting guide](http://www.nuttx.org/Documentation/NuttxPortingGuide.html) for porting NuttX to a new computing platform.

For all NuttX based flight controllers (e.g. the Pixhawk series) the OS is loaded as part of the application build.

The configuration files for NuttX based boards, including linker scripts and other required settings are located under [platforms/nuttx/nuttx-configs](https://github.com/PX4/Firmware/tree/master/platforms/nuttx/nuttx-configs).

The following example uses FMUv5 as it is a recent [reference configuration](../debug/reference-design.md) for NuttX based flight controllers:
* Running `make px4fmu-v5_default` from the `src/Firmware` directory will build the FMUv5 config
* The base FMUv5 configuration files are located in: [platforms/nuttx/nuttx-configs/px4fmu-v5](https://github.com/PX4/Firmware/tree/master/platforms/nuttx/nuttx-configs/px4fmu-v5).
* Board specific header: [platforms/nuttx/nuttx-configs/px4fmu-v5/include/board.h](https://github.com/PX4/Firmware/blob/master/platforms/nuttx/nuttx-configs/px4fmu-v5/include/board.h).
* NuttX OS config (created with Nuttx menuconfig): [nuttx-configs/px4fmu-v5/nsh/defconfig](https://github.com/PX4/Firmware/blob/master/platforms/nuttx/nuttx-configs/px4fmu-v5/nsh/defconfig).
* Build configuration: [cmake/configs/nuttx\_px4fmu-v5\_default.cmake](https://github.com/PX4/Firmware/blob/master/cmake/configs/nuttx_px4fmu-v5_default.cmake).

The function of each of these files, and perhaps more, will need to be duplicated for a new flight controller board.

#### NuttX Menuconfig
If you need to modify the NuttX OS configuration, you can do this via [menuconfig](https://bitbucket.org/nuttx/nuttx) using the PX4 shortcuts:
```sh
make px4fmu-v5_default menuconfig
make px4fmu-v5_default qconfig
```

### Linux

Linux boards do not include the OS and kernel configuration. These are already provided by the Linux image available for the board (which needs to support the inertial sensors out of the box).

* [cmake/configs/posix\_rpi\_cross.cmake](https://github.com/PX4/Firmware/blob/master/cmake/configs/posix_rpi_cross.cmake) - RPI cross-compilation.

## Middleware Components and Configuration

This section describes the various middleware components, and the configuration file updates needed to port them to new flight controller hardware.

### QuRT / Hexagon

* The start script is located in [posix-configs/](https://github.com/PX4/Firmware/tree/master/posix-configs).
* The OS configuration is part of the default Linux image (TODO: Provide location of LINUX IMAGE and flash instructions).
* The PX4 middleware configuration is located in [src/drivers/boards](https://github.com/PX4/Firmware/tree/master/src/drivers/boards). TODO: ADD BUS CONFIG
* Drivers: [DriverFramework](https://github.com/px4/DriverFramework).
* Reference config: Running `make eagle_default` builds the Snapdragon Flight reference config.

## Related Information

* [Device Drivers](../middleware/drivers.md) - How to support new peripheral hardware (device drivers)
* [Building the Code](../setup/building_px4.md) - How to build source and upload firmware 
* Supported Flight Controllers:
  * [Autopilot Hardware](https://docs.px4.io/en/flight_controller/) (PX4 User Guide)
  * [Supported boards list](https://github.com/PX4/Firmware/#supported-hardware) (Github)
* [Supported Peripherals](https://docs.px4.io/en/peripherals/) (PX4 User Guide)


## RC UART Wiring Recommendations

It is generally recommended to connect RC via separate RX and TX pins to the microcontroller. 
If however RX and TX are connected together, the UART has to be put into singlewire mode to prevent any contention. 
This is done via board config and manifest files. 
One example is [px4fmu-v5](https://github.com/PX4/Firmware/blob/master/src/drivers/boards/px4fmu-v5/manifest.c).


## Accepting a Board into PX4 Codelines

Manufacturers may wish to contribute board ports to the PX4 project codeline in order more closely align with the project.

We encourage board manufacturers to aim for full compatibility with the [FMU spec](https://pixhawk.org/).
With full compatibility you benefit from the ongoing day-to-day development of PX4, but have none of the maintenance costs that come from supporting deviations from the specification.

> **Tip** Manufacturers should carefully consider the cost of maintenance before deviating from the specification
  (the cost to the manufacturer is proportional to the level of divergence).

It's also important to note that the PX4 dev team has a responsibility to release safe software, and as such we require any board manufacturer to commit any resources necessary to keep their port up-to-date, and in a working state.

We welcome any individual or company that is willing to follow our [Code of Conduct](../contribute/README.md#code-of-conduct) and work with the Dev Team to provide a safe and fulfilling PX4 experience to their customers.

Every officially supported board benefits from:
* Your port available in the PX4 repository
* Automatic firmware builds that are accessible from *QGroundControl*
* Compatibility with the rest of the ecosystem
* Automated checks via CI - safety remains paramount to this community.
* [Flight testing](../test_and_ci/test_flights.md)

In summary, if you want to have your board officially supported in PX4:
* Your hardware must be available in the market (i.e. it can be purchased by any developer without restriction).
* Hardware must be made available to the PX4 Dev Team so that they can validate the port (contact [lorenz@px4.io](mailto:lorenz@px4.io) for guidance on where to ship hardware for testing).
* The board must pass full [test suite](../test_and_ci/README.md) and [flight testing](../test_and_ci/test_flights.md).

**The PX4 project reserves the right to refuse acceptance of new ports (or remove current ports) for failure to meet the requirements set by the project.**
