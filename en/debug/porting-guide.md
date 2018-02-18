# Flight Controller Porting Guide

This topic is for developers who want to port PX4 to work with *new* flight controller hardware.

## Architecture

PX4 consists of two main layers: The board support and middleware layer on top of the host OS (NuttX, Linux or any other POSIX platform like Mac OS). And the applications (Flight Stack)(in [src/modules](https://github.com/PX4/Firmware/tree/master/src/modules)\).

This guide is focused only on the middleware as the applications will run on any board target.

## NuttX Boards

The location of the main files for a NuttX board are:

* Board startup and PX4 board configuration: [src/drivers/boards](https://github.com/PX4/Firmware/tree/master/src/drivers/boards). It contains bus and GPIO mappings and the board initialization code. 
  * FMUv5 example: [src/drivers/boards/px4fmu-v5](https://github.com/PX4/Firmware/tree/master/src/drivers/boards/px4fmu-v5). 
* NuttX board config: [nuttx-configs](https://github.com/PX4/Firmware/tree/master/nuttx-configs). The OS gets loaded as part of the application build. 
  * FMUv5 example: [nuttx-configs/px4fmu-v5/include/board.h](https://github.com/PX4/Firmware/blob/master/nuttx-configs/px4fmu-v5/include/board.h).
* NuttX OS config (created with menuconfig): [nuttx-configs/px4fmu-v5/nsh/defconfig](https://github.com/PX4/Firmware/blob/master/nuttx-configs/px4fmu-v5/nsh/defconfig).
* Linker scripts and other required settings: [nuttx-configs](https://github.com/PX4/Firmware/tree/master/nuttx-configs). 
  * FMUv5 example: [nuttx-configs/px4fmu-v5](https://github.com/PX4/Firmware/tree/master/nuttx-configs/px4fmu-v5).
* Boot file system (startup script): [ROMFS/px4fmu\_common](https://github.com/PX4/Firmware/tree/master/ROMFS/px4fmu_common)
* Board build configuration: [cmake/configs/nuttx\_px4fmu-v5\_default.cmake](https://github.com/PX4/Firmware/blob/master/cmake/configs/nuttx_px4fmu-v5_default.cmake).
* Drivers: [src/drivers](https://github.com/PX4/Firmware/tree/master/src/drivers).
* Reference config: Running `make px4fmu-v4_default` builds the FMUv4 config, which is the current NuttX reference configuration.


### NuttX Menuconfig

If you need to modify the NuttX [menuconfig](https://bitbucket.org/nuttx/nuttx) you can do this using the PX4 shortcuts:
```sh
make px4fmu-v2_default menuconfig
make px4fmu-v2_default qconfig
```

## QuRT / Hexagon

* The start script is located in [posix-configs/](https://github.com/PX4/Firmware/tree/master/posix-configs).
* The OS configuration is part of the default Linux image (TODO: Provide location of LINUX IMAGE and flash instructions).
* The PX4 middleware configuration is located in [src/drivers/boards](https://github.com/PX4/Firmware/tree/master/src/drivers/boards). TODO: ADD BUS CONFIG
* Drivers: [DriverFramework](https://github.com/px4/DriverFramework).
* Reference config: Running `make qurt_eagle_release` builds the Snapdragon Flight reference config.


## Linux Boards

Linux boards do not include the OS and kernel configuration. These are already provided by the Linux image available for the board (which needs to support the inertial sensors out of the box).

* [cmake/configs/posix\_rpi\_cross.cmake](https://github.com/PX4/Firmware/blob/master/cmake/configs/posix_rpi_cross.cmake) - RPI cross-compilation.





## Related Information

* [Device Drivers](../middleware/drivers.md) - How to support new peripheral hardware (device drivers)
* [Building the Code](../setup/building_px4.md) - How to build source and upload firmware 
* Supported Flight Controllers:
  * [Autopilot Hardware](https://docs.px4.io/en/flight_controller/) (PX4 User Guide)
  * [Supported boards list](https://github.com/PX4/Firmware/#supported-hardware) (Github)
* [Supported Peripherals](https://docs.px4.io/en/peripherals/) (PX4 User Guide)
