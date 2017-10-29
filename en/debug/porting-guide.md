# Porting Guide

PX4 consists of two main layers: The board support and middleware layer on top of the host OS \(NuttX, Linux or any other POSIX platform like Mac OS\). And the applications \(in [src/modules](https://github.com/PX4/Firmware/tree/master/src/modules)\).

This guide is focused only on the middleware as the applications will run on any board target.

## NuttX Boards

The main files for a NuttX board are located at \(for the FMUv5 example\):

* [src/drivers/boards/px4fmu-v5](https://github.com/PX4/Firmware/tree/master/src/drivers/boards/px4fmu-v5) - board startup and PX4 board configuration
* [nuttx-configs/px4fmu-v5/include/board.h](https://github.com/PX4/Firmware/blob/master/nuttx-configs/px4fmu-v5/include/board.h) - NuttX board config
* [nuttx-configs/px4fmu-v5/nsh/defconfig](https://github.com/PX4/Firmware/blob/master/nuttx-configs/px4fmu-v5/nsh/defconfig) - NuttX OS config - created with menuconfig
* [nuttx-configs/px4fmu-v5](https://github.com/PX4/Firmware/tree/master/nuttx-configs/px4fmu-v5) - Linker scripts and other required settings
* [ROMFS/px4fmu\_common](https://github.com/PX4/Firmware/tree/master/ROMFS/px4fmu_common) - the boot file system
* [cmake/configs/nuttx\_px4fmu-v5\_default.cmake](https://github.com/PX4/Firmware/blob/master/cmake/configs/nuttx_px4fmu-v5_default.cmake) - board build configuration

## Linux Boards





