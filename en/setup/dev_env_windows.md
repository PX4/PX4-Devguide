# Windows Installation Instructions

> **Warning** Although a Windows toolchain is available, its not officially supported and we discourage its use. It is unbearably slow during Firmware compilation and does not support new boards like Snapdragon Flight. It also cannot run the standard robotics software packages many developers use to prototype computer vision and navigation. Before starting to develop on Windows, consider installing a dual-boot environment with [Ubuntu](http://ubuntu.com).

## Development Environment Installation

Download and install these on your system:

  * [Qt Creator IDE](http://www.qt.io/download-open-source/#section-6)
  * [PX4 Toolchain Installer v14 for Windows Download](http://firmware.diydrones.com/Tools/PX4-tools/px4_toolchain_installer_v14_win.exe) (32/64 bit systems, complete build system, drivers)
  * [PX4 USB Drivers](http://pixhawk.org/static/px4driver.msi) (32/64 bit systems)

Now continue to run the [first build](../setup/building_px4.md)!

## NEW! Bash on Windows

There is a new option for Windows users which is to run Bash shell natively then follow the Linux
build instructions.  See [BashOnWindows](https://github.com/Microsoft/BashOnWindows).  We have 
verified that the PX4 build succeeds in this environment.  It cannot yet flash the firmware, but
you can use the Mission Planner or QGroundControl to flash custom firwmare on Windows.
