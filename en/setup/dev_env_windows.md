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

Note: to build Pixhawk ARM firmware you will need to use a [64 bit arm-none-eabi compiler](https://github.com/SolinGuo/arm-none-eabi-bash-on-win10-.git)
since BashOnWindows doesn't run 32 bit ELF programs and the default compiler from `https://launchpad.net/gcc-arm-embedded` is 32 bit.
So if you download the *.tar.bz2 file that SolinGuo created to your machine and unpack it using this command line in BashOnWindows console:

`tar -xvf gcc-arm-none-eabi-5_4-2017q2-20170512-linux.tar.bz2`

you will get the following folder which contains the arm gcc cross-compiler:

`gcc-arm-none-eabi-5_4-2017q2/bin`

If you add this folder to your PATH using the usual export PATH=... Linux trick then the PX4 build will be able to find and run this compiler. After that, you can run `make px4fmu-v2_default` in BashOnWindows and the firmware will appear here: `build_px4fmu-v2_default/src/firmware/nuttx/px4fmu-v2_default.px4`. You can then flash this new firmware on your Pixhawk using QGroundControl.
