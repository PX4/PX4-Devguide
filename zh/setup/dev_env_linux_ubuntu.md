# Ubuntu LTS/Debian Linux上开发环境的搭建

The supported/tested Linux OS versions for PX4 development are [Ubuntu Linux LTS](https://wiki.ubuntu.com/LTS) 18.04 (Bionic Beaver) and 20.04 (Focal Fossa). These allow you to build for the [most PX4 targets](../setup/dev_env.md#supported-targets) (NuttX based hardware, *Qualcomm Snapdragon Flight* hardware, Linux-based hardware, Simulation).

我们提供了Bash脚本来方便你根据不同的平台安装开发环境：

* **[ubuntu.sh](https://github.com/PX4/PX4-Autopilot/blob/{{ book.px4_version }}/Tools/setup/ubuntu.sh)**: Installs [Gazebo 9](../simulation/gazebo.md) and [jMAVSim](../simulation/jmavsim.md) simulators and/or [NuttX/Pixhawk](../setup/building_px4.md#nuttx) tools. 不包含[FastRTPS](#fast_rtps)所依赖的工具。
* **[ubuntu_sim_ros_melodic.sh](https://raw.githubusercontent.com/PX4/Devguide/{{ book.px4_version }}/build_scripts/ubuntu_sim_ros_melodic.sh)**: Installs [ROS "Melodic"](#rosgazebo) and PX4 on Ubuntu 18.04 LTS (and later).

> **Tip** The scripts have been tested on *clean* Ubuntu 18.04 LTS and Ubuntu 20.04 LTS installations. They *may* not work as expected if installed "on top" of an existing system, or on a different Ubuntu release.

本说明将在下面解释如何下载并使用这些脚本。

<a id="sim_nuttx"></a>

## Gazebo, JMAVSim and NuttX (Pixhawk) Targets

Use the [ubuntu.sh](https://github.com/PX4/PX4-Autopilot/blob/{{ book.px4_version }}/Tools/setup/ubuntu.sh) script to set up a development environment that includes [Gazebo 9](../simulation/gazebo.md) and [jMAVSim](../simulation/jmavsim.md) simulators, and/or the [NuttX/Pixhawk](../setup/building_px4.md#nuttx) toolchain.

To install the toolchain:

1. [下载 PX4 源代码](../setup/building_px4.md)：
        bash
        git clone https://github.com/PX4/PX4-Autopilot.git --recursive

2. 在bash shell中不带参数地运行ubuntu.sh来安装所有的依赖工具：
        bash
        bash ./Tools/setup/ubuntu.sh

    * 在安装过程中确认并通过所有的提示。
    * 你可以通过传输参数`--no-nuttx` 和 `--no-sim-tools` 来跳过 nuttx 和/或 仿真器工具的安装。
3. 完成后重新启动计算机。

> **Note** You can alternatively download [ubuntu.sh](https://github.com/PX4/PX4-Autopilot/blob/{{ book.px4_version }}/Tools/setup/ubuntu.sh) and [requirements.txt](https://github.com/PX4/PX4-Autopilot/blob/{{ book.px4_version }}/Tools/setup/requirements.txt) from the PX4 source repository (**/Tools/setup/**) and run ubuntu.sh in place:
> `wget https://raw.githubusercontent.com/PX4/PX4-Autopilot/{{ book.px4_version }}/Tools/setup/ubuntu.sh`
> `wget https://raw.githubusercontent.com/PX4/PX4-Autopilot/{{ book.px4_version }}/Tools/setup/requirements.txt`
> `bash ubuntu.sh`

Notes:

* PX4需要跟Gazebo 7, 8, 或者 9一起工作， 该脚本使用[gazebosim.org instructions](http://gazebosim.org/tutorials?tut=install_ubuntu&cat=install)来安装Gazebo9。
* 如果你正在使用ROS工作，那么请遵循[ROS/Gazebo](#rosgazebo)的介绍来安装（它是自动转安装的，作为ROS安装的一部分）。
* 你可以通过确认gcc的版本来验证Nuttx的安装：

    ```bash
    $arm-none-eabi-gcc --version

    arm-none-eabi-gcc (GNU Tools for Arm Embedded Processors 7-2017-q4-major) 7.2.1 20170904 (release) [ARM/embedded-7-branch revision 255204]
    Copyright (C) 2017 Free Software Foundation, Inc.
    This is free software; see the source for copying conditions.  There is NO
    warranty; not even for MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
    ```

<!-- Do we need to add to our scripts or can we assume correct version installs over?
Remove any old versions of the arm-none-eabi toolchain.
```sh
sudo apt-get remove gcc-arm-none-eabi gdb-arm-none-eabi binutils-arm-none-eabi gcc-arm-embedded
sudo add-apt-repository --remove ppa:team-gcc-arm-embedded/ppa
```
-->

<a id="raspberry-pi-hardware"></a>

## Raspberry Pi

The following instructions explain how to set up a build toolchain for RasPi on *Ubuntu 18.04*.

> **Warning** To build for Ubuntu 20.04 (focal) you must use docker (the GCC toolchain on Ubuntu 20.04 can build PX4, but the generated binary files are too new to run on actual Pi). For more information see [PilotPi with Raspberry Pi OS Developer Quick Start > Alternative build method using docker](https://docs.px4.io/master/en/flight_controller/raspberry_pi_pilotpi_rpios.html#alternative-build-method-using-docker).

To get the common dependencies for Raspberry Pi:

1. Download [ubuntu.sh](https://github.com/PX4/PX4-Autopilot/blob/{{ book.px4_version }}/Tools/setup/ubuntu.sh) and [requirements.txt](https://github.com/PX4/PX4-Autopilot/blob/{{ book.px4_version }}/Tools/setup/requirements.txt) from the PX4 source repository (**/Tools/setup/**):
    `wget https://raw.githubusercontent.com/PX4/PX4-Autopilot/{{ book.px4_version }}/Tools/setup/ubuntu.sh`
    `wget https://raw.githubusercontent.com/PX4/PX4-Autopilot/{{ book.px4_version }}/Tools/setup/requirements.txt`
2. 在终端中运行**ubuntu.sh**，获取一般的依赖模块：
        bash
        bash ubuntu.sh --no-nuttx --no-sim-tools

3. Then setup an cross-compiler (either GCC or clang) as described in the following sections.

### GCC (armhf)

Ubuntu software repository provides a set of pre-compiled toolchains. Note that Ubuntu Focal comes up with `gcc-9-arm-linux-gnueabihf` as its default installation which is not fully supported, so we must manually install `gcc-8-arm-linux-gnueabihf` and set it as the default toolchain. This guide also applies to earlier Ubuntu releases (Bionic). The following instruction assumes you haven't installed any version of arm-linux-gnueabihf, and will set up the default executable with `update-alternatives`. Install them with the terminal command:

```sh
sudo apt-get install -y gcc-8-arm-linux-gnueabihf g++-8-arm-linux-gnueabihf
```

Set them as default:

```sh
sudo update-alternatives --install /usr/bin/arm-linux-gnueabihf-gcc arm-linux-gnueabihf-gcc /usr/bin/arm-linux-gnueabihf-8 100 --slave /usr/bin/arm-linux-gnueabihf-g++ arm-linux-gnueabihf-g++ /usr/bin/arm-linux-gnueabihf-g++-8
sudo update-alternatives --config arm-linux-gnueabihf-gcc
```

### GCC (aarch64)

If you want to build PX4 for ARM64 devices, this section is required.

```sh
sudo apt-get install -y gcc-8-aarch64-linux-gnu g++-8-aarch64-linux-gnu
sudo update-alternatives --install /usr/bin/aarch64-linux-gnu-gcc aarch64-linux-gnu-gcc /usr/bin/aarch64-linux-gnu-gcc-8 100 --slave /usr/bin/aarch64-linux-gnu-g++ aarch64-linux-gnu-g++ /usr/bin/aarch64-linux-gnu-g++-8
sudo update-alternatives --config aarch64-linux-gnu-gcc
```

### Clang (optional)

First [install GCC](#gcc) (needed to use clang).

We recommend you to get clang from the Ubuntu software repository, as shown below:

    sudo apt-get install clang


Example below for building PX4 firmware out of tree, using *CMake*.

```sh
cd <PATH-TO-PX4-SRC>
mkdir build/px4_raspberrypi_default_clang
cd build/px4_raspberrypi_default_clang
cmake \
-G"Unix Makefiles" \
-DCONFIG=px4_raspberrypi_default \
-UCMAKE_C_COMPILER \
-DCMAKE_C_COMPILER=clang \
-UCMAKE_CXX_COMPILER \
-DCMAKE_CXX_COMPILER=clang++ \
../..
make
```

### Detailed Information

Additional developer information for using PX4 on Raspberry Pi (including building PX4 natively) can be found here:

* [Raspberry Pi 2/3 Navio2 Autopilot](https://docs.px4.io/master/en/flight_controller/raspberry_pi_navio2.html).
* [Raspberry Pi 2/3/4 PilotPi Shield](https://docs.px4.io/master/en/flight_controller/raspberry_pi_pilotpi.html).

<a id="rosgazebo"></a>

## ROS/Gazebo

This section explains how to install [ROS/Gazebo](../ros/README.md) ("Melodic") for use with PX4.

To install the development toolchain:

1. 在bash shell中下载脚本：
    `wget https://raw.githubusercontent.com/PX4/Devguide/{{ book.px4_version }}/build_scripts/ubuntu_sim_ros_melodic.sh`
2. 运行脚本：
        bash
        bash ubuntu_sim_ros_melodic.sh 随着脚本的运行，可能需要根据提示进行确认。

Note:

* ROS Melodic is installed with Gazebo9 by default.
* Your catkin (ROS build system) workspace is created at **~/catkin_ws/**.
* The script uses instructions from the ROS Wiki "Melodic" [Ubuntu page](http://wiki.ros.org/melodic/Installation/Ubuntu).

## 高通骁龙飞控（Snapdragon Flight）

Setup instructions for Snapdragon Flight are provided in the *PX4 User Guide*:

* [Development Environment](https://docs.px4.io/master/en/flight_controller/snapdragon_flight_dev_environment_installation.html)
* [Software Installation](https://docs.px4.io/master/en/flight_controller/snapdragon_flight_software_installation.html)
* [Configuration](https://docs.px4.io/master/en/flight_controller/snapdragon_flight_configuration.html)

<a id="fast_rtps"></a>

## Fast RTPS installation

[eProsima Fast RTPS](http://eprosima-fast-rtps.readthedocs.io/en/latest/) is a C++ implementation of the RTPS (Real Time Publish Subscribe) protocol. FastRTPS is used, via the [RTPS/ROS2 Interface: PX4-FastRTPS Bridge](../middleware/micrortps.md), to allow PX4 uORB topics to be shared with offboard components.

Follow the instructions in [Fast RTPS Installation](../setup/fast-rtps-installation.md) to install it.

## 附加工具

After setting up the build/simulation toolchain, see [Additional Tools](../setup/generic_dev_tools.md) for information about other useful tools.

## 后续步骤

Once you have finished setting up the environment, continue to the [build instructions](../setup/building_px4.md).
