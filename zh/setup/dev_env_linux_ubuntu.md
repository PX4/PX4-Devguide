# Ubuntu LTS/Debian Linux上开发环境的搭建

The supported/tested Linux OS versions for PX4 development are [Ubuntu Linux LTS](https://wiki.ubuntu.com/LTS) 18.04 (Bionic Beaver) and 20.04 (Focal Fossa). These allow you to build for the [most PX4 targets](../setup/dev_env.md#supported-targets) (NuttX based hardware, *Qualcomm Snapdragon Flight* hardware, Linux-based hardware, Simulation).

我们提供了Bash脚本来方便你根据不同的平台安装开发环境：

* **[ubuntu.sh](https://github.com/PX4/Firmware/blob/{{ book.px4_version }}/Tools/setup/ubuntu.sh)**：安装 [Gazebo 9](../simulation/gazebo.md) 和 [jMAVSim](../simulation/jmavsim.md) 仿真器 以及/或者 [NuttX/Pixhawk](../setup/building_px4.md#nuttx) 工具。 不包含[FastRTPS](#fast_rtps)所依赖的工具。
* **[ubuntu_sim_ros_melodic.sh](https://raw.githubusercontent.com/PX4/Devguide/{{ book.px4_version }}/build_scripts/ubuntu_sim_ros_melodic.sh)**: Installs [ROS "Melodic"](#rosgazebo) and PX4 on Ubuntu 18.04 LTS (and later).

> **Tip** The scripts have been tested on *clean* Ubuntu 18.04 LTS and Ubuntu 20.04 LTS installations. They *may* not work as expected if installed "on top" of an existing system, or on a different Ubuntu release.

本说明将在下面解释如何下载并使用这些脚本。

## Gazebo，JMAVSim 与 NuttX（Pixhawk）编译目标 {#sim_nuttx}

使用[ubuntu.sh](https://github.com/PX4/Firmware/blob/{{ book.px4_version }}/Tools/setup/ubuntu.sh)这个脚本来安装开发环境以支持[Gazebo 9](../simulation/gazebo.md)和[jMAVSim](../simulation/jmavsim.md)仿真器，以及/或者[NuttX/Pixhawk](../setup/building_px4.md#nuttx)工具链。

执行步骤：

1. [下载 PX4 源代码](../setup/building_px4.md)： 
        bash
        git clone https://github.com/PX4/Firmware.git --recursive

2. 在bash shell中不带参数地运行ubuntu.sh来安装所有的依赖工具： 
        bash
        bash ./Tools/setup/ubuntu.sh
    
      
    * 在安装过程中确认并通过所有的提示。
    * 你可以通过传输参数`--no-nuttx` 和 `--no-sim-tools` 来跳过 nuttx 和/或 仿真器工具的安装。
3. 完成后重新启动计算机。

> **Note** 你也可以从 PX4 源码库（**/Tools/setup/**）手动下载 [ubuntu.sh](https://github.com/PX4/Firmware/blob/{{ book.px4_version }}/Tools/setup/ubuntu.sh) 与 [requirements.txt](https://github.com/PX4/Firmware/blob/{{ book.px4_version }}/Tools/setup/requirements.txt) 文件并运行 ubuntu.sh：   
> `wget https://raw.githubusercontent.com/PX4/Firmware/{{ book.px4_version }}/Tools/setup/ubuntu.sh`   
> `wget https://raw.githubusercontent.com/PX4/Firmware/{{ book.px4_version }}/Tools/setup/requirements.txt`   
> `bash ubuntu.sh`

备注：

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
Remove any old versions of the arm-none-eabi toolchain.</p>

<pre><code class="sh">sudo apt-get remove gcc-arm-none-eabi gdb-arm-none-eabi binutils-arm-none-eabi gcc-arm-embedded
sudo add-apt-repository --remove ppa:team-gcc-arm-embedded/ppa
</code></pre>

<p>-->

## Raspberry Pi {#raspberry-pi-hardware}

<!-- NOTE: RaPi docker toolchain (for comparison) here: https://github.com/PX4/containers/blob/master/docker/Dockerfile_armhf -->

获取用于树莓派 Raspberry Pi 的构建工具链：

1. 从PX4源码仓库下载[ubuntu.sh](https://github.com/PX4/Firmware/blob/{{ book.px4_version }}/Tools/setup/ubuntu.sh) 和[requirements.txt](https://github.com/PX4/Firmware/blob/{{ book.px4_version }}/Tools/setup/requirements.txt)：   
    `wget https://raw.githubusercontent.com/PX4/Firmware/{{ book.px4_version }}/Tools/setup/ubuntu.sh`   
    `wget https://raw.githubusercontent.com/PX4/Firmware/{{ book.px4_version }}/Tools/setup/requirements.txt`
2. 在终端中运行**ubuntu.sh**，获取一般的依赖模块： 
        bash
        bash ubuntu.sh --no-nuttx --no-sim-tools

3. 然后根据下面的描述安装ARMv7交叉编译器(GCC或者Clang)。

### GCC

The official Raspberry Pi toolchains are not supported as PX4 has requires C++14 (which they do not support).

Ubuntu provides a set of pre-compiled toolchains that you can use instead. Install these with the terminal command:

    sudo apt-get install -y gcc-arm-linux-gnueabihf g++-arm-linux-gnueabihf
    

These package contains GCC/G++ 7.4.0 at time of writing. To test the toolchain, please execute:

    arm-linux-gnueabihf-gcc -v
    arm-linux-gnueabihf-g++ -v
    

### Clang

First [install GCC](#gcc) (needed to use clang).

We recommend you to get clang from the Ubuntu software repository as follows:

    sudo apt-get install clang
    

Example below for building PX4 firmware out of tree, using CMake.

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

### 本地编译

Additional developer information for using PX4 on Raspberry Pi (including building PX4 natively) can be found here: [Raspberry Pi 2/3 Navio2 Autopilot](https://docs.px4.io/master/en/flight_controller/raspberry_pi_navio2.html).

## ROS/Gazebo {#rosgazebo}

This section explains how to install [ROS/Gazebo](../ros/README.md) ("Melodic") for use with PX4.

To install the development toolchain:

1. 在bash shell中下载脚本：   
    `wget https://raw.githubusercontent.com/PX4/Devguide/{{ book.px4_version }}/build_scripts/ubuntu_sim_ros_melodic.sh`
2. 运行脚本： 
        bash
        bash ubuntu_sim_ros_melodic.sh 随着脚本的运行，可能需要根据提示进行确认。

注：

* ROS Melodic 已经默认跟Gazebo9一起安装了。
* 你的catkin（ROS编译系统）工作空间已经创建在**~/catkin_ws/**中。
* 脚本中使用的指令来自于ROS的wiki的"Melodic" [Ubuntu page](http://wiki.ros.org/melodic/Installation/Ubuntu)。

## 高通骁龙飞控（Snapdragon Flight）

在 *PX4 用户指南* 中提供了高通骁龙飞控的安装说明：

* [开发环境](https://docs.px4.io/master/en/flight_controller/snapdragon_flight_dev_environment_installation.html)
* [软件安装](https://docs.px4.io/master/en/flight_controller/snapdragon_flight_software_installation.html)
* [配置](https://docs.px4.io/master/en/flight_controller/snapdragon_flight_configuration.html)

## Fast RTPS installation {#fast_rtps}

[eProsima Fast RTPS](http://eprosima-fast-rtps.readthedocs.io/en/latest/) 是 RTPS（Real Time Publish Subscribe 实时发布订阅）协议的 C++ 实现库。 通过 [RTPS/ROS2 接口：PX4-FastRTPS 桥接](../middleware/micrortps.md) 使用 FastRTPS，允许与 Offboard 组件共享 PX4 uORB 话题。

Follow the instructions in [Fast RTPS Installation](../setup/fast-rtps-installation.md) to install it.

## 附加工具

After setting up the build/simulation toolchain, see [Additional Tools](../setup/generic_dev_tools.md) for information about other useful tools.

## 后续步骤

Once you have finished setting up the environment, continue to the [build instructions](../setup/building_px4.md).