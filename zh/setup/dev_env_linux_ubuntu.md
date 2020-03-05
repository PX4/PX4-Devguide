# Ubuntu LTS/Debian Linux上开发环境的搭建

[Ubuntu linux LTS](https://wiki.ubuntu.com/LTS) 16.04是标准/推荐的Linux开发操作系统。 你可以在这上面编译所有的PX4对象（基于NuttX平台的硬件，高通骁龙飞行硬件，基于Linux平台的硬件以及仿真）

我们提供了Bash脚本来方便你根据不同的平台安装开发环境：

* **[ubuntu.sh](https://github.com/PX4/Firmware/blob/{{ book.px4_version }}/Tools/setup/ubuntu.sh)**: 安装 [Gazebo 9](../simulation/gazebo.md) 和 [jMAVSim](../simulation/jmavsim.md) 仿真器 以及/或者 [NuttX/Pixhawk](../setup/building_px4.md#nuttx) 工具。 不包含[FastRTPS](#fast_rtps)所依赖的工具。
* **[ubuntu_sim_ros_melodic.sh](https://raw.githubusercontent.com/PX4/Devguide/{{ book.px4_version }}/build_scripts/ubuntu_sim_ros_melodic.sh)**: 安装 [ROS "Melodic"](#rosgazebo) 以及 PX4 到 Ubuntu 18.04 LTS上。

> **提示**这些脚本已经在纯净的Ubuntu16.04和Ubuntu18.04 LTS上测试过了。 如果你在一个已经安装过这些工具的系统上或者一些其他的Ubuntu发行版上执行安装，它也有可能会安装不成功。

本说明将在下面解释如何下载并使用这些脚本。

## Gazebo, JMAVSim and NuttX (Pixhawk) {#sim_nuttx}

使用[ubuntu.sh](https://github.com/PX4/Firmware/blob/{{ book.px4_version }}/Tools/setup/ubuntu.sh)这个脚本来安装开发环境以支持[Gazebo 9](../simulation/gazebo.md)和[jMAVSim](../simulation/jmavsim.md)仿真器，以及/或者[NuttX/Pixhawk](../setup/building_px4.md#nuttx)工具链。

执行步骤：

1. [Download PX4 Source Code](../setup/building_px4.md): 
        bash
        git clone https://github.com/PX4/Firmware.git --recursive

2. 在bash shell中不带参数地运行ubuntu.sh来安装所有的依赖工具： 
        bash
        bash ./Tools/setup/ubuntu.sh
    
      
    * 在安装过程中确认并通过所有的提示
    * 你可以通过传输参数`--no-nuttx` 和 `--no-sim-tools` 来跳过 nuttx 和/或 仿真器工具的安装。
3. 完成后重新启动计算机。

> **Note** You can alternatively download [ubuntu.sh](https://github.com/PX4/Firmware/blob/{{ book.px4_version }}/Tools/setup/ubuntu.sh) and [requirements.txt](https://github.com/PX4/Firmware/blob/{{ book.px4_version }}/Tools/setup/requirements.txt) from the PX4 source repository (**/Tools/setup/**) and run ubuntu.sh in place:   
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

To get the build toolchain for Raspberry Pi:

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

Additional developer information for using PX4 on Raspberry Pi (including building PX4 natively) can be found here: [Raspberry Pi 2/3 Navio2 Autopilot](https://docs.px4.io/en/flight_controller/raspberry_pi_navio2.html).

## ROS/Gazebo {#rosgazebo}

This section explains how to install [ROS/Gazebo](../ros/README.md) ("Melodic") for use with PX4.

> **注：** PX4 已经跟ROS Melodic 在 Ubuntu 18.04 上一同测试过。 ROS Melodic 不适用于Ubuntu 16.04。

To install the development toolchain:

1. 在bash shell中下载脚本：   
    `wget https://raw.githubusercontent.com/PX4/Devguide/{{ book.px4_version }}/build_scripts/ubuntu_sim_ros_melodic.sh`
2. 运行脚本： 
        bash
        bash ubuntu_sim_ros_melodic.sh 随着脚本的运行，可能需要根据提示进行确认。

Note:

* ROS Melodic 已经默认跟Gazebo9一起安装了。
* 你的catkin（ROS编译系统）工作空间已经创建在**~/catkin_ws/**中。
* 脚本中使用的指令来自于ROS的wiki的"Melodic" [Ubuntu page](http://wiki.ros.org/melodic/Installation/Ubuntu)。

## Snapdragon Flight

Setup instructions for Snapdragon Flight are provided in the *PX4 User Guide*:

* [开发环境](https://docs.px4.io/en/flight_controller/snapdragon_flight_dev_environment_installation.html)
* [软件安装](https://docs.px4.io/en/flight_controller/snapdragon_flight_software_installation.html)
* [配置](https://docs.px4.io/en/flight_controller/snapdragon_flight_configuration.html)

## FastRTPS installation {#fast_rtps}

[eProsima Fast RTPS](http://eprosima-fast-rtps.readthedocs.io/en/latest/) is a C++ implementation of the RTPS (Real Time Publish Subscribe) protocol. FastRTPS is used, via the [RTPS/ROS2 Interface: PX4-FastRTPS Bridge](../middleware/micrortps.md), to allow PX4 uORB topics to be shared with offboard components.

The following instructions can be used to install the FastRTPS 1.7.1 binaries to your home directory.

```sh
wget https://www.eprosima.com/index.php/component/ars/repository/eprosima-fast-rtps/eprosima-fast-rtps-1-7-1/eprosima_fastrtps-1-7-1-linux-tar-gz -O eprosima_fastrtps-1-7-1-linux.tar.gz
tar -xzf eprosima_fastrtps-1-7-1-linux.tar.gz eProsima_FastRTPS-1.7.1-Linux/
tar -xzf eprosima_fastrtps-1-7-1-linux.tar.gz requiredcomponents
tar -xzf requiredcomponents/eProsima_FastCDR-1.0.8-Linux.tar.gz
```

> **注：**下面的这几行命令用来编译FastCDR以及FastRTPS库，`make`命令与`-j2`选项一起执行。 这个选项指定了在编译源码时使用多少个并行的线程。 修改 `-j2` 为 `-j<系统cpu的内核数目>`可以加速库的编译速度。

```sh
(cd eProsima_FastCDR-1.0.8-Linux && ./configure --libdir=/usr/lib && make -j2 && sudo make install)
(cd eProsima_FastRTPS-1.7.1-Linux && ./configure --libdir=/usr/lib && make -j2 && sudo make install)
rm -rf requiredcomponents eprosima_fastrtps-1-7-1-linux.tar.gz
```

> **注：** 更多一般性的指令，需要额外从源安装的，可以从这里找到：[Fast RTPS installation](../setup/fast-rtps-installation.md)。

## Additional Tools

After setting up the build/simulation toolchain, see [Additional Tools](../setup/generic_dev_tools.md) for information about other useful tools.

## Next Steps

Once you have finished setting up the environment, continue to the [build instructions](../setup/building_px4.md).