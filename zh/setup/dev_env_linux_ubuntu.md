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

1. 从PX4的源码仓库中下载[ubuntu.sh](https://github.com/PX4/Firmware/blob/{{ book.px4_version }}/Tools/setup/ubuntu.sh) 和 [requirements.txt](https://github.com/PX4/Firmware/blob/{{ book.px4_version }}/Tools/setup/requirements.txt)（在**/Tools/setup/**目录下）；   
    `wget https://raw.githubusercontent.com/PX4/Firmware/{{ book.px4_version }}/Tools/setup/ubuntu.sh`   
    `wget https://raw.githubusercontent.com/PX4/Firmware/{{ book.px4_version }}/Tools/setup/requirements.txt`
2. 在bash shell中不带参数地运行ubuntu.sh来安装所有的依赖工具： 
        bash
        source ubuntu.sh
    
    * 在安装过程中确认并通过所有的提示
    * 你可以通过传输参数`--no-nuttx` 和 `--no-sim-tools` 来跳过 nuttx 和/或 仿真器工具的安装。
3. 完成后重新启动计算机。

> **或者** 你也可以直接下载PX4的全部源码然后运行里面的脚本： 
> 
>     git clone https://github.com/PX4/Firmware.git
>       source Firmware/Tools/setup/ubuntu.sh

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

To get the build toolchain for Raspberry Pi:

1. Download [ubuntu.sh](https://github.com/PX4/Firmware/blob/{{ book.px4_version }}/Tools/setup/ubuntu.sh) and [requirements.txt](https://github.com/PX4/Firmware/blob/{{ book.px4_version }}/Tools/setup/requirements.txt) from the PX4 source repository (**/Tools/setup/**):   
    `wget https://raw.githubusercontent.com/PX4/Firmware/{{ book.px4_version }}/Tools/setup/ubuntu.sh`   
    `wget https://raw.githubusercontent.com/PX4/Firmware/{{ book.px4_version }}/Tools/setup/requirements.txt`
2. Run **ubuntu.sh** in a terminal to get just the common dependencies: 
        bash
        source ubuntu.sh --no-nuttx --no-sim-tools

3. Then setup an ARMv7 cross-compiler (either GCC or clang) as described in the following sections.

### GCC

The current recommended toolchain for raspbian can be cloned from `https://github.com/raspberrypi/tools.git` (at time of writing 4.9.3). The `PATH` environmental variable should include the path to the gcc cross-compiler collection of tools (e.g. gcc, g++, strip) prefixed with `arm-linux-gnueabihf-`.

```sh
git clone https://github.com/raspberrypi/tools.git ${HOME}/rpi-tools

# test compiler
$HOME/rpi-tools/arm-bcm2708/gcc-linaro-arm-linux-gnueabihf-raspbian-x64/bin/arm-linux-gnueabihf-gcc -v

# permanently update PATH variable by modifying ~/.profile
echo 'export PATH=$PATH:$HOME/rpi-tools/arm-bcm2708/gcc-linaro-arm-linux-gnueabihf-raspbian-x64/bin' >> ~/.profile

# update PATH variable only for this session
export PATH=$PATH:$HOME/rpi-tools/arm-bcm2708/gcc-linaro-arm-linux-gnueabihf-raspbian-x64/bin
```

### Clang

In order to use clang, you also need GCC.

Download clang for your specific distribution from [LLVM Download page](http://releases.llvm.org/download.html) and unpack it. Assuming that you've unpacked clang to `CLANG_DIR`, and `clang` binary is available in `CLANG_DIR/bin`, and you have the GCC cross-compiler in `GCC_DIR`, you will need to setup the symlinks for clang in the `GCC_DIR` bin dir, and add `GCC_DIR/bin` to `PATH`.

Example below for building PX4 firmware out of tree, using CMake.

```sh
ln -s <CLANG_DIR>/bin/clang <GCC_DIR>/bin/clang
ln -s <CLANG_DIR>/bin/clang++ <GCC_DIR>/bin/clang++
export PATH=<GCC_DIR>/bin:$PATH

cd <PATH-TO-PX4-SRC>
mkdir build/posix_rpi_cross_clang
cd build/posix_rpi_cross_clang
cmake \
-G"Unix Makefiles" \
-DCONFIG=posix_rpi_cross \
-DCMAKE_C_COMPILER=clang \
-DCMAKE_CXX_COMPILER=clang++ \
..
```

### Native Builds

Additional developer information for using PX4 on Raspberry Pi (including building PX4 natively) can be found here: [Raspberry Pi 2/3 Navio2 Autopilot](https://docs.px4.io/en/flight_controller/raspberry_pi_navio2.html).

## Parrot Bebop

Developers working with the Parrot Bebop should first install the [Raspberry Pi Linux Toolchain](#raspberry-pi-hardware) as described above.

Then install ADB:

```sh
sudo apt-get install android-tools-adb -y
```

## ROS/Gazebo {#rosgazebo}

This section explains how to install [ROS/Gazebo](../ros/README.md) ("Melodic") for use with PX4.

> **Note** PX4 is tested with ROS Melodic on Ubuntu 18.04 LTS. ROS Melodic does not work on Ubuntu 16.04.

To install the development toolchain:

1. Download the script in a bash shell:   
    `wget https://raw.githubusercontent.com/PX4/Devguide/{{ book.px4_version }}/build_scripts/ubuntu_sim_ros_melodic.sh`
2. Run the script: 
        bash
        source ubuntu_sim_ros_melodic.sh You may need to acknowledge some prompts as the script progresses.

Note:

* ROS Melodic is installed with Gazebo9 by default.
* Your catkin (ROS build system) workspace is created at **~/catkin_ws/**.
* The script uses instructions from the ROS Wiki "Melodic" [Ubuntu page](http://wiki.ros.org/melodic/Installation/Ubuntu).

## Snapdragon Flight

Setup instructions for Snapdragon Flight are provided in the *PX4 User Guide*:

* [Development Environment](https://docs.px4.io/en/flight_controller/snapdragon_flight_dev_environment_installation.html)
* [Software Installation](https://docs.px4.io/en/flight_controller/snapdragon_flight_software_installation.html)
* [Configuration](https://docs.px4.io/en/flight_controller/snapdragon_flight_configuration.html)

## FastRTPS installation {#fast_rtps}

[eProsima Fast RTPS](http://eprosima-fast-rtps.readthedocs.io/en/latest/) is a C++ implementation of the RTPS (Real Time Publish Subscribe) protocol. FastRTPS is used, via the [RTPS/ROS2 Interface: PX4-FastRTPS Bridge](../middleware/micrortps.md), to allow PX4 uORB topics to be shared with offboard components.

The following instructions can be used to install the FastRTPS 1.7.1 binaries to your home directory.

```sh
wget https://www.eprosima.com/index.php/component/ars/repository/eprosima-fast-rtps/eprosima-fast-rtps-1-7-1/eprosima_fastrtps-1-7-1-linux-tar-gz -O eprosima_fastrtps-1-7-1-linux.tar.gz
tar -xzf eprosima_fastrtps-1-7-1-linux.tar.gz eProsima_FastRTPS-1.7.1-Linux/
tar -xzf eprosima_fastrtps-1-7-1-linux.tar.gz requiredcomponents
tar -xzf requiredcomponents/eProsima_FastCDR-1.0.8-Linux.tar.gz
```

> **Note** In the following lines where we compile the FastCDR and FastRTPS libraries, the `make` command is issued with the `-j2` option. This option defines the number of parallel threads (or `j`obs) that are used to compile the source code. Change `-j2` to `-j<number_of_cpu_cores_in_your_system>` to speed up the compilation of the libraries.

```sh
(cd eProsima_FastCDR-1.0.8-Linux && ./configure --libdir=/usr/lib && make -j2 && sudo make install)
(cd eProsima_FastRTPS-1.7.1-Linux && ./configure --libdir=/usr/lib && make -j2 && sudo make install)
rm -rf requiredcomponents eprosima_fastrtps-1-7-1-linux.tar.gz
```

> **Note** More "generic" instructions, which additionally cover installation from source, can be found here: [Fast RTPS installation](../setup/fast-rtps-installation.md).

## 额外工具

After setting up the build/simulation toolchain, see [Additional Tools](../setup/generic_dev_tools.md) for information about other useful tools.

## 后续步骤

Once you have finished setting up the environment, continue to the [build instructions](../setup/building_px4.md).