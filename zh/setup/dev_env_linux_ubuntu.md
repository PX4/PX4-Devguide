# Ubuntu LTS/Debian Linux 的开发环境

< 0>Ubuntu linux lts</a>（16.04）是标准的/首选的 Linux 开发操作系统。 Linux允许您构建[所有PX4目标](../setup/dev_env.md#supported-targets)（基于NuttX的硬件、高通骁龙飞控硬件、基于Linux的硬件、仿真、ROS）。

以下说明说明了如何 *手动* 设置每个受支持的目标的开发环境。

> **Tip** 我们建议您使用 [一键安装脚本 ](#convenience-bash-scripts) 来安装模拟器和/或 Nuttx 工具链（这比在下面的说明中键入更容易）。 然后再参考其他目标（如高通骁龙飞控、Bebop、树莓派等的附加说明）。

<span></span>

> **Tip** 在设置构建/模拟工具链之后，有关其他有用工具的信息，请参阅 [附加工具](../setup/generic_dev_tools.md)。

## 一键安装脚本

我们已经创建了许多 bash 脚本，您可以使用这些脚本来安装模拟器和 Nuttx 工具链。 以下脚本作用分别是安装*Qt Creator IDE*、[ Ninja构建系统](#ninja-build-system)、[通用依赖项](#common-dependencies)、[FastRTPS](#fastrtps-installation)，以及将PX4源下载到您的目录（**~/src/Firmware**）。

> **Tip** 该脚本已经在全新Ubuntu 16.04安装测试通过。 如果安装在除上述提到的系统或其他Ubuntu版本上，则它们*可能*无法正常工作。

这些脚本是:

* **<a href="https://raw.githubusercontent.com/PX4/Devguide/master/build_scripts/ubuntu_sim_common_deps.sh" target="_blank" download>ubuntu_sim_common_deps.sh</a>**：[通用依赖](#common-dependencies)，[jMAVSim](#jmavsim) 模拟器
  
  * 该脚本包含了编译 PX4 需要的依赖。 当你运行任何其他脚本时，它会自动下载并运行。
  * 在安装[高通骁龙飞控](#snapdragon-flight) 或 [树莓派/Parrot Bebop](#raspberry-pi-hardware) 之前， 你可以先运行它。

* **<a href="https://raw.githubusercontent.com/PX4/Devguide/master/build_scripts/ubuntu_sim.sh" target="_blank" download>ubuntu_sim.sh</a>**: **ubuntu_sim_common_deps.sh** + [Gazebo8](#gazebo) 模拟器。

* **<a href="https://raw.githubusercontent.com/PX4/Devguide/master/build_scripts/ubuntu_sim_nuttx.sh" target="_blank" download>ubuntu_sim_nuttx.sh</a>**：**ubuntu_sim.sh** + NuttX 工具。 
  * *完成安装后需要重启。*
* **<a href="https://raw.githubusercontent.com/PX4/Devguide/master/build_scripts/ubuntu_sim_ros_gazebo.sh" target="_blank" download>ubuntu_sim_ros_gazebo.sh</a>**: **ubuntu_sim_common_deps.sh** + [ROS/Gazebo and MAVROS](#rosgazebo). 
  * ROS Kinetic 默认与 Gazebo 7 一起安装（为了简化 ROS 的开发，我们使用的默认而不是 Gazebo 8）。
  * 你的 catkin （ROS 构建系统）工作目录生成在**~/catkin_ws/**。

### 如何使用脚本

使用脚本：

1. 将用户添加到 ”dialout“ 组中（只需做一次）： 
  1. 打开终端输入： 
        sh
          sudo usermod -a -G dialout $USER
  
  2. 注销重新登录（必须重新登录后才能生效）。
2. 下载脚本
3. 运行 bash 脚本（比如运行 **ubuntu_sim.sh** ）： 
      bash
       source ubuntu_sim.sh 所有弹出的提示均确认通过。

## 权限设置

> **Warning** 绝对不要用 `sudo` 试图解决权限问题！！！ 这会带来更多的权限问题甚至需要重装系统来解决！！！

The user needs to be part of the group "dialout":

```sh
sudo usermod -a -G dialout $USER
```

Then logout and login again (the change is only made after a new login).

## Remove the modemmanager

Ubuntu comes with a serial modem manager which interferes heavily with any robotics related use of a serial port \(or USB serial\). It can removed/deinstalled without side effects:

```sh
sudo apt-get remove modemmanager
```

## Common Dependencies

Update the package list and install the following dependencies for all PX4 build targets.

```sh
sudo apt-get update -y
sudo apt-get install git zip qtcreator cmake \
    build-essential genromfs ninja-build exiftool vim-common -y
# Required python packages
sudo apt-get install python-argparse \
    python-empy python-toml python-numpy python-yaml \
    python-dev python-pip -y
sudo -H pip install --upgrade pip 
sudo -H pip install pandas jinja2 pyserial cerberus
```

You may also wish to install [pyulog](https://github.com/PX4/pyulog#pyulog). This is is a useful python package that contains scripts to parse *ULog* files and display them.

    # optional python tools
    sudo -H pip install pyulog
    

<!-- import docs ninja build system --> {% include "_ninja_build_system.txt" %}

## FastRTPS installation

[eProsima Fast RTPS](http://eprosima-fast-rtps.readthedocs.io/en/latest/) is a C++ implementation of the RTPS (Real Time Publish Subscribe) protocol. FastRTPS is used, via the [RTPS/ROS2 Interface: PX4-FastRTPS Bridge](../middleware/micrortps.md), to allow PX4 uORB topics to be shared with offboard components.

The following instructions can be used to install the FastRTPS 1.5 binaries to your home directory.

```sh
wget http://www.eprosima.com/index.php/component/ars/repository/eprosima-fast-rtps/eprosima-fast-rtps-1-5-0/eprosima_fastrtps-1-5-0-linux-tar-gz -O eprosima_fastrtps-1-5-0-linux.tar.gz
tar -xzf eprosima_fastrtps-1-5-0-linux.tar.gz eProsima_FastRTPS-1.5.0-Linux/
tar -xzf eprosima_fastrtps-1-5-0-linux.tar.gz requiredcomponents
tar -xzf requiredcomponents/eProsima_FastCDR-1.0.7-Linux.tar.gz
```

> **Note** In the following lines where we compile the FastCDR and FastRTPS libraries, the `make` command is issued with the `-j2` option. This option defines the number of parallel threads (or `j`obs) that are used to compile the source code. Change `-j2` to `-j<number_of_cpu_cores_in_your_system>` to speed up the compilation of the libraries.

```sh
cd eProsima_FastCDR-1.0.7-Linux; ./configure --libdir=/usr/lib; make -j2; sudo make install
cd ..
cd eProsima_FastRTPS-1.5.0-Linux; ./configure --libdir=/usr/lib; make -j2; sudo make install
cd ..
rm -rf requiredcomponents eprosima_fastrtps-1-5-0-linux.tar.gz
```

> **Note** More "generic" instructions, which additionally cover installation from source, can be found here: [Fast RTPS installation](../setup/fast-rtps-installation.md).

## Simulation Dependencies

The dependencies for the Gazebo and jMAVSim simulators listed below. You should minimally install jMAVSim to make it easy to test the installation. Additional information about these and other supported simulators is covered in: [Simulation](../simulation/README.md).

### jMAVSim

Install the dependencies for [jMAVSim Simulation](../simulation/jmavsim.md).

    # jMAVSim simulator
    sudo apt-get install ant openjdk-8-jdk openjdk-8-jre -y
    

### Gazebo

> **Note** If you're going work with ROS then follow the [ROS/Gazebo](#rosgazebo) instructions in the following section (these install Gazebo automatically, as part of the ROS installation).

Install the dependencies for [Gazebo Simulation](../simulation/gazebo.md).

    # Gazebo simulator
    sudo apt-get install protobuf-compiler libeigen3-dev libopencv-dev -y
    sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
    ## Setup keys
    wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
    ## Update the debian database:
    sudo apt-get update -y
    ## Install Gazebo9
    sudo apt-get install gazebo9 -y
    ## For developers (who work on top of Gazebo) one extra package
    sudo apt-get install libgazebo9-dev -y
    

> **Tip** PX4 works with Gazebo 7, 8, and 9. The [installation instructions](http://gazebosim.org/tutorials?tut=install_ubuntu&cat=install) above are for installing Gazebo 9.

<!-- these dependencies left over when I separated the dependencies. These appear to both be for using Clang. MOve them down?
sudo apt-get install clang-3.5 lldb-3.5 -y
-->

### ROS/Gazebo

Install the dependencies for [ROS/Gazebo](../ros/README.md) ("Kinetic"). These include Gazebo7 (at time of writing, the default version that comes with ROS). The instructions come from the ROS Wiki [Ubuntu page](http://wiki.ros.org/kinetic/Installation/Ubuntu).

```sh
# ROS Kinetic/Gazebo
## Gazebo dependencies
sudo apt-get install protobuf-compiler libeigen3-dev libopencv-dev -y

## ROS Gazebo: http://wiki.ros.org/kinetic/Installation/Ubuntu
## Setup keys
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
## For keyserver connection problems substitute hkp://pgp.mit.edu:80 or hkp://keyserver.ubuntu.com:80 above.
sudo apt-get update
## Get ROS/Gazebo
sudo apt-get install ros-kinetic-desktop-full -y
## Initialize rosdep
sudo rosdep init
rosdep update
## Setup environment variables
rossource="source /opt/ros/kinetic/setup.bash"
if grep -Fxq "$rossource" ~/.bashrc; then echo ROS setup.bash already in .bashrc;
else echo "$rossource" >> ~/.bashrc; fi
source ~/.bashrc
## Get rosinstall
sudo apt-get install python-rosinstall -y
```

Install the [MAVROS \(MAVLink on ROS\)](../ros/mavros_installation.md) package. This enables MAVLink communication between computers running ROS, MAVLink enabled autopilots, and MAVLink enabled GCS.

> **Tip** MAVROS can be installed as an ubuntu package or from source. Source is recommended for developers.

```sh
## Create catkin workspace (ROS build system)
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws

## Install dependencies
sudo apt-get install python-wstool python-rosinstall-generator python-catkin-tools -y

## Initialise wstool
wstool init ~/catkin_ws/src

## Build MAVROS
### Get source (upstream - released)
rosinstall_generator --upstream mavros | tee /tmp/mavros.rosinstall
### Get latest released mavlink package
rosinstall_generator mavlink | tee -a /tmp/mavros.rosinstall
### Setup workspace & install deps
wstool merge -t src /tmp/mavros.rosinstall
wstool update -t src
rosdep install --from-paths src --ignore-src --rosdistro kinetic -y
```

> **Note** If you use an ubuntu-based distro and the command `rosdep install --from-paths src --ignore-src --rosdistro kinetic -y` fails, you can try to force the command to run by executing `rosdep install --from-paths src --ignore-src --rosdistro kinetic -y --os ubuntu:xenial`

```sh
## Build!
catkin build
## Re-source environment to reflect new packages/build environment
catkin_ws_source="source ~/catkin_ws/devel/setup.bash"
if grep -Fxq "$catkin_ws_source" ~/.bashrc; then echo ROS catkin_ws setup.bash already in .bashrc;
else echo "$catkin_ws_source" >> ~/.bashrc; fi
source ~/.bashrc
```

## NuttX-based Hardware

Install the following dependencies to build for NuttX based hardware: Pixhawk, Pixfalcon, Pixracer, Pixhawk 3, Intel® Aero Ready to Fly Drone.

> **Note** Packages with specified versions should be installed with the specified package version.

```sh
sudo apt-get install python-serial openocd \
    flex bison libncurses5-dev autoconf texinfo \
    libftdi-dev libtool zlib1g-dev -y
```

Remove any old versions of the arm-none-eabi toolchain.

```sh
sudo apt-get remove gcc-arm-none-eabi gdb-arm-none-eabi binutils-arm-none-eabi gcc-arm-embedded
sudo add-apt-repository --remove ppa:team-gcc-arm-embedded/ppa
```

<!-- import GCC toolchain common documentation --> {% include "_gcc_toolchain_installation.txt" %}

## Snapdragon Flight

Setup instructions for Snapdragon Flight are provided in the *PX4 User Guide*:

* [Development Environment](https://docs.px4.io/en/flight_controller/snapdragon_flight_dev_environment_installation.html)
* [Software Installation](https://docs.px4.io/en/flight_controller/snapdragon_flight_software_installation.html)
* [Configuration](https://docs.px4.io/en/flight_controller/snapdragon_flight_configuration.html)

## Raspberry Pi Hardware

Developers working on Raspberry Pi hardware need to download a ARMv7 cross-compiler, either GCC or clang. The current recommended toolchain for raspbian can be cloned from `https://github.com/raspberrypi/tools.git` (at time of writing 4.9.3). The `PATH` environmental variable should include the path to the gcc cross-compiler collection of tools (e.g. gcc, g++, strip) prefixed with `arm-linux-gnueabihf-`.

```sh
git clone https://github.com/raspberrypi/tools.git ${HOME}/rpi-tools

# test compiler
$HOME/rpi-tools/arm-bcm2708/gcc-linaro-arm-linux-gnueabihf-raspbian-x64/bin/arm-linux-gnueabihf-gcc -v

# permanently update PATH variable by modifying ~/.profile
echo 'export PATH=$PATH:$HOME/rpi-tools/arm-bcm2708/gcc-linaro-arm-linux-gnueabihf-raspbian-x64/bin' >> ~/.profile

# update PATH variable only for this session
export PATH=$PATH:$HOME/rpi-tools/arm-bcm2708/gcc-linaro-arm-linux-gnueabihf-raspbian-x64/bin
```

### clang

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

Developers working with the Parrot Bebop should install the RPi Linux Toolchain. Follow the description under [Raspberry Pi hardware](#raspberry-pi-hardware).

Next, install ADB.

```sh
sudo apt-get install android-tools-adb -y
```

## Additional Tools

After setting up the build/simulation toolchain, see [Additional Tools](../setup/generic_dev_tools.md) for information about other useful tools.

## Next Steps

Once you have finished setting up the environment, continue to the [build instructions](../setup/building_px4.md).