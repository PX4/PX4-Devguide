# Ubuntu LTS/Debian Linux 的开发环境

[Ubuntu Linux LTS](https://wiki.ubuntu.com/LTS) 16.04 is the standard/preferred Linux development OS. It allows you to build for [most PX4 targets](../setup/dev_env.md#supported-targets) (NuttX based hardware, Qualcomm Snapdragon Flight hardware, Linux-based hardware, Simulation).

> **Note** Ubuntu 18.04 is required if you want to work with *ROS Melodic* (which does not install on Ubuntu 16.04).

以下说明说明了如何 *手动* 设置每个受支持的目标的开发环境。

> **Tip** We recommend that you use the [Convenience bash scripts](#convenience-bash-scripts) to install the Simulators and/or NuttX toolchain (this is easier than typing in the instructions below). Then follow just the additional instructions for other targets (e.g. Qualcomm Snapdragon Flight, Bebop, Raspberry Pi, etc.)

<span></span>

> **Tip** After setting up the build/simulation toolchain, see [Additional Tools](../setup/generic_dev_tools.md) for information about other useful tools.

## 一键安装脚本

我们已经创建了许多 bash 脚本，您可以使用这些脚本来安装模拟器和 Nuttx 工具链。 以下脚本作用分别是安装*Qt Creator IDE*、[ Ninja构建系统](#ninja-build-system)、[通用依赖项](#common-dependencies)、[FastRTPS](#fastrtps-installation)，以及将PX4源下载到您的目录（**~/src/Firmware**）。

> **Tip** The scripts have been tested on clean Ubuntu 16.04 and 18.04 LTS installations. They *may* not work as expected if installed on top of an existing system or a different Ubuntu release.

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

> **Warning** Never ever fix permission problems by using `sudo`. It will create more permission problems in the process and require a system re-installation to fix them.

用户应先加入组 ”dialout“：

```sh
sudo usermod -a -G dialout $USER
```

注销重新登录（必须重新登录后才能生效）。

## 删除 modemmanager

Ubuntu 自带串口调试解调器管理器，这会和很多机器人使用的串口（或 USB 串口）发生冲突。 卸载并不会产生边际效应：

```sh
sudo apt-get remove modemmanager
```

## 通用依赖

更新包列表，并且安装以下依赖：

```sh
sudo apt-get update -y
sudo apt-get install git zip qtcreator cmake \
    build-essential genromfs ninja-build exiftool -y

# Install xxd (package depends on version)
which xxd || sudo apt install xxd -y || sudo apt-get install vim-common --no-install-recommends -y

# Required python packages
sudo apt-get install python-argparse \
    python-empy python-toml python-numpy python-yaml \
    python-dev python-pip -y
sudo -H pip install --upgrade pip 
sudo -H pip install pandas jinja2 pyserial cerberus
```

或许你会希望安装 [pyulog](https://github.com/PX4/pyulog#pyulog)。 它是一个实用的 Python 包，可以解析 *ULog* 文件并显示。

    # optional python tools
    sudo -H pip install pyulog
    

<!-- import docs ninja build system --> {% include "_ninja_build_system.md" %}

## FastRTPS 安装

[eProsima Fast RTPS](http://eprosima-fast-rtps.readthedocs.io/en/latest/) 是 RTPS协议的 C++ 实现库。 通过 [RTPS/ROS2 接口: px4-frtps bridge ](../middleware/micrortps.md) 使用 FastRTPS，允许与离板组件共享 PX4 uORB 话题。

以下说明可用于将 FastRTPS 1.5 二进制文件安装到您的主目录中。

```sh
wget http://www.eprosima.com/index.php/component/ars/repository/eprosima-fast-rtps/eprosima-fast-rtps-1-5-0/eprosima_fastrtps-1-5-0-linux-tar-gz -O eprosima_fastrtps-1-5-0-linux.tar.gz
tar -xzf eprosima_fastrtps-1-5-0-linux.tar.gz eProsima_FastRTPS-1.5.0-Linux/
tar -xzf eprosima_fastrtps-1-5-0-linux.tar.gz requiredcomponents
tar -xzf requiredcomponents/eProsima_FastCDR-1.0.7-Linux.tar.gz
```

> **Note** In the following lines where we compile the FastCDR and FastRTPS libraries, the `make` command is issued with the `-j2` option. This option defines the number of parallel threads (or `j`obs) that are used to compile the source code. Change `-j2` to `-j<number_of_cpu_cores_in_your_system>` to speed up the compilation of the libraries.

```sh
(cd eProsima_FastCDR-1.0.7-Linux && ./configure --libdir=/usr/lib && make -j2 && sudo make install)
(cd eProsima_FastRTPS-1.5.0-Linux && ./configure --libdir=/usr/lib && make -j2 && sudo make install)
rm -rf requiredcomponents eprosima_fastrtps-1-5-0-linux.tar.gz
```

> **Note** More "generic" instructions, which additionally cover installation from source, can be found here: [Fast RTPS installation](../setup/fast-rtps-installation.md).

## 模拟器依赖

下面列出的 Gazebo 和 jMAVSim 模拟器的依赖关系。 你可以先将 jMAVSim 最小安装以验证安装是否成功。 更多信息及模拟器支持参见：[模拟器](../simulation/README.md)。

### jMAVSim

为 [jMAVSim Simulation](../simulation/jmavsim.md) 安装依赖。

    # jMAVSim simulator
    sudo apt-get install ant openjdk-8-jdk openjdk-8-jre -y
    

### Gazebo

> **Note** If you're going work with ROS then follow the [ROS/Gazebo](#rosgazebo) instructions in the following section (these install Gazebo automatically, as part of the ROS installation).

为 [jMAVSim Simulation](../simulation/gazebo.md) 安装依赖。

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

### ROS/Gazebo

Install the dependencies for [ROS/Gazebo](../ros/README.md) ("Melodic"). These include Gazebo9 (the default version that comes with ROS Melodic). 这些说明来自 ROS Wiki [Ubuntu 页 ](http://wiki.ros.org/kinetic/Installation/Ubuntu)。

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

安装 [MAVROS （运行于 ROS 的 MAVLink ）](../ros/mavros_installation.md) 包。 这启动了运行 ROS 电脑之间的 MAVLink 连接， MAVLink 启动飞控，并且启动 QCS。

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

## 基于 Nuttx 的硬件

安装以下依赖项，以构建基于 Nuttx 的硬件：Pixhawk、Pixfalcon、Pixracer、Pixhawk 3、Intel® Aero Ready to Fly Drone。

> **Note** Packages with specified versions should be installed with the specified package version.

```sh
sudo apt-get install python-serial openocd \
    flex bison libncurses5-dev autoconf texinfo \
    libftdi-dev libtool zlib1g-dev -y
```

删除任何旧版本的 arm-none-eabi 工具链。

```sh
sudo apt-get remove gcc-arm-none-eabi gdb-arm-none-eabi binutils-arm-none-eabi gcc-arm-embedded
sudo add-apt-repository --remove ppa:team-gcc-arm-embedded/ppa
```

<!-- import GCC toolchain common documentation --> {% include "_gcc_toolchain_installation.md" %}

## 高通骁龙飞控（Snapdragon Flight）

在 *PX4用户指南* 中提供了高通骁龙飞控的安装说明:

* [开发环境](https://docs.px4.io/en/flight_controller/snapdragon_flight_dev_environment_installation.html)
* [软件安装](https://docs.px4.io/en/flight_controller/snapdragon_flight_software_installation.html)
* [配置](https://docs.px4.io/en/flight_controller/snapdragon_flight_configuration.html)

## 树莓派硬件

使用树莓派硬件的开发人员需要下载 ARMv7 交叉编译器，可以是 gcc，也可以是 clang。 当前推荐的树莓工具链可以从 `https://github.com/raspberrypi/tools.git` 下载（在编写4.9.3 时）。 `PATH` 环境变量应包括以 `arm-linux-gnueabihf-` 为前缀的 gcc 跨编译器工具集合的路径 （例如 gcc、g ++、strip）。

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

为了使用 clang，您还需要 GCC。

从 [LLVM 下载页面](http://releases.llvm.org/download.html) 下载您的特定发行版并将其解包。 假设您已解压缩 clang 到 `CLANG_DIR`，并且 `clang` 二进制文件在 `CLANG_DIR/bin` 中可用, 并且您在 `GCC_DIR` 中有 GCC 交叉编译器，则需要在 `GCC_DIR` bin dir 中设置 clang 的符号链接, 并添加 `GCC_DIR/bin</0 > 到 <code>PATH`。

下面的示例，用于使用 CMake 构建 PX4 固件。

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

### 本地编译

有关在树莓派上使用 PX4（包括本地构建 PX4）的其他开发人员信息，请参见此处：[Raspberry pi 2/navio2 autopilot](https://docs.px4.io/en/flight_controller/raspberry_pi_navio2.html)。

## Parrot Bebop

使用 Parrot Bebop 开发应该安装 RPi Linux 工具链。 参见 [树莓派硬件](#raspberry-pi-hardware) 描述。

接下来，安装 ADB。

```sh
sudo apt-get install android-tools-adb -y
```

## 其他工具

在设置构建/模拟工具链之后，有关其他有用工具的信息，请参阅 其他工具</0 >。</p> 

## 下一步

设置完环境后，请继续执行 构建说明</0 >。</p>