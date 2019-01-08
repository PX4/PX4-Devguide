# 리눅스 개발환경

리눅스에서는 [모든 PX4 타겟들](../setup/dev_env.md#supported-targets) (NuttX based hardware, Qualcomm Snapdragon Flight hardware, Linux-based hardware, Simulation, ROS)에 대해서 빌드가 가능합니다.

다음은 지원하는 타겟의 각 개발환경을 *수동*으로 셋업하는 방법을 설명합니다.

> **Tip** We recommend that you use the [Convenience bash scripts](#convenience-bash-scripts) to install the Simulators and/or NuttX toolchain (this is easier than typing in the instructions below). Then follow just the additional instructions for other targets (e.g. Qualcomm Snapdragon Flight, Bebop, Raspberry Pi, etc.)



## 편리한 Bash 스크립트 {#convenience-bash-scripts}

We've created a number of bash scripts that you can use to install the Simulators and/or NuttX toolchain. All the scripts include the *Qt Creator IDE*, [Ninja Build System](#ninja-build-system), [Common Dependencies](#common-dependencies), [FastRTPS](#fastrtps-installation), and also download the PX4 source to your computer (**~/src/Firmware**).

> **Tip** The scripts have been tested on a clean Ubuntu LTS installation. They *may* not work as expected if installed on top of an existing system.

The scripts are:

* <strong><a href="https://raw.githubusercontent.com/PX4/Devguide/master/build_scripts/ubuntu_sim.sh" target="_blank" download>ubuntu_sim.sh</a></strong>: [Common Dependencies](#common-dependencies), [jMAVSim](#jmavsim) simulator, [Gazebo8](#gazebo) simulator.
  * This contains the common dependencies for all PX4 build targets.
  * You can run this before installing the remaining dependencies for [Qualcomm Snapdragon Flight](#snapdragon-flight) or [Raspberry Pi/Parrot Bebop](#raspberry-pi-hardware).
* <strong><a href="https://raw.githubusercontent.com/PX4/Devguide/master/build_scripts/ubuntu_sim_nuttx.sh" target="_blank" download>ubuntu_sim_nuttx.sh</a></strong>: **ubuntu_sim.sh** + NuttX tools. *This requires computer restart on completion.*
* <strong><a href="https://raw.githubusercontent.com/PX4/Devguide/master/build_scripts/ubuntu_sim_ros_gazebo.sh" target="_blank" download>ubuntu_sim_ros_gazebo.sh</a></strong>: [ROS/Gazebo and MAVROS](#rosgazebo).
  * ROS is installed with Gazebo7 by default (we have chosen to use the default rather than Gazebo8 to simplify ROS development).
  * Your catkin (ROS build system) workspace is created at **~/catkin_ws/**.


### How to use the scripts

To use the scripts:
1. Make the user a member of the group "dialout" (this only has to be done once):
   1. Open a terminal and enter the following command:
      ```sh
      sudo usermod -a -G dialout $USER
      ```
   1. Logout and login again (the change is only made after a new login).
1. Download the desired script
1. Run the script in a bash shell (e.g. to run **ubuntu_sim.sh**):
   ```bash
   source ubuntu_sim.sh
   ```
   Acknowledge any prompts as the scripts progress.


## Permission Setup

> **Warning** Never ever fix permission problems by using `sudo`. It will create more permission problems in the process and require a system re-installation to fix them.

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

<!-- import docs ninja build system -->
{% include "_ninja_build_system.txt" %}


## Common Dependencies

Update the package list and install the following dependencies for all PX4 build targets.

```sh
sudo add-apt-repository ppa:george-edison55/cmake-3.x -y
sudo apt-get update
sudo apt-get install python-argparse git-core wget zip \
    python-empy qtcreator cmake build-essential genromfs -y
# required python packages
sudo apt-get install python-dev -y
sudo apt-get install python-pip
sudo -H pip install pandas jinja2
pip install pyserial
```

You may also wish to install [pyulog](https://github.com/PX4/pyulog#pyulog). This is is a useful python package that contains scripts to parse *ULog* files and display them.
```
# optional python tools
pip install pyulog
```

## FastRTPS installation

[eProsima Fast RTPS](http://eprosima-fast-rtps.readthedocs.io/en/latest/) is a C++ implementation of the RTPS (Real Time Publish Subscribe) protocol. FastRTPS is used, via the [RTPS/ROS2 Interface: PX4-FastRTPS Bridge](../middleware/micrortps.md), to allow PX4 uORB topics to be shared with offboard components.

The following instructions can be used to install the FastRTPS 1.5 binaries to your home directory.

```sh
wget http://www.eprosima.com/index.php/component/ars/repository/eprosima-fast-rtps/eprosima-fast-rtps-1-5-0/eprosima_fastrtps-1-5-0-linux-tar-gz
mv eprosima_fastrtps-1-5-0-linux-tar-gz eprosima_fastrtps-1-5-0-linux.tar.gz
tar -xzf eprosima_fastrtps-1-5-0-linux.tar.gz eProsima_FastRTPS-1.5.0-Linux/
tar -xzf eprosima_fastrtps-1-5-0-linux.tar.gz requiredcomponents
tar -xzf requiredcomponents/eProsima_FastCDR-1.0.7-Linux.tar.gz
(cd eProsima_FastCDR-1.0.7-Linux && ./configure --libdir=/usr/lib && make && sudo make install)
(cd eProsima_FastRTPS-1.5.0-Linux && ./configure --libdir=/usr/lib && make && sudo make install)
```

> **Note** More "generic" instructions, which additionally cover installation from source, can be found here: [Fast RTPS installation](../setup/fast-rtps-installation.md).


## Simulation Dependencies
The dependencies for the Gazebo and jMAVSim simulators listed below. You should minimally install jMAVSim to make it easy to test the installation. Additional information about these and other supported simulators is covered in: [Simulation](../simulation/README.md).

### jMAVSim

Install the dependencies for [jMAVSim Simulation](../simulation/jmavsim.md).

```
# jMAVSim simulator
sudo apt-get install ant openjdk-8-jdk openjdk-8-jre -y
```

### Gazebo

> **Note** If you're going work with ROS then follow the [ROS/Gazebo](#rosgazebo) instructions in the following section (these install Gazebo automatically, as part of the ROS installation).

Install the dependencies for [Gazebo Simulation](../simulation/gazebo.md).

```
# Gazebo simulator
sudo apt-get install protobuf-compiler libeigen3-dev libopencv-dev -y
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
## Setup keys
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
## Update the debian database:
sudo apt-get update -y
## Install Gazebo8
sudo apt-get install gazebo8 -y
## For developers (who work on top of Gazebo) one extra package
sudo apt-get install libgazebo8-dev
```

<!-- these dependencies left over when I separated the dependencies. These appear to both be for using Clang. MOve them down?
sudo apt-get install clang-3.5 lldb-3.5 -y
-->

### ROS/Gazebo

Install the dependencies for [ROS/Gazebo](../ros/README.md)) ("Kinetic"). These include Gazebo7 (at time of writing, the default version that comes with ROS). The instructions come from the ROS Wiki [Ubuntu page](http://wiki.ros.org/kinetic/Installation/Ubuntu).

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
echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
## Get rosinstall
sudo apt-get install python-rosinstall -y
```

Install the [MAVROS \(MAVLink on ROS\)](../ros/mavros_installation.md) package. This enables MAVLink communication between computers running ROS, MAVLink enabled autopilots, and MAVLink enabled GCS.

> **Tip** MAVROS can be installed as a ubuntu package or from source. Source is recommended for developers.


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
## Build!
catkin build
## Re-source environment to reflect new packages/build environment
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
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
<!-- removed duplicate deps from "common" set: build-essential python-empy -->

Remove any old versions of the arm-none-eabi toolchain.

```sh
sudo apt-get remove gcc-arm-none-eabi gdb-arm-none-eabi binutils-arm-none-eabi gcc-arm-embedded
sudo add-apt-repository --remove ppa:team-gcc-arm-embedded/ppa
```

Execute the script below to install 5.4:

```sh
pushd .
cd ~
wget https://launchpad.net/gcc-arm-embedded/5.0/5-2016-q2-update/+download/gcc-arm-none-eabi-5_4-2016q2-20160622-linux.tar.bz2
tar -jxf gcc-arm-none-eabi-5_4-2016q2-20160622-linux.tar.bz2
exportline="export PATH=$HOME/gcc-arm-none-eabi-5_4-2016q2/bin:\$PATH"
if grep -Fxq "$exportline" ~/.bash_profile; then echo nothing to do ; else echo $exportline >> ~/.profile; fi
popd
```

Run these commands to install the 32 bit support libraries (this might fail and can be skipped if running a 32 bit OS):

```sh
sudo dpkg --add-architecture i386
sudo apt-get update

sudo apt-get install libc6:i386 libgcc1:i386 libstdc++5:i386 libstdc++6:i386
sudo apt-get install gcc-4.6-base:i386
```

Now restart your machine.

**Troubleshooting**

Check the version by entering the following command:

```sh
arm-none-eabi-gcc --version
```

The output should be something similar to:

```sh
arm-none-eabi-gcc (GNU Tools for ARM Embedded Processors) 5.4.1 20160609 (release) [ARM/embedded-5-branch revision 237715]
Copyright (C) 2015 Free Software Foundation, Inc.
This is free software; see the source for copying conditions.  There is NO
warranty; not even for MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
```

If you get the following output, make sure you have the 32bit libs installed properly as described in the installation steps:

```sh
arm-none-eabi-gcc --version
arm-none-eabi-gcc: No such file or directory
```

## Snapdragon Flight

Setup instructions for Snapdragon Flight are provided in the *PX4 User Guide*:
* [Development Environment](https://docs.px4.io/en/flight_controller/snapdragon_flight_dev_environment_installation.html)
* [Software Installation](https://docs.px4.io/en/flight_controller/snapdragon_flight_software_installation.html)
* [Configuration](https://docs.px4.io/en/flight_controller/snapdragon_flight_configuration.html)

## Raspberry Pi Hardware

Developers working on Raspberry Pi hardware need to download a ARMv7 cross-compiler, either GCC or clang.
The recommended toolchain for raspbian is GCC 4.8.3 and can be cloned from `https://github.com/raspberrypi/tools.git`.
The `PATH` environmental variable should include the path to the gcc cross-compiler collection of tools (e.g. gcc, g++, strip) prefixed with `arm-linux-gnueabihf-`.

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

Download clang for your specific distribution from [LLVM Download page](http://releases.llvm.org/download.html) and unpack it.
Assuming that you've unpacked clang to `CLANG_DIR`, and `clang` binary is available in `CLANG_DIR/bin`, and you have the GCC cross-compiler in `GCC_DIR`, you will need to setup the symlinks for clang in the `GCC_DIR` bin dir, and add `GCC_DIR/bin` to `PATH`.

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

## Parrot Bebop

Developers working with the Parrot Bebop should install the RPi Linux Toolchain. Follow the
description under [Raspberry Pi hardware](https://docs.px4.io/en/flight_controller/raspberry_pi_navio2.html).

Next, install ADB.

```sh
sudo apt-get install android-tools-adb -y
```

<!-- import docs for other tools and next steps. -->
{% include "_addition_dev_tools.txt" %}
