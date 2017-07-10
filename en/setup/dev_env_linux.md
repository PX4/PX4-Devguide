# Development Environment on Ubuntu LTS / Debian Linux

[Ubuntu Linux LTS](https://wiki.ubuntu.com/LTS) is the main PX4 development platform, allowing you to build for [all PX4 targets](../setup/dev_env.md#supported-targets) (NuttX based hardware, Qualcomm Snapdragon Flight hardware, Linux-based hardware, Simulation).

> **Tip** We have standardized on Debian / Ubuntu LTS as the supported Linux distribution. Installation instructions are also provided for [boutique distributions](../setup/dev_env_linux_boutique.md) (Cent OS and Arch Linux).

The following instructions explain how to set up a development environment each of the supported targets.

## Permission Setup

> **Warning** Never ever fix permission problems by using `sudo`. It will create more permission problems in the process and require a system reinstallation to fix them.

The user needs to be part of the group "dialout":

```sh
sudo usermod -a -G dialout $USER
```

Then logout and login again (the change is only made after a new login).



## Convenience Bash Scripts

We've created a number of bash scripts that you can use to install the Simulators and/or NuttX toolchain. This is a lot easier/more reliable than copying/typing the instructions in the rest of this topic.


### How to use the scripts
To use the scripts, download them to your computer, make them executable, and then run them. For example:
```bash
chmod +x ubuntu_sim.sh
./ubuntu_sim.sh
```

### Scripts

All of the scripts include the [Ninja Build System](#ninja-build-system), [Common Dependencies](#common-dependencies), and [Gazebo & jMAVSim Simulators](#simulation-dependencies), and also download the PX4 source to your computer (**~/src/Firmware**). 

The scripts are:

* <strong><a href="https://raw.githubusercontent.com/hamishwillee/Devguide/tidy_toolchain/build_scripts/ubuntu_sim.sh" target="_blank" download>ubuntu_sim.sh</a></strong>: Common dependencies, Simulator builds. Used as "base" for other scripts.
* <strong><a href="https://raw.githubusercontent.com/hamishwillee/Devguide/tidy_toolchain/build_scripts/ubuntu_sim_nuttx.sh" target="_blank" download>ubuntu_sim_nuttx.sh</a></strong>: Common dependencies, Simulator builds and NuttX tools. *This requires computer restart on completion.*

> **Tip** The **ubuntu_sim.sh** script contains the common dependencies for all PX4 build targets. You can run this before installing the remaining dependencies for [Qualcomm Snapdragon Flight](#snapdragon-flight) or [Raspberry Pi/Parrot Bebop](#raspberry-pi-hardware).

## Ubuntu Configuration

Ubuntu comes with a serial modem manager which interferes heavily with any robotics related use of a serial port \(or USB serial\). It can removed/deinstalled without side effects:

```sh
sudo apt-get remove modemmanager
```

## Ninja Build System

[Ninja](https://github.com/martine/ninja) is a faster build system than *Make* and the PX4 *CMake* generators support it. Unfortunately Ubuntu currently carries a very outdated version of *Ninja*. To install a recent version, download the binary and add it to your path:


```sh
mkdir -p $HOME/ninja
cd $HOME/ninja
wget https://github.com/martine/ninja/releases/download/v1.6.0/ninja-linux.zip
unzip ninja-linux.zip
rm ninja-linux.zip
exportline="export PATH=$HOME/ninja:\$PATH"
if grep -Fxq "$exportline" ~/.profile; then echo nothing to do ; else echo $exportline >> ~/.profile; fi
. ~/.profile
```

## Common Dependencies

Update the package list and install the following dependencies for all PX4 build targets. 

> **Info** Install the [Ninja Build System](#ninja-build-system) for faster build times than with `Make`. It will be automatically selected if installed.

```sh
sudo add-apt-repository ppa:george-edison55/cmake-3.x -y
sudo apt-get update
sudo apt-get install python-argparse git-core wget zip \
    python-empy qtcreator cmake build-essential genromfs -y
# required python packages
sudo apt-get install python-pip
sudo -H pip install pandas jinja2
```

## Simulation Dependencies
The dependencies for the Gazebo and jMAVSim simulators listed below. You should minimally install jMAVSim to make it easy to test the installation. Additional information about these and other supported simulators is covered in: [Simulation](../simulation/README.md).

### jMAVSim

Install the dependencies for [jMAVSim Simulation](../simulation/sitl.md).

```
# jMAVSim simulator
sudo apt-get install ant openjdk-8-jdk openjdk-8-jre -y
```

### Gazebo

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

### (Cross) Toolchain installation

```sh
sudo apt-get install android-tools-adb android-tools-fastboot \
    fakechroot fakeroot unzip xz-utils wget python python-empy -y
```

Please follow the instructions on https://github.com/ATLFlight/cross_toolchain for the toolchain installation.

Load the new configuration:

```sh
source ~/.bashrc
```

### Sysroot Installation

A sysroot is required to provide the libraries and header files needed to cross compile applications for the Snapdragon Flight applications processor.

The qrlSDK sysroot provides the required header files and libraries for the camera, GPU, etc.

Download the file [Flight\_3.1.1\_qrlSDK.zip](http://support.intrinsyc.com/attachments/download/690/Flight_3.1.1_qrlSDK.zip) and save it in `cross_toolchain/download/`.

```sh
cd cross_toolchain
unset HEXAGON_ARM_SYSROOT
./qrlinux_sysroot.sh
```

Append the following to your ~/.bashrc:

```sh
export HEXAGON_ARM_SYSROOT=${HOME}/Qualcomm/qrlinux_v3.1.1_sysroot
```

Load the new configuration:

```sh
source ~/.bashrc
```

For more sysroot options see [Sysroot Installation](https://github.com/ATLFlight/cross_toolchain/blob/sdk3/README.md#sysroot-installation)

### Update ADSP firmware

Before building, flashing and running code, you'll need to update the [ADSP firmware](../flight_controller/snapdragon_flight_advanced.md#updating-the-adsp-firmware).

### References

There is a an external set of documentation for Snapdragon Flight toolchain and SW setup and verification:
[ATLFlightDocs](https://github.com/ATLFlight/ATLFlightDocs/blob/master/README.md)

Messages from the DSP can be viewed using mini-dm.

```sh
${HEXAGON_SDK_ROOT}/tools/debug/mini-dm/Linux_Debug/mini-dm
```

Note: Alternatively, especially on Mac, you can also use [nano-dm](https://github.com/kevinmehall/nano-dm).

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
mkdir build_posix_rpi_cross_clang
cd build_posix_rpi_cross_clang
cmake \
-G"Unix Makefiles" \
-DCONFIG=posix_rpi_cross \
-DCMAKE_C_COMPILER=clang \
-DCMAKE_CXX_COMPILER=clang++ \
..

```

## Parrot Bebop

Developers working with the Parrot Bebop should install the RPi Linux Toolchain. Follow the
description under [Raspberry Pi hardware](../flight_controller/raspberry_pi_navio2.md).

Next, install ADB.

```sh
sudo apt-get install android-tools-adb -y
```

<!-- import docs for other tools and next steps. -->
{% include "_addition_dev_tools.txt" %}
