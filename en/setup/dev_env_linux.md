# Development Environment on Linux

We have standardized on Debian / Ubuntu LTS as the supported Linux distribution, but [boutique distribution instructions](../setup/dev_env_linux_boutique.md) are available for Cent OS and Arch Linux.

## Permission Setup

> **Warning** Never ever fix permission problems by using 'sudo'. It will create more permission problems in the process and require a system reinstallation to fix them.

The user needs to be part of the group "dialout":

```sh
sudo usermod -a -G dialout $USER
```

And then you have to logout and login again, as this is only changed after a new login.

## Installation

Update the package list and install the following dependencies for all PX4 build targets. PX4 supports four main families:

* NuttX based hardware: [Pixhawk](../flight_controller/pixhawk.md), [Pixfalcon](../flight_controller/pixfalcon.md),
  [Pixracer](../flight_controller/pixracer.md), [Crazyflie](../flight_controller/crazyflie2.md),
  [Intel® Aero Ready to Fly Drone](../flight_controller/intel_aero.md)
* [Qualcomm Snapdragon Flight hardware](../flight_controller/snapdragon_flight.md)
* Linux-based hardware: [Raspberry Pi 2/3](../flight_controller/raspberry_pi.md), Parrot Bebop
* Host simulation: [jMAVSim SITL](../simulation/sitl.md) and [Gazebo SITL](../simulation/gazebo.md)

> **Info** Install the [Ninja Build System](../setup/dev_env_linux_boutique.md#ninja-build-system) for faster build times than with Make. It will be automatically selected if installed.

```sh
sudo add-apt-repository ppa:george-edison55/cmake-3.x -y
sudo apt-get update
sudo apt-get install python-argparse git-core wget zip \
    python-empy qtcreator cmake build-essential genromfs -y
# simulation tools
sudo apt-get install ant protobuf-compiler libeigen3-dev libopencv-dev openjdk-8-jdk openjdk-8-jre clang-3.5 lldb-3.5 -y
```

### NuttX based hardware

Ubuntu comes with a serial modem manager which interferes heavily with any robotics related use of a serial port \(or USB serial\). It can deinstalled without side effects:

```sh
sudo apt-get remove modemmanager
```

Update the package list and install the following dependencies. Packages with specified versions should be installed with this particular package version.

```sh
sudo apt-get install python-serial openocd \
    flex bison libncurses5-dev autoconf texinfo build-essential \
    libftdi-dev libtool zlib1g-dev \
    python-empy  -y
```

Make sure to remove leftovers before adding the arm-none-eabi toolchain.

```sh
sudo apt-get remove gcc-arm-none-eabi gdb-arm-none-eabi binutils-arm-none-eabi gcc-arm-embedded
sudo add-apt-repository --remove ppa:team-gcc-arm-embedded/ppa
```

Then follow the [toolchain installation instructions](../setup/dev_env_linux_boutique.md#toolchain-installation) to install the arm-none-eabi toolchain version 4.9 or 5.4 manually.

### Snapdragon Flight

#### Toolchain installation

```sh
sudo apt-get install android-tools-adb android-tools-fastboot fakechroot fakeroot unzip xz-utils wget python python-empy -y
```

Please follow the instructions on https://github.com/ATLFlight/cross_toolchain for the toolchain installation.

Load the new configuration:

```sh
source ~/.bashrc
```

#### Sysroot Installation

A sysroot is required to provide the libraries and header files needed to cross compile applications for the Snapdragon Flight applications processor.

The qrlSDK sysroot provies the required header files and libraries for the camera, GPU, etc.

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

#### Update ADSP firmware

Before building, flashing and running code, you'll need to update the [ADSP firmware](../flight_controller/snapdragon_flight_advanced.md#updating-the-adsp-firmware).

#### References

There is a an external set of documentation for Snapdragon Flight toolchain and SW setup and verification:
[ATLFlightDocs](https://github.com/ATLFlight/ATLFlightDocs/blob/master/README.md)

Messages from the DSP can be viewed using mini-dm.

```sh
${HEXAGON_SDK_ROOT}/tools/debug/mini-dm/Linux_Debug/mini-dm
```

Note: Alternatively, especially on Mac, you can also use [nano-dm](https://github.com/kevinmehall/nano-dm).

### Raspberry Pi hardware

Developers working on Raspberry Pi hardware should download the RPi Linux toolchain from below. The installation script will automatically install the cross-compiler toolchain. If you are looking for the _native_ Raspberry Pi toolchain to compile directly on the Pi, see [here](../flight_controller/raspberry_pi.md#native-builds-optional)

```sh
git clone https://github.com/pixhawk/rpi_toolchain.git
cd rpi_toolchain
./install_cross.sh
```

You will be required to enter your password for toolchain installation to complete successfully.

You can pass a different path to the installer script if you wouldn't like to install the toolchain to the default location of `/opt/rpi_toolchain`. Run `./install_cross.sh <PATH>`. The installer will automatically configure required environment variables as well.

Finally, run the following command to update the evironmental variables:
```
source ~/.profile
```

### Parrot Bebop

Developers working with the Parrot Bebop should install the RPi Linux Toolchain. Follow the
description under [Raspberry Pi hardware](../flight_controller/raspberry_pi.md).

Next, install ADB.

``sh
sudo apt-get install android-tools-adb -y` ``

## Finishing Up

Now continue to run the [first build](../setup/building_px4.md)!
