# Development Environment on Ubuntu LTS / Debian Linux

[Ubuntu Linux LTS](https://wiki.ubuntu.com/LTS) 16.04 is the standard/preferred Linux development OS. It allows you to build for the [most PX4 targets](../setup/dev_env.md#supported-targets) (NuttX based hardware, *Qualcomm Snapdragon Flight* hardware, Linux-based hardware, Simulation).

Bash scripts are provided to help make it easy to install development environment for different target platforms:

* **[ubuntu.sh](https://github.com/PX4/Firmware/blob/{{ book.px4_version }}/Tools/setup/ubuntu.sh)**: Installs [Gazebo 9](../simulation/gazebo.md) and [jMAVSim](../simulation/jmavsim.md) simulators and/or [NuttX/Pixhawk](../setup/building_px4.md#nuttx) tools. Does not include dependencies for [FastRTPS](#fast_rtps).
* **[ubuntu_sim_ros_melodic.sh](https://raw.githubusercontent.com/PX4/Devguide/{{ book.px4_version }}/build_scripts/ubuntu_sim_ros_melodic.sh)**: Installs [ROS "Melodic"](#rosgazebo) and PX4 on Ubuntu 18.04 LTS.

> **Tip** The scripts have been tested on clean Ubuntu 16.04 and 18.04 LTS installations. They *may* not work as expected if installed on top of an existing system or a different Ubuntu release.

The instructions below explain how to download and use the scripts.

## Gazebo, JMAVSim and NuttX (Pixhawk) Targets {#sim_nuttx}

Use the [ubuntu.sh](https://github.com/PX4/Firmware/blob/{{ book.px4_version }}/Tools/setup/ubuntu.sh) script to set up a development environment that includes [Gazebo 9](../simulation/gazebo.md) and [jMAVSim](../simulation/jmavsim.md) simulators, and/or the [NuttX/Pixhawk](../setup/building_px4.md#nuttx) toolchain.

To install the toolchain:

1. [Download PX4 Source Code](../setup/building_px4.md): 
        bash
        git clone https://github.com/PX4/Firmware.git --recursive

2. Run the **ubuntu.sh** with no arguments (in a bash shell) to install everything: 
        bash
        bash ./Tools/setup/ubuntu.sh
    
      
    * Acknowledge any prompts as the script progress.
    * You can use the `--no-nuttx` and `--no-sim-tools` to omit the nuttx and/or simulation tools.
3. Restart the computer on completion.

> **Note** You can alternatively download [ubuntu.sh](https://github.com/PX4/Firmware/blob/{{ book.px4_version }}/Tools/setup/ubuntu.sh) and [requirements.txt](https://github.com/PX4/Firmware/blob/{{ book.px4_version }}/Tools/setup/requirements.txt) from the PX4 source repository (**/Tools/setup/**) and run ubuntu.sh in place:   
> `wget https://raw.githubusercontent.com/PX4/Firmware/{{ book.px4_version }}/Tools/setup/ubuntu.sh`   
> `wget https://raw.githubusercontent.com/PX4/Firmware/{{ book.px4_version }}/Tools/setup/requirements.txt`   
> `bash ubuntu.sh`

Notes:

* PX4 works with Gazebo 7, 8, and 9. The script uses [gazebosim.org instructions](http://gazebosim.org/tutorials?tut=install_ubuntu&cat=install) to install Gazebo9.
* If you're going work with ROS then follow the [ROS/Gazebo](#rosgazebo) instructions instead (these install Gazebo automatically, as part of the ROS installation).
* You can verify the the NuttX installation by confirming the gcc version as shown:
    
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

1. Download [ubuntu.sh](https://github.com/PX4/Firmware/blob/{{ book.px4_version }}/Tools/setup/ubuntu.sh) and [requirements.txt](https://github.com/PX4/Firmware/blob/{{ book.px4_version }}/Tools/setup/requirements.txt) from the PX4 source repository (**/Tools/setup/**):   
    `wget https://raw.githubusercontent.com/PX4/Firmware/{{ book.px4_version }}/Tools/setup/ubuntu.sh`   
    `wget https://raw.githubusercontent.com/PX4/Firmware/{{ book.px4_version }}/Tools/setup/requirements.txt`
2. Run **ubuntu.sh** in a terminal to get just the common dependencies: 
        bash
        bash ubuntu.sh --no-nuttx --no-sim-tools

3. Then setup an ARMv7 cross-compiler (either GCC or clang) as described in the following sections.

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

### Native Builds

Additional developer information for using PX4 on Raspberry Pi (including building PX4 natively) can be found here: [Raspberry Pi 2/3 Navio2 Autopilot](https://docs.px4.io/en/flight_controller/raspberry_pi_navio2.html).

## ROS/Gazebo {#rosgazebo}

This section explains how to install [ROS/Gazebo](../ros/README.md) ("Melodic") for use with PX4.

> **Note** PX4 is tested with ROS Melodic on Ubuntu 18.04 LTS. ROS Melodic does not work on Ubuntu 16.04.

To install the development toolchain:

1. Download the script in a bash shell:   
    `wget https://raw.githubusercontent.com/PX4/Devguide/{{ book.px4_version }}/build_scripts/ubuntu_sim_ros_melodic.sh`
2. Run the script: 
        bash
        bash ubuntu_sim_ros_melodic.sh You may need to acknowledge some prompts as the script progresses.

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

## Additional Tools

After setting up the build/simulation toolchain, see [Additional Tools](../setup/generic_dev_tools.md) for information about other useful tools.

## Next Steps

Once you have finished setting up the environment, continue to the [build instructions](../setup/building_px4.md).