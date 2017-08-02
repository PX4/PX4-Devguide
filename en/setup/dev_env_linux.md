# Development Environment on Linux

Linux allows you to build for [all PX4 targets](../setup/dev_env.md#supported-targets) (NuttX based hardware, Qualcomm Snapdragon Flight hardware, Linux-based hardware, Simulation, ROS).

> **Tip** We have standardized on Debian / [Ubuntu Linux LTS](https://wiki.ubuntu.com/LTS) (16.04) as the supported Linux distribution. Installation instructions are also provided for [boutique distributions](../setup/dev_env_linux_boutique.md) (Cent OS and Arch Linux).

The following instructions explain how to set up a development environment on Ubuntu LTS using convenience bash scripts. Instructions for *manually installing* these and additional targets can be found in [Ubuntu/Debian Linux](../setup/dev_env_linux_ubuntu.md).


## Development Toolchain

The instructions below show how you can use bash scripts to install the developer toolchain on Ubuntu LTS. All the scripts include the *Qt Creator IDE*, [Ninja Build System](https://ninja-build.org/), [Common Dependencies](../setup/dev_env_linux_ubuntu.md#common-dependencies), and also download the PX4 source to your computer (**~/src/Firmware**).

> **Tip** The scripts have been tested on a clean Ubuntu LTS 16.04 installation. They *may* not work as expected if installed on top of an existing system or on another Ubuntu release. If you have any problems then follow the [manual installation instructions](../setup/dev_env_linux_ubuntu.md).

First make the user a member of the group "dialout"
1. On the command prompt enter:
   ```sh
   sudo usermod -a -G dialout $USER
   ```
1. Logout and login again (the change is only made after a new login).

Then follow the instructions for your development target in the sections below.

### Pixhawk/NuttX and jMAVSim/Gazebo Simulation

To install the development toolchain:

1. Download <a href="https://raw.githubusercontent.com/PX4/Devguide/master/build_scripts/ubuntu_sim_nuttx.sh" target="_blank" download>ubuntu_sim_nuttx.sh</a>.
1. Run the script in a bash shell:
   ```bash
   source ubuntu_sim_nuttx.sh
   ```
   You may need to acknowledge some prompts as the script progresses.
1. Restart the computer on completion.


### Snapdragon Flight or Raspberry Pi

To install the development toolchain:
1. Download <a href="https://raw.githubusercontent.com/PX4/Devguide/master/build_scripts/ubuntu_sim.sh" target="_blank" download>ubuntu_sim.sh</a> (this contains the simulators and common toolchain dependencies).
1. Run the script in a bash shell:
   ```bash
   source ubuntu_sim.sh
   ```
   You may need to acknowledge some prompts as the script progresses.
1. Follow the platform-specific instructions in [Ubuntu/Debian Linux](../setup/dev_env_linux_ubuntu.md) for your target:
   * [Snapdragon Flight](../setup/dev_env_linux_ubuntu.md#snapdragon-flight)
   * [Raspberry Pi](../setup/dev_env_linux_ubuntu.md#raspberry-pi-hardware)

### Parrot Bepop

Follow the (manual) instructions here: [Ubuntu/Debian Linux > Parrot Bebop](../setup/dev_env_linux_ubuntu.md#raspberry-pi-hardware).


### Gazebo with ROS

To install the development toolchain:

1. Download <a href="https://raw.githubusercontent.com/PX4/Devguide/master/build_scripts/ubuntu_sim_ros_gazebo.sh" target="_blank" download>ubuntu_sim_ros_gazebo.sh</a>.
1. Run the script in a bash shell:
   ```bash
   source ubuntu_sim_ros_gazebo.sh
   ```
   You may need to acknowledge some prompts as the script progresses.

Note: 
* ROS is installed with Gazebo7 by default (we have chosen to use the default rather than Gazebo8 to simplify ROS development).
* Your catkin (ROS build system) workspace is created at **~/catkin_ws/**.


<!-- Additional DevTools Common to all Platforms -->

## Ground Control Software

Download and install the [QGroundControl Daily Build](https://docs.qgroundcontrol.com/en/releases/daily_builds.html).

![QGroundControl](../../assets/qgc_goto.jpg)


## Editor / IDE

The development team often use:

* [Sublime Text](https://www.sublimetext.com): a fast and lean text editor. 
* [Qt Creator](http://www.qt.io/download-open-source/#section-6): A popular open-source IDE.
  > **Note** The installation scripts automatically install *Qt Creator* as part of the common dependencies. You can launch it by entering `qtcreator` in a bash terminal.

## Next Steps

Once you have finished setting up the environment, continue to the [build instructions](../setup/building_px4.md).
