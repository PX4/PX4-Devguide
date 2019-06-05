# Development Environment on Linux

Linux allows you to build for [all PX4 targets](../setup/dev_env.md#supported-targets) (NuttX based hardware, Qualcomm Snapdragon Flight hardware, Linux-based hardware, Simulation, ROS).

> **Tip** [Ubuntu Linux LTS](https://wiki.ubuntu.com/LTS) 16.04 is the tested/supported Linux distribution for most development.
  Ubuntu 18.04 LTS with ROS Melodic is used for [ROS development](#ros). 
  Instructions are also provided for [CentOS](../setup/dev_env_linux_centos.md) and [Arch Linux](../setup/dev_env_linux_arch.md).

The following instructions explain how to set up a development environment on Ubuntu LTS using convenience bash scripts. 
Instructions for *manually installing* these and additional targets can be found in [Ubuntu/Debian Linux](../setup/dev_env_linux_ubuntu.md).


## Development Toolchain

The instructions below show how you can use our [convenience bash scripts](../setup/dev_env_linux_ubuntu.md#convenience-bash-scripts) to setup the developer toolchain on Ubuntu LTS. 
All the scripts install the *Qt Creator IDE*, [Ninja Build System](https://ninja-build.org/), [Common Dependencies](../setup/dev_env_linux_ubuntu.md#common-dependencies), [FastRTPS](../setup/dev_env_linux_ubuntu.md#fastrtps-installation), and also download the PX4 source to your computer (**~/src/Firmware**).

> **Tip** The scripts have been tested on clean Ubuntu LTS 16.04 and Ubuntu LTS 18.04 installations.
  They *may* not work as expected if installed on top of an existing system or on another Ubuntu release. 
  If you have any problems then follow the [manual installation instructions](../setup/dev_env_linux_ubuntu.md).

First make the user a member of the group "dialout":
1. On the command prompt enter:
   ```sh
   sudo usermod -a -G dialout $USER
   ```
1. Logout and login again (the change is only made after a new login).

Then follow the instructions for your development target in the sections below.


### Pixhawk/NuttX (and jMAVSim)

To install the development toolchain:

1. Download the script in a bash shell:
   ```bash
   wget https://raw.githubusercontent.com/PX4/Devguide/master/build_scripts/ubuntu_sim_nuttx.sh
   ```
1. Run the script:
   ```bash
   source ubuntu_sim_nuttx.sh
   ```
   You may need to acknowledge some prompts as the script progresses.
1. Restart the computer on completion.


### Snapdragon Flight

Setup instructions for Snapdragon Flight are provided in the *PX4 User Guide*:
* [Development Environment](https://docs.px4.io/en/flight_controller/snapdragon_flight_dev_environment_installation.html)
* [Software Installation](https://docs.px4.io/en/flight_controller/snapdragon_flight_software_installation.html)
* [Configuration](https://docs.px4.io/en/flight_controller/snapdragon_flight_configuration.html)

   
### Raspberry Pi

To install the development toolchain:
1. Download the script in a bash shell (this contains the jMAVSim simulator and common toolchain dependencies):
   ```bash
   wget https://raw.githubusercontent.com/PX4/Devguide/master/build_scripts/ubuntu_sim_common_deps.sh
   ```
1. Run the script:
   ```bash
   source ubuntu_sim_common_deps.sh
   ```
   You may need to acknowledge some prompts as the script progresses.
1. Follow setup instructions in [Ubuntu/Debian Linux](../setup/dev_env_linux_ubuntu.md) for [Raspberry Pi](../setup/dev_env_linux_ubuntu.md#raspberry-pi-hardware).

### Parrot Bepop

Follow the (manual) instructions here: [Ubuntu/Debian Linux > Parrot Bebop](../setup/dev_env_linux_ubuntu.md#raspberry-pi-hardware).


### jMAVSim/Gazebo Simulation

To install the Gazebo9 and jMAVSim simulators:

1. Download the script in a bash shell:
   ```bash
   wget https://raw.githubusercontent.com/PX4/Devguide/master/build_scripts/ubuntu_sim.sh
   ```
1. Run the script:
   ```bash
   source ubuntu_sim.sh
   ```
   You may need to acknowledge some prompts as the script progresses.

> **Tip** If you just need jMAVSim, instead download and run <a href="https://raw.githubusercontent.com/PX4/Devguide/{{ book.px4_version }}/build_scripts/ubuntu_sim_common_deps.sh" target="_blank" download>ubuntu_sim_common_deps.sh</a>.

<span><span>
> **Note** PX4 works with Gazebo 7, 8, and 9. 
  The script installs Gazebo 9.

### Gazebo with ROS Melodic {#ros}

> **Note** PX4 is tested with ROS Melodic on Ubuntu 18.04 LTS.
  ROS Melodic does not work on Ubuntu 16.04.


To install the development toolchain:

1. Download the script in a bash shell:
   ```bash
   wget https://raw.githubusercontent.com/PX4/Devguide/master/build_scripts/ubuntu_sim_ros_melodic.sh
   ```
1. Run the script:
   ```bash
   source ubuntu_sim_ros_melodic.sh
   ```
   You may need to acknowledge some prompts as the script progresses.

Note: 
* ROS Melodic is installed with Gazebo9 by default.
* Your catkin (ROS build system) workspace is created at **~/catkin_ws/**.

## Additional Tools

After setting up the build/simulation toolchain, see [Additional Tools](../setup/generic_dev_tools.md) for information about other useful tools.

## Next Steps

Once you have finished setting up the environment, continue to the [build instructions](../setup/building_px4.md).
