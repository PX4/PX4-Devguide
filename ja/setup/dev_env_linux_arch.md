# Development Environment on Arch Linux

![Gazebo on Arch](../../assets/gazebo/arch-gazebo.png)

> **Note** The instructions have been tested on [Manjaro](https://manjaro.org/) (Arch based distribution) as it is much easier to set up than Arch Linux.

Using the the setup script provided in the Firmware repository is very convenient to set your Arch installation up for PX4 development. It installs all tools to build PX4 (without RTPS) for NuttX targets and run simulation with jMAVsim or gazebo. Here are the steps to get you started from scratch.

## Script parameters

- `--gazebo`   
    Gazebo simulation is not installed by default, add this parameter parameter to install gazebo from the [AUR](https://aur.archlinux.org/packages/gazebo/). Note that gazebo gets compiled from source and therefore takes some time to install and requires entering the sudo password multiple times for dependencies.
- `--no-nuttx`   
    If you only plan to use simulation you can omit the microcontroller target toolchain with this parameter.
- `--no-sim-tools`   
    If you only plan to use build for microcontroller targets you can omit all simulatior tools with this parameter.

## Option 1 Clone PX4, install Toolchain

```sh
git clone https://github.com/PX4/Firmware.git
source Firmware/Tools/setup/arch.sh # optionally append --gazebo
```

## Option 2 only install Toolchain

```sh
wget https://raw.githubusercontent.com/PX4/Firmware/master/Tools/setup/arch.sh
source arch.sh # optionally append --gazebo
```